"""雷达 + 深度视觉避障决策。

订阅：
    /scan                       sensor_msgs/LaserScan
    /camera/depth/image_raw     sensor_msgs/Image（可选，Astra 深度）

发布：
    /control/avoid_scale        std_msgs/Float32   巡线速度缩放 0~1
    /control/avoid_bias         std_msgs/Float32   附加角速度 rad/s
    /control/avoid_stop         std_msgs/Bool      紧急停车
    /control/avoid_state        std_msgs/String    JSON 详细状态
    /control/avoid_cmd          geometry_msgs/Twist 仅用于 HUD/记录
    /perception/obstacle_distance std_msgs/Float32 最近障碍距离（米，-1=未知）

思想：避障不直接抢 /cmd_vel，而是给仲裁器提供“缩放 + 偏置 + 急停”，
巡线仍然负责把车拉回线上，两者不会互相覆盖。

对应功能：⑪ 视觉/雷达避障。
"""

import json
import math
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String

from obstacle_avoider.core import (CLEAR, SLOW, STOP, depth_roi_min,
                                   evaluate_scan, state_for)


class ObstacleAvoider(Node):
    """把雷达/深度数据压缩成速度缩放、转向偏置和急停信号。"""

    def __init__(self):
        super().__init__('obstacle_avoider')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('enable_scan', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('depth_scale', 0.0)  # 0=按编码自动

        self.declare_parameter('forward_half_deg', 30.0)
        self.declare_parameter('side_half_deg', 75.0)
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.85)
        self.declare_parameter('min_valid', 0.05)
        self.declare_parameter('max_valid', 8.0)
        self.declare_parameter('min_scale', 0.15)
        self.declare_parameter('turn_gain', 1.2)
        self.declare_parameter('max_bias', 0.9)

        self.declare_parameter('depth_x_min', 0.35)
        self.declare_parameter('depth_x_max', 0.65)
        self.declare_parameter('depth_y_min', 0.35)
        self.declare_parameter('depth_y_max', 0.65)
        self.declare_parameter('depth_stop_distance', 0.20)
        self.declare_parameter('depth_slow_distance', 0.50)

        self.declare_parameter('scan_timeout', 0.6)
        self.declare_parameter('depth_timeout', 0.6)
        self.declare_parameter('stop_hold_sec', 0.5)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('debug', True)

        self.enable_scan = bool(self.get_parameter('enable_scan').value)
        self.enable_depth = bool(self.get_parameter('enable_depth').value)
        self.depth_scale_param = float(self.get_parameter('depth_scale').value)
        self.forward_half = float(self.get_parameter('forward_half_deg').value)
        self.side_half = float(self.get_parameter('side_half_deg').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.min_valid = float(self.get_parameter('min_valid').value)
        self.max_valid = float(self.get_parameter('max_valid').value)
        self.min_scale = float(self.get_parameter('min_scale').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.max_bias = float(self.get_parameter('max_bias').value)
        self.depth_stop = float(
            self.get_parameter('depth_stop_distance').value)
        self.depth_slow = float(
            self.get_parameter('depth_slow_distance').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.depth_timeout = float(self.get_parameter('depth_timeout').value)
        self.stop_hold = float(self.get_parameter('stop_hold_sec').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.bridge = CvBridge()
        self._scan = None
        self._scan_time = 0.0
        self._depth = None
        self._depth_scale = 0.001
        self._depth_time = 0.0
        self._hold_until = 0.0
        self._last_state = CLEAR
        self._last_log = 0.0

        self.pub_scale = self.create_publisher(
            Float32, '/control/avoid_scale', 10)
        self.pub_bias = self.create_publisher(
            Float32, '/control/avoid_bias', 10)
        self.pub_stop = self.create_publisher(Bool, '/control/avoid_stop', 10)
        self.pub_state = self.create_publisher(
            String, '/control/avoid_state', 10)
        self.pub_cmd = self.create_publisher(Twist, '/control/avoid_cmd', 10)
        self.pub_distance = self.create_publisher(
            Float32, '/perception/obstacle_distance', 10)

        if self.enable_scan:
            self.create_subscription(
                LaserScan, str(self.get_parameter('scan_topic').value),
                self.on_scan, qos_profile_sensor_data)
        if self.enable_depth:
            self.create_subscription(
                Image, str(self.get_parameter('depth_topic').value),
                self.on_depth, qos_profile_sensor_data)

        rate = max(float(self.get_parameter('rate').value), 1.0)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            '避障启动：雷达=%s 深度=%s 停车阈值=%.2fm 减速阈值=%.2fm' % (
                self.enable_scan, self.enable_depth,
                self.stop_distance, self.slow_distance))

    # ------------------------------------------------------------------
    def on_scan(self, msg):
        self._scan = msg
        self._scan_time = time.monotonic()

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error('深度图转换失败：%s' % exc,
                                    throttle_duration_sec=10.0)
            return
        self._depth = depth
        self._depth_time = time.monotonic()
        if self.depth_scale_param > 0.0:
            self._depth_scale = self.depth_scale_param
        elif msg.encoding == '32FC1':
            self._depth_scale = 1.0
        else:
            self._depth_scale = 0.001

    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        scan_decision = None
        depth_distance = math.inf
        source = 'none'

        if (self.enable_scan and self._scan is not None
                and now - self._scan_time <= self.scan_timeout):
            scan_decision = evaluate_scan(
                self._scan.ranges,
                self._scan.angle_min,
                self._scan.angle_increment,
                forward_half_deg=self.forward_half,
                side_half_deg=self.side_half,
                stop_distance=self.stop_distance,
                slow_distance=self.slow_distance,
                min_valid=self.min_valid,
                max_valid=self.max_valid,
                min_scale=self.min_scale,
                turn_gain=self.turn_gain,
                max_bias=self.max_bias)
            source = 'scan'

        if (self.enable_depth and self._depth is not None
                and now - self._depth_time <= self.depth_timeout):
            depth_distance = depth_roi_min(
                self._depth,
                x_min=float(self.get_parameter('depth_x_min').value),
                x_max=float(self.get_parameter('depth_x_max').value),
                y_min=float(self.get_parameter('depth_y_min').value),
                y_max=float(self.get_parameter('depth_y_max').value),
                min_valid=self.min_valid,
                max_valid=min(self.max_valid, 4.0),
                depth_scale=self._depth_scale)
            if math.isfinite(depth_distance):
                source = 'scan+depth' if source == 'scan' else 'depth'

        state = CLEAR
        scale = 1.0
        bias = 0.0
        distance = math.inf

        if scan_decision is not None:
            state = scan_decision.state
            scale = scan_decision.scale
            bias = scan_decision.bias
            distance = scan_decision.front_min

        if math.isfinite(depth_distance):
            depth_state, depth_scale = state_for(
                depth_distance, self.depth_stop, self.depth_slow,
                min_scale=self.min_scale)
            if _severity(depth_state) > _severity(state):
                state = depth_state
            scale = min(scale, depth_scale)
            distance = min(distance, depth_distance)

        if state == STOP:
            self._hold_until = max(self._hold_until, now + self.stop_hold)
        elif now < self._hold_until:
            state = STOP
            scale = 0.0
            bias = 0.0

        if state != self._last_state:
            self._last_state = state
            if state == STOP:
                self.get_logger().warning(
                    '前方 %.2fm，停车' % (distance if math.isfinite(distance) else -1))
            elif state == SLOW:
                self.get_logger().info(
                    '前方 %.2fm，减速到 %.0f%%，偏置 %.2f' % (
                        distance, scale * 100.0, bias))
            else:
                self.get_logger().info('前方通畅，恢复巡线')

        self.pub_scale.publish(Float32(data=float(scale)))
        self.pub_bias.publish(Float32(data=float(bias if state == SLOW else 0.0)))
        self.pub_stop.publish(Bool(data=(state == STOP)))
        self.pub_distance.publish(
            Float32(data=float(distance if math.isfinite(distance) else -1.0)))

        cmd = Twist()
        cmd.angular.z = float(bias if state == SLOW else 0.0)
        self.pub_cmd.publish(cmd)

        payload = {
            'state': state,
            'distance': round(float(distance), 3) if math.isfinite(distance)
            else -1.0,
            'scale': round(float(scale), 3),
            'bias': round(float(bias if state == SLOW else 0.0), 3),
            'source': source,
            'scan_age': round(now - self._scan_time, 2) if self._scan_time else -1.0,
            'depth_age': round(now - self._depth_time, 2) if self._depth_time else -1.0,
        }
        self.pub_state.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

        if self.debug and now - self._last_log >= 5.0:
            self._last_log = now
            self.get_logger().info(
                '避障状态 %s 距离 %.2fm 来源 %s' % (
                    state, distance if math.isfinite(distance) else -1.0,
                    source))


def _severity(state):
    return {STOP: 3, SLOW: 2, CLEAR: 1}.get(state, 0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleAvoider()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
