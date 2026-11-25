"""二维码引导的定点泊车与精准停靠。

订阅：
    /camera/color/image_raw     sensor_msgs/Image
    /camera/depth/image_raw     sensor_msgs/Image（可选，用于测距）
    /control/parking_enable     std_msgs/Bool   true 开始泊车，false 中止

发布：
    /cmd_vel_park               geometry_msgs/Twist  泊车速度（经仲裁器）
    /control/parking_active     std_msgs/Bool        是否接管底盘
    /control/parking_state      std_msgs/String      JSON 状态
    /mission/parked             std_msgs/Bool        完成=true / 超时=false

流程：原地慢转找码 → 转到画面中心 → 低速直行靠近 →
      码足够大或深度 ≤ target_distance → 停稳 hold_sec 秒 → 完成。

对应功能：⑨ 定点泊车与精准投递（不含机械抓取）。
现场标定：
    1. 把投递点二维码贴在车上相机高度；
    2. 先看 /control/parking_state 的 height_ratio，把 target_height_ratio
       设为“距离合适时”的实测值；
    3. 若 Astra 深度可用，优先用 target_distance（米），更稳。
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from parking.core import FAILED, DockController

try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except Exception:
    pyzbar = None
    HAS_PYZBAR = False


class Parking(Node):
    def __init__(self):
        super().__init__('parking')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('marker_source', 'qr')
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_scale', 0.0)  # 0=自动

        self.declare_parameter('kp_angular', 1.4)
        self.declare_parameter('center_tol', 0.12)
        self.declare_parameter('align_exit_tol', 0.20)
        self.declare_parameter('target_height_ratio', 0.28)
        self.declare_parameter('target_distance', 0.45)
        self.declare_parameter('approach_speed', 0.16)
        self.declare_parameter('search_speed', 0.35)
        self.declare_parameter('search_direction', 1.0)
        self.declare_parameter('hold_sec', 1.5)
        self.declare_parameter('timeout', 35.0)

        self.declare_parameter('detect_rate', 5.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('marker_timeout', 0.8)
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('debug', True)
        # zbar 对斜着的码很挑角度，直接解不出时按这些角度补转再试。
        # 和 vision_detect 的默认值保持一致。
        self.declare_parameter('qr_retry_angles',
                               [10.0, -10.0, 20.0, -20.0,
                                30.0, -30.0, 45.0, -45.0])

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.use_depth = bool(self.get_parameter('use_depth').value)
        self.depth_scale_param = float(self.get_parameter('depth_scale').value)
        self.detect_interval = 1.0 / max(
            float(self.get_parameter('detect_rate').value), 1.0)
        self.control_rate = max(
            float(self.get_parameter('control_rate').value), 1.0)
        self.marker_timeout = float(
            self.get_parameter('marker_timeout').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.qr_retry_angles = [
            float(a) for a in
            (self.get_parameter('qr_retry_angles').value or [])]

        self.controller = DockController(
            kp_angular=float(self.get_parameter('kp_angular').value),
            center_tol=float(self.get_parameter('center_tol').value),
            align_exit_tol=float(self.get_parameter('align_exit_tol').value),
            target_height_ratio=float(
                self.get_parameter('target_height_ratio').value),
            target_distance=float(
                self.get_parameter('target_distance').value),
            approach_speed=float(
                self.get_parameter('approach_speed').value),
            search_speed=float(self.get_parameter('search_speed').value),
            search_direction=float(
                self.get_parameter('search_direction').value),
            hold_sec=float(self.get_parameter('hold_sec').value),
            timeout=float(self.get_parameter('timeout').value))

        self.bridge = CvBridge()
        self._frame = None
        self._depth = None
        self._depth_scale = 0.001
        self._active = bool(self.get_parameter('auto_enable').value)
        self._marker = None
        self._marker_time = 0.0
        self._last_detect = 0.0
        self._last_qr_angle = 0.0
        self._last_state = None
        self._last_output = None

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_park', 10)
        self.pub_active = self.create_publisher(
            Bool, '/control/parking_active', 10)
        self.pub_state = self.create_publisher(
            String, '/control/parking_state', 10)
        self.pub_parked = self.create_publisher(Bool, '/mission/parked', 10)

        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(
                Image, str(self.get_parameter('depth_topic').value),
                self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/control/parking_enable', self.on_enable, 10)
        self.create_timer(1.0 / self.control_rate, self.tick)

        if not HAS_PYZBAR:
            self.get_logger().error(
                '未安装 pyzbar，泊车无法识别二维码。')
        if self._active:
            self._activate()
        self.get_logger().info(
            '泊车节点启动：目标高度比 %.2f 目标距离 %.2fm 搜索速度 %.2f' % (
                self.controller.target_height_ratio,
                self.controller.target_distance,
                self.controller.search_speed))

    # ------------------------------------------------------------------
    def on_image(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('彩色图转换失败：%s' % exc,
                                    throttle_duration_sec=10.0)

    def on_depth(self, msg):
        try:
            self._depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error('深度图转换失败：%s' % exc,
                                    throttle_duration_sec=10.0)
            return
        if self.depth_scale_param > 0.0:
            self._depth_scale = self.depth_scale_param
        elif msg.encoding == '32FC1':
            self._depth_scale = 1.0
        else:
            self._depth_scale = 0.001

    def on_enable(self, msg):
        if msg.data and not self._active:
            self._activate()
        elif not msg.data and self._active:
            self._deactivate('manual stop')

    def _activate(self):
        self._active = True
        self.controller.reset()
        self._marker = None
        self._marker_time = 0.0
        self._last_detect = 0.0
        self._last_state = None
        self._last_output = None
        self.pub_active.publish(Bool(data=True))
        self.get_logger().info('泊车开始：搜索二维码')

    def _deactivate(self, reason):
        self._active = False
        self._marker = None
        self.pub_cmd.publish(Twist())
        self.pub_active.publish(Bool(data=False))
        self.get_logger().info('泊车结束：%s' % reason)

    # ------------------------------------------------------------------
    def _detect_marker(self, now):
        if not HAS_PYZBAR or self._frame is None:
            return
        if now - self._last_detect < self.detect_interval:
            return
        self._last_detect = now
        gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)
        try:
            results = pyzbar.decode(gray)
        except Exception as exc:
            self.get_logger().warning('二维码解码失败：%s' % exc,
                                      throttle_duration_sec=10.0)
            return

        angle = 0.0
        if not results and self.qr_retry_angles:
            results, angle = self._decode_rotated(gray)
        if not results:
            return

        item = max(results, key=lambda r: r.rect.width * r.rect.height)
        rect = item.rect
        cx = float(rect.left + rect.width / 2.0)
        cy = float(rect.top + rect.height / 2.0)
        if angle:
            # 补转后 zbar 量到的坐标是“转过之后”那帧里的，必须换回原帧，
            # 否则对准偏差和深度采样都会落到错误位置
            cx, cy = self._unrotate_point(
                cx, cy, angle, gray.shape[1], gray.shape[0])
        self._marker = (cx, cy, float(rect.width), float(rect.height))
        self._marker_time = now

    def _decode_rotated(self, gray):
        """按 qr_retry_angles 逐个补转尝试，返回 (结果, 使用的角度)。"""
        height, width = gray.shape[:2]
        center = (width / 2.0, height / 2.0)
        # 上次成功的角度放最前面：车绕着码转时角度变化慢，通常一次就中，
        # 找不到码的搜索阶段才不会每次都把 8 个角度全试一遍
        angles = list(self.qr_retry_angles)
        if self._last_qr_angle in angles:
            angles.remove(self._last_qr_angle)
            angles.insert(0, self._last_qr_angle)
        for angle in angles:
            matrix = cv2.getRotationMatrix2D(center, float(angle), 1.0)
            rotated = cv2.warpAffine(
                gray, matrix, (width, height),
                flags=cv2.INTER_LINEAR, borderValue=255)
            try:
                results = pyzbar.decode(rotated)
            except Exception:
                continue
            if results:
                self._last_qr_angle = float(angle)
                return results, float(angle)
        return [], 0.0

    @staticmethod
    def _unrotate_point(cx, cy, angle, width, height):
        """getRotationMatrix2D 的逆变换：补转图坐标 -> 原图坐标。"""
        theta = math.radians(angle)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        tx = (1.0 - cos_t) * width / 2.0 - sin_t * height / 2.0
        ty = sin_t * width / 2.0 + (1.0 - cos_t) * height / 2.0
        x = cx - tx
        y = cy - ty
        return (cos_t * x - sin_t * y, sin_t * x + cos_t * y)

    def _depth_at_marker(self):
        if not self.use_depth or self._depth is None or self._marker is None:
            return None
        arr = self._depth
        if arr.ndim != 2 or arr.size == 0:
            return None
        cx, cy, w, h = self._marker
        height, width = arr.shape
        half_w = max(2, int(w * 0.25))
        half_h = max(2, int(h * 0.25))
        x0 = max(0, int(cx) - half_w)
        x1 = min(width, int(cx) + half_w)
        y0 = max(0, int(cy) - half_h)
        y1 = min(height, int(cy) + half_h)
        if x1 <= x0 or y1 <= y0:
            return None
        roi = arr[y0:y1, x0:x1].astype(np.float32) * self._depth_scale
        roi = roi[np.isfinite(roi)]
        roi = roi[roi > 0.05]
        roi = roi[roi < 5.0]
        if roi.size < 4:
            return None
        return float(np.median(roi))

    def tick(self):
        if not self._active:
            return

        now = time.monotonic()
        self._detect_marker(now)

        marker = None
        if (self._marker is not None
                and now - self._marker_time <= self.marker_timeout):
            marker = self._marker

        frame_w = self._frame.shape[1] if self._frame is not None else 0
        frame_h = self._frame.shape[0] if self._frame is not None else 0
        depth_m = self._depth_at_marker()

        out = self.controller.update(
            now, marker, frame_w, frame_h, depth_m=depth_m)

        cmd = Twist()
        cmd.linear.x = float(out.linear)
        cmd.angular.z = float(out.angular)
        self.pub_cmd.publish(cmd)

        payload = {
            'active': True,
            'state': out.state,
            'progress': round(float(out.progress), 3),
            'error_x': round(float(out.error_x), 3),
            'height_ratio': round(float(out.height_ratio), 3),
            'distance': round(float(out.distance), 3),
            'has_marker': marker is not None,
        }
        self.pub_state.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

        if out.state != self._last_state:
            self._last_state = out.state
            self.get_logger().info(
                '泊车状态 -> %s（进度 %.0f%% 偏差 %.2f 高度比 %.2f 距离 %.2fm）' % (
                    out.state, out.progress * 100.0, out.error_x,
                    out.height_ratio, out.distance))

        if out.done:
            self.pub_cmd.publish(Twist())
            self.pub_parked.publish(Bool(data=True))
            self._deactivate('到达投递点')
        elif out.state == FAILED:
            self.pub_cmd.publish(Twist())
            self.pub_parked.publish(Bool(data=False))
            self._deactivate('超时未找到二维码')

        if self.debug:
            self._last_output = out

    def destroy_node(self):
        try:
            self.pub_cmd.publish(Twist())
            self.pub_active.publish(Bool(data=False))
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Parking()
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
