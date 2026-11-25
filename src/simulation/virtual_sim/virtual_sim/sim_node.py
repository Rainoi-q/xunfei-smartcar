"""二维虚拟仿真节点：不需要 Gazebo，直接产出相机/深度/雷达/里程计。

订阅：
    /cmd_vel     geometry_msgs/Twist   与实车同一接口
    /sim/reset   std_msgs/Bool         true 回到起点

发布：
    /camera/color/image_raw  sensor_msgs/Image (rgb8)
    /camera/depth/image_raw  sensor_msgs/Image (32FC1，米)
    /scan                    sensor_msgs/LaserScan
    /odom                    nav_msgs/Odometry
    /sim/pose                std_msgs/String (JSON)
    TF: map -> odom -> base_link

用途：
    - 对应功能⑭ 虚拟仿真交互；
    - 没有场地/没上电时，用同一套感知与控制节点跑通巡线、避障、二维码、泊车；
    - 演示视频可录制 /hud/image，画面上同时有真实算法输出与仿真场景。

用法见 smartcar_bringup/launch/sim.launch.py。
"""

import array
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from virtual_sim.core import Pose2D, SimCar, World


class VirtualSim(Node):
    def __init__(self):
        super().__init__('virtual_sim')

        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('near', 0.25)
        self.declare_parameter('far', 2.25)
        self.declare_parameter('lateral_span', 1.6)
        self.declare_parameter('size_m', 6.0)
        self.declare_parameter('ppm', 300)

        self.declare_parameter('start_x', 0.90)
        self.declare_parameter('start_y', 1.00)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.5)

        self.declare_parameter('camera_rate', 10.0)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('physics_rate', 50.0)
        self.declare_parameter('odom_rate', 20.0)
        self.declare_parameter('rays', 360)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('range_noise', 0.0)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        # 排查用：每 5 秒打印各定时器实际回调次数和相机耗时
        self.declare_parameter('debug_stats', False)
        # 演示用：任务 DONE/FAILED 后自动复位，循环跑（网页上一直是活动的）
        self.declare_parameter('auto_loop', False)
        self.declare_parameter('loop_delay', 6.0)

        self.world = World(
            size_m=float(self.get_parameter('size_m').value),
            ppm=int(self.get_parameter('ppm').value),
            near=float(self.get_parameter('near').value),
            far=float(self.get_parameter('far').value),
            lateral_span=float(self.get_parameter('lateral_span').value),
            camera_w=int(self.get_parameter('width').value),
            camera_h=int(self.get_parameter('height').value))
        start = Pose2D(
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
            float(self.get_parameter('start_yaw').value))
        self.car = SimCar(
            start,
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value))

        self.camera_rate = max(float(self.get_parameter('camera_rate').value), 1.0)
        self.scan_rate = max(float(self.get_parameter('scan_rate').value), 1.0)
        self.physics_rate = max(float(self.get_parameter('physics_rate').value), 5.0)
        self.odom_rate = max(float(self.get_parameter('odom_rate').value), 1.0)
        self.rays = max(int(self.get_parameter('rays').value), 30)
        self.max_range = float(self.get_parameter('max_range').value)
        self.range_noise = max(float(self.get_parameter('range_noise').value), 0.0)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)

        self._cmd = (0.0, 0.0)
        self._last_v = 0.0
        self._last_w = 0.0
        self._last_time = time.monotonic()
        self._last_collision_log = 0.0

        self.pub_image = self.create_publisher(
            Image, str(self.get_parameter('camera_topic').value),
            qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(
            Image, str(self.get_parameter('depth_topic').value),
            qos_profile_sensor_data)
        self.pub_scan = self.create_publisher(
            LaserScan, str(self.get_parameter('scan_topic').value),
            qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 10)
        self.pub_pose = self.create_publisher(String, '/sim/pose', 10)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(
            Twist, str(self.get_parameter('cmd_topic').value),
            self.on_cmd, 10)
        self.create_subscription(Bool, '/sim/reset', self.on_reset, 10)
        # auto_loop 用：任务结束后复位整个流程（仿真车 + 任务状态机 + 巡线）
        self.pub_mission_reset = self.create_publisher(
            Bool, '/mission/reset', 10)
        self.pub_line_enable = self.create_publisher(
            Bool, '/control/line_enable', 10)
        self.create_subscription(
            String, '/mission/status', self.on_mission_status, 10)

        self.debug_stats = bool(self.get_parameter('debug_stats').value)
        self.auto_loop = bool(self.get_parameter('auto_loop').value)
        self.loop_delay = max(float(self.get_parameter('loop_delay').value), 0.0)
        self._done_since = None
        self._loop_line_at = 0.0
        self._stats = {'physics': 0, 'camera': 0, 'scan': 0, 'odom': 0}
        self._cam_ms = 0.0
        self._cam_parts = (0.0,) * 7
        self._stats_t = time.monotonic()

        self.create_timer(1.0 / self.physics_rate, self.physics)
        self.create_timer(1.0 / self.camera_rate, self.publish_camera)
        self.create_timer(1.0 / self.scan_rate, self.publish_scan)
        self.create_timer(1.0 / self.odom_rate, self.publish_odom)
        self.create_timer(5.0, self._report_stats)
        self.create_timer(1.0, self._maybe_loop)

        self.get_logger().info(
            '虚拟仿真启动：世界 %.1fm 起点(%.2f, %.2f, %.2f) 相机 %.0fx%.0f@%.0fHz' % (
                self.world.size_m, start.x, start.y, start.theta,
                self.world.camera_w, self.world.camera_h, self.camera_rate))

    # ------------------------------------------------------------------
    def on_cmd(self, msg):
        self._cmd = (float(msg.linear.x), float(msg.angular.z))

    def on_reset(self, msg):
        if msg.data:
            self.car.reset()
            self._cmd = (0.0, 0.0)
            self.get_logger().info('仿真车已复位')

    def on_mission_status(self, msg):
        try:
            state = str(json.loads(msg.data).get('state', ''))
        except (ValueError, TypeError):
            return
        if state in ('DONE', 'FAILED'):
            if self._done_since is None:
                self._done_since = time.monotonic()
        else:
            self._done_since = None

    def _maybe_loop(self):
        """auto_loop：任务结束后等 loop_delay 秒，复位整车状态再跑一轮。"""
        if not self.auto_loop:
            return
        now = time.monotonic()
        if self._loop_line_at and now >= self._loop_line_at:
            # 等 mission_node 的 on_reset 先把它自己的 line_enable=false 发完，
            # 再恢复巡线，否则两条消息先后顺序不定，车会一直停着
            self._loop_line_at = 0.0
            self.pub_line_enable.publish(Bool(data=True))
        if self._done_since is None:
            return
        if now - self._done_since < self.loop_delay:
            return
        self._done_since = None
        self.on_reset(Bool(data=True))
        self.pub_mission_reset.publish(Bool(data=True))
        self._loop_line_at = now + 1.5
        self.get_logger().info('auto_loop：任务已结束，复位后重新跑一轮')

    # ------------------------------------------------------------------
    def _report_stats(self):
        now = time.monotonic()
        span = max(now - self._stats_t, 1e-3)
        self._stats_t = now
        stats = dict(self._stats)
        for key in self._stats:
            self._stats[key] = 0
        if not bool(self.get_parameter('debug_stats').value):
            return
        parts = self._cam_parts
        self.get_logger().info(
            'SIM STATS 5s: physics %.1f/s camera %.1f/s scan %.1f/s odom %.1f/s '
            'camera %.0fms = rnd %.0f stamp %.0f build %.0f pub %.0f '
            'drnd %.0f dbuild %.0f dpub %.0f' % (
                stats['physics'] / span, stats['camera'] / span,
                stats['scan'] / span, stats['odom'] / span, self._cam_ms,
                parts[0], parts[1], parts[2], parts[3],
                parts[4], parts[5], parts[6]))

    def physics(self):
        self._stats['physics'] += 1
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        if dt <= 0.0:
            dt = 1.0 / self.physics_rate
        elif dt > 0.5:
            # 单线程 executor 里相机回调很重（RDK X5 上每帧 ~100ms），
            # 物理定时器会被推迟到 ~10Hz；此时若把 dt 强改成 1/50s，
            # 仿真车会变成 5~10 倍慢动作。真卡停后再按 0.5s 封顶。
            dt = 0.5

        before = Pose2D(self.car.pose.x, self.car.pose.y, self.car.pose.theta)
        v, w = self.car.step(self._cmd[0], self._cmd[1], dt)
        self._last_v, self._last_w = v, w

        if self.car.collides(self.world):
            self.car.pose = before
            self._cmd = (0.0, 0.0)
            self._last_v = self._last_w = 0.0
            if now - self._last_collision_log > 2.0:
                self._last_collision_log = now
                self.get_logger().warning('仿真车撞到障碍物，已停止')
        else:
            self.car.clamp_to_world(self.world)

    def publish_camera(self):
        self._stats['camera'] += 1
        t0 = time.monotonic()
        frame = self.world.render_camera(self.car.pose)
        t1 = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        t2 = time.monotonic()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        # 绝不能用 bytes 赋值：rclpy 会对 Sequence 做逐元素校验，
        # 921600 字节要 0.7 秒；array.array('B') 走快路径，约 1 ms
        msg.data = array.array('B', frame.tobytes())
        t3 = time.monotonic()
        self.pub_image.publish(msg)
        t4 = time.monotonic()

        depth = self.world.render_depth(self.car.pose).astype(np.float32)
        t5 = time.monotonic()
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.camera_frame
        depth_msg.height, depth_msg.width = depth.shape[:2]
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = 0
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = array.array('B', depth.tobytes())
        t6 = time.monotonic()
        self.pub_depth.publish(depth_msg)
        t7 = time.monotonic()
        self._cam_ms = (t7 - t0) * 1000.0
        ms = lambda a, b: (b - a) * 1000.0
        self._cam_parts = (ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t3, t4),
                           ms(t4, t5), ms(t5, t6), ms(t6, t7))

    def publish_scan(self):
        self._stats['scan'] += 1
        ranges, angle_min, increment = self.world.scan(
            self.car.pose, rays=self.rays, max_range=self.max_range)
        if self.range_noise > 0.0:
            rng = np.random.default_rng(2026)
            noise = rng.normal(0.0, self.range_noise, len(ranges))
            ranges = [float(max(0.05, min(self.max_range, r + n)))
                      for r, n in zip(ranges, noise)]

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.angle_min = angle_min
        msg.angle_max = angle_min + increment * (len(ranges) - 1)
        msg.angle_increment = increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.scan_rate
        msg.range_min = 0.05
        msg.range_max = self.max_range
        msg.ranges = ranges
        msg.intensities = []
        self.pub_scan.publish(msg)

    def publish_odom(self):
        self._stats['odom'] += 1
        pose = self.car.pose
        stamp = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(pose.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose.theta / 2.0)
        msg.twist.twist.linear.x = self._last_v
        msg.twist.twist.angular.z = self._last_w
        self.pub_odom.publish(msg)

        self.tf.sendTransform(self._tf(self.map_frame, self.odom_frame,
                                       stamp, 0.0, 0.0, 0.0))
        self.tf.sendTransform(self._tf(self.odom_frame, self.base_frame,
                                       stamp, pose.x, pose.y, pose.theta))

        self.pub_pose.publish(String(data=json.dumps({
            'x': round(pose.x, 3),
            'y': round(pose.y, 3),
            'yaw': round(pose.theta, 3),
            'v': round(self._last_v, 3),
            'w': round(self._last_w, 3),
        }, ensure_ascii=False)))

    @staticmethod
    def _tf(parent, child, stamp, x, y, yaw):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VirtualSim()
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
