"""速度指令仲裁器：全车唯一向 /cmd_vel 发布速度的节点。

订阅：
    /cmd_vel_line          geometry_msgs/Twist  巡线输出
    /cmd_vel_park          geometry_msgs/Twist  泊车输出
    /control/parking_active std_msgs/Bool       泊车是否接管
    /control/avoid_scale   std_msgs/Float32     避障速度缩放
    /control/avoid_bias    std_msgs/Float32     避障转向偏置
    /control/avoid_stop    std_msgs/Bool        急停
    /control/nav_active    std_msgs/Bool        任务导航阶段（nav2 接管 /cmd_vel）

发布：
    /cmd_vel               geometry_msgs/Twist
    /control/arbiter_state std_msgs/String      JSON 状态（HUD 用）

优先级：急停 > 导航 > 泊车 > 巡线（乘避障缩放 + 加偏置）> 停车。
任何来源超过超时时间没更新都会自动落回 0，防止话题断流后车继续跑。

nav_active=true 时（mission_manager 正在让 nav2 走过去）本节点不输出速度，
只发状态，由 nav2 单独占 /cmd_vel，避免两个节点抢方向盘；
但避障急停仍然生效（急停时输出 0 覆盖 nav2）。
"""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from cmd_vel_arbiter.core import CommandArbiter


class CmdVelArbiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_arbiter')

        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('line_timeout', 0.5)
        self.declare_parameter('park_timeout', 0.5)
        self.declare_parameter('enable', True)
        self.declare_parameter('debug', True)

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.enabled = bool(self.get_parameter('enable').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.arbiter = CommandArbiter(
            max_linear=float(self.get_parameter('max_linear').value),
            max_angular=float(self.get_parameter('max_angular').value),
            line_timeout=float(self.get_parameter('line_timeout').value),
            park_timeout=float(self.get_parameter('park_timeout').value))

        self._line = None
        self._line_time = 0.0
        self._park = None
        self._park_time = 0.0
        self._park_active = False
        self._avoid_scale = 1.0
        self._avoid_bias = 0.0
        self._emergency = False
        self._nav_active = False
        self._last_mode = None
        self._last_log = 0.0

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.pub_state = self.create_publisher(
            String, '/control/arbiter_state', 10)

        self.create_subscription(Twist, '/cmd_vel_line', self.on_line, 10)
        self.create_subscription(Twist, '/cmd_vel_park', self.on_park, 10)
        self.create_subscription(
            Bool, '/control/parking_active', self.on_park_active, 10)
        self.create_subscription(
            Float32, '/control/avoid_scale', self.on_avoid_scale, 10)
        self.create_subscription(
            Float32, '/control/avoid_bias', self.on_avoid_bias, 10)
        self.create_subscription(
            Bool, '/control/avoid_stop', self.on_avoid_stop, 10)
        self.create_subscription(
            Bool, '/control/nav_active', self.on_nav_active, 10)

        rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            '仲裁器启动：输出=%s 频率=%.0fHz 最大 v=%.2f w=%.2f' % (
                self.output_topic, rate,
                self.arbiter.max_linear, self.arbiter.max_angular))

    def on_line(self, msg):
        self._line = (float(msg.linear.x), float(msg.angular.z))
        self._line_time = time.monotonic()

    def on_park(self, msg):
        self._park = (float(msg.linear.x), float(msg.angular.z))
        self._park_time = time.monotonic()

    def on_park_active(self, msg):
        self._park_active = bool(msg.data)

    def on_avoid_scale(self, msg):
        self._avoid_scale = float(msg.data)

    def on_avoid_bias(self, msg):
        self._avoid_bias = float(msg.data)

    def on_avoid_stop(self, msg):
        self._emergency = bool(msg.data)

    def on_nav_active(self, msg):
        active = bool(msg.data)
        if active != self._nav_active:
            self.get_logger().info(
                '导航阶段 %s /cmd_vel 控制权' % (
                    '让给 nav2，仲裁器暂停输出' if active
                    else '交还仲裁器'))
        self._nav_active = active

    def tick(self):
        now = time.monotonic()

        if self._nav_active and not self._emergency:
            # nav2 自己在发 /cmd_vel，本节点只报状态不抢控制权
            self._publish_state('nav', 0.0, 0.0, now, 'nav2 owns /cmd_vel')
            return

        line = None
        if self._line is not None:
            line = (self._line[0], self._line[1], self._line_time)
        park = None
        if self._park is not None:
            park = (self._park[0], self._park[1], self._park_time)

        if not self.enabled:
            cmd = self.arbiter.compute(now, emergency=True)
            cmd.reason = 'arbiter disabled'
        else:
            cmd = self.arbiter.compute(
                now, line=line, park=park,
                park_active=self._park_active,
                avoid_scale=self._avoid_scale,
                avoid_bias=self._avoid_bias,
                emergency=self._emergency)

        msg = Twist()
        msg.linear.x = float(cmd.linear)
        msg.angular.z = float(cmd.angular)
        self.pub.publish(msg)

        self._publish_state(cmd.mode, cmd.linear, cmd.angular, now,
                            cmd.reason)

        if cmd.mode != self._last_mode:
            self._last_mode = cmd.mode
            self.get_logger().info('仲裁切换：%s（%s）' % (
                cmd.mode, cmd.reason))
        elif (self.debug and cmd.mode == 'line'
              and now - self._last_log > 10.0):
            self._last_log = now
            self.get_logger().info(
                '巡线输出 v=%.2f w=%.2f 避障缩放=%.2f 偏置=%.2f' % (
                    cmd.linear, cmd.angular, self._avoid_scale,
                    self._avoid_bias))

    def _publish_state(self, mode, linear, angular, now, reason):
        payload = {
            'mode': mode,
            'reason': reason,
            'linear': round(float(linear), 3),
            'angular': round(float(angular), 3),
            'line_age': round(now - self._line_time, 2) if self._line_time else -1.0,
            'park_age': round(now - self._park_time, 2) if self._park_time else -1.0,
            'avoid_scale': round(self._avoid_scale, 3),
            'avoid_bias': round(self._avoid_bias, 3),
            'emergency': self._emergency,
            'nav_active': self._nav_active,
        }
        self.pub_state.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def destroy_node(self):
        try:
            self.pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelArbiter()
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
