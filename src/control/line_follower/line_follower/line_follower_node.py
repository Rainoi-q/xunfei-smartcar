"""巡线控制节点（PID）。

订阅：
    /perception/line   std_msgs/Float32
        横向偏移，范围 [-1, 1]，正值表示线在画面右侧，NaN 表示丢失。
    /control/line_enable std_msgs/Bool
        false=暂停巡线（任务开始后由 mission_manager 关闭，
        nav2 导航/泊车阶段不允许巡线抢方向盘）；true=恢复。

发布：
    /cmd_vel_line      geometry_msgs/Twist
        注意：不直接发 /cmd_vel。最终速度由 cmd_vel_arbiter 仲裁，
        以便避障和泊车能安全接管。

逻辑：
    angular = -(kp*e + ki*∫e dt + kd*de/dt)
    偏移越大越减速，保证转弯时不会冲出去。
    超过 line_timeout 没有收到有效偏移时输出 0（安全第一）。

运行时开关：
    ros2 param set /line_follower enable false

对应功能：⑫ 巡线（控制部分）。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from line_follower.core import LinePid


class LineFollower(Node):
    """把偏移量转换成底盘速度指令。"""

    def __init__(self):
        super().__init__('line_follower')

        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.10)
        self.declare_parameter('target_speed', 0.30)
        self.declare_parameter('min_speed_ratio', 0.40)
        self.declare_parameter('max_angular', 1.50)
        self.declare_parameter('line_timeout', 0.50)
        self.declare_parameter('integral_limit', 1.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('output_topic', '/cmd_vel_line')
        self.declare_parameter('enable', True)

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.line_timeout = float(self.get_parameter('line_timeout').value)
        self.enabled = bool(self.get_parameter('enable').value)

        self.pid = LinePid(
            kp=float(self.get_parameter('kp').value),
            ki=float(self.get_parameter('ki').value),
            kd=float(self.get_parameter('kd').value),
            target_speed=float(self.get_parameter('target_speed').value),
            min_speed_ratio=float(self.get_parameter('min_speed_ratio').value),
            max_angular=float(self.get_parameter('max_angular').value),
            integral_limit=float(self.get_parameter('integral_limit').value))

        self._error = math.nan
        self._last_msg_time = 0.0
        self._last_time = time.monotonic()

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Float32, '/perception/line', self.on_line, 10)
        self.create_subscription(
            Bool, '/control/line_enable', self.on_enable_topic, 10)

        rate = max(float(self.get_parameter('control_rate').value), 1.0)
        self.create_timer(1.0 / rate, self.control_loop)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info(
            '巡线控制启动：输出=%s kp=%.2f ki=%.2f kd=%.2f 速度=%.2f m/s' % (
                self.output_topic, self.pid.kp, self.pid.ki, self.pid.kd,
                self.pid.target_speed))

    def on_set_parameters(self, params):
        """支持运行时调参与启停，改完立刻生效。"""
        for param in params:
            if param.name == 'enable':
                self.enabled = bool(param.value)
                self.pid.reset()
                self.get_logger().info('巡线 %s' % (
                    '使能' if self.enabled else '关闭'))
            elif param.name in ('kp', 'ki', 'kd', 'target_speed'):
                setattr(self.pid, param.name, float(param.value))
        return SetParametersResult(successful=True)

    def on_line(self, msg):
        self._error = float(msg.data)
        if not math.isnan(self._error):
            self._last_msg_time = time.monotonic()

    def on_enable_topic(self, msg):
        """任务状态机的开/停巡线指令。"""
        enabled = bool(msg.data)
        if enabled == self.enabled:
            return
        self.enabled = enabled
        self.pid.reset()
        self.get_logger().info('巡线 %s（任务状态机指令）' % (
            '恢复' if enabled else '暂停'))

    def control_loop(self):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now

        cmd = Twist()
        lost = math.isnan(self._error)
        stale = (now - self._last_msg_time) > self.line_timeout
        if not self.enabled or lost or stale:
            self.pid.reset()
            self.pub.publish(cmd)
            return

        linear, angular = self.pid.update(self._error, dt)
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub.publish(cmd)

    def destroy_node(self):
        # 退出时务必发一帧零速度，防止车还在动
        try:
            if self.pub is not None:
                self.pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollower()
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
