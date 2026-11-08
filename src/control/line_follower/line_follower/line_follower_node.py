"""巡线控制节点（PID）。

订阅：
    /perception/line   std_msgs/Float32
        横向偏移，范围 [-1, 1]，正值表示线在画面右侧，NaN 表示丢失。

发布：
    /cmd_vel           geometry_msgs/Twist

逻辑：
    angular = -(kp*e + ki*∫e dt + kd*de/dt)
    偏移越大越减速，保证转弯时不会冲出去。
    超过 line_timeout 没有收到有效偏移时停车（安全第一）。

对应功能：⑫ 巡线（控制部分）。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


def _clamp(value, low, high):
    return max(low, min(high, value))


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
        self.declare_parameter('enable', True)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.min_speed_ratio = float(self.get_parameter('min_speed_ratio').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.line_timeout = float(self.get_parameter('line_timeout').value)
        self.integral_limit = float(self.get_parameter('integral_limit').value)
        self.enabled = bool(self.get_parameter('enable').value)

        self._error = math.nan
        self._integral = 0.0
        self._prev_error = None
        self._last_msg_time = 0.0
        self._last_time = time.monotonic()

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/perception/line', self.on_line, 10)

        rate = max(float(self.get_parameter('control_rate').value), 1.0)
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            '巡线控制已启动 kp=%.2f ki=%.2f kd=%.2f 速度=%.2f m/s' % (
                self.kp, self.ki, self.kd, self.target_speed))

    def on_line(self, msg):
        self._error = float(msg.data)
        if not math.isnan(self._error):
            self._last_msg_time = time.monotonic()

    def control_loop(self):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05

        cmd = Twist()

        lost = math.isnan(self._error)
        stale = (now - self._last_msg_time) > self.line_timeout
        if not self.enabled or lost or stale:
            # 停车并清空积分项，避免恢复时猛打方向
            self._integral = 0.0
            self._prev_error = None
            self.pub.publish(cmd)
            return

        error = self._error
        self._integral = _clamp(
            self._integral + error * dt, -self.integral_limit, self.integral_limit)

        if self._prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error

        angular = -(self.kp * error + self.ki * self._integral + self.kd * derivative)
        angular = _clamp(angular, -self.max_angular, self.max_angular)

        linear = self.target_speed * max(self.min_speed_ratio, 1.0 - abs(error))

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
