"""巡线 PID 的纯逻辑，方便在电脑上做阶跃/正弦跟踪测试。"""


def _clamp(value, low, high):
    return max(low, min(high, value))


class LinePid:
    """偏移量 [-1,1] → (线速度, 角速度)。"""

    def __init__(self, kp=1.2, ki=0.05, kd=0.10, target_speed=0.30,
                 min_speed_ratio=0.40, max_angular=1.50,
                 integral_limit=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.target_speed = float(target_speed)
        self.min_speed_ratio = _clamp(float(min_speed_ratio), 0.0, 1.0)
        self.max_angular = abs(float(max_angular))
        self.integral_limit = abs(float(integral_limit))
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        """error: 正=线在右侧。返回 (linear, angular)。"""
        error = float(error)
        dt = float(dt)
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05

        self.integral = _clamp(
            self.integral + error * dt,
            -self.integral_limit, self.integral_limit)

        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        angular = -(self.kp * error + self.ki * self.integral
                    + self.kd * derivative)
        angular = _clamp(angular, -self.max_angular, self.max_angular)
        linear = self.target_speed * max(
            self.min_speed_ratio, 1.0 - abs(error))
        return linear, angular

    def set_params(self, kp=None, ki=None, kd=None, target_speed=None):
        if kp is not None:
            self.kp = float(kp)
        if ki is not None:
            self.ki = float(ki)
        if kd is not None:
            self.kd = float(kd)
        if target_speed is not None:
            self.target_speed = float(target_speed)
