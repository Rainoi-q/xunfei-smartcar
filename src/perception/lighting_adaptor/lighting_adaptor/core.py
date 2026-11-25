"""光照判断与曝光闭环的纯逻辑（不依赖 ROS / OpenCV）。

BrightnessClassifier  把每帧平均亮度平滑后分成 暗 / 中 / 亮 三档，带迟滞。
ExposureController    按目标亮度增量调节曝光值，带死区、步长与上下限。

两个类都只在 update() 里维护状态，方便在电脑上做序列仿真。
"""


def _clamp(value, low, high):
    return max(low, min(high, value))


class BrightnessClassifier:
    """亮度分档器（迟滞 + 指数平滑），避免画面轻微变化时档位来回跳。"""

    DARK = 0
    NORMAL = 1
    BRIGHT = 2

    def __init__(self, dark_threshold=60.0, bright_threshold=180.0,
                 hysteresis=8.0, alpha=0.3, initial_level=None):
        self.dark_threshold = float(dark_threshold)
        self.bright_threshold = float(bright_threshold)
        self.hysteresis = abs(float(hysteresis))
        self.alpha = _clamp(float(alpha), 0.01, 1.0)
        self.smoothed = None
        if initial_level is None:
            initial_level = self.NORMAL
        self.level = int(initial_level)

    def update(self, mean_luma):
        """输入一帧平均亮度，返回 (level, smoothed_luma)。"""
        value = float(mean_luma)
        if self.smoothed is None:
            self.smoothed = value
        else:
            self.smoothed = self.alpha * value + (1.0 - self.alpha) * self.smoothed

        s = self.smoothed
        h = self.hysteresis
        if self.level == self.DARK:
            if s > self.dark_threshold + h:
                self.level = self.NORMAL
        elif self.level == self.NORMAL:
            if s < self.dark_threshold - h:
                self.level = self.DARK
            elif s > self.bright_threshold + h:
                self.level = self.BRIGHT
        else:  # BRIGHT
            if s < self.bright_threshold - h:
                self.level = self.NORMAL

        return self.level, self.smoothed

    @staticmethod
    def level_name(level):
        return {0: 'DARK', 1: 'NORMAL', 2: 'BRIGHT'}.get(int(level), 'UNKNOWN')


class ExposureController:
    """把“当前亮度 → 目标亮度”的误差转成新的曝光绝对值。

    约定：曝光值越大画面越亮。若某款相机相反，把 invert=True。
    """

    def __init__(self, target_luma=110.0, deadband=8.0, gain=2.0,
                 min_exposure=1, max_exposure=5000, max_step=80,
                 initial_exposure=120, invert=False, change_epsilon=1):
        self.target_luma = float(target_luma)
        self.deadband = abs(float(deadband))
        self.gain = float(gain)
        self.min_exposure = int(min_exposure)
        self.max_exposure = int(max_exposure)
        self.max_step = max(1, int(max_step))
        self.invert = bool(invert)
        self.change_epsilon = max(0, int(change_epsilon))
        self.exposure = int(_clamp(initial_exposure,
                                   self.min_exposure, self.max_exposure))
        self.reason = 'init'

    def update(self, mean_luma):
        """返回新的曝光值；在死区内或没有变化时返回 None。"""
        error = self.target_luma - float(mean_luma)
        if abs(error) < self.deadband:
            self.reason = 'deadband'
            return None

        step = error * self.gain
        step = _clamp(step, -self.max_step, self.max_step)
        if self.invert:
            step = -step

        new_value = int(round(_clamp(self.exposure + step,
                                     self.min_exposure, self.max_exposure)))
        if abs(new_value - self.exposure) < max(1, self.change_epsilon):
            self.reason = 'epsilon'
            return None

        self.exposure = new_value
        self.reason = 'adjust'
        return new_value

    def set_range(self, min_exposure, max_exposure):
        """v4l2 查询到真实范围后回填。"""
        self.min_exposure = int(min_exposure)
        self.max_exposure = int(max_exposure)
        self.exposure = int(_clamp(self.exposure,
                                   self.min_exposure, self.max_exposure))
        return self.exposure
