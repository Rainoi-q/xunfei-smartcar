"""速度指令仲裁的纯逻辑。

优先级：急停 > 泊车 > 巡线（叠加避障缩放/偏置）> 停车。
所有来源都有超时保护：话题断流时输出 0，绝不让车保持上一次速度。
"""

from dataclasses import dataclass


@dataclass
class Cmd:
    linear: float = 0.0
    angular: float = 0.0
    mode: str = 'idle'
    reason: str = ''


def _clamp(value, low, high):
    return max(low, min(high, value))


class CommandArbiter:
    """输入各来源的 (v, w, 时间戳)，输出最终速度。"""

    def __init__(self, max_linear=0.5, max_angular=1.5,
                 line_timeout=0.5, park_timeout=0.5):
        self.max_linear = abs(float(max_linear))
        self.max_angular = abs(float(max_angular))
        self.line_timeout = abs(float(line_timeout))
        self.park_timeout = abs(float(park_timeout))

    def compute(self, now, line=None, park=None, park_active=False,
                avoid_scale=1.0, avoid_bias=0.0, emergency=False):
        """line/park: (v, w, timestamp) 或 None。"""
        if emergency:
            return Cmd(0.0, 0.0, 'estop', 'obstacle stop')

        if park_active and park is not None:
            v, w, stamp = park
            if now - stamp <= self.park_timeout:
                return self._clamp(Cmd(v, w, 'park', 'parking active'))

        if line is not None:
            v, w, stamp = line
            if now - stamp <= self.line_timeout:
                scale = _clamp(float(avoid_scale), 0.0, 1.0)
                bias = float(avoid_bias)
                if scale <= 0.0:
                    return Cmd(0.0, 0.0, 'hold', 'avoid scale 0')
                out_v = v * scale
                out_w = w * scale + bias
                return self._clamp(Cmd(out_v, out_w, 'line', 'line follow'))

        return Cmd(0.0, 0.0, 'idle', 'no fresh command')

    def _clamp(self, cmd):
        cmd.linear = _clamp(cmd.linear, -self.max_linear, self.max_linear)
        cmd.angular = _clamp(cmd.angular, -self.max_angular, self.max_angular)
        if abs(cmd.linear) < 1e-4:
            cmd.linear = 0.0
        if abs(cmd.angular) < 1e-4:
            cmd.angular = 0.0
        return cmd
