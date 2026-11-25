"""二维码引导泊车的纯状态机（不依赖 ROS）。

状态：
    SEARCH   原地慢转找二维码
    ALIGN    把二维码转到画面中心（只转不走）
    APPROACH 低速直行靠近，直到二维码足够大 / 深度足够近
    HOLD     原地停稳 hold_sec 秒
    DONE     完成
    FAILED   超时

输入 marker = (cx, cy, w, h)（像素）或 None；depth_m 为可选的深度距离（米）。
输出 (linear, angular, state, progress, done)。
"""

import math
from dataclasses import dataclass

SEARCH = 'SEARCH'
ALIGN = 'ALIGN'
APPROACH = 'APPROACH'
HOLD = 'HOLD'
DONE = 'DONE'
FAILED = 'FAILED'


def _clamp(value, low, high):
    return max(low, min(high, value))


@dataclass
class DockOutput:
    linear: float = 0.0
    angular: float = 0.0
    state: str = SEARCH
    progress: float = 0.0
    done: bool = False
    error_x: float = 0.0
    height_ratio: float = 0.0
    distance: float = -1.0


class DockController:
    def __init__(self, kp_angular=1.4, center_tol=0.12,
                 align_exit_tol=0.20, target_height_ratio=0.28,
                 target_distance=0.45, approach_speed=0.16,
                 search_speed=0.35, search_direction=1.0,
                 hold_sec=1.5, timeout=35.0):
        self.kp_angular = float(kp_angular)
        self.center_tol = abs(float(center_tol))
        self.align_exit_tol = max(self.center_tol,
                                  abs(float(align_exit_tol)))
        self.target_height_ratio = float(target_height_ratio)
        self.target_distance = float(target_distance)
        self.approach_speed = abs(float(approach_speed))
        self.search_speed = abs(float(search_speed))
        self.search_direction = 1.0 if search_direction >= 0 else -1.0
        self.hold_sec = max(0.0, float(hold_sec))
        self.timeout = max(1.0, float(timeout))

        self.state = SEARCH
        self.start_time = None
        self.hold_until = 0.0

    def reset(self):
        self.state = SEARCH
        self.start_time = None
        self.hold_until = 0.0

    def update(self, now, marker, frame_w, frame_h, depth_m=None):
        if self.start_time is None:
            self.start_time = now
        if now - self.start_time > self.timeout:
            self.state = FAILED
            return DockOutput(state=FAILED)

        if self.state in (DONE, FAILED):
            return DockOutput(state=self.state,
                              done=(self.state == DONE))

        if marker is None:
            if self.state == HOLD:
                # 已到点位，短暂丢码也保持停稳
                if now < self.hold_until:
                    return DockOutput(state=HOLD, progress=1.0)
                self.state = DONE
                return DockOutput(state=DONE, progress=1.0, done=True)
            self.state = SEARCH
            return DockOutput(angular=self.search_speed * self.search_direction,
                              state=SEARCH)

        cx, _cy, _w, h = marker
        if frame_w <= 0 or frame_h <= 0:
            return DockOutput(state=self.state)

        error_x = (float(cx) - frame_w / 2.0) / (frame_w / 2.0)
        height_ratio = float(h) / float(frame_h)
        distance = float(depth_m) if depth_m is not None and depth_m > 0 else -1.0
        progress = 0.0
        if self.target_height_ratio > 0:
            progress = _clamp(height_ratio / self.target_height_ratio, 0.0, 1.0)
        if distance > 0 and self.target_distance > 0:
            progress = max(progress, 1.0 - distance / max(self.target_distance, 1e-6))
            progress = _clamp(progress, 0.0, 1.0)

        if self.state == HOLD:
            if now < self.hold_until:
                return DockOutput(state=HOLD, progress=1.0,
                                  error_x=error_x, height_ratio=height_ratio,
                                  distance=distance)
            self.state = DONE
            return DockOutput(state=DONE, progress=1.0, done=True,
                              error_x=error_x, height_ratio=height_ratio,
                              distance=distance)

        # 对准优先：偏差过大时只转不走
        tolerance = (self.center_tol if self.state != ALIGN
                     else self.align_exit_tol)
        if abs(error_x) > tolerance:
            self.state = ALIGN
            angular = -self.kp_angular * error_x
            angular = _clamp(angular, -0.8, 0.8)
            return DockOutput(angular=angular, state=ALIGN,
                              progress=progress, error_x=error_x,
                              height_ratio=height_ratio, distance=distance)

        reached = height_ratio >= self.target_height_ratio
        if distance > 0 and self.target_distance > 0:
            reached = reached or distance <= self.target_distance
        if reached:
            self.state = HOLD
            self.hold_until = now + self.hold_sec
            return DockOutput(state=HOLD, progress=1.0,
                              error_x=error_x, height_ratio=height_ratio,
                              distance=distance)

        self.state = APPROACH
        speed = self.approach_speed * max(
            0.35, 1.0 - 0.6 * abs(error_x))
        angular = -0.5 * self.kp_angular * error_x
        angular = _clamp(angular, -0.5, 0.5)
        return DockOutput(linear=speed, angular=angular, state=APPROACH,
                          progress=progress, error_x=error_x,
                          height_ratio=height_ratio, distance=distance)
