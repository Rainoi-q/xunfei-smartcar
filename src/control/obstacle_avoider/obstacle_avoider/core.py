"""避障决策的纯逻辑（不依赖 ROS）。

evaluate_scan   从 LaserScan 距离数组里算前向最小距离、左右余量和减速比例。
depth_roi_min   从深度图 ROI 里算最小有效距离（米）。
state_for       用距离阈值给出 CLEAR / SLOW / STOP。
"""

import math
from dataclasses import dataclass

import numpy as np


CLEAR = 'CLEAR'
SLOW = 'SLOW'
STOP = 'STOP'


@dataclass
class ObstacleDecision:
    state: str = CLEAR
    distance: float = math.inf
    scale: float = 1.0         # 巡线速度缩放系数 0~1
    bias: float = 0.0          # 附加角速度（rad/s），正值左转
    front_min: float = math.inf
    left_min: float = math.inf
    right_min: float = math.inf
    valid_points: int = 0

    def as_dict(self):
        return {
            'state': self.state,
            'distance': self.distance,
            'scale': self.scale,
            'bias': self.bias,
            'front_min': self.front_min,
            'left_min': self.left_min,
            'right_min': self.right_min,
            'points': self.valid_points,
        }


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(value)


def _sector_min(ranges, angle_min, angle_increment, low_deg, high_deg,
                min_valid, max_valid):
    if not ranges or angle_increment == 0.0:
        return math.inf, 0
    low = math.radians(low_deg)
    high = math.radians(high_deg)
    best = math.inf
    count = 0
    for index, value in enumerate(ranges):
        if not _finite(value):
            continue
        angle = angle_min + index * angle_increment
        # 归一到 [-pi, pi]
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if angle < low or angle > high:
            continue
        if value < min_valid or value > max_valid:
            continue
        count += 1
        if value < best:
            best = value
    return best, count


def evaluate_scan(ranges, angle_min, angle_increment,
                  forward_half_deg=30.0, side_half_deg=75.0,
                  stop_distance=0.35, slow_distance=0.85,
                  min_valid=0.05, max_valid=8.0,
                  min_scale=0.15, turn_gain=1.2, max_bias=0.9):
    """雷达避障决策。角度约定：0 为正前方，正值向左。"""
    if angle_increment == 0.0:
        return ObstacleDecision()

    front_min, front_count = _sector_min(
        ranges, angle_min, angle_increment,
        -forward_half_deg, forward_half_deg, min_valid, max_valid)
    left_min, _ = _sector_min(
        ranges, angle_min, angle_increment,
        forward_half_deg, side_half_deg, min_valid, max_valid)
    right_min, _ = _sector_min(
        ranges, angle_min, angle_increment,
        -side_half_deg, -forward_half_deg, min_valid, max_valid)

    if front_count == 0 or not math.isfinite(front_min):
        return ObstacleDecision(state=CLEAR, left_min=left_min,
                                right_min=right_min)

    state, scale = state_for(front_min, stop_distance, slow_distance,
                             min_scale=min_scale)
    bias = 0.0
    if state == SLOW:
        # 哪边空就往哪边偏：左侧更空 → 正角速度（左转）
        clearance = left_min - right_min
        if math.isfinite(clearance):
            direction = 1.0 if clearance > 0.0 else -1.0
            strength = min(1.0, abs(clearance) / max(slow_distance, 1e-6))
            bias = direction * turn_gain * (1.0 - scale) * strength
            bias = max(-max_bias, min(max_bias, bias))

    return ObstacleDecision(
        state=state, distance=front_min, scale=scale, bias=bias,
        front_min=front_min, left_min=left_min, right_min=right_min,
        valid_points=front_count)


def state_for(distance, stop_distance, slow_distance, min_scale=0.15):
    """距离 → (state, scale)。"""
    if not math.isfinite(distance):
        return CLEAR, 1.0
    if distance <= stop_distance:
        return STOP, 0.0
    if distance <= slow_distance:
        span = max(slow_distance - stop_distance, 1e-6)
        ratio = (distance - stop_distance) / span
        scale = min_scale + (1.0 - min_scale) * max(0.0, min(1.0, ratio))
        return SLOW, scale
    return CLEAR, 1.0


def depth_roi_min(depth_image, x_min=0.35, x_max=0.65,
                  y_min=0.35, y_max=0.65,
                  min_valid=0.05, max_valid=4.0,
                  depth_scale=0.001):
    """深度图 ROI 最小有效值，单位米。

    Astra 深度通常为 16UC1（毫米），depth_scale=0.001；
    若为 32FC1（米），把 depth_scale 设为 1.0。
    """
    arr = np.asarray(depth_image)
    if arr.ndim != 2 or arr.size == 0:
        return math.inf

    height, width = arr.shape
    x0 = int(max(0.0, min(1.0, x_min)) * width)
    x1 = int(max(0.0, min(1.0, x_max)) * width)
    y0 = int(max(0.0, min(1.0, y_min)) * height)
    y1 = int(max(0.0, min(1.0, y_max)) * height)
    if x1 <= x0 or y1 <= y0:
        return math.inf

    roi = arr[y0:y1, x0:x1].astype(np.float32)
    roi = roi[np.isfinite(roi)]
    if roi.size == 0:
        return math.inf
    roi = roi[roi > 0]
    if roi.size == 0:
        return math.inf

    meters = roi * float(depth_scale)
    meters = meters[(meters >= min_valid) & (meters <= max_valid)]
    if meters.size == 0:
        return math.inf
    return float(meters.min())
