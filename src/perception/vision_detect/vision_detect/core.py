"""视觉感知的纯算法部分（不依赖 ROS，可在电脑上单独测试）。

包含两块：
    LineTracker   从二值掩膜中提取巡线横向偏移，带自下而上的跟踪与直线拟合。
    parse_qr_payload  把二维码原文解析成多字段 JSON，并校验必填字段。

设计原则：
    - 这里不 import cv2 / rclpy，方便用 pytest 在 Windows/Ubuntu 上直接跑。
    - 掩膜的二值化（灰度阈值 / HSV）由节点负责，本模块只吃 numpy 数组。
"""

import json
import math
from dataclasses import dataclass, field

import numpy as np


# ----------------------------------------------------------------------
# 巡线
# ----------------------------------------------------------------------
@dataclass
class LineResult:
    """一帧的巡线结果。"""

    offset: float = math.nan       # [-1, 1]，正=线在画面右侧；丢失为 NaN
    lost: bool = True
    bottom_x: float = math.nan     # 拟合出的画面底边中线 x 像素
    slope: float = 0.0             # dx/dy（向上为正方向），正值表示线向右斜
    points: list = field(default_factory=list)  # [(x, y_up)] y 以 ROI 底边为 0
    bands_used: int = 0


class LineTracker:
    """自下而上的分带跟踪器。

    相比“整块区域取平均”，分带跟踪的好处：
        1. 弯道时只相信靠近车头的线，远处噪点不影响；
        2. 遇到白线/反光形成的碎块，按“离上一带最近”选择，不会把路沿算进去；
        3. 能给出斜率，便于下游做前馈转向。
    """

    def __init__(self, num_bands=10, band_height_px=12, min_pixels=300,
                 min_run_width=3, max_center_jump=0.30, max_slope=1.5):
        self.num_bands = max(2, int(num_bands))
        self.band_height_px = max(2, int(band_height_px))
        self.min_pixels = max(0, int(min_pixels))
        self.min_run_width = max(1, int(min_run_width))
        self.max_center_jump = float(max_center_jump)   # 占画宽比例
        self.max_slope = float(max_slope)

    def extract(self, mask):
        """mask: (H, W) 的 bool / uint8 二值图（非 0 为线）。返回 LineResult。"""
        mask = np.asarray(mask)
        if mask.ndim != 2 or mask.size == 0:
            return LineResult()

        height, width = mask.shape
        total = int(np.count_nonzero(mask))
        if total < self.min_pixels:
            return LineResult()

        band_h = min(self.band_height_px,
                     max(4, height // self.num_bands))
        max_jump = max(8.0, self.max_center_jump * width)

        prev_center = width / 2.0
        points = []
        masses = []

        i = 0
        while True:
            y1 = height - i * band_h
            y0 = max(0, y1 - band_h)
            if y1 <= y0:
                break
            i += 1

            band = mask[y0:y1, :]
            col = band.sum(axis=0).astype(np.float32)
            min_col = max(1, int(round((y1 - y0) * 0.20)))
            runs = self._find_runs(col >= min_col)
            if not runs:
                continue

            candidates = []
            for start, end in runs:
                if end - start + 1 < self.min_run_width:
                    continue
                mass = float(col[start:end + 1].sum())
                if mass <= 0.0:
                    continue
                xs = np.arange(start, end + 1, dtype=np.float32)
                center = float((col[start:end + 1] * xs).sum() / mass)
                jump = abs(center - prev_center)
                if i > 1 and jump > max_jump:
                    continue  # 突然跳到另一条线/路沿，丢弃
                candidates.append((jump, -mass, center, mass))

            if not candidates:
                continue
            candidates.sort(key=lambda item: (item[0], item[1]))
            _, _, center, mass = candidates[0]
            prev_center = center

            y_img = (y0 + y1) / 2.0
            points.append((center, float(height - y_img)))
            masses.append(mass)

        if not points:
            return LineResult(bands_used=0)

        if len(points) >= 2:
            ys = np.array([p[1] for p in points], dtype=np.float64)
            xs = np.array([p[0] for p in points], dtype=np.float64)
            ws = np.sqrt(np.array(masses, dtype=np.float64))
            # 加权最小二乘：x = slope * y + intercept
            try:
                slope, intercept = np.polyfit(ys, xs, 1, w=ws)
            except (np.linalg.LinAlgError, ValueError):
                slope, intercept = 0.0, float(points[0][0])
            if not math.isfinite(slope) or abs(slope) > self.max_slope:
                slope = 0.0
                intercept = float(np.average(xs, weights=ws))
            bottom_x = float(intercept)
        else:
            bottom_x = float(points[0][0])
            slope = 0.0

        offset = (bottom_x - width / 2.0) / (width / 2.0)
        offset = max(-1.0, min(1.0, offset))
        return LineResult(
            offset=offset,
            lost=False,
            bottom_x=bottom_x,
            slope=float(slope),
            points=points,
            bands_used=len(points),
        )

    @staticmethod
    def _find_runs(flags):
        """返回 True 连续段的 [start, end] 闭区间列表。"""
        flags = np.asarray(flags, dtype=bool)
        if not flags.any():
            return []
        padded = np.concatenate(([False], flags, [False]))
        diff = np.diff(padded.astype(np.int8))
        starts = np.flatnonzero(diff == 1)
        ends = np.flatnonzero(diff == -1) - 1
        return list(zip(starts.tolist(), ends.tolist()))


# ----------------------------------------------------------------------
# 二维码
# ----------------------------------------------------------------------
DEFAULT_REQUIRED_FIELDS = ('order_id', 'consignee', 'destination')


@dataclass
class QrPayload:
    """规范化后的二维码内容。"""

    json_text: str = '{}'          # 发给 /perception/qr_payload 的字符串
    data: dict = field(default_factory=dict)
    valid: bool = False
    missing: list = field(default_factory=list)
    raw: str = ''
    is_json: bool = False

    def summary(self):
        """一行可读摘要，打印到日志 / HUD。"""
        if not self.data:
            return '(empty)'
        order = {
            'order_id': 0, 'consignee': 1, 'destination': 2,
            'priority': 3, 'weight_kg': 4,
        }
        keys = sorted(self.data.keys(), key=lambda k: (order.get(k, 99), str(k)))
        parts = []
        for key in keys:
            value = self.data[key]
            if isinstance(value, float):
                value = ('%.2f' % value).rstrip('0').rstrip('.')
            parts.append('%s=%s' % (key, value))
        return ' '.join(parts)


def parse_qr_payload(raw, required_fields=DEFAULT_REQUIRED_FIELDS,
                     allow_plain_text=True):
    """把二维码原文转成 QrPayload。

    - 合法 JSON 对象：检查 required_fields，缺字段记入 missing；
    - 普通文本：包装成 {"raw": ...}，allow_plain_text=True 时视为有效；
    - 其余（JSON 数组/数字等）：视为无效。
    """
    text = (raw or '').strip()
    required = tuple(required_fields or ())

    if not text:
        return QrPayload(json_text='{}', raw='', valid=False)

    try:
        data = json.loads(text)
        is_json = True
    except (ValueError, TypeError):
        data = None
        is_json = False

    if is_json:
        if not isinstance(data, dict):
            return QrPayload(
                json_text=json.dumps({'raw': text}, ensure_ascii=False),
                data={'raw': text}, valid=False, raw=text, is_json=True)
        missing = [key for key in required
                   if key not in data or data[key] in (None, '')]
        valid = not missing
        json_text = json.dumps(data, ensure_ascii=False, sort_keys=True)
        return QrPayload(json_text=json_text, data=data, valid=valid,
                         missing=missing, raw=text, is_json=True)

    data = {'raw': text}
    json_text = json.dumps(data, ensure_ascii=False, sort_keys=True)
    return QrPayload(json_text=json_text, data=data,
                     valid=bool(allow_plain_text), raw=text, is_json=False)
