"""光照/曝光、避障、OCR 纯逻辑的测试。"""

import math

import numpy as np
import pytest

from lighting_adaptor.core import BrightnessClassifier, ExposureController
from obstacle_avoider.core import (CLEAR, SLOW, STOP, depth_roi_min,
                                   evaluate_scan, state_for)
from ocr_reader.core import clean_text, parse_tesseract_tsv, text_quality


# ----------------------------------------------------------------------
# 光照分档
# ----------------------------------------------------------------------
def _warm(classifier, value, frames=10):
    for _ in range(frames):
        classifier.update(value)


def test_brightness_dark_normal_bright():
    classifier = BrightnessClassifier(dark_threshold=60.0,
                                      bright_threshold=170.0,
                                      hysteresis=8.0, alpha=0.5)
    _warm(classifier, 30.0)
    assert classifier.level == BrightnessClassifier.DARK
    _warm(classifier, 110.0)
    assert classifier.level == BrightnessClassifier.NORMAL
    _warm(classifier, 230.0)
    assert classifier.level == BrightnessClassifier.BRIGHT


def test_brightness_hysteresis_blocks_small_change():
    classifier = BrightnessClassifier(dark_threshold=60.0,
                                      bright_threshold=170.0,
                                      hysteresis=8.0, alpha=1.0)
    _warm(classifier, 100.0)
    assert classifier.level == BrightnessClassifier.NORMAL
    _warm(classifier, 175.0)     # 170+8 以内，不应跳到 BRIGHT
    assert classifier.level == BrightnessClassifier.NORMAL
    _warm(classifier, 190.0)
    assert classifier.level == BrightnessClassifier.BRIGHT


def test_brightness_level_name():
    assert BrightnessClassifier.level_name(0) == 'DARK'
    assert BrightnessClassifier.level_name(1) == 'NORMAL'
    assert BrightnessClassifier.level_name(2) == 'BRIGHT'


def test_exposure_deadband_returns_none():
    controller = ExposureController(target_luma=110.0, deadband=8.0,
                                    initial_exposure=200)
    assert controller.update(105.0) is None
    assert controller.exposure == 200


def test_exposure_increases_when_dark():
    controller = ExposureController(target_luma=110.0, deadband=8.0,
                                    gain=2.0, initial_exposure=200,
                                    max_step=60)
    value = controller.update(40.0)
    assert value == 260


def test_exposure_decreases_when_bright():
    controller = ExposureController(target_luma=110.0, deadband=8.0,
                                    gain=2.0, initial_exposure=200,
                                    max_step=60)
    value = controller.update(200.0)
    assert value == 140


def test_exposure_invert():
    controller = ExposureController(target_luma=110.0, deadband=8.0,
                                    gain=2.0, initial_exposure=200,
                                    max_step=60, invert=True)
    value = controller.update(40.0)
    assert value == 140


def test_exposure_clamped_to_range():
    controller = ExposureController(target_luma=110.0, deadband=8.0,
                                    gain=2.0, initial_exposure=4990,
                                    max_exposure=5000, max_step=60)
    assert controller.update(10.0) == 5000


def test_exposure_set_range_clamps_current_value():
    controller = ExposureController(initial_exposure=4000)
    assert controller.set_range(1, 1000) == 1000
    assert controller.exposure == 1000


# ----------------------------------------------------------------------
# 避障
# ----------------------------------------------------------------------
RAYS = 360
ANGLE_MIN = -math.pi
INCREMENT = 2.0 * math.pi / RAYS


def _ranges(fill=10.0):
    return [fill] * RAYS


def _index(angle_rad):
    return int(round((angle_rad - ANGLE_MIN) / INCREMENT)) % RAYS


def test_scan_clear_when_far_away():
    decision = evaluate_scan(_ranges(10.0), ANGLE_MIN, INCREMENT)
    assert decision.state == CLEAR
    assert decision.scale == pytest.approx(1.0)


def test_scan_stops_for_close_obstacle():
    ranges = _ranges(10.0)
    ranges[_index(0.0)] = 0.2
    decision = evaluate_scan(ranges, ANGLE_MIN, INCREMENT,
                             stop_distance=0.35, slow_distance=0.85)
    assert decision.state == STOP
    assert decision.scale == 0.0


def test_scan_slow_and_bias_towards_open_side():
    ranges = _ranges(10.0)
    for deg in range(-20, 21, 2):
        ranges[_index(math.radians(deg))] = 0.6
    for deg in range(35, 70, 5):
        ranges[_index(math.radians(deg))] = 2.0      # 左侧空
        ranges[_index(math.radians(-deg))] = 0.7     # 右侧近
    decision = evaluate_scan(ranges, ANGLE_MIN, INCREMENT,
                             stop_distance=0.35, slow_distance=0.85)
    assert decision.state == SLOW
    assert 0.0 < decision.scale < 1.0
    assert decision.bias > 0.0        # 左转


def test_scan_ignores_non_finite_ranges():
    ranges = _ranges(float('inf'))
    ranges[_index(0.0)] = float('nan')
    decision = evaluate_scan(ranges, ANGLE_MIN, INCREMENT)
    assert decision.state == CLEAR


def test_state_for_boundaries():
    assert state_for(0.1, 0.35, 0.85)[0] == STOP
    assert state_for(0.6, 0.35, 0.85)[0] == SLOW
    assert state_for(2.0, 0.35, 0.85) == (CLEAR, 1.0)
    assert state_for(math.inf, 0.35, 0.85) == (CLEAR, 1.0)


def test_depth_roi_min_millimeters():
    depth = np.full((480, 640), 3000, dtype=np.uint16)
    depth[200:260, 280:360] = 300
    value = depth_roi_min(depth, depth_scale=0.001)
    assert value == pytest.approx(0.3)


def test_depth_roi_min_float_meters():
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    depth[100:200, 100:200] = 0.8
    value = depth_roi_min(depth, depth_scale=1.0)
    assert value == pytest.approx(2.0)     # 0.8 不在 ROI 内


def test_depth_roi_min_ignores_zero():
    depth = np.zeros((480, 640), dtype=np.uint16)
    assert math.isinf(depth_roi_min(depth, depth_scale=0.001))


# ----------------------------------------------------------------------
# OCR TSV
# ----------------------------------------------------------------------
HEADER = ('level\tpage_num\tblock_num\tpar_num\tline_num\tword_num\t'
          'left\ttop\twidth\theight\tconf\ttext')


def _row(block, par, line, word, conf, text):
    return '\t'.join(['5', '1', str(block), str(par), str(line), str(word),
                      '10', '10', '40', '20', str(conf), text])


def test_parse_english_line():
    tsv = '\n'.join([HEADER,
                     _row(1, 1, 1, 1, 96, 'Hello'),
                     _row(1, 1, 1, 2, 91, 'World')])
    text, conf, count = parse_tesseract_tsv(tsv)
    assert text == 'Hello World'
    assert conf == pytest.approx(93.5)
    assert count == 2


def test_parse_chinese_joins_without_space():
    tsv = '\n'.join([HEADER,
                     _row(1, 1, 1, 1, 95, '配送'),
                     _row(1, 1, 1, 2, 93, '中心')])
    text, _, _ = parse_tesseract_tsv(tsv)
    assert text == '配送中心'


def test_parse_filters_low_confidence():
    tsv = '\n'.join([HEADER,
                     _row(1, 1, 1, 1, 96, 'ABC'),
                     _row(1, 1, 1, 2, 10, 'XYZ')])
    text, conf, count = parse_tesseract_tsv(tsv, min_conf=30.0)
    assert text == 'ABC'
    assert count == 2          # 置信度统计仍包含低分词
    assert 10.0 <= conf <= 96.0


def test_parse_multiple_lines_ordered():
    tsv = '\n'.join([HEADER,
                     _row(1, 1, 2, 1, 90, 'second'),
                     _row(1, 1, 1, 1, 90, 'first')])
    text, _, _ = parse_tesseract_tsv(tsv)
    assert text == 'first second'


def test_parse_bad_rows_are_skipped():
    tsv = '\n'.join([HEADER, 'garbage', '1\t2\t3'])
    text, conf, count = parse_tesseract_tsv(tsv)
    assert text == ''
    assert count == 0


def test_clean_text_removes_symbol_lines():
    text = clean_text('  Hello   World  \n|\n~~\n\n配送 中心')
    assert text == 'Hello World 配送 中心'


def test_clean_text_truncates():
    text = clean_text('a' * 500, max_chars=20)
    assert len(text) == 20


def test_text_quality():
    assert text_quality('') == 0.0
    assert text_quality('abc123') == pytest.approx(1.0)
    assert text_quality('!!') == pytest.approx(0.0)
    assert text_quality('A!!') == pytest.approx(1.0 / 3.0)
