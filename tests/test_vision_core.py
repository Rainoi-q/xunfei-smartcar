"""巡线跟踪器与二维码解析的测试。"""

import math

import numpy as np

from vision_detect.core import (DEFAULT_REQUIRED_FIELDS, LineTracker,
                                parse_qr_payload)


def _tracker(**kwargs):
    params = dict(num_bands=10, band_height_px=12, min_pixels=200,
                  max_slope=1.5)
    params.update(kwargs)
    return LineTracker(**params)


def test_straight_line_at_center():
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[:, 300:320] = 255
    result = _tracker().extract(mask)
    assert not result.lost
    assert abs(result.offset) < 0.05
    assert result.bands_used >= 5


def test_line_on_right_gives_positive_offset():
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[:, 490:520] = 255
    result = _tracker().extract(mask)
    assert not result.lost
    assert result.offset > 0.4


def test_line_on_left_gives_negative_offset():
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[:, 110:140] = 255
    result = _tracker().extract(mask)
    assert not result.lost
    assert result.offset < -0.4


def test_diagonal_line_reports_slope():
    height, width = 480, 640
    mask = np.zeros((height, width), dtype=np.uint8)
    for y in range(height):
        # 越往上（y 变小）线越靠右
        center = int(200 + (height - y) * 0.4)
        mask[y, max(0, center - 6):min(width, center + 6)] = 255
    result = _tracker().extract(mask)
    assert not result.lost
    assert result.slope != 0.0


def test_empty_mask_is_lost():
    mask = np.zeros((480, 640), dtype=np.uint8)
    result = _tracker().extract(mask)
    assert result.lost
    assert math.isnan(result.offset)


def test_too_few_pixels_is_lost():
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[470:474, 318:322] = 255   # 只有 16 个像素
    result = _tracker(min_pixels=200).extract(mask)
    assert result.lost


def test_offset_never_exceeds_one():
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[:, 0:8] = 255
    result = _tracker().extract(mask)
    assert -1.0 <= result.offset <= 1.0


def test_qr_json_with_all_fields_is_valid():
    payload = parse_qr_payload(
        '{"order_id":"A03","consignee":"zhang_san","destination":"door_3"}')
    assert payload.valid
    assert payload.is_json
    assert payload.data['destination'] == 'door_3'
    assert payload.missing == []


def test_qr_missing_field_is_invalid():
    payload = parse_qr_payload('{"order_id":"A03","destination":"door_3"}')
    assert not payload.valid
    assert payload.missing == ['consignee']


def test_qr_empty_value_counts_as_missing():
    payload = parse_qr_payload(
        '{"order_id":"A03","consignee":"","destination":"door_3"}')
    assert not payload.valid
    assert 'consignee' in payload.missing


def test_plain_text_allowed_by_default():
    payload = parse_qr_payload('door_3')
    assert payload.valid
    assert payload.data == {'raw': 'door_3'}
    assert not payload.is_json


def test_plain_text_can_be_rejected():
    payload = parse_qr_payload('door_3', allow_plain_text=False)
    assert not payload.valid


def test_json_array_is_invalid():
    payload = parse_qr_payload('[1, 2, 3]')
    assert not payload.valid
    assert payload.data == {'raw': '[1, 2, 3]'}


def test_empty_payload():
    payload = parse_qr_payload('   ')
    assert not payload.valid
    assert payload.json_text == '{}'


def test_required_fields_default():
    assert DEFAULT_REQUIRED_FIELDS == ('order_id', 'consignee', 'destination')


def test_summary_is_sorted_by_business_order():
    payload = parse_qr_payload(
        '{"weight_kg":1.5,"destination":"door_3","order_id":"A03",'
        '"consignee":"zhang_san"}')
    text = payload.summary()
    assert text.index('order_id') < text.index('consignee')
    assert text.index('consignee') < text.index('destination')
    assert text.index('destination') < text.index('weight_kg')
