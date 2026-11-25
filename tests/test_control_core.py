"""巡线 PID、速度仲裁、泊车状态机的测试。"""

import math

import pytest

from cmd_vel_arbiter.core import CommandArbiter
from line_follower.core import LinePid
from parking.core import ALIGN, APPROACH, DONE, FAILED, HOLD, SEARCH, \
    DockController


# ----------------------------------------------------------------------
# 巡线 PID
# ----------------------------------------------------------------------
def test_pid_turns_right_when_line_is_right():
    pid = LinePid(kp=1.2, ki=0.0, kd=0.0, target_speed=0.3, max_angular=1.5)
    linear, angular = pid.update(0.5, 0.05)
    assert angular < 0.0
    assert 0.0 < linear <= 0.3


def test_pid_turns_left_when_line_is_left():
    pid = LinePid(kp=1.2, ki=0.0, kd=0.0)
    _, angular = pid.update(-0.5, 0.05)
    assert angular > 0.0


def test_pid_on_center_goes_straight_at_full_speed():
    pid = LinePid(target_speed=0.3)
    linear, angular = pid.update(0.0, 0.05)
    assert angular == pytest.approx(0.0)
    assert linear == pytest.approx(0.3)


def test_pid_slows_down_with_large_error():
    pid = LinePid(target_speed=0.3, min_speed_ratio=0.4)
    linear, _ = pid.update(1.0, 0.05)
    assert linear == pytest.approx(0.3 * 0.4)


def test_pid_angular_is_clamped():
    pid = LinePid(kp=1.2, max_angular=1.5)
    _, angular = pid.update(1.0, 0.05)
    assert abs(angular) <= 1.5 + 1e-9


def test_pid_integral_is_limited():
    pid = LinePid(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5)
    for _ in range(100):
        pid.update(1.0, 0.1)
    assert abs(pid.integral) <= 0.5 + 1e-9


def test_pid_reset_clears_history():
    pid = LinePid()
    pid.update(0.5, 0.05)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prev_error is None


# ----------------------------------------------------------------------
# 速度仲裁
# ----------------------------------------------------------------------
def _arbiter(**kwargs):
    params = dict(max_linear=0.5, max_angular=1.5,
                  line_timeout=0.5, park_timeout=0.5)
    params.update(kwargs)
    return CommandArbiter(**params)


def test_arbiter_emergency_stops_everything():
    cmd = _arbiter().compute(
        10.0, line=(0.3, 0.0, 10.0), park=(0.2, 0.0, 10.0),
        park_active=True, emergency=True)
    assert cmd.mode == 'estop'
    assert cmd.linear == 0.0 and cmd.angular == 0.0


def test_arbiter_park_beats_line():
    cmd = _arbiter().compute(
        10.0, line=(0.3, 0.4, 10.0), park=(0.1, -0.2, 10.0),
        park_active=True)
    assert cmd.mode == 'park'
    assert cmd.linear == pytest.approx(0.1)
    assert cmd.angular == pytest.approx(-0.2)


def test_arbiter_stale_park_falls_back_to_line():
    cmd = _arbiter().compute(
        10.0, line=(0.3, 0.0, 10.0), park=(0.1, 0.0, 8.0),
        park_active=True)
    assert cmd.mode == 'line'


def test_arbiter_park_without_active_flag_is_ignored():
    cmd = _arbiter().compute(
        10.0, line=(0.3, 0.0, 10.0), park=(0.1, 0.0, 10.0),
        park_active=False)
    assert cmd.mode == 'line'


def test_arbiter_applies_avoid_scale_and_bias():
    cmd = _arbiter().compute(
        10.0, line=(0.4, 0.2, 10.0), avoid_scale=0.5, avoid_bias=0.3)
    assert cmd.linear == pytest.approx(0.2)
    assert cmd.angular == pytest.approx(0.4)


def test_arbiter_zero_scale_holds():
    cmd = _arbiter().compute(10.0, line=(0.4, 0.2, 10.0), avoid_scale=0.0)
    assert cmd.mode == 'hold'
    assert cmd.linear == 0.0 and cmd.angular == 0.0


def test_arbiter_stale_line_goes_idle():
    cmd = _arbiter().compute(10.0, line=(0.4, 0.2, 9.0))
    assert cmd.mode == 'idle'
    assert cmd.linear == 0.0


def test_arbiter_clamps_to_limits():
    cmd = _arbiter(max_linear=0.3, max_angular=0.8).compute(
        10.0, line=(5.0, 5.0, 10.0))
    assert cmd.linear == pytest.approx(0.3)
    assert cmd.angular == pytest.approx(0.8)


# ----------------------------------------------------------------------
# 泊车状态机
# ----------------------------------------------------------------------
def _controller(**kwargs):
    params = dict(kp_angular=1.4, center_tol=0.12, align_exit_tol=0.20,
                  target_height_ratio=0.4, target_distance=0.5,
                  approach_speed=0.16, search_speed=0.35,
                  hold_sec=1.0, timeout=20.0)
    params.update(kwargs)
    return DockController(**params)


def test_dock_search_without_marker():
    controller = _controller()
    out = controller.update(1.0, None, 640, 480)
    assert out.state == SEARCH
    assert out.angular > 0.0


def test_dock_aligns_when_marker_off_center():
    controller = _controller()
    out = controller.update(1.0, (560.0, 240.0, 60.0, 60.0), 640, 480)
    assert out.state == ALIGN
    assert out.angular < 0.0      # 码在右边 → 右转
    assert out.linear == 0.0


def test_dock_aligns_left_marker():
    controller = _controller()
    out = controller.update(1.0, (80.0, 240.0, 60.0, 60.0), 640, 480)
    assert out.angular > 0.0


def test_dock_approaches_when_centered_and_far():
    controller = _controller()
    out = controller.update(1.0, (320.0, 240.0, 60.0, 60.0), 640, 480)
    assert out.state == APPROACH
    assert out.linear > 0.0
    assert out.angular == pytest.approx(0.0)


def test_dock_approaches_uses_depth():
    controller = _controller(target_distance=0.5)
    out = controller.update(1.0, (320.0, 240.0, 60.0, 60.0), 640, 480,
                            depth_m=1.2)
    assert out.state == APPROACH
    out = controller.update(1.2, (320.0, 240.0, 60.0, 60.0), 640, 480,
                            depth_m=0.4)
    assert out.state == HOLD


def test_dock_holds_then_done():
    controller = _controller(hold_sec=1.0, target_distance=0.5)
    out = controller.update(1.0, (320.0, 240.0, 60.0, 60.0), 640, 480,
                            depth_m=0.4)
    assert out.state == HOLD
    out = controller.update(1.5, (320.0, 240.0, 60.0, 60.0), 640, 480,
                            depth_m=0.4)
    assert out.state == HOLD and not out.done
    out = controller.update(2.5, (320.0, 240.0, 60.0, 60.0), 640, 480,
                            depth_m=0.4)
    assert out.state == DONE and out.done


def test_dock_times_out():
    controller = _controller(timeout=5.0)
    controller.update(1.0, None, 640, 480)
    out = controller.update(6.5, None, 640, 480)
    assert out.state == FAILED


def test_dock_losing_marker_during_hold_still_finishes():
    controller = _controller(hold_sec=1.0, target_distance=0.5)
    controller.update(1.0, (320.0, 240.0, 60.0, 60.0), 640, 480,
                      depth_m=0.4)
    out = controller.update(1.2, None, 640, 480)
    assert out.state == HOLD
    out = controller.update(2.5, None, 640, 480)
    assert out.state == DONE


def test_dock_error_x_sign():
    controller = _controller()
    out = controller.update(1.0, (480.0, 240.0, 60.0, 60.0), 640, 480)
    assert out.error_x == pytest.approx(0.5)


def test_dock_accepts_bytes_like_height():
    controller = _controller()
    out = controller.update(1.0, (320.0, 240.0, 60.0, 60), 640, 480)
    assert math.isfinite(out.height_ratio)
