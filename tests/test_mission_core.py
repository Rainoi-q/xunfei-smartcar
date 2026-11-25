"""航点文件解析与任务状态机的测试。"""

import math

import pytest

from mission_manager.core import (DOCKING, DONE, FAILED, IDLE, NAVIGATING,
                                  MissionMachine)
from waypoint_nav.core import (Waypoint, load_config, quaternion_to_yaw,
                               resolve_route, yaw_to_quaternion)

WAYPOINTS_YAML = """
frame_id: map
waypoints:
  dock:    {x: 1.2, y: 0.5, yaw: 0.0}
  shelf_a: {x: 0.8, y: 2.0, yaw: 1.57}
  door_3:  {x: 3.5, y: 2.2, yaw: 3.14}
routes:
  deliver_3: [shelf_a, door_3]
  home: [dock]
  empty: []
"""


def _write(tmp_path, text=WAYPOINTS_YAML):
    path = tmp_path / 'waypoints.yaml'
    path.write_text(text, encoding='utf-8')
    return str(path)


def test_load_config(tmp_path):
    frame, points, routes = load_config(_write(tmp_path))
    assert frame == 'map'
    assert set(points) == {'dock', 'shelf_a', 'door_3'}
    assert points['door_3'].x == pytest.approx(3.5)
    assert routes['deliver_3'] == ['shelf_a', 'door_3']


def test_load_config_defaults(tmp_path):
    path = tmp_path / 'w.yaml'
    path.write_text('waypoints:\n  a: {x: 1, y: 2}\n', encoding='utf-8')
    frame, points, routes = load_config(str(path))
    assert frame == 'map'
    assert points['a'] == Waypoint('a', 1.0, 2.0, 0.0)
    assert routes == {}


def test_load_config_rejects_bad_waypoint(tmp_path):
    path = tmp_path / 'w.yaml'
    path.write_text('waypoints:\n  a: 5\n', encoding='utf-8')
    with pytest.raises(ValueError):
        load_config(str(path))


def test_load_config_rejects_bad_route(tmp_path):
    path = tmp_path / 'w.yaml'
    path.write_text('waypoints: {}\nroutes: {r: dock}\n', encoding='utf-8')
    with pytest.raises(ValueError):
        load_config(str(path))


def test_resolve_route_by_route_name(tmp_path):
    _, points, routes = load_config(_write(tmp_path))
    route = resolve_route('deliver_3', points, routes)
    assert [p.name for p in route] == ['shelf_a', 'door_3']


def test_resolve_route_by_waypoint_name(tmp_path):
    _, points, routes = load_config(_write(tmp_path))
    route = resolve_route('door_3', points, routes)
    assert len(route) == 1 and route[0].name == 'door_3'


def test_resolve_unknown_route_raises(tmp_path):
    _, points, routes = load_config(_write(tmp_path))
    with pytest.raises(KeyError):
        resolve_route('nowhere', points, routes)


def test_resolve_dangling_waypoint_raises():
    points = {'a': Waypoint('a', 0.0, 0.0, 0.0)}
    routes = {'r': ['a', 'b']}
    with pytest.raises(KeyError):
        resolve_route('r', points, routes)


def test_resolve_empty_route_raises(tmp_path):
    _, points, routes = load_config(_write(tmp_path))
    with pytest.raises(KeyError):
        resolve_route('empty', points, routes)


def test_quaternion_round_trip():
    for yaw in (-3.0, -1.57, 0.0, 0.3, 1.57, 3.0):
        x, y, z, w = yaw_to_quaternion(yaw)
        assert quaternion_to_yaw(x, y, z, w) == pytest.approx(yaw, abs=1e-6)


def test_quaternion_is_unit_length():
    x, y, z, w = yaw_to_quaternion(0.7)
    assert math.sqrt(x * x + y * y + z * z + w * w) == pytest.approx(1.0)


# ----------------------------------------------------------------------
# 任务状态机
# ----------------------------------------------------------------------
def test_mission_happy_path():
    machine = MissionMachine(timeout=60.0)
    assert machine.state == IDLE
    assert machine.start('door_3', 'deliver_3', {'order_id': 'A03'}, 0.0)
    assert machine.state == NAVIGATING
    assert machine.current.route == 'deliver_3'
    assert machine.current.order['order_id'] == 'A03'

    assert machine.nav_done(True, 10.0)
    assert machine.state == DOCKING

    assert machine.parked(True, 20.0)
    assert machine.state == DONE
    assert machine.current.message == 'delivered'


def test_mission_navigation_failure():
    machine = MissionMachine()
    machine.start('door_3', 'deliver_3', {}, 0.0)
    assert machine.nav_done(False, 5.0)
    assert machine.state == FAILED


def test_mission_docking_failure():
    machine = MissionMachine()
    machine.start('door_3', 'deliver_3', {}, 0.0)
    machine.nav_done(True, 5.0)
    assert machine.parked(False, 8.0)
    assert machine.state == FAILED


def test_mission_cannot_start_twice():
    machine = MissionMachine()
    machine.start('a', 'a', {}, 0.0)
    assert not machine.start('b', 'b', {}, 1.0)
    assert machine.current.destination == 'a'


def test_mission_ignores_wrong_events():
    machine = MissionMachine()
    assert not machine.nav_done(True, 1.0)
    assert not machine.parked(True, 1.0)
    assert machine.state == IDLE


def test_mission_timeout():
    machine = MissionMachine(timeout=10.0)
    machine.start('a', 'a', {}, 0.0)
    assert not machine.tick(9.0)
    assert machine.tick(11.0)
    assert machine.state == FAILED


def test_mission_timeout_ignored_when_idle():
    machine = MissionMachine(timeout=10.0)
    assert not machine.tick(100.0)
    assert machine.state == IDLE


def test_mission_reset():
    machine = MissionMachine()
    machine.start('a', 'a', {}, 0.0)
    machine.reset(3.0)
    assert machine.state == IDLE
    assert machine.current.destination == ''


def test_mission_restart_after_failure():
    machine = MissionMachine()
    machine.start('a', 'a', {}, 0.0)
    machine.nav_done(False, 1.0)
    assert machine.start('b', 'b', {}, 2.0)
    assert machine.state == NAVIGATING
