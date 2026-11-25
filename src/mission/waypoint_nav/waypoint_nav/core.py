"""航点文件与路线的纯逻辑。

waypoints.yaml 约定：

    frame_id: map
    waypoints:
      dock:      {x: 1.2, y: 0.5, yaw: 0.0}
      shelf_a:   {x: 0.8, y: 2.0, yaw: 1.57}
      door_3:    {x: 3.5, y: 2.2, yaw: 3.14}
    routes:
      deliver_3: [shelf_a, door_3]
      home:      [dock]

yaw 单位弧度；routes 里按顺序执行。
"""

import math
from dataclasses import dataclass

import yaml


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


def load_config(path):
    """读取 YAML，返回 (frame_id, waypoints dict, routes dict)。"""
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    frame_id = str(data.get('frame_id', 'map'))
    raw_points = data.get('waypoints') or {}
    waypoints = {}
    for name, value in raw_points.items():
        if not isinstance(value, dict):
            raise ValueError('航点 %s 必须是 {x, y, yaw} 结构' % name)
        waypoints[str(name)] = Waypoint(
            name=str(name),
            x=float(value.get('x', 0.0)),
            y=float(value.get('y', 0.0)),
            yaw=float(value.get('yaw', 0.0)))

    raw_routes = data.get('routes') or {}
    routes = {}
    for name, seq in raw_routes.items():
        if not isinstance(seq, list):
            raise ValueError('路线 %s 必须是航点名称列表' % name)
        routes[str(name)] = [str(item) for item in seq]

    return frame_id, waypoints, routes


def resolve_route(name, waypoints, routes):
    """名字既可以是路线，也可以是单个航点。"""
    name = str(name).strip()
    if not name:
        raise KeyError('路线名为空')
    if name in routes:
        names = routes[name]
    elif name in waypoints:
        names = [name]
    else:
        raise KeyError('未知路线/航点：%s' % name)

    result = []
    for item in names:
        if item not in waypoints:
            raise KeyError('路线 %s 引用了不存在的航点 %s' % (name, item))
        result.append(waypoints[item])
    if not result:
        raise KeyError('路线 %s 为空' % name)
    return result


def yaw_to_quaternion(yaw):
    """绕 z 轴旋转的四元数 (x, y, z, w)。"""
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x, y, z, w):
    """四元数 → 平面 yaw（弧度）。"""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)
