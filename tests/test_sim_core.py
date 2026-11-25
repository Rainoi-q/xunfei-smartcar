"""虚拟仿真核心的测试：几何、渲染方向、深度与雷达。"""

import math

import numpy as np
import pytest

from virtual_sim.core import Pose2D, SimCar, World
from virtual_sim.qr_asset import QR_MATRIX, QR_PAYLOAD


@pytest.fixture(scope='module')
def world():
    return World(ppm=150, camera_w=320, camera_h=240, size_m=6.0)


def test_qr_asset_is_square_and_decodable_size():
    assert len(QR_MATRIX) == 37
    assert all(len(row) == 37 for row in QR_MATRIX)
    assert all(set(row) <= {'0', '1'} for row in QR_MATRIX)
    assert 'door_3' in QR_PAYLOAD


def test_start_is_inside_world(world):
    assert 0 < world.start.x < world.size_m
    assert 0 < world.start.y < world.size_m


def test_camera_shape_and_dtype(world):
    frame = world.render_camera(world.start)
    assert frame.shape == (240, 320, 3)
    assert frame.dtype == np.uint8


def test_camera_is_not_mirrored(world):
    """车头朝 +x 时，世界 -y 在图像右侧——与真实相机一致。"""
    world.image.putpixel((int(1.9 * world.ppm), int(0.6 * world.ppm)),
                         (0, 0, 255))
    for dx in range(3):
        for dy in range(3):
            world.image.putpixel(
                (int(1.9 * world.ppm) + dx, int(0.6 * world.ppm) + dy),
                (0, 0, 255))
            world.image.putpixel(
                (int(1.9 * world.ppm) + dx, int(1.4 * world.ppm) + dy),
                (0, 255, 0))

    frame = world.render_camera(world.start).astype(int)
    blue = (frame[:, :, 2] > 180) & (frame[:, :, 0] < 80)
    green = (frame[:, :, 1] > 180) & (frame[:, :, 0] < 80)
    assert blue.any() and green.any()

    blue_x = np.argwhere(blue)[:, 1].mean()
    green_x = np.argwhere(green)[:, 1].mean()
    assert blue_x > 160, '右侧物体应出现在图像右半边'
    assert green_x < 160, '左侧物体应出现在图像左半边'


def test_camera_looks_forward(world):
    """车正前方 1.5m 处的标记应出现在画面中。"""
    px = int((world.start.x + 1.5) * world.ppm)
    py = int(world.start.y * world.ppm)
    for dx in range(-3, 4):
        for dy in range(-3, 4):
            world.image.putpixel((px + dx, py + dy), (255, 0, 255))
    frame = world.render_camera(world.start).astype(int)
    magenta = ((frame[:, :, 0] > 180) & (frame[:, :, 2] > 180)
               & (frame[:, :, 1] < 90))
    assert magenta.any()
    rows = np.argwhere(magenta)[:, 0]
    assert rows.max() < 240


def test_depth_shape_and_range(world):
    depth = world.render_depth(world.start)
    assert depth.shape == (240, 320)
    assert depth.dtype == np.float32
    finite = depth[np.isfinite(depth)]
    assert finite.min() >= world.near - 1e-6
    assert finite.max() <= world.far + 1e-6


def test_depth_top_is_further_than_bottom(world):
    depth = world.render_depth(world.start)
    column = depth[:, 160]
    assert column[0] > column[-1]


def test_depth_obstacle_occludes(world):
    """障碍物正前方时，其后方的深度应被“拉近”。"""
    pose = Pose2D(1.0, 3.70, 0.0)      # 正对 (1.35, 3.70) 的障碍
    depth = world.render_depth(pose)
    center = depth[:, 160]
    # 障碍在正前方 0.35m，画面中心列不应出现 1.5m 以上的远距离
    assert np.nanmin(center) < 0.6


def test_scan_ranges(world):
    ranges, angle_min, increment = world.scan(world.start, rays=90,
                                              max_range=8.0)
    assert len(ranges) == 90
    assert angle_min == pytest.approx(-math.pi)
    assert increment == pytest.approx(2.0 * math.pi / 90)
    assert all(0.0 < value <= 8.0 for value in ranges)


def test_scan_detects_obstacle(world):
    pose = Pose2D(1.0, 3.70, 0.0)
    ranges, angle_min, increment = world.scan(pose, rays=360)
    # 正前方 0° 对应的索引
    index = int(round((0.0 - angle_min) / increment)) % 360
    assert ranges[index] < 0.5


def test_scan_wall_ahead(world):
    pose = Pose2D(5.0, 3.0, 0.0)
    ranges, angle_min, increment = world.scan(pose, rays=360)
    index = int(round((0.0 - angle_min) / increment)) % 360
    assert ranges[index] == pytest.approx(world.size_m - world.wall - 5.0,
                                          abs=0.05)


def test_simcar_straight_line():
    car = SimCar(Pose2D(1.0, 1.0, 0.0))
    car.step(0.5, 0.0, 1.0)
    assert car.pose.x == pytest.approx(1.5)
    assert car.pose.y == pytest.approx(1.0)
    assert car.pose.theta == pytest.approx(0.0)


def test_simcar_turn_left_increases_yaw():
    car = SimCar(Pose2D(1.0, 1.0, 0.0))
    car.step(0.0, 1.0, 0.5)
    assert car.pose.theta == pytest.approx(0.5, abs=1e-6)


def test_simcar_clamps_command():
    car = SimCar(Pose2D(1.0, 1.0, 0.0), max_linear=0.3, max_angular=0.5)
    v, w = car.step(5.0, 5.0, 0.1)
    assert v == pytest.approx(0.3)
    assert w == pytest.approx(0.5)


def test_simcar_reset_and_collision(world):
    car = SimCar(Pose2D(world.start.x, world.start.y, world.start.theta))
    car.step(0.2, 0.0, 0.5)
    car.reset()
    assert car.pose.x == pytest.approx(world.start.x)

    obstacle = world.obstacles[0]
    car.pose = Pose2D(obstacle[0], obstacle[1], 0.0)
    assert car.collides(world)

    car.pose = Pose2D(1.0, 1.0, 0.0)
    assert not car.collides(world)


def test_simcar_clamped_inside_walls(world):
    car = SimCar(Pose2D(0.0, 0.0, 0.0))
    car.clamp_to_world(world)
    assert car.pose.x >= world.wall
    assert car.pose.y >= world.wall
