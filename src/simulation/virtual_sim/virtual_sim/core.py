"""二维虚拟仿真核心（不依赖 ROS）。

World         生成俯视世界图、合成相机画面、深度图与激光雷达扫描。
SimCar        差速底盘运动积分与简单碰撞检测。

设计目标：
    - 不需要 Gazebo，一条命令就能在车上/电脑上跑通巡线、避障、二维码、泊车；
    - 相机画面是“俯视前向裁剪”，与真实相机视角不同，但巡线/二维码算法通用；
    - 画面非镜像：图像右侧 = 车右侧，与真实相机一致，转向符号不用改；
    - 雷达与深度图由同一世界模型生成，避障逻辑与实车一致。
"""

import math
from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw

from virtual_sim.qr_asset import QR_MATRIX


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float


class World:
    def __init__(self, size_m=6.0, ppm=300, near=0.25, far=2.25,
                 lateral_span=1.6, camera_w=640, camera_h=480):
        self.size_m = float(size_m)
        self.ppm = int(ppm)
        self.near = float(near)
        self.far = float(far)
        self.lateral_span = float(lateral_span)
        self.camera_w = int(camera_w)
        self.camera_h = int(camera_h)

        self.wall = 0.12
        # 障碍物放在赛道外侧，保证巡线路线本身畅通；
        # 想演示避障，把半径改大或手动把车开向它们即可。
        self.obstacles = [
            (1.35, 3.70, 0.18),
            (4.75, 1.20, 0.16),
        ]
        # 赛道终点 (4.6, 4.4) 朝 45°，二维码板正好在正前方 1.2m，
        # 这样相机在巡线末端就能完整看到码，泊车可以直接接管。
        self.path = [
            (0.70, 1.00), (1.60, 1.00), (2.20, 1.30), (2.80, 2.20),
            (3.40, 3.20), (4.20, 4.00), (4.60, 4.40),
        ]
        self.qr_center = (5.45, 5.25)
        self.qr_size = 0.70
        self.start = Pose2D(0.90, 1.00, 0.0)

        self.image = self._build_image()
        self._depth_column = self._build_depth_column()

    # ------------------------------------------------------------------
    def _m(self, value):
        return value * self.ppm

    def _build_image(self):
        size = int(round(self.size_m * self.ppm))
        image = Image.new('RGB', (size, size), (236, 236, 236))
        draw = ImageDraw.Draw(image)

        # 1 米网格，方便肉眼判断位置
        grid = max(1, int(self.ppm))
        for tick in range(1, int(round(self.size_m))):
            xy = tick * grid
            draw.line([(xy, 0), (xy, size)], fill=(222, 222, 222), width=1)
            draw.line([(0, xy), (size, xy)], fill=(222, 222, 222), width=1)

        # 边界墙
        wall = int(self.wall * self.ppm)
        draw.rectangle([0, 0, size - 1, size - 1], outline=(55, 55, 60),
                       width=wall)

        # 巡线路径
        points = [(self._m(x), self._m(y)) for x, y in self.path]
        draw.line(points, fill=(25, 25, 25),
                  width=max(4, int(0.045 * self.ppm)), joint='curve')

        # 障碍物
        for x, y, radius in self.obstacles:
            cx, cy = self._m(x), self._m(y)
            r = max(6, int(radius * self.ppm))
            draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                         fill=(210, 70, 60), outline=(120, 20, 20), width=3)

        # 二维码地标
        qx, qy = self._m(self.qr_center[0]), self._m(self.qr_center[1])
        half = int(self.qr_size * self.ppm / 2)
        draw.rectangle([qx - half, qy - half, qx + half, qy + half],
                       fill=(255, 255, 255), outline=(20, 20, 20), width=2)
        qr_img = self._qr_image()
        qr_px = int(self.qr_size * 0.82 * self.ppm)
        qr_img = qr_img.resize((qr_px, qr_px), Image.NEAREST)
        image.paste(qr_img, (int(qx - qr_px / 2), int(qy - qr_px / 2)))

        draw.text((int(self._m(self.qr_center[0])) - 24,
                   int(self._m(self.qr_center[1])) + half + 8),
                  'DOCK', fill=(20, 20, 20))
        draw.text((int(self._m(self.start.x)) - 22,
                   int(self._m(self.start.y)) - 22),
                  'START', fill=(20, 20, 20))
        return image

    @staticmethod
    def _qr_image():
        n = len(QR_MATRIX)
        img = Image.new('RGB', (n, n), (255, 255, 255))
        pixels = img.load()
        for row, line in enumerate(QR_MATRIX):
            for col, value in enumerate(line):
                if value == '1':
                    pixels[col, row] = (0, 0, 0)
        return img

    def _build_depth_column(self):
        """每一行像素对应的前向地面距离（米），上远下近。"""
        span = self.far - self.near
        depth = np.empty(self.camera_h, dtype=np.float32)
        for v in range(self.camera_h):
            depth[v] = self.near + span * (1.0 - v / max(1, self.camera_h - 1))
        return depth

    # ------------------------------------------------------------------
    def render_camera(self, pose):
        """返回 (H, W, 3) uint8 RGB 前向俯视图。

        非镜像约定：图像右侧对应车体右侧（ROS 中 -y），
        这样巡线的左右符号与真实相机完全一致。
        PIL 的 AFFINE 是“输出 → 输入”的逆映射。
        """
        width, height = self.camera_w, self.camera_h
        ppm = self.ppm
        ls = self.lateral_span / max(1, width - 1)
        k = (self.far - self.near) / max(1, height - 1)
        cx = (width - 1) / 2.0
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)

        a = ppm * ls * sin_t
        b = -ppm * k * cos_t
        c = ppm * (pose.x + self.far * cos_t - cx * ls * sin_t)
        d = -ppm * ls * cos_t
        e = -ppm * k * sin_t
        f = ppm * (pose.y + self.far * sin_t + cx * ls * cos_t)

        view = self.image.transform(
            (width, height), Image.AFFINE, (a, b, c, d, e, f),
            resample=Image.BILINEAR, fillcolor=(0, 0, 0))
        return np.asarray(view, dtype=np.uint8)

    def render_depth(self, pose):
        """返回 (H, W) float32，单位米。

        每列是一条与车头同向的平行射线：先求该列到墙/障碍的距离，
        像素深度取“地面距离”和“遮挡距离”的较小值，
        因此障碍物会在深度图上形成一段近值，与真实深度相机一致。
        """
        width, height = self.camera_w, self.camera_h
        ls = self.lateral_span / max(1, width - 1)
        cx = (width - 1) / 2.0
        max_range = self.far

        u = np.arange(width, dtype=np.float64)
        lateral = -(u - cx) * ls      # 车体系 y（左正右负）
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)

        ox = pose.x - lateral * sin_t
        oy = pose.y + lateral * cos_t

        hit = np.full(width, max_range, dtype=np.float64)

        if abs(cos_t) > 1e-9:
            target = (self.size_m - self.wall if cos_t > 0.0
                      else self.wall)
            t = (target - ox) / cos_t
            hit = np.minimum(hit, np.where(t > 0.02, t, max_range))
        if abs(sin_t) > 1e-9:
            target = (self.size_m - self.wall if sin_t > 0.0
                      else self.wall)
            t = (target - oy) / sin_t
            hit = np.minimum(hit, np.where(t > 0.02, t, max_range))

        for obs_x, obs_y, radius in self.obstacles:
            fx = ox - obs_x
            fy = oy - obs_y
            b = 2.0 * (fx * cos_t + fy * sin_t)
            c = fx * fx + fy * fy - radius * radius
            disc = b * b - 4.0 * c
            valid = disc >= 0.0
            if not bool(valid.any()):
                continue
            root = np.sqrt(np.maximum(disc, 0.0))
            t = (-b - root) / 2.0
            t = np.where(valid & (t > 0.02), t, max_range)
            hit = np.minimum(hit, t)

        depth = np.minimum(self._depth_column[:, None].astype(np.float64),
                           hit[None, :])
        return depth.astype(np.float32)

    # ------------------------------------------------------------------
    def scan(self, pose, rays=360, max_range=8.0):
        """返回 (ranges list, angle_min, angle_increment)。"""
        ranges = []
        angle_min = -math.pi
        increment = 2.0 * math.pi / rays
        xmin, xmax = self.wall, self.size_m - self.wall
        ymin, ymax = self.wall, self.size_m - self.wall

        for i in range(rays):
            angle = angle_min + i * increment
            dx = math.cos(angle)
            dy = math.sin(angle)
            distance = max_range

            # 到四面墙
            if dx > 1e-9:
                candidate = (xmax - pose.x) / dx
                if candidate >= 0:
                    distance = min(distance, candidate)
            elif dx < -1e-9:
                candidate = (xmin - pose.x) / dx
                if candidate >= 0:
                    distance = min(distance, candidate)
            if dy > 1e-9:
                candidate = (ymax - pose.y) / dy
                if candidate >= 0:
                    distance = min(distance, candidate)
            elif dy < -1e-9:
                candidate = (ymin - pose.y) / dy
                if candidate >= 0:
                    distance = min(distance, candidate)

            # 到圆形障碍
            for ox, oy, radius in self.obstacles:
                fx, fy = pose.x - ox, pose.y - oy
                b = 2.0 * (fx * dx + fy * dy)
                c = fx * fx + fy * fy - radius * radius
                disc = b * b - 4.0 * c
                if disc < 0.0:
                    continue
                root = math.sqrt(disc)
                t1 = (-b - root) / 2.0
                t2 = (-b + root) / 2.0
                for t in (t1, t2):
                    if t > 0.02:
                        distance = min(distance, t)
                        break

            ranges.append(float(min(max_range, distance)))
        return ranges, angle_min, increment


class SimCar:
    def __init__(self, pose, max_linear=0.5, max_angular=1.5,
                 body_radius=0.11):
        self.pose = Pose2D(pose.x, pose.y, pose.theta)
        self.start = Pose2D(pose.x, pose.y, pose.theta)
        self.max_linear = abs(float(max_linear))
        self.max_angular = abs(float(max_angular))
        self.body_radius = float(body_radius)

    def reset(self):
        self.pose = Pose2D(self.start.x, self.start.y, self.start.theta)

    def step(self, linear, angular, dt):
        v = max(-self.max_linear, min(self.max_linear, float(linear)))
        w = max(-self.max_angular, min(self.max_angular, float(angular)))
        half = 0.5 * w * dt
        self.pose.x += v * math.cos(self.pose.theta + half) * dt
        self.pose.y += v * math.sin(self.pose.theta + half) * dt
        self.pose.theta = math.atan2(
            math.sin(self.pose.theta + half * 2.0),
            math.cos(self.pose.theta + half * 2.0))
        return v, w

    def clamp_to_world(self, world):
        margin = world.wall + self.body_radius
        self.pose.x = max(margin, min(world.size_m - margin, self.pose.x))
        self.pose.y = max(margin, min(world.size_m - margin, self.pose.y))

    def collides(self, world):
        for ox, oy, radius in world.obstacles:
            if math.hypot(self.pose.x - ox,
                          self.pose.y - oy) < radius + self.body_radius:
                return True
        return False
