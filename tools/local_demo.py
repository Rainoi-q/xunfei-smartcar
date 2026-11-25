"""本地纯算法闭环演示：不需要 ROS，也不需要上车。

用 virtual_sim 的世界模型 + 真实的 core 逻辑（巡线跟踪、PID、避障、仲裁、
泊车状态机）跑一遍“巡线 → 发现二维码 → 泊车停稳”，并输出：

    out/frame_XX.png    相机视角 + 叠加信息（类似 HUD）
    out/trajectory.png  俯视轨迹
    out/summary.txt     运行结论
    out/preview.html    把所有结果放在一个网页里看

注意：
    - 二维码框是“几何投影”模拟的（车上有 pyzbar 真解码）；
    - 其他算法与车上节点用的是同一份 core 代码。

用法：
    python tools/local_demo.py
    python tools/local_demo.py --seconds 60 --out out
"""

import argparse
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for rel in [
        'src/perception/vision_detect',
        'src/control/line_follower',
        'src/control/obstacle_avoider',
        'src/control/cmd_vel_arbiter',
        'src/control/parking',
        'src/simulation/virtual_sim']:
    path = os.path.join(ROOT, rel)
    if path not in sys.path:
        sys.path.insert(0, path)

from cmd_vel_arbiter.core import Cmd, CommandArbiter   # noqa: E402
from line_follower.core import LinePid                 # noqa: E402
from obstacle_avoider.core import evaluate_scan         # noqa: E402
from parking.core import FAILED, DockController        # noqa: E402
from virtual_sim.core import SimCar, World             # noqa: E402
from virtual_sim.qr_asset import QR_PAYLOAD            # noqa: E402
from vision_detect.core import LineTracker             # noqa: E402

DT = 0.05            # 20Hz 控制
VISION_EVERY = 2     # 10Hz 视觉
SCAN_EVERY = 2       # 10Hz 雷达
THRESHOLD = 100      # 与 params.yaml 一致：白底黑线
ROI_TOP = 0.60
SNAP_TIMES = [6.0, 12.0, 18.0, 24.0, 30.0, 36.0, 42.0, 48.0]


def _font(size=16):
    for name in ('arial.ttf', 'segoeui.ttf', 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            continue
    return ImageFont.load_default()


def project_board(pose, world):
    """把二维码板四角投到相机画面，返回 (bbox, 中心前向距离) 或 None。

    与 render_camera 的仿射投影严格一致，用来在本地模拟 pyzbar 的检测结果。
    """
    width, height = world.camera_w, world.camera_h
    ls = world.lateral_span / max(1, width - 1)
    k = (world.far - world.near) / max(1, height - 1)
    cx_px = (width - 1) / 2.0
    cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
    half = world.qr_size / 2.0
    bx, by = world.qr_center

    us, vs = [], []
    for dx in (-half, half):
        for dy in (-half, half):
            wx, wy = bx + dx, by + dy
            ddx, ddy = wx - pose.x, wy - pose.y
            forward = ddx * cos_t + ddy * sin_t
            left = -ddx * sin_t + ddy * cos_t
            if not (world.near < forward < world.far):
                return None
            u = cx_px + (-left) / ls
            v = (world.far - forward) / k
            if not (-2.0 <= u <= width + 1.0 and -2.0 <= v <= height + 1.0):
                return None
            us.append(u)
            vs.append(v)

    u0, u1 = min(us), max(us)
    v0, v1 = min(vs), max(vs)
    bbox = ((u0 + u1) / 2.0, (v0 + v1) / 2.0, u1 - u0, v1 - v0)
    forward = world.far - bbox[1] * k
    return bbox, forward


def depth_at(depth, bbox):
    cx, cy, w, h = bbox
    y0 = int(max(0, cy - h * 0.25))
    y1 = int(min(depth.shape[0], cy + h * 0.25))
    x0 = int(max(0, cx - w * 0.25))
    x1 = int(min(depth.shape[1], cx + w * 0.25))
    if y1 <= y0 or x1 <= x0:
        return None
    roi = depth[y0:y1, x0:x1]
    roi = roi[np.isfinite(roi)]
    roi = roi[(roi > 0.05) & (roi < 5.0)]
    if roi.size < 4:
        return None
    return float(np.median(roi))


def draw_frame(frame, info, tracker_result, bbox):
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, 'RGBA')
    font = _font(15)
    small = _font(13)

    draw.rectangle([0, 0, image.width, 62], fill=(0, 0, 0, 165))
    draw.rectangle([0, image.height - 58, image.width, image.height],
                   fill=(0, 0, 0, 160))

    draw.text((10, 6), 'XUNFEI CUP 2026  SMART CAR', fill=(0, 220, 255),
              font=font)
    draw.text((10, 26), 'MISSION: ' + info['mission'], fill=(255, 255, 255),
              font=font)
    draw.text((10, 44), 'QR: ' + info['qr'], fill=(255, 255, 255), font=small)
    draw.text((image.width - 190, 8), 'T+%05.1fs' % info['t'],
              fill=(210, 210, 210), font=font)

    # 巡线拟合点
    if tracker_result is not None and not tracker_result.lost:
        pts = [(x, image.height - y) for x, y in tracker_result.points]
        for x, y in pts:
            draw.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(0, 255, 0, 220))
        if len(pts) >= 2:
            draw.line(pts, fill=(0, 255, 0), width=2)
        bx = tracker_result.bottom_x
        draw.ellipse([bx - 8, image.height - 20, bx + 8, image.height - 4],
                     fill=(0, 255, 0, 220))
    else:
        draw.text((10, 76), 'LINE LOST', fill=(255, 60, 60), font=font)

    if bbox is not None:
        cx, cy, w, h = bbox
        draw.rectangle([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2],
                       outline=(255, 0, 255), width=3)
        draw.text((cx - w / 2, cy - h / 2 - 18), 'QR', fill=(255, 0, 255),
                  font=font)

    y = image.height - 50
    draw.text((10, y), 'LINE %s' % info['line'], fill=(0, 255, 0), font=small)
    draw.text((10, y + 18), 'V %+.2f  W %+.2f' % (info['v'], info['w']),
              fill=(0, 255, 0), font=small)
    draw.text((230, y), 'OBSTACLE %s' % info['obstacle'],
              fill=(255, 220, 0), font=small)
    draw.text((230, y + 18), 'PARK %s' % info['park'],
              fill=(255, 220, 0), font=small)
    draw.text((470, y), 'MODE %s' % info['mode'], fill=(255, 255, 255),
              font=small)
    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--out', default=os.path.join(ROOT, 'out'))
    args = parser.parse_args()

    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    # 清掉上一次运行留下的帧，避免新旧结果混在一起
    for name in os.listdir(out):
        if name.startswith('frame_') and name.endswith('.png'):
            try:
                os.remove(os.path.join(out, name))
            except OSError:
                pass

    world = World(ppm=300, camera_w=640, camera_h=480)
    car = SimCar(world.start)
    tracker = LineTracker(num_bands=10, band_height_px=12, min_pixels=300,
                          max_slope=1.5)
    pid = LinePid(kp=1.2, ki=0.05, kd=0.10, target_speed=0.25,
                  min_speed_ratio=0.40, max_angular=1.5, integral_limit=1.0)
    dock = DockController(kp_angular=1.4, center_tol=0.12,
                          align_exit_tol=0.20, target_height_ratio=2.0,
                          target_distance=1.0, approach_speed=0.18,
                          search_speed=0.35, hold_sec=1.5, timeout=45.0)
    arbiter = CommandArbiter(max_linear=0.5, max_angular=1.5,
                             line_timeout=0.5, park_timeout=0.5)

    mission = 'IDLE'
    parking_active = False
    qr_seen = False
    qr_text = '--'
    marker = None
    depth_latest = None
    result_latest = None
    offset = math.nan
    scan_state = 'CLEAR'
    scan_distance = math.inf
    avoid_scale = 1.0
    avoid_bias = 0.0
    dock_out = None
    cmd = Cmd()
    min_clearance = math.inf
    max_error = 0.0
    dock_trigger_t = None
    snap_index = 0
    frames = []
    trajectory = [(car.pose.x, car.pose.y)]

    def snapshot(now, result=None, bbox=None, frame=None):
        if frame is None:
            frame = world.render_camera(car.pose)
        frames.append(_frame_payload(
            frame, result, bbox, mission, qr_text, offset, scan_state,
            scan_distance, dock_out, cmd, now))

    t = 0.0
    step = 0
    while t < args.seconds and mission not in ('DONE', 'FAILED'):
        pose = car.pose

        # ---------------- 视觉 10Hz ----------------
        if step % VISION_EVERY == 0:
            frame = world.render_camera(pose)
            gray = np.dot(frame[..., :3].astype(np.float32),
                          np.array([0.299, 0.587, 0.114],
                                   dtype=np.float32)).astype(np.uint8)
            top = int(gray.shape[0] * ROI_TOP)
            mask = (gray[top:] < THRESHOLD).astype(np.uint8) * 255
            result_latest = tracker.extract(mask)
            offset = result_latest.offset

            depth_latest = world.render_depth(pose)
            projected = project_board(pose, world)
            marker = projected[0] if projected is not None else None

            if marker is not None and not qr_seen:
                qr_seen = True
                qr_text = 'order A03 -> door_3'
                mission = 'DOCKING'
                parking_active = True
                dock.reset()
                dock_trigger_t = t
                print('T+%5.1fs  检测到二维码，开始泊车（距板 %.2fm）' % (
                    t, projected[1]))
                snapshot(t, result_latest, marker, frame)
            elif snap_index < len(SNAP_TIMES) and t >= SNAP_TIMES[snap_index]:
                snap_index += 1
                snapshot(t, result_latest, marker, frame)
        else:
            frame = None

        # ---------------- 雷达 10Hz ----------------
        if step % SCAN_EVERY == 0:
            ranges, angle_min, increment = world.scan(pose, rays=180)
            decision = evaluate_scan(ranges, angle_min, increment,
                                     stop_distance=0.35, slow_distance=0.85)
            scan_state = decision.state
            scan_distance = decision.front_min
            if math.isfinite(scan_distance):
                min_clearance = min(min_clearance, scan_distance)
            avoid_scale = decision.scale
            avoid_bias = decision.bias

        # ---------------- 控制 20Hz ----------------
        line_cmd = (0.0, 0.0, math.nan)
        if not math.isnan(offset):
            max_error = max(max_error, abs(offset))
            v, w = pid.update(offset, DT)
            line_cmd = (v, w, t)

        park_cmd = None
        if parking_active:
            depth_m = depth_at(depth_latest, marker) if (
                depth_latest is not None and marker is not None) else None
            dock_out = dock.update(t, marker, world.camera_w,
                                   world.camera_h, depth_m=depth_m)
            park_cmd = (dock_out.linear, dock_out.angular, t)
            if dock_out.done:
                mission = 'DONE'
                parking_active = False
                print('T+%5.1fs  已停稳，任务完成' % t)
            elif dock_out.state == FAILED:
                mission = 'FAILED'
                parking_active = False

        cmd = arbiter.compute(
            t,
            line=None if math.isnan(line_cmd[2]) else line_cmd,
            park=park_cmd,
            park_active=parking_active and park_cmd is not None,
            avoid_scale=avoid_scale, avoid_bias=avoid_bias,
            emergency=(scan_state == 'STOP'))

        car.step(cmd.linear, cmd.angular, DT)
        if car.collides(world):
            mission = 'FAILED'
            print('T+%5.1fs  仿真车撞到障碍物' % t)
        car.clamp_to_world(world)

        if step % 10 == 0:
            trajectory.append((car.pose.x, car.pose.y))

        t += DT
        step += 1

    trajectory.append((car.pose.x, car.pose.y))
    snapshot(t, result_latest, marker)

    # ---------------- 输出 ----------------
    saved = []
    for index, payload in enumerate(frames):
        image = draw_frame(payload['frame'], payload['info'],
                           payload.get('result'), payload.get('bbox'))
        path = os.path.join(out, 'frame_%02d.png' % index)
        image.save(path)
        saved.append((path, payload['info']['t']))

    trajectory_path = os.path.join(out, 'trajectory.png')
    _plot_trajectory(world, trajectory, car, trajectory_path)

    distance_to_board = math.hypot(car.pose.x - world.qr_center[0],
                                   car.pose.y - world.qr_center[1])
    summary = [
        '本地闭环演示结果',
        '================',
        '任务结果：%s' % mission,
        '仿真时长：%.1f s' % t,
        '触发泊车用时：%s' % ('%.1f s' % dock_trigger_t
                              if dock_trigger_t is not None else '未触发'),
        '终点到底板中心距离：%.2f m' % distance_to_board,
        '巡线最大偏移：%.2f（0=居中，1=最右）' % max_error,
        '最近障碍距离：%s' % ('%.2f m' % min_clearance
                              if math.isfinite(min_clearance) else '无'),
        '二维码内容：%s' % QR_PAYLOAD,
        '',
        '说明：二维码框在本地是按几何投影模拟的；',
        '车上由 pyzbar 真正解码，其余算法完全一致。',
    ]
    summary_text = '\n'.join(summary)
    with open(os.path.join(out, 'summary.txt'), 'w', encoding='utf-8') as fh:
        fh.write(summary_text + '\n')
    print(summary_text)

    html = ['<!doctype html><html lang="zh-CN"><meta charset="utf-8">',
            '<title>讯飞杯 2026 智能车 本地预览</title>',
            '<style>body{background:#111;color:#eee;font-family:Segoe UI,'
            'Microsoft YaHei,sans-serif;margin:24px}img{max-width:100%;'
            'border:1px solid #333;margin:8px 0}pre{background:#1b1b1b;'
            'padding:12px;border:1px solid #333}</style>',
            '<h1>智能车闭环演示（本地、无 ROS）</h1>',
            '<pre>%s</pre>' % summary_text.replace('&', '&amp;').replace(
                '<', '&lt;'),
            '<h2>俯视轨迹</h2>',
            '<img src="trajectory.png">',
            '<h2>相机视角（模拟 HUD）</h2>']
    for path, stamp in saved:
        html.append('<p>T+%.1fs</p><img src="%s">' % (
            stamp, os.path.basename(path)))
    html.append('</html>')
    html_path = os.path.join(out, 'preview.html')
    with open(html_path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(html))

    print('\n已生成：')
    print('  ' + html_path)
    print('  ' + trajectory_path)
    print('  帧图片 %d 张在 %s' % (len(saved), out))
    return 0 if mission == 'DONE' else 1


def _frame_payload(frame, result, bbox, mission, qr_text, offset,
                   scan_state, scan_distance, dock_out, cmd, t):
    if result is not None:
        line_text = ('LOST' if result.lost else 'off %+.2f' % result.offset)
    else:
        line_text = 'off %+.2f' % offset if not math.isnan(offset) else '--'

    park_text = '--'
    if dock_out is not None:
        park_text = dock_out.state
        if dock_out.state in ('APPROACH', 'HOLD', 'DONE'):
            park_text += ' %d%%' % int(dock_out.progress * 100)

    return {
        'frame': frame,
        'result': result,
        'bbox': bbox,
        'info': {
            't': t,
            'mission': mission,
            'qr': qr_text,
            'line': line_text,
            'obstacle': ('%s %.2fm' % (scan_state, scan_distance)
                         if math.isfinite(scan_distance) else scan_state),
            'park': park_text,
            'v': cmd.linear,
            'w': cmd.angular,
            'mode': cmd.mode,
        },
    }


def _plot_trajectory(world, trajectory, car, path):
    image = world.image.copy()
    draw = ImageDraw.Draw(image)
    scale = world.ppm
    points = [(x * scale, y * scale) for x, y in trajectory]
    draw.line(points, fill=(30, 110, 220), width=5)

    sx, sy = world.start.x * scale, world.start.y * scale
    draw.ellipse([sx - 9, sy - 9, sx + 9, sy + 9], fill=(0, 150, 0))
    ex, ey = car.pose.x * scale, car.pose.y * scale
    draw.ellipse([ex - 9, ey - 9, ex + 9, ey + 9], fill=(220, 30, 30))
    draw.text((sx + 12, sy - 20), 'START', fill=(0, 90, 0))
    draw.text((ex + 12, ey + 6), 'STOP', fill=(150, 0, 0))

    width = 900
    ratio = width / image.width
    image = image.resize((width, int(image.height * ratio)),
                         Image.LANCZOS)
    image.save(path)


if __name__ == '__main__':
    sys.exit(main())
