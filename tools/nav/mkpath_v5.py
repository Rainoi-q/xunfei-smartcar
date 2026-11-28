#!/usr/bin/env python3
# mkpath_v5 —— 离墙加权 Dijkstra 路径生成（2026-11-28）
#
# 相对旧 mkpath.py 的修改：
#   1) unknown(205) 默认不可通行（旧版把 unknown 当 free，路径可能穿出已建图区）。
#      --allow-unknown 可恢复旧行为（会打印警告）。
#   2) MINC 可配（--minc，默认 0.15）；净空用“到不可通行格”的距离。
#   3) 平滑后逐点复核净空，低于 MINC*0.9 就退回未平滑路径，避免拐角内切。
#   4) 终点用 waypoints.yaml 的精确坐标+yaw；吸附偏移 >0.03 m 时把精确终点补回路径末尾。
#   5) 输出到 ~/drive4/route_path3.json（重启不丢），并打印每段长度/最小净空/吸附偏移。
#
# 用法：
#   python3 mkpath_v5.py                          # 默认 MINC=0.15
#   python3 mkpath_v5.py --minc 0.14 --out ~/drive4/route_path3.json
#   python3 mkpath_v5.py --allow-unknown          # 旧行为，仅作对照

import argparse, heapq, json, math, os, sys

import numpy as np
import cv2, yaml

HOME = os.path.expanduser('~')
DEF_MAP = os.path.join(HOME, 'wheeltec_ros2/install/wheeltec_nav2/share/wheeltec_nav2/map/WHEELTEC.yaml')
DEF_WP = os.path.join(HOME, 'smartcar_ws/src/smartcar_bringup/config/waypoints.yaml')
DEF_OUT = os.path.join(HOME, 'drive4/route_path3.json')

ap = argparse.ArgumentParser()
ap.add_argument('--map', default=DEF_MAP)
ap.add_argument('--wp', default=DEF_WP)
ap.add_argument('--out', default=DEF_OUT)
ap.add_argument('--minc', type=float, default=0.15, help='起点/中途允许的最小净空（米）')
ap.add_argument('--allow-unknown', action='store_true', help='把 unknown 当可通行（旧行为）')
ap.add_argument('--wall-weight', type=float, default=0.40, help='离墙加权系数')
ap.add_argument('--goal-minc', type=float, default=0.10, help='终点吸附允许的最小净空（米）')
a = ap.parse_args()

meta = yaml.safe_load(open(a.map))
img = meta['image']
img_path = img if img.startswith('/') else os.path.dirname(a.map) + '/' + img
pgm = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
if pgm is None:
    print('MAP_LOAD_FAIL %s' % img_path)
    sys.exit(1)
H, W = pgm.shape
res = float(meta['resolution'])
ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

free = pgm >= 250
if a.allow_unknown:
    trav = pgm >= 200
    print('WARN: --allow-unknown 把未建图区域当可通行')
else:
    trav = free
dist = cv2.distanceTransform(trav.astype(np.uint8), cv2.DIST_L2, 5) * res
occ = pgm < 100
d_occ = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
passable = trav & (dist >= a.minc)

print('map=%s %dx%d res=%.3f free=%.2fm2 minc=%.2f allow_unknown=%s passable=%d'
      % (img_path, H, W, res, float(free.sum()) * res * res, a.minc,
         a.allow_unknown, int(passable.sum())), flush=True)


def w2c(x, y):
    return (H - 1 - int(round((y - oy) / res)), int(round((x - ox) / res)))


def c2w(r, c):
    return (c * res + ox, (H - 1 - r) * res + oy)


def nearest_passable(r0, c0, max_rad=40, min_clear=None):
    """在半径 max_rad 格内找最近的 passable 格；min_clear 可放宽终点净空。"""
    if min_clear is not None:
        ok = trav & (dist >= min_clear)
    else:
        ok = passable
    if 0 <= r0 < H and 0 <= c0 < W and ok[r0, c0]:
        return r0, c0, 0.0
    best = None
    for rad in range(1, max_rad + 1):
        r_lo, r_hi = max(0, r0 - rad), min(H, r0 + rad + 1)
        c_lo, c_hi = max(0, c0 - rad), min(W, c0 + rad + 1)
        for rr in range(r_lo, r_hi):
            for cc in (c_lo, c_hi - 1):
                if ok[rr, cc]:
                    dd = math.hypot(rr - r0, cc - c0)
                    if best is None or dd < best[0]:
                        best = (dd, rr, cc)
        for cc in range(c_lo, c_hi):
            for rr in (r_lo, r_hi - 1):
                if ok[rr, cc]:
                    dd = math.hypot(rr - r0, cc - c0)
                    if best is None or dd < best[0]:
                        best = (dd, rr, cc)
        if best is not None:
            return best[1], best[2], best[0] * res
    return None


def plan(p, q, goal_clear=None):
    r0, c0 = w2c(*p)
    r1, c1 = w2c(*q)
    if not (0 <= r0 < H and 0 <= c0 < W and 0 <= r1 < H and 0 <= c1 < W):
        return None
    s = nearest_passable(r0, c0, 40, None)
    g = nearest_passable(r1, c1, 40, goal_clear if goal_clear is not None else None)
    if s is None or g is None:
        return None
    r0, c0, s_shift = s
    r1, c1, g_shift = g
    INF = 1e18
    D = np.full((H, W), INF)
    D[r0, c0] = 0.0
    prev = {}
    pq = [(0.0, r0, c0)]
    nb = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
          (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    while pq:
        d, r, c = heapq.heappop(pq)
        if d > D[r, c]:
            continue
        if (r, c) == (r1, c1):
            break
        for dr, dc, w in nb:
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and passable[rr, cc]:
                cl = max(float(dist[rr, cc]), 0.05)
                nd = d + w * res * (1.0 + (a.wall_weight / cl) ** 3)
                if nd < D[rr, cc]:
                    D[rr, cc] = nd
                    prev[(rr, cc)] = (r, c)
                    heapq.heappush(pq, (nd, rr, cc))
    if D[r1, c1] >= INF:
        return None
    cells = [(r1, c1)]
    cur = (r1, c1)
    while cur != (r0, c0):
        cur = prev[cur]
        cells.append(cur)
    cells.reverse()
    pts = [c2w(r, c) for r, c in cells]
    return pts, s_shift, g_shift


def smooth(pts, win=5, iters=3):
    P = np.array(pts, dtype=float)
    out = P.copy()
    for _ in range(iters):
        Q = out.copy()
        for i in range(1, len(out) - 1):
            aa = max(0, i - win // 2)
            bb = min(len(out), i + win // 2 + 1)
            Q[i] = out[aa:bb].mean(axis=0)
        out = Q
    return out


def clearance_at(x, y):
    r, c = w2c(x, y)
    if not (0 <= r < H and 0 <= c < W):
        return -1.0
    return float(dist[r, c])


def occ_clearance_at(x, y):
    """到真实障碍(occupied)格中心的距离；比 dist 宽松（不含 unknown），用于限速。"""
    r, c = w2c(x, y)
    if not (0 <= r < H and 0 <= c < W):
        return 0.0
    return float(d_occ[r, c])


def resample(P, step=0.05):
    R = [P[0]]
    acc = 0.0
    for k in range(1, len(P)):
        d = np.hypot(*(P[k] - P[k - 1]))
        acc += d
        if acc >= step:
            R.append(P[k])
            acc = 0.0
    if np.hypot(*(R[-1] - P[-1])) > 0.02:
        R.append(P[-1])
    return np.array(R)


wp = yaml.safe_load(open(a.wp))['waypoints']
names = ['start', 'dock_1', 'dock_2', 'dock_3']
for nm in names:
    if nm not in wp:
        print('WP_MISSING %s' % nm)
        sys.exit(1)
WPS = [(float(wp[nm]['x']), float(wp[nm]['y'])) for nm in names]

out = []
for i in range(3):
    res_pl = plan(WPS[i], WPS[i + 1], a.goal_minc)
    if res_pl is None:
        print('leg%d NO_PATH at minc=%.2f' % (i + 1, a.minc), flush=True)
        sys.exit(2)
    pts, s_shift, g_shift = res_pl
    P = np.array(pts, dtype=float)
    S = smooth(P, 5, 3)
    bad = [clearance_at(x, y) for x, y in S]
    if min(bad) < a.minc * 0.9:
        print('leg%d smooth rejected min_clear=%.3f' % (i + 1, min(bad)), flush=True)
        S = P
    # 终点精确补回
    goal = WPS[i + 1]
    if math.hypot(S[-1, 0] - goal[0], S[-1, 1] - goal[1]) > 0.03:
        S = np.vstack([S, np.array(goal)])
    # 起点精确补回（第一段）
    start = WPS[i]
    if math.hypot(S[0, 0] - start[0], S[0, 1] - start[1]) > 0.03:
        S = np.vstack([np.array(start), S])
    R = resample(S, 0.05)
    clr = [clearance_at(x, y) for x, y in R]
    occlr = [occ_clearance_at(x, y) for x, y in R]
    kmin = int(np.argmin(occlr))
    L = sum(np.hypot(*(R[k] - R[k - 1])) for k in range(1, len(R)))
    print('leg%d len=%.2fm min_clear=%.3f min_wall=%.3f at (%.2f,%.2f) pts=%d snap start=%.2f goal=%.2f'
          % (i + 1, L, min(clr), min(occlr), R[kmin, 0], R[kmin, 1], len(R), s_shift, g_shift),
          flush=True)
    out.append({
        'leg': i + 1,
        'goal': [float(goal[0]), float(goal[1]), float(wp[names[i + 1]]['yaw'])],
        'pts': [[round(float(x), 3), round(float(y), 3)] for x, y in R],
        'clr': [round(float(v), 3) for v in occlr],
    })

os.makedirs(os.path.dirname(a.out), exist_ok=True)
json.dump(out, open(a.out, 'w'))
print('saved %s' % a.out, flush=True)
