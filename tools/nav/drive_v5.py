#!/usr/bin/env python3
# drive_v5 —— 按车体轮廓的安全路径跟踪（2026-11-28）
#
# 相对 v4 的修改：
#   1) /scan 与 map->base_footprint 都做时效检查；断流先停车，超 3 s 退出。
#   2) 位姿跳变检测（AMCL 重定位/打滑时不会突然打满方向）。
#   3) 异常、Ctrl-C、正常结束都保证向 /cmd_vel 发零速（底盘源码里没有超时保护）。
#   4) 删除后向盲倒：只有后向真的有点（>=12 个）且余量 >0.25 m 才允许倒车；
#      当前 120°~230° 被 angle_disable 屏蔽，实际上永远不会倒。
#   5) 到点对齐改用里程计相对航向（AMCL 航向 ±20° 抖动不会再让它原地来回转）。
#   6) GUARD 先停车复核；只有在两侧都有真实数据、余量 >ROT_SIDE_MIN、且前角扫掠
#      方向余量足够时才小步慢转，最多试 3 次，否则安全停住（宁停不蹭）。
#   7) 余量计算向量化，并按扫描角度网格缓存 env，4500 点不再逐点算三角函数。
#   8) 近距盲区检测：一侧原本能看到 <0.26 m 的墙、随后整侧变空，会打印 DEADZONE。
#      （这是给 min_range=0.2 现状兜底观察用的；把 min_range 降下来后就不会出现。）
#   9) DRY=1 只打印 10 s 诊断（range_min/raw 余量/位姿），不发任何速度。
#  10) 正前扇区从 ±57° 收窄到 ±34°（窄道侧墙不再拖累 cf）；前角扫掠窗口加 ±1.15° 余量，
#      避免采样错位把角点本身漏掉。
#
# 环境变量：
#   ROUTE=~/drive4/route_path3.json   START_LEG=0   DRY=0|1
#   VMAX=0.05  VMIN=0.02  STOP_F=0.06  STOP_S=0.03
#   ALIGN_CLEAR=0.08  ROT_SIDE_MIN=0.10  DEADZONE_GUARD=0|1
#   ODOM_TOPIC=/odometry/filtered
#
# 注意：STOP_F/STOP_S 是“车体到墙”的余量；当前雷达 min_range=0.2，
# 这两个阈值落在盲区里不会触发——到现场把 min_range 降到 0.10 左右后再跑。

import math, os, sys, time, json, traceback

import numpy as np
import rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

HOME = os.path.expanduser('~')
DRY = int(os.environ.get('DRY', '0'))
START_LEG = int(os.environ.get('START_LEG', '0'))
VMAX = float(os.environ.get('VMAX', '0.05'))
VMIN = float(os.environ.get('VMIN', '0.02'))
STOP_F = float(os.environ.get('STOP_F', '0.06'))
STOP_S = float(os.environ.get('STOP_S', '0.03'))
ALIGN_CLEAR = float(os.environ.get('ALIGN_CLEAR', '0.08'))
ROT_SIDE_MIN = float(os.environ.get('ROT_SIDE_MIN', '0.10'))
DEADZONE_GUARD = int(os.environ.get('DEADZONE_GUARD', '0'))
ODOM_TOPIC = os.environ.get('ODOM_TOPIC', '/odometry/filtered')

# 车体轮廓（激光坐标系）：激光在 base_link 前方 0.0316 m
ENV_XF = 0.100     # 车头在激光前 0.100 m
ENV_XR = 0.167     # 车尾在激光后 0.167 m
ENV_Y = 0.116      # 轮胎外缘 ±0.116 m
CORNER_R = math.hypot(ENV_XF, ENV_Y)   # 前角到激光 0.153 m
CORNER_A = math.atan2(ENV_Y, ENV_XF)   # 前角方位 49.2°
# “正前”只取 ±34°：若像 v4 那样取 ±57°，在窄道里 45°~57° 的光线打到侧墙，
# cf 会被侧向距离拖到几厘米，车会在本来走得通的直道上误触发前向停车。
FRONT_HALF = 0.60                      # rad，约 34°
SIDE_MAX = 2.2                         # rad，约 126°；再往后是 120°~230° 屏蔽区


def pick_route():
    p = os.environ.get('ROUTE', '')
    if p:
        if not os.path.exists(p):
            print('ROUTE_MISSING %s' % p, flush=True)
            sys.exit(1)
        return p
    for cand in (os.path.join(HOME, 'drive4', 'route_path3.json'),
                 os.path.join(HOME, 'drive4', 'route_path2.json'),
                 '/tmp/route_path2.json'):
        if os.path.exists(cand):
            return cand
    print('ROUTE_MISSING', flush=True)
    sys.exit(1)


ROUTE = pick_route()
LEGS = json.load(open(ROUTE))


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


class C(Node):
    def __init__(self):
        super().__init__('drive_v5')
        self.scan = None
        self.scan_mono = 0.0
        self.geom = None
        self.odom = None
        self.odom_mono = 0.0
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.tf, self)

    def cb(self, m):
        self.scan = m
        self.scan_mono = time.monotonic()

    def odom_cb(self, m):
        self.odom = m
        self.odom_mono = time.monotonic()

    def odom_yaw(self):
        if self.odom is None or time.monotonic() - self.odom_mono > 0.5:
            return None
        q = self.odom.pose.pose.orientation
        return 2 * math.atan2(q.z, q.w)


def pose(c):
    """返回 (x, y, yaw, age) 或 None。age 为最新 TF 的时间戳年龄（秒）。"""
    try:
        t = c.tf.lookup_transform('map', 'base_footprint', Time())
        q = t.transform.rotation
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = 2 * math.atan2(q.z, q.w)
        st = t.header.stamp
        age = c.get_clock().now().nanoseconds / 1e9 - (st.sec + st.nanosec / 1e9)
        if not math.isfinite(age):
            age = 0.0
        return x, y, math.atan2(math.sin(yaw), math.cos(yaw)), max(0.0, age)
    except Exception:
        return None


def scan_clear(c):
    m = c.scan
    n = len(m.ranges)
    key = (n, float(m.angle_min), float(m.angle_increment))
    if c.geom is None or c.geom[0] != key:
        ang = m.angle_min + np.arange(n) * m.angle_increment
        ang = np.arctan2(np.sin(ang), np.cos(ang))
        ca = np.cos(ang)
        sa = np.sin(ang)
        tx = np.where(ca > 1e-6, ENV_XF / np.where(ca > 1e-6, ca, 1.0),
             np.where(ca < -1e-6, ENV_XR / np.where(ca < -1e-6, -ca, 1.0), 1e9))
        ty = np.where(sa > 1e-6, ENV_Y / np.where(sa > 1e-6, sa, 1.0),
             np.where(sa < -1e-6, ENV_Y / np.where(sa < -1e-6, -sa, 1.0), 1e9))
        env = np.minimum(tx, ty)
        masks = (np.abs(ang) <= FRONT_HALF,
                 (ang > FRONT_HALF) & (ang <= SIDE_MAX),
                 (ang >= -SIDE_MAX) & (ang < -FRONT_HALF),
                 np.abs(ang) > SIDE_MAX)
        c.geom = (key, ang, env, masks)
    _, ang, env, masks = c.geom
    r = np.asarray(m.ranges, dtype=np.float64)
    if r.shape[0] != ang.shape[0]:
        # 长度在缓存后变了：清缓存重算一次
        c.geom = None
        return scan_clear(c)
    rmin = float(m.range_min) if float(m.range_min) > 0 else 0.02
    rmin = max(0.02, rmin - 0.005)
    valid = np.isfinite(r) & (r >= rmin) & (r < 6.0)
    clr = r - env
    mf, ml, mr, mrear = masks

    def mn(mask):
        sel = valid & mask
        return float(clr[sel].min()) if sel.any() else 9.0

    def mnraw(mask):
        sel = valid & mask
        return float(r[sel].min()) if sel.any() else 9.0

    return {
        'cf': mn(mf), 'cl': mn(ml), 'cr': mn(mr),
        'rawf': mnraw(mf), 'rawl': mnraw(ml), 'rawr': mnraw(mr),
        'rearmin': mnraw(mrear),
        'rearn': int(np.count_nonzero(valid & mrear)),
        'nl': int(np.count_nonzero(valid & ml)),
        'nr': int(np.count_nonzero(valid & mr)),
        'nvalid': int(np.count_nonzero(valid)),
        'ang': ang, 'r': r, 'valid': valid,
    }


def sweep_ok(inf, direction, step=0.06):
    """转 direction 方向前，检查前左/前右角扫过的那一小段角度的真实余量。"""
    ang, r, valid = inf['ang'], inf['r'], inf['valid']
    for phi in (CORNER_A, -CORNER_A):
        lo, hi = sorted((phi, phi + direction * step))
        lo -= 0.02                     # 把角点本身那根光线也包进来，避免采样错位漏检
        hi += 0.02
        sel = valid & (ang >= lo) & (ang <= hi)
        if sel.any() and float(r[sel].min() - CORNER_R) < 0.02:
            return False
    return True


def stop(c, n=15):
    t = Twist()
    for _ in range(n):
        try:
            c.pub.publish(t)
            rclpy.spin_once(c, timeout_sec=0.02)
        except Exception:
            break


def log_state(tag, t0, leg, k, n, p, rem, e, psi, cf, cl, cr, inf, v, w, mc):
    print('  t=%5.1f %s leg%d %3d/%3d pose(%.2f,%.2f,%.0f) rem=%.2f e=%.2f psi=%.0f '
          'cf%.3f cl%.3f cr%.3f mclr%.2f | rawf%.2f rawl%.2f rawr%.2f rearn%d n%d | v=%.2f w=%.2f' %
          (time.time() - t0, tag, leg + 1, k + 1, n, p[0], p[1], math.degrees(p[2]), rem,
           e, math.degrees(psi), cf, cl, cr, mc, inf['rawf'], inf['rawl'], inf['rawr'],
           inf['rearn'], inf['nvalid'], v, w), flush=True)


def align_to(c, target, x, y, yaw, inf, t0):
    """用里程计相对角度转到 target；AMCL 只用来给初始误差。"""
    err0 = norm(target - yaw)
    if abs(err0) < 0.10:
        print('ALIGN ok err=%.0f' % math.degrees(err0), flush=True)
        return
    if min(inf['cf'], inf['cl'], inf['cr']) < ALIGN_CLEAR:
        print('ALIGN_SKIP clear cf%.2f cl%.2f cr%.2f err=%.0f' %
              (inf['cf'], inf['cl'], inf['cr'], math.degrees(err0)), flush=True)
        return
    direction = 1.0 if err0 > 0 else -1.0
    base = c.odom_yaw()
    dur = min(6.0, max(0.6, abs(err0) / 0.15))
    print('ALIGN start err=%.0f dur=%.1fs odom=%s' %
          (math.degrees(err0), dur, base is not None), flush=True)
    t_start = time.time()
    last = t_start
    while time.time() - t_start < dur and rclpy.ok():
        if time.monotonic() - c.scan_mono > 0.5:
            print('ALIGN scan stale, stop', flush=True)
            break
        tw = Twist()
        tw.angular.z = 0.15 * direction
        c.pub.publish(tw)
        rclpy.spin_once(c, timeout_sec=0.05)
        inf2 = scan_clear(c)
        if min(inf2['cf'], inf2['cl'], inf2['cr']) < max(0.04, ALIGN_CLEAR * 0.5):
            print('ALIGN blocked cf%.2f cl%.2f cr%.2f' %
                  (inf2['cf'], inf2['cl'], inf2['cr']), flush=True)
            break
        o = c.odom_yaw()
        if o is not None and base is not None:
            swept = abs(norm(o - base))
            if swept >= abs(err0) - 0.05:
                break
        if time.time() - last > 0.5:
            last = time.time()
            o2 = c.odom_yaw()
            print('  ALIGN %.2fs odom_delta=%.0f' %
                  (time.time() - t_start,
                   math.degrees(norm(o2 - base)) if (o2 is not None and base is not None) else -999),
                  flush=True)
    stop(c, 10)
    o = c.odom_yaw()
    print('ALIGN done odom_delta=%s' %
          ('%.0f' % math.degrees(norm(o - base)) if (o is not None and base is not None) else 'n/a'),
          flush=True)


def run(c):
    t0 = time.time()
    print('V5 route=%s legs=%d DRY=%d' % (ROUTE, len(LEGS), DRY), flush=True)
    print('V5 body front %.3f rear %.3f side %.3f corner %.3f | VMAX %.2f STOP_F %.2f STOP_S %.2f'
          % (ENV_XF, ENV_XR, ENV_Y, CORNER_R, VMAX, STOP_F, STOP_S), flush=True)
    for li, leg in enumerate(LEGS):
        cl = leg.get('clr') or []
        pts = leg['pts']
        if len(cl) != len(pts):
            continue
        groups = []
        for k, v in enumerate(cl):
            if v < 0.18:
                if groups and k == groups[-1][1] + 1:
                    groups[-1][1] = k
                else:
                    groups.append([k, k])
        for k0, k1 in groups:
            print('V5 tight leg%d k%d-%d (%.2f,%.2f)-(%.2f,%.2f) min_wall %.2f'
                  % (li + 1, k0, k1, pts[k0][0], pts[k0][1], pts[k1][0], pts[k1][1],
                     min(cl[k0:k1 + 1])), flush=True)

    # 启动等待
    t_wait = time.time()
    while c.scan is None and time.time() - t_wait < 8 and rclpy.ok():
        rclpy.spin_once(c, timeout_sec=0.1)
    if c.scan is None:
        print('NO_SCAN', flush=True)
        return
    m = c.scan
    print('V5 scan: n=%d angle_min=%.1fdeg inc=%.3fdeg range_min=%.2f range_max=%.1f'
          % (len(m.ranges), math.degrees(m.angle_min), math.degrees(m.angle_increment),
             m.range_min, m.range_max), flush=True)

    if DRY:
        print('DRY: 只打印诊断，不发速度', flush=True)
        while time.time() - t_wait < 10 and rclpy.ok():
            rclpy.spin_once(c, timeout_sec=0.1)
            if c.scan is None or time.monotonic() - c.scan_mono > 0.5:
                continue
            inf = scan_clear(c)
            p = pose(c)
            if p is None:
                print('  DRY pose=None', flush=True)
                continue
            print('  DRY pose(%.2f,%.2f,%.0f) cf%.3f cl%.3f cr%.3f | rawf%.2f rawl%.2f rawr%.2f '
                  'rearn%d nvalid%d | scan_age%.2fs tf_age%.2fs' %
                  (p[0], p[1], math.degrees(p[2]), inf['cf'], inf['cl'], inf['cr'],
                   inf['rawf'], inf['rawl'], inf['rawr'], inf['rearn'], inf['nvalid'],
                   time.monotonic() - c.scan_mono, p[3]), flush=True)
            time.sleep(1.0)
        return

    p = None
    t_wait = time.time()
    while p is None and time.time() - t_wait < 20 and rclpy.ok():
        rclpy.spin_once(c, timeout_sec=0.1)
        p = pose(c)
    if p is None:
        print('NO_POSE', flush=True)
        return
    print('V5 start pose %.2f %.2f %.0f' % (p[0], p[1], math.degrees(p[2])), flush=True)
    e_lp = 0.0
    last_pose = (p[0], p[1], p[2])
    jumps = []

    for leg in range(START_LEG, len(LEGS)):
        P = np.array(LEGS[leg]['pts'], dtype=np.float64)
        goal = LEGS[leg]['goal']
        CLR = np.asarray(LEGS[leg].get('clr', []), dtype=np.float64)
        if CLR.shape[0] != len(P):
            CLR = np.full(len(P), 9.0)
        print('=== leg %d/%d goal (%.2f,%.2f,%.0fdeg) pts %d'
              % (leg + 1, len(LEGS), goal[0], goal[1], math.degrees(goal[2]), len(P)), flush=True)
        leg_t0 = time.time()
        idx_prev = 0
        stuck = 0
        guard_start = None
        guard_attempts = 0
        last_tick = 0.0
        scan_seen = -1.0
        prev_rawl = 9.0
        prev_rawr = 9.0
        dz_until = 0.0
        pose_lost_since = None

        while rclpy.ok():
            rclpy.spin_once(c, timeout_sec=0.05)
            now = time.monotonic()

            # ---- 传感器时效 ----
            if c.scan is None or now - c.scan_mono > 0.5:
                stop(c, 3)
                if now - c.scan_mono > 3.0:
                    print('SCAN_LOST abort', flush=True)
                    return
                continue
            inf = scan_clear(c)
            p = pose(c)
            if p is None or p[3] > 0.6:
                stop(c, 3)
                if pose_lost_since is None:
                    pose_lost_since = time.time()
                if time.time() - pose_lost_since > 3.0:
                    print('POSE_LOST abort age=%s' %
                          ('none' if p is None else '%.2fs' % p[3]), flush=True)
                    return
                continue
            pose_lost_since = None
            x, y, yaw = p[0], p[1], p[2]

            # ---- 位姿跳变 ----
            dd = math.hypot(x - last_pose[0], y - last_pose[1])
            dyaw = abs(norm(yaw - last_pose[2]))
            if dd > 0.30 or dyaw > 0.50:
                jumps = [t for t in jumps if time.time() - t < 2.0] + [time.time()]
                print('POSE_JUMP d=%.2f dyaw=%.0f (%d/3)' %
                      (dd, math.degrees(dyaw), len(jumps)), flush=True)
                e_lp = 0.0
                last_pose = (x, y, yaw)
                if len(jumps) >= 3:
                    print('POSE_UNSTABLE abort', flush=True)
                    return
                stop(c, 3)
                continue
            last_pose = (x, y, yaw)

            # ---- 近距盲区观察 ----
            if c.scan_mono != scan_seen:
                scan_seen = c.scan_mono
                if prev_rawl < 0.26 and inf['nl'] == 0:
                    print('DEADZONE_L prev_rawl=%.2f (墙进入 %.2f 盲区)' % (prev_rawl, m.range_min), flush=True)
                    dz_until = time.time() + 1.0
                if prev_rawr < 0.26 and inf['nr'] == 0:
                    print('DEADZONE_R prev_rawr=%.2f (墙进入 %.2f 盲区)' % (prev_rawr, m.range_min), flush=True)
                    dz_until = time.time() + 1.0
                prev_rawl = inf['rawl']
                prev_rawr = inf['rawr']

            # ---- 路径跟踪 ----
            lo = max(0, idx_prev - 20)
            hi = min(len(P), idx_prev + 45)
            d = np.hypot(P[lo:hi, 0] - x, P[lo:hi, 1] - y)
            k = int(np.argmin(d)) + lo
            dn = float(np.min(d))
            if dn > 0.5:
                d2 = np.hypot(P[:, 0] - x, P[:, 1] - y)
                k = int(np.argmin(d2))
                dn = float(np.min(d2))
            idx_prev = k
            mc = float(CLR[k])          # 路径点到最近真实障碍的图上距离（限速用）
            k2 = min(k + 4, len(P) - 1)
            k1 = max(0, k - 2)
            tx = P[k2, 0] - P[k1, 0]
            ty = P[k2, 1] - P[k1, 1]
            tn = math.hypot(tx, ty) + 1e-9
            tx /= tn
            ty /= tn
            path_h = math.atan2(ty, tx)
            ex = x - P[k, 0]
            ey = y - P[k, 1]
            e = -(ex * ty - ey * tx)          # >0：车在路径左侧
            e_lp = 0.6 * e_lp + 0.4 * max(-0.3, min(0.3, e))
            psi = norm(path_h - yaw)
            rem = float(np.hypot(P[-1, 0] - x, P[-1, 1] - y))

            j = min(k + 1, len(P) - 1)
            acc = 0.0
            while j < len(P) - 1 and acc < 0.30:
                acc += math.hypot(P[j + 1, 0] - P[j, 0], P[j + 1, 1] - P[j, 1])
                j += 1
            j2 = min(j + 3, len(P) - 1)
            bend = abs(norm(math.atan2(P[j2, 1] - P[j, 1], P[j2, 0] - P[j, 0]) - path_h)) if j2 > j else 0.0

            cf, cl, cr = inf['cf'], inf['cl'], inf['cr']
            tw = Twist()

            # ---- 到点 ----
            if rem < 0.14 and k >= len(P) - 4:
                print('leg %d arrive pose(%.2f,%.2f,%.0f) clearance cf%.2f cl%.2f cr%.2f'
                      % (leg + 1, x, y, math.degrees(yaw), cf, cl, cr), flush=True)
                if goal[2] is None:
                    print('ALIGN_SKIP no goal yaw', flush=True)
                else:
                    align_to(c, goal[2], x, y, yaw, inf, t0)
                stop(c, 10)
                print('leg %d ARRIVED (%.2f,%.2f,%.0f)' % (leg + 1, x, y, math.degrees(yaw)), flush=True)
                e_lp = 0.0
                break

            # ---- GUARD ----
            if cf < STOP_F or min(cl, cr) < STOP_S:
                tw.linear.x = 0.0
                tw.angular.z = 0.0
                c.pub.publish(tw)
                stuck += 1
                if guard_start is None:
                    guard_start = time.time()
                if stuck % 10 == 1:
                    print('GUARD cf%.3f cl%.3f cr%.3f rawl%.2f rawr%.2f'
                          % (cf, cl, cr, inf['rawl'], inf['rawr']), flush=True)
                if time.time() - guard_start < 0.6:
                    continue
                away = -1.0 if cl < cr else 1.0
                sides_known = inf['nl'] > 0 and inf['nr'] > 0
                if sides_known and min(cl, cr) > ROT_SIDE_MIN and sweep_ok(inf, away) \
                        and guard_attempts < 3:
                    guard_attempts += 1
                    print('GUARD rotate dir=%+.0f (attempt %d)' % (away, guard_attempts), flush=True)
                    for _ in range(16):
                        tw2 = Twist()
                        tw2.angular.z = 0.15 * away
                        c.pub.publish(tw2)
                        rclpy.spin_once(c, timeout_sec=0.05)
                        if time.monotonic() - c.scan_mono > 0.5:
                            break
                        inf2 = scan_clear(c)
                        if inf2['cf'] < STOP_F or min(inf2['cl'], inf2['cr']) < STOP_S:
                            break
                        if not sweep_ok(inf2, away):
                            break
                    stop(c, 3)
                    guard_start = None
                    continue
                if inf['rearn'] >= 12 and inf['rearmin'] > 0.25:
                    print('ESCAPE reverse rearmin=%.2f' % inf['rearmin'], flush=True)
                    for _ in range(12):
                        tw2 = Twist()
                        tw2.linear.x = -0.03
                        c.pub.publish(tw2)
                        rclpy.spin_once(c, timeout_sec=0.05)
                    stop(c, 3)
                    guard_start = None
                    continue
                print('GUARD_ABORT cf%.3f cl%.3f cr%.3f sides_known=%s attempts=%d'
                      % (cf, cl, cr, sides_known, guard_attempts), flush=True)
                stop(c, 10)
                return
            else:
                guard_start = None
                guard_attempts = 0
                stuck = max(0, stuck - 3)

                sc = 1.0
                if cf < 0.10:
                    sc = 0.45
                elif cf < 0.20:
                    sc = 0.65
                elif cf < 0.35:
                    sc = 0.85
                if min(cl, cr) < 0.06:
                    sc *= 0.6
                elif min(cl, cr) < 0.10:
                    sc *= 0.8
                if abs(psi) > 0.5:
                    sc *= 0.7
                if bend > 0.7:
                    sc *= 0.65
                elif bend > 0.4:
                    sc *= 0.85

                # 图上离墙很近的位置（比如 leg1 的窄口）再降速，给雷达/避障留反应时间
                if mc < 0.16:
                    sc *= 0.55
                elif mc < 0.19:
                    sc *= 0.80

                wz = -1.6 * e_lp + 1.1 * psi        # 横向误差 + 航向误差（Stanley 符号）
                if min(cl, cr) < 0.30:
                    wz += max(-0.20, min(0.20, 1.5 * (cl - cr)))   # 窄处按左右余量居中
                wz = max(-0.40, min(0.40, wz))
                v = max(VMIN, VMAX * sc * (1.0 - 0.35 * min(1.0, abs(psi) / 1.2)))

                if DEADZONE_GUARD and time.time() < dz_until:
                    v = min(v, 0.02)
                    if inf['nl'] == 0 and wz > 0:
                        wz = 0.0
                    if inf['nr'] == 0 and wz < 0:
                        wz = 0.0

                tw.angular.z = wz
                tw.linear.x = v
                c.pub.publish(tw)

                if time.time() - last_tick > 1.0:
                    last_tick = time.time()
                    log_state('drive', t0, leg, k, len(P), p, rem, e, psi,
                              cf, cl, cr, inf, v, wz, mc)

            if time.time() - leg_t0 > 420:
                print('LEG_TIMEOUT %d' % (leg + 1), flush=True)
                stop(c, 10)
                return

    print('ROUTE_DONE', flush=True)


def main():
    rclpy.init()
    c = None
    try:
        c = C()
        run(c)
    except KeyboardInterrupt:
        print('SIGINT', flush=True)
    except Exception:
        traceback.print_exc()
    finally:
        if c is not None:
            stop(c, 20)
            try:
                c.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
