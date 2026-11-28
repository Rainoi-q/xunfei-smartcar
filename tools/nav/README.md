# 实车导航 v5（预规划路径 + 车身轮廓跟线）

2026-11-28 离线完成并做了合成扫描/地图自测，**尚未在本场地实车验证**。
上车前必须先把雷达 `min_range` 从 0.2 降到 0.10 左右（`lsn10p.yaml`），否则 20 cm 内的墙是盲区。

## 文件

| 文件 | 说明 |
|---|---|
| `drive_v5.py` | 控制器：订阅 `/scan`、`/odometry/filtered`，按路线 JSON 纯跟踪，直接发 `/cmd_vel` |
| `mkpath_v5.py` | 路径生成：离墙加权 Dijkstra，unknown 默认不可通行，输出每点离墙距离用于限速 |
| `route_path3.json` | 用当前 `WHEELTEC.pgm` + `waypoints.yaml` 生成的路线（三段 6.41 / 2.54 / 3.28 m） |

## 用法

```bash
# 生成路线（地图或航点改了才需要重跑）
python3 mkpath_v5.py --minc 0.15 --out ~/drive4/route_path3.json

# 干跑：只打印雷达/位姿诊断，不发速度
DRY=1 python3 ~/drive4/drive_v5.py

# 正式跑
python3 ~/drive4/drive_v5.py

# 从第 2 段开始
START_LEG=1 python3 ~/drive4/drive_v5.py
```

环境变量：`ROUTE`（默认 `~/drive4/route_path3.json`）、`START_LEG`、`DRY`、
`VMAX`(0.05)、`VMIN`(0.02)、`STOP_F`(0.06)、`STOP_S`(0.03)、
`ALIGN_CLEAR`(0.08)、`ROT_SIDE_MIN`(0.10)、`ODOM_TOPIC`(`/odometry/filtered`)。

## 关键行为

- 扫描断流 0.5 s 停车，3 s 退出；`map→base_footprint` 同样处理。
- 位姿跳变（2 s 内 3 次 >0.30 m 或 >29°）退出，避免 AMCL 跳变时打满方向。
- GUARD：先停车；两侧都有真实数据、余量 >0.10 m 且前角扫掠通过才小步慢转，最多 3 次，否则停住。
- 后向 120°~230° 仍被 `angle_disable` 屏蔽，所以**基本不会倒车**（代码仅在后方 ≥12 点且 >0.25 m 时倒）。
- 到点用里程计相对航向对齐，避免 AMCL 抖动导致原地来回转。
- 前向扇区 ±34°（v4 的 ±57° 会被窄道侧墙拖累 `cf` 误停）。

## 已知风险（到场重点看）

- **leg1 窄口**在地图 `(1.55, 1.19)` 附近，图上缝约 0.30 m；leg1 大部分点离墙只有 0.18~0.21 m。
  `mkpath_v5.py` 在 `MINC=0.15` 是上限（0.16 起无解）。
- 到场顺序：降 `min_range` → `DRY=1` 看 leg1 `(1.3~1.6, 1.1~1.8)` 的真实 `cf/cl/cr` → 再正式跑。
- 若窄口 `GUARD_ABORT`：真缝 ≥0.35 m 可试 `STOP_S=0.02 STOP_F=0.04`；真缝接近 0.30 m 不要硬过。
