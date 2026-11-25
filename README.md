# 天津仁爱学院第二届"讯飞杯"AI创新大赛（2026）

## 智能驾驶赛道 · 城市末端配送智能车

> 本仓库为参赛项目主仓库，对应《方案一：原方案（全员开发）》。

---

## 一、赛事信息

| 项目 | 内容 |
|---|---|
| 赛道 | 智能驾驶（智能车） |
| 初赛提交 | **2026年11月30日（周一）** |
| 提交邮箱 | jxliu33@iflytek.com |
| 提交材料 | 功能演示视频 + 参赛项目PPT或技术方案 |
| 决赛 | 2026年12月下旬，大活中心，现场演示+答辩 |
| 开发环境 | **Ubuntu 22.04 LTS + ROS2 Humble** |
| 评分方式 | 14项功能，**按完成项数评定成绩** |

---

## 二、仓库结构

```
smartcar_ws/
├── src/
│   ├── smartcar_msgs/        # 公共消息定义（保留；本轮 ①②③⑧⑩ 不用）
│   ├── smartcar_bringup/     # 一键启动 launch + 全局参数 + 航点/别名
│   ├── hardware/
│   │   └── camera_driver/    # UVC 相机 / 视频回放（实车 Astra 用厂家节点）
│   ├── perception/
│   │   ├── vision_detect/    # 巡线 + 二维码多字段
│   │   ├── lighting_adaptor/ # 光照分级 + 曝光自适应（⑥⑦）
│   │   └── ocr_reader/       # tesseract OCR（⑤）
│   ├── control/
│   │   ├── line_follower/    # 巡线 PID（⑫）
│   │   ├── obstacle_avoider/ # 雷达 + 深度避障（⑪）
│   │   ├── cmd_vel_arbiter/  # 唯一 /cmd_vel 出口
│   │   └── parking/          # 二维码引导泊车（⑨）
│   ├── mission/
│   │   ├── waypoint_nav/     # nav2 航点导航 + 航点录制（⑬）
│   │   └── mission_manager/  # 任务状态机
│   ├── interaction/
│   │   └── hud/              # /hud/image 叠加画面
│   └── simulation/
│       └── virtual_sim/      # 2D 虚拟仿真（⑭）
├── tests/                    # 102 个 pytest（纯算法，无 ROS）
├── tools/
│   ├── local_demo.py         # Windows 上无 ROS 闭环演示
│   ├── make_qr_samples.py    # 生成可打印二维码
│   └── gen_qr_asset.py       # 重新生成仿真二维码位图
├── docs/
│   ├── 使用说明.md            # ⭐ 上车/仿真/本地演示全流程
│   ├── 功能对照表.md          # 功能编号 ↔ 代码文件
│   ├── 接口文档.md            # Topic 契约（v2.0）
│   ├── 功能清单.md            # 完成度追踪
│   ├── 分支与协作规范.md
│   └── 硬件参数.md
└── README.md
```

> 本轮交付 9 项功能：④⑤⑥⑦⑨⑪⑫⑬⑭（①②③⑧⑩ 明确不做）。
> 详细用法见 **`docs/使用说明.md`**，功能与代码对应见 **`docs/功能对照表.md`**。

---

## 三、快速开始

### 3.1 环境

实车是 **WHEELTEC RDK X5**（aarch64，Ubuntu 22.04 + ROS2 Humble），
出厂工程在 `~/wheeltec_ros2`，本工程独立放在 `~/smartcar_ws`，互不改动。

### 3.2 编译

```bash
cd ~/smartcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> ⚠️ 必须放在 Linux 家目录（`~/smartcar_ws`），**不要放在 `/mnt/c/...` 或共享文件夹**。

### 3.3 本地验证（不接车）

```bash
python -m pytest tests -q                 # 102 个用例，纯算法
python tools/local_demo.py                # 闭环演示 → out/preview.html
ros2 launch smartcar_bringup sim.launch.py  # ROS 虚拟仿真
```

### 3.4 实车启动

先启动厂家底盘/相机/雷达（launch 名以 `~/wheeltec_ros2` 为准），确认
`/camera/color/image_raw`、`/scan`、`/odom` 在线，然后：

```bash
ros2 launch smartcar_bringup bringup.launch.py start_web_video:=true
```

详细步骤、参数标定、航点录制、排障见 **`docs/使用说明.md`**。

---

## 四、无车阶段联调（视频回放）

用一段巡线视频验证「相机 → 视觉 → 巡线 → 仲裁」整条链路（不接底盘，车不会动）：

```bash
ros2 launch smartcar_bringup demo.launch.py video_path:=$HOME/Videos/line.mp4

# 另开终端观察
ros2 topic hz  /camera/color/image_raw      # ~30 Hz
ros2 topic echo /perception/line            # [-1,1] 偏移，NaN=丢失
ros2 topic echo /perception/qr_payload      # 画面里出现二维码时
ros2 topic echo /control/arbiter_state      # mode/速度
ros2 run rqt_image_view rqt_image_view /perception/debug_image
```

**用真实 UVC 摄像头**（实车 Astra 用厂家节点，不需要本节）：

```bash
ros2 run camera_driver camera_node            # source=device
ros2 run vision_detect vision_node
```

**调参纪律**：只改 `src/smartcar_bringup/config/params.yaml`，不要改代码默认值。
改完重新 `colcon build` 或重发 launch。

---

## 五、团队分工

| 代号 | 角色 | 负责模块 | 对应功能编号 |
|---|---|---|---|
| A | 队长 / 系统集成 | 仓库与接口、任务状态机、全场联调、场地设计、视频脚本、答辩主讲 | ②③ 及全部集成 |
| B | 视觉算法 | 巡线、红绿灯、二维码、OCR、货物识别、光照与曝光 | ④⑤⑥⑦⑩⑫ |
| C | 运动控制 | 底盘驱动、避障、泊车、巡线PID、路径规划 | ⑨⑪⑫⑬ |
| D | 交互与仿真 | 语音识别、LLM语义解析、多指令调度、Gazebo场景 | ①②③⑭ |

指导教师（≤3位）：报名盖章、方向把关、决赛前模拟答辩。

---

## 六、分支与提交规范

### 分支

| 分支 | 用途 |
|---|---|
| `main` | 随时可编译、可运行，只接受合并 |
| `feat/a-*` | A 的开发分支，如 `feat/a-mission` |
| `feat/b-*` | B 的开发分支，如 `feat/b-vision` |
| `feat/c-*` | C 的开发分支 |
| `feat/d-*` | D 的开发分支 |

### 提交信息

格式：`[功能编号] 简述`

```
[④] 二维码解析支持多字段
[⑩] 红绿灯识别增加面积阈值过滤
[docs] 更新接口文档 Topic 表
```

### 纪律

1. 每天收工前 push 一次，禁止只存在本地
2. 合并到 `main` 前必须自测通过
3. 接口字段变更提前一天发群说明，A 确认后执行
4. 不改他人节点内部逻辑，只提 issue

详细规范见 `docs/分支与协作规范.md`。

---

## 七、当前里程碑（11-30 初赛提交前）

- [x] 9 项功能代码完成，102 个单元测试通过
- [x] 本地闭环演示跑通（巡线 → 读码 → 泊车）
- [x] 虚拟仿真一键启动（`sim.launch.py`）
- [x] 使用说明 / 接口文档 / 功能对照表
- [ ] 代码部署到 RDK X5 并 `colcon build` 通过
- [ ] 实车巡线 + 避障 + 二维码 + 泊车逐项验证（见 `docs/功能清单.md` 勾选表）
- [ ] 录制功能演示视频，提交 PPT

---

## 八、功能完成度追踪

见 `docs/功能清单.md`（每周日晚更新）。

14项功能：巡线、避障、红绿灯、二维码、OCR、光照判断、曝光自适应、货物识别搬运、定点泊车投递、路径规划、语音交互、多指令连续执行、多级语义识别、虚拟仿真交互。

---

## 九、相关文档

- `docs/使用说明.md` — ⭐ 上车部署、启动顺序、任务操作、参数标定、排障
- `docs/功能对照表.md` — 功能编号 ↔ 包/文件/话题 ↔ 本地证据
- `docs/接口文档.md` — Topic 与消息契约（v2.0）
- `docs/功能清单.md` — 14 项完成度追踪
- `docs/分支与协作规范.md` — 日常协作细则
- 《方案一：原方案（全员开发）》《方案二：D方案》— 桌面文档（立项时）

---

**仓库可见性建议：Private（私有）**，比赛结束前不公开。
