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
│   ├── smartcar_msgs/        # 公共消息定义（全组依赖，已实现）
│   ├── smartcar_bringup/     # 一键启动、参数配置（已实现）
│   ├── hardware/             # 硬件驱动层
│   │   ├── camera_driver/    # 相机采集/视频回放（已实现）
│   │   ├── base_driver/      # 底盘驱动（选车后实现）
│   │   └── range_driver/     # 超声波/测距（选车后实现）
│   ├── perception/           # 感知层
│   │   ├── vision_detect/    # 巡线/红绿灯/二维码（已实现）
│   │   └── lighting_adaptor/ # 光照判断 + 曝光自适应
│   ├── control/              # 运动控制
│   │   ├── line_follower/    # 巡线 PID（已实现）
│   │   ├── obstacle_avoider/
│   │   └── parking/
│   ├── interaction/          # 交互
│   │   ├── voice_input/
│   │   └── nlp_planner/
│   ├── mission/              # 任务状态机（核心）
│   │   └── mission_manager/
│   └── simulation/           # Gazebo 场景（对应功能⑭）
├── docs/
│   ├── 接口文档.md            # Topic 契约，改动需全组评审
│   └── 分支与协作规范.md
└── README.md
```

> 已实现：`smartcar_msgs`、`smartcar_bringup`、`camera_driver`、`vision_detect`、`line_follower`。
> 其余目录为**待建设的包**，由对应负责人在第一周内按 README 创建。各目录下的 README 写明了包名与职责。

---

## 三、快速开始

### 3.1 环境

Ubuntu 22.04 + ROS2 Humble。安装方式见《方案一》附录B。

### 3.2 克隆与编译

```bash
git clone <本仓库地址> ~/smartcar_ws
cd ~/smartcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> ⚠️ 必须克隆到 Linux 家目录（`~/smartcar_ws`），**不要放在 `/mnt/c/...` 或共享文件夹**，否则编译极慢且权限异常。

### 3.3 验证环境

```bash
ros2 interface show smartcar_msgs/msg/TaskPlan
ros2 launch smartcar_bringup bringup.launch.py
```

第一条能打印消息结构、第二条能正常启动，说明环境与仓库都通了。

如果编译时报缺少 `cv_bridge` / `cv2` / `pyzbar`，执行：

```bash
sudo apt install -y ros-humble-cv-bridge python3-opencv python3-numpy \
                    python3-pyzbar libzbar0
```

---

## 四、无车阶段联调（视频回放）

车辆到位前，用一段道路视频验证「相机 → 视觉 → 控制」整条链路。

```bash
# 准备一段巡线视频，例如 ~/Videos/line.mp4
ros2 launch smartcar_bringup demo.launch.py video_path:=$HOME/Videos/line.mp4

# 另开终端观察
ros2 topic hz  /camera/image_raw              # 应有 ~30 Hz
ros2 topic echo /perception/line              # 应有 [-1,1] 的偏移
ros2 topic echo /perception/traffic_light     # 出现红绿灯时有有效帧
ros2 topic echo /cmd_vel                      # 有 PID 输出
ros2 run rqt_image_view rqt_image_view /perception/debug_image
```

**用真实摄像头**：把 `params.yaml` 里 `camera_driver.source` 改成 `device`，
`device` 指向设备节点（默认 `/dev/video0`），然后直接 `ros2 run`：

```bash
ros2 run camera_driver camera_node            # 只开相机
ros2 run vision_detect vision_node            # 只开视觉
```

**调参纪律**：只改 `src/smartcar_bringup/config/params.yaml`，不要改代码里的默认值。
红绿灯 HSV 阈值必须按场地实测灯光重新标定（见 `docs/功能清单.md` ⑩）。

> ⚠️ 本阶段 `/cmd_vel` 只发到话题，没有底盘节点执行，车不会动。上实车前先确认
> `base_driver` 已实现、且 `line_follower` 的 `enable` 参数可控。

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

## 七、第一周里程碑（11月15日前）

- [ ] 全员能克隆并 `colcon build` 通过
- [ ] `smartcar_msgs` 消息定义冻结
- [ ] 各成员完成自己包的创建与空节点
- [ ] 仿真环境跑通巡线 + 避障 + 语音下令
- [ ] 车型锁定，硬件参数写入 `docs/硬件参数.md`

---

## 八、功能完成度追踪

见 `docs/功能清单.md`（每周日晚更新）。

14项功能：巡线、避障、红绿灯、二维码、OCR、光照判断、曝光自适应、货物识别搬运、定点泊车投递、路径规划、语音交互、多指令连续执行、多级语义识别、虚拟仿真交互。

---

## 九、相关文档

- 《方案一：原方案（全员开发）》— 桌面文档
- 《方案二：D方案（成果与验证负责人）》— 桌面文档
- `docs/接口文档.md` — Topic 与消息契约
- `docs/分支与协作规范.md` — 日常协作细则

---

**仓库可见性建议：Private（私有）**，比赛结束前不公开。
