# mission — 任务层（核心）

> **负责人：A**
> 这是全车的"大脑"，把语音任务、感知结果、运动控制串成完整业务流程。

## 待创建的包

`mission_manager`：任务状态机

## 输入

| Topic | 类型 | 用途 |
|---|---|---|
| `/nlp/task_plan` | smartcar_msgs/TaskPlan | 待执行的任务序列 |
| `/perception/traffic_light` | smartcar_msgs/TrafficLight | 路口决策 |
| `/perception/objects` | smartcar_msgs/DetectedObjects | 货物与投递点定位 |
| `/perception/qr_payload` | std_msgs/String | 包裹信息校验 |
| `/odom` | nav_msgs/Odometry | 进度与到达判定 |

## 输出

| Topic | 类型 | 用途 |
|---|---|---|
| `/mission/state` | std_msgs/String | 供 HUD 显示 |
| `/gripper/command` | std_msgs/Bool | 抓取/释放 |

## 状态机

```
IDLE ──> PLANNING ──> NAVIGATING ──> GRABBING ──> DELIVERING ──> DONE
             │             │             │              │
             └─────────────┴─────────────┴──────────────┴──> FAILED
                                                              │
                                                         重试(≤2次)
```

## 设计要求

1. **状态迁移表先写文档再写代码**，第一周完成
2. 每条指令有超时（默认 120 秒），超时进 FAILED 并语音播报
3. 失败自动重试 ≤ 2 次，仍失败则跳过该指令继续下一条（保证演示能跑完）
4. 所有状态变化必须能出现在 HUD 上

## 验收标准

- [ ] 一句"把A货送到3号门再回来"能完整跑通
- [ ] 中途人为遮挡 3 秒后能自动恢复
- [ ] 单条指令卡住时超时生效并继续执行后续指令
