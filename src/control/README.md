# control — 运动控制层

> **负责人：C**

## 待创建的包

| 包名 | 输入 | 输出 | 对应功能 |
|---|---|---|---|
| `line_follower` | `/perception/line`、`/odom` | `/cmd_vel` | ⑫ 巡线 |
| `obstacle_avoider` | `/scan` | `/cmd_vel`（优先级高于巡线） | ⑪ 避障 |
| `parking` | `/perception/objects` | `/cmd_vel` | ⑨ 定点泊车 |

## 关键设计：控制权仲裁

三个控制节点都会发 `/cmd_vel`，必须约定优先级，否则会打架：

```
避障（最高） > 泊车 > 巡线（最低）
```

推荐做法：`obstacle_avoider` 发布时同时置位一个 `/control/override` 话题，
其他节点订阅后让出控制权；或统一由一个 `motion_arbiter` 节点汇总后发布 `/cmd_vel`。

**第一周就要定下来，不要等到联调才发现。**

## 建议实现顺序

1. 巡线 PID（先在 Gazebo 里调参）
2. 避障（阈值刹车最简单，先保命再优化）
3. 红绿灯停走（订阅 `/perception/traffic_light` 即可）
4. 定点泊车（最难，11月20日后再投入）

## 验收标准

- [ ] 沿黑线连续 30 米不脱线
- [ ] 障碍物 30 cm 内停车
- [ ] 红灯停、绿灯行，连续 5 次成功 ≥ 4 次
- [ ] 急停响应时间 ≤ 200 ms
