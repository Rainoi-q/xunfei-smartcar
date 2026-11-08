# hardware — 硬件驱动层

> **负责人：C（底盘/测距）、A（相机）**
> 选车后实现。**这是全仓唯一与具体车型绑定的目录**，上层算法一律不依赖这里。

## 待创建的包

| 包名 | 可执行文件 | 输入 | 输出 | 说明 |
|---|---|---|---|---|
| `camera_driver` | `camera_driver_node` | 设备 `/dev/video0` | `/camera/image_raw` (sensor_msgs/Image) | 支持手动曝光（功能⑦的关键） |
| `base_driver` | `base_driver_node` | `/cmd_vel` | 轮速控制 + `/odom` | 速度与里程计标定 |
| `range_driver` | `range_driver_node` | 雷达/超声波 | `/scan` (sensor_msgs/LaserScan) | 多个测距源合并成一帧 |

## 创建 Python 包模板

```bash
cd src/hardware
ros2 pkg create --build-type ament_python camera_driver \
  --dependencies rclpy sensor_msgs
```

## 验收标准

- [ ] `ros2 topic hz /camera/image_raw` 稳定 ≥ 25 Hz
- [ ] `v4l2-ctl --list-ctrls` 能看到 `exposure_absolute`，且能通过代码修改生效
- [ ] 手动推动车 1 米，`/odom` 累计误差 ≤ 5%
- [ ] 障碍物距 30 cm 时 `/scan` 数值正确
