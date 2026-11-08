# perception — 感知层

> **负责人：B**
> 总目标：从图像中稳定提取 6 项功能所需的全部信息。

## 待创建的包

### vision_detect（主感知节点）

| 输出 Topic | 类型 | 对应功能 |
|---|---|---|
| `/perception/line` | std_msgs/Float32 | ⑫ 巡线 |
| `/perception/traffic_light` | smartcar_msgs/TrafficLight | ⑩ 红绿灯 |
| `/perception/qr_payload` | std_msgs/String | ④ 二维码 |
| `/perception/objects` | smartcar_msgs/DetectedObjects | ⑧ 货物识别 |
| `/perception/ocr_text` | std_msgs/String | ⑤ OCR |

### lighting_adaptor（光照与曝光）

| 输出 | 说明 |
|---|---|
| `/camera/brightness` (std_msgs/Int8) | 0暗 / 1中 / 2亮 |
| 直接调用 `cv2.VideoCapture.set(CAP_PROP_EXPOSURE, ...)` | ⑦ 自适应曝光 |

> 注意：功能⑦要求**主动控制曝光**。若相机设为自动曝光，评委可能不认可。实现时务必切换为手动模式再调节。

## 建议实现顺序

1. 巡线（最容易，也是其他功能的调试基础）
2. 二维码（pyzbar，半小时能出结果）
3. 红绿灯（HSV 颜色分割，加面积过滤）
4. 光照判断 + 曝光（逻辑简单，分数好拿）
5. OCR（PaddleOCR 本地部署）
6. 货物识别（YOLOv8n，需要数据集）

## 硬性要求

- [ ] 所有阈值从 `params.yaml` 读取，禁止硬编码
- [ ] 每项功能有 50 次测试记录（`docs/data/`）
- [ ] 暗光与强光两种条件下均需验收
- [ ] 单帧处理耗时 ≤ 50 ms（否则影响控制实时性）
