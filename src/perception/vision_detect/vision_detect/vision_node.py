"""视觉感知节点。

订阅：
    /camera/image_raw          sensor_msgs/Image (bgr8)

发布：
    /perception/line           std_msgs/Float32
        巡线横向偏移，范围 [-1, 1]，正值表示线在画面右侧；
        丢失时发布 NaN，下游据此停车。
    /perception/traffic_light  smartcar_msgs/TrafficLight
        红绿灯颜色、置信度、像素面积。
    /perception/qr_payload     std_msgs/String
        二维码内容（JSON 字符串），内容变化时才发布。需要安装 pyzbar。
    /perception/debug_image    sensor_msgs/Image
        可视化调试图，仅 debug=true 时发布。

对应功能：④ 二维码、⑩ 红绿灯、⑫ 巡线（视觉部分）。
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from smartcar_msgs.msg import TrafficLight

try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except Exception:  # pyzbar 未安装时不影响其余功能
    pyzbar = None
    HAS_PYZBAR = False


COLOR_MAP = {
    'RED': TrafficLight.RED,
    'YELLOW': TrafficLight.YELLOW,
    'GREEN': TrafficLight.GREEN,
}


class VisionDetect(Node):
    """把一帧图像转换成巡线偏移与红绿灯状态。"""

    def __init__(self):
        super().__init__('vision_detect')

        # ---------------- 巡线 ----------------
        self.declare_parameter('threshold', 100)
        self.declare_parameter('roi_top', 0.60)
        self.declare_parameter('invert', False)
        self.declare_parameter('min_line_pixels', 300)

        # ---------------- 红绿灯 ----------------
        self.declare_parameter('red_h_min', 0)
        self.declare_parameter('red_h_max', 10)
        self.declare_parameter('red2_h_min', 170)
        self.declare_parameter('red2_h_max', 180)
        self.declare_parameter('green_h_min', 40)
        self.declare_parameter('green_h_max', 90)
        self.declare_parameter('yellow_h_min', 20)
        self.declare_parameter('yellow_h_max', 35)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('v_min', 80)
        self.declare_parameter('min_area_px', 300)
        self.declare_parameter('confidence_area', 3000.0)
        self.declare_parameter('smooth_frames', 5)

        # ---------------- 运行 ----------------
        self.declare_parameter('process_rate', 10.0)
        self.declare_parameter('qr_interval', 0.5)
        self.declare_parameter('debug', False)

        self.threshold = int(self.get_parameter('threshold').value)
        self.roi_top = float(self.get_parameter('roi_top').value)
        self.invert = bool(self.get_parameter('invert').value)
        self.min_line_pixels = int(self.get_parameter('min_line_pixels').value)

        self.min_area_px = float(self.get_parameter('min_area_px').value)
        self.confidence_area = float(self.get_parameter('confidence_area').value)
        self.smooth_frames = max(1, int(self.get_parameter('smooth_frames').value))

        self.process_rate = float(self.get_parameter('process_rate').value)
        self.qr_interval = float(self.get_parameter('qr_interval').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.bridge = CvBridge()
        self._kernel = np.ones((5, 5), np.uint8)
        self._latest_frame = None

        # 红绿灯稳定器：连续 N 帧同色才切换，避免闪烁
        self._cand_name = None
        self._cand_count = 0
        self._stable_name = None

        self._last_qr_text = None
        self._last_qr_time = 0.0

        self.pub_line = self.create_publisher(Float32, '/perception/line', 10)
        self.pub_light = self.create_publisher(
            TrafficLight, '/perception/traffic_light', 10)
        self.pub_qr = self.create_publisher(String, '/perception/qr_payload', 10)
        self.pub_debug = self.create_publisher(Image, '/perception/debug_image', 10)

        self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)

        period = 1.0 / max(self.process_rate, 1.0)
        self.create_timer(period, self.process)

        if not HAS_PYZBAR:
            self.get_logger().warning(
                '未安装 pyzbar，二维码功能不可用。安装命令：'
                'sudo apt install -y python3-pyzbar libzbar0')

        self.get_logger().info(
            '视觉感知已启动：处理频率 %.1f Hz，debug=%s' % (
                self.process_rate, self.debug))

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def on_image(self, msg):
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error('图像转换失败：%s' % exc)

    def process(self):
        frame = self._latest_frame
        if frame is None:
            return

        try:
            offset, binary, center_x = self._detect_line(frame)
            self.pub_line.publish(Float32(data=float(offset)))

            light = self._detect_traffic_light(frame)
            self.publish_traffic_light(light)
            self._maybe_publish_qr(frame)

            if self.debug:
                self.publish_debug(frame, binary, center_x, light)
        except Exception as exc:
            self.get_logger().error('处理帧时出错：%s' % exc, throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # 巡线（功能⑫视觉部分）
    # ------------------------------------------------------------------
    def _detect_line(self, frame):
        """返回 (偏移量, 二值图, 中心x)；丢失时偏移量为 NaN。"""
        height, width = frame.shape[:2]
        top = min(int(height * self.roi_top), max(0, height - 10))
        roi = frame[top:height, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        flag = cv2.THRESH_BINARY if self.invert else cv2.THRESH_BINARY_INV
        _, binary = cv2.threshold(gray, self.threshold, 255, flag)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self._kernel)

        rows, cols = np.nonzero(binary)
        if cols.size < self.min_line_pixels:
            return math.nan, binary, None

        center_x = float(cols.mean())
        offset = (center_x - width / 2.0) / (width / 2.0)
        offset = max(-1.0, min(1.0, offset))
        return offset, binary, int(center_x)

    # ------------------------------------------------------------------
    # 红绿灯（功能⑩）
    # ------------------------------------------------------------------
    def _build_masks(self, hsv):
        s_min = int(self.get_parameter('s_min').value)
        v_min = int(self.get_parameter('v_min').value)

        red = cv2.inRange(
            hsv,
            np.array([self.get_parameter('red_h_min').value, s_min, v_min], dtype=np.uint8),
            np.array([self.get_parameter('red_h_max').value, 255, 255], dtype=np.uint8))
        red |= cv2.inRange(
            hsv,
            np.array([self.get_parameter('red2_h_min').value, s_min, v_min], dtype=np.uint8),
            np.array([self.get_parameter('red2_h_max').value, 255, 255], dtype=np.uint8))
        green = cv2.inRange(
            hsv,
            np.array([self.get_parameter('green_h_min').value, s_min, v_min], dtype=np.uint8),
            np.array([self.get_parameter('green_h_max').value, 255, 255], dtype=np.uint8))
        yellow = cv2.inRange(
            hsv,
            np.array([self.get_parameter('yellow_h_min').value, s_min, v_min], dtype=np.uint8),
            np.array([self.get_parameter('yellow_h_max').value, 255, 255], dtype=np.uint8))

        return (('RED', red), ('YELLOW', yellow), ('GREEN', green))

    def _detect_traffic_light(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        candidates = []

        for name, mask in self._build_masks(hsv):
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area_px:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if h <= 0 or float(w) / h > 4.0:
                    continue  # 过滤长条形背景色块
                candidates.append((name, area, (x, y, w, h)))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[1], reverse=True)
        return candidates[0]

    def publish_traffic_light(self, light):
        msg = TrafficLight()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        if light is None:
            self._cand_count = max(0, self._cand_count - 1)
            if self._cand_count == 0:
                self._cand_name = None
            msg.valid = False
            msg.color = COLOR_MAP.get(self._stable_name, TrafficLight.UNKNOWN)
            msg.confidence = 0.0
            msg.area_px = 0
            self.pub_light.publish(msg)
            return

        name, area, _bbox = light
        if name == self._cand_name:
            self._cand_count += 1
        else:
            self._cand_name = name
            self._cand_count = 1

        if self._cand_count >= self.smooth_frames:
            self._stable_name = name

        msg.valid = self._stable_name is not None
        msg.color = COLOR_MAP.get(self._stable_name, TrafficLight.UNKNOWN)
        msg.confidence = float(min(1.0, area / self.confidence_area))
        msg.area_px = int(area)
        self.pub_light.publish(msg)

    # ------------------------------------------------------------------
    # 二维码（功能④）
    # ------------------------------------------------------------------
    def _maybe_publish_qr(self, frame):
        if not HAS_PYZBAR or self.qr_interval <= 0.0:
            return

        now = time.monotonic()
        if now - self._last_qr_time < self.qr_interval:
            return
        self._last_qr_time = now

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = pyzbar.decode(gray)
        if not results:
            return

        raw = results[0].data.decode('utf-8', errors='ignore')
        text = self._normalize_payload(raw)
        if text == self._last_qr_text:
            return

        self._last_qr_text = text
        self.pub_qr.publish(String(data=text))
        self.get_logger().info('二维码解析成功：%s' % text)

    @staticmethod
    def _normalize_payload(raw):
        """把二维码内容统一成 JSON 字符串，便于下游解析。"""
        try:
            return json.dumps(json.loads(raw), ensure_ascii=False, sort_keys=True)
        except (ValueError, TypeError):
            return json.dumps({'raw': raw}, ensure_ascii=False)

    # ------------------------------------------------------------------
    # 调试可视化
    # ------------------------------------------------------------------
    def publish_debug(self, frame, binary, center_x, light):
        height, width = frame.shape[:2]
        canvas = frame.copy()

        top = int(height * self.roi_top)
        cv2.line(canvas, (0, top), (width, top), (0, 255, 255), 2)

        if center_x is not None:
            cv2.circle(canvas, (center_x, height - 10), 12, (0, 255, 0), -1)
            cv2.putText(
                canvas, 'offset %.2f' % ((center_x - width / 2.0) / (width / 2.0)),
                (10, top + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(canvas, 'LINE LOST', (10, top + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        if light is not None:
            name, area, (x, y, w, h) = light
            color = (0, 0, 255) if name == 'RED' else (
                (0, 255, 0) if name == 'GREEN' else (0, 255, 255))
            cv2.rectangle(canvas, (x, y), (x + w, y + h), color, 2)
            cv2.putText(canvas, '%s %d' % (name, area), (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        if binary is not None and binary.ndim == 2:
            thumb_w, thumb_h = width // 4, (height - top) // 4
            if thumb_w > 0 and thumb_h > 0:
                small = cv2.resize(binary, (thumb_w, thumb_h))
                small = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
                canvas[0:thumb_h, width - thumb_w:width] = small

        msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.pub_debug.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionDetect()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
