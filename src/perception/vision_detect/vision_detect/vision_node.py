"""视觉感知节点：巡线偏移 + 二维码多信息解析。

订阅：
    /camera/color/image_raw   sensor_msgs/Image (bgr8)
        WHEELTEC 出厂 Astra 彩色话题；换相机时改 image_topic 参数。

发布：
    /perception/line          std_msgs/Float32
        横向偏移 [-1, 1]，正值表示线在画面右侧；丢失发 NaN。
    /perception/qr_payload    std_msgs/String
        二维码规范化后的 JSON；内容变化时才发布。
    /perception/qr_valid      std_msgs/Bool
        最近一次二维码是否包含全部必填字段。
    /perception/debug_image   sensor_msgs/Image
        调试图（ROI、二值图、跟踪点、拟合线、二维码框）。

对应功能：
    ④ 二维码多信息解析、⑫ 巡线（视觉部分）。
    ⑩ 红绿灯按本次参赛范围不实现，相关参数与话题均不再占用 CPU。

参数全部来自 smartcar_bringup/config/params.yaml 的 vision_detect 段。
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from vision_detect.core import LineTracker, parse_qr_payload

try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except Exception:  # pyzbar 未安装时只关闭二维码
    pyzbar = None
    HAS_PYZBAR = False


class VisionDetect(Node):
    """把一帧图像转换成巡线偏移与结构化二维码信息。"""

    def __init__(self):
        super().__init__('vision_detect')

        # ---------------- 输入/输出 ----------------
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('line_enable', True)
        self.declare_parameter('qr_enable', True)
        self.declare_parameter('process_rate', 10.0)
        self.declare_parameter('debug', True)

        # ---------------- 巡线 ----------------
        self.declare_parameter('threshold', 100)
        self.declare_parameter('invert', False)       # False=白底黑线
        self.declare_parameter('roi_top', 0.60)
        self.declare_parameter('use_hsv', False)      # 彩色线时用 HSV
        self.declare_parameter('hsv_low', [0, 0, 0])
        self.declare_parameter('hsv_high', [180, 255, 80])
        self.declare_parameter('num_bands', 10)
        self.declare_parameter('band_height_px', 12)
        self.declare_parameter('min_line_pixels', 300)
        self.declare_parameter('max_slope', 1.5)

        # ---------------- 二维码 ----------------
        self.declare_parameter('qr_interval', 0.5)
        self.declare_parameter('qr_repeat_sec', 1.5)  # 消失多久后允许再次上报
        # zbar 对歪斜/旋转的二维码不敏感（仿真里码面与相机夹角约 45°），
        # 直接解码失败时按这些角度补转再试一遍
        self.declare_parameter('qr_retry_angles',
                               [10.0, -10.0, 20.0, -20.0,
                                30.0, -30.0, 45.0, -45.0])
        self.declare_parameter('qr_required_fields',
                               'order_id,consignee,destination')
        self.declare_parameter('qr_allow_plain_text', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.line_enable = bool(self.get_parameter('line_enable').value)
        self.qr_enable = bool(self.get_parameter('qr_enable').value)
        self.process_rate = max(float(self.get_parameter('process_rate').value), 1.0)
        self.debug = bool(self.get_parameter('debug').value)

        self.threshold = int(self.get_parameter('threshold').value)
        self.invert = bool(self.get_parameter('invert').value)
        self.roi_top = min(max(float(self.get_parameter('roi_top').value), 0.0), 0.95)
        self.use_hsv = bool(self.get_parameter('use_hsv').value)
        self.hsv_low = np.array(self.get_parameter('hsv_low').value, dtype=np.uint8)
        self.hsv_high = np.array(self.get_parameter('hsv_high').value, dtype=np.uint8)
        self.min_line_pixels = int(self.get_parameter('min_line_pixels').value)

        self.qr_interval = max(float(self.get_parameter('qr_interval').value), 0.0)
        self.qr_retry_angles = [
            float(a) for a in
            (self.get_parameter('qr_retry_angles').value or [])]
        self.qr_repeat_sec = max(float(self.get_parameter('qr_repeat_sec').value), 0.0)
        fields = str(self.get_parameter('qr_required_fields').value or '')
        self.qr_required_fields = tuple(
            item.strip() for item in fields.split(',') if item.strip())
        self.qr_allow_plain_text = bool(
            self.get_parameter('qr_allow_plain_text').value)

        self.bridge = CvBridge()
        self._kernel = np.ones((3, 3), np.uint8)
        self._latest_frame = None
        self._latest_stamp = None
        self._frame_w = 0
        self._frame_h = 0

        self.tracker = LineTracker(
            num_bands=int(self.get_parameter('num_bands').value),
            band_height_px=int(self.get_parameter('band_height_px').value),
            min_pixels=self.min_line_pixels,
            max_slope=float(self.get_parameter('max_slope').value))

        self._last_qr_text = None
        self._last_qr_seen = 0.0
        self._last_qr_time = 0.0
        self._last_debug_time = 0.0
        self._fps = 0.0

        self.pub_line = self.create_publisher(Float32, '/perception/line', 10)
        self.pub_qr = self.create_publisher(String, '/perception/qr_payload', 10)
        self.pub_qr_valid = self.create_publisher(Bool, '/perception/qr_valid', 10)
        self.pub_debug = self.create_publisher(Image, '/perception/debug_image', 10)

        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        self.create_timer(1.0 / self.process_rate, self.process)

        if self.qr_enable and not HAS_PYZBAR:
            self.get_logger().warning(
                '未安装 pyzbar，二维码功能关闭。安装：'
                'sudo apt install -y python3-pyzbar libzbar0')

        self.get_logger().info(
            '视觉感知启动：图像=%s 处理=%.1fHz 巡线=%s 二维码=%s' % (
                self.image_topic, self.process_rate,
                self.line_enable, self.qr_enable and HAS_PYZBAR))

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('图像转换失败：%s' % exc,
                                    throttle_duration_sec=5.0)
            return
        self._latest_frame = frame
        self._latest_stamp = msg.header.stamp
        self._frame_h, self._frame_w = frame.shape[:2]

    def process(self):
        frame = self._latest_frame
        if frame is None:
            return

        t0 = time.monotonic()
        result = None
        mask = None
        roi_top_px = 0
        qr_box = None

        try:
            if self.line_enable:
                mask, roi_top_px = self._build_mask(frame)
                result = self.tracker.extract(mask)
                self.pub_line.publish(Float32(data=float(result.offset)))

            if self.qr_enable and HAS_PYZBAR:
                qr_box = self._maybe_publish_qr(frame)

            if self.debug:
                self._publish_debug(frame, mask, roi_top_px, result, qr_box)
        except Exception as exc:
            self.get_logger().error('处理帧出错：%s' % exc,
                                    throttle_duration_sec=5.0)
        finally:
            dt = time.monotonic() - t0
            if dt > 0:
                self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)

    # ------------------------------------------------------------------
    # 巡线
    # ------------------------------------------------------------------
    def _build_mask(self, frame):
        height, width = frame.shape[:2]
        top = min(int(height * self.roi_top), max(0, height - 10))
        roi = frame[top:height, :]

        if self.use_hsv:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        else:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            flag = cv2.THRESH_BINARY if self.invert else cv2.THRESH_BINARY_INV
            _, mask = cv2.threshold(gray, self.threshold, 255, flag)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        return mask, top

    # ------------------------------------------------------------------
    # 二维码
    # ------------------------------------------------------------------
    def _maybe_publish_qr(self, frame):
        now = time.monotonic()
        if now - self._last_qr_time < self.qr_interval:
            return None
        self._last_qr_time = now

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            results = pyzbar.decode(gray)
        except Exception as exc:
            self.get_logger().warning('pyzbar 解码异常：%s' % exc,
                                      throttle_duration_sec=10.0)
            return None

        if not results and now - self._last_qr_seen > 2.0:
            # 只在还没稳定看到码时补转，避免每帧都付这个开销
            results = self._decode_rotated(gray)

        if not results:
            if (self._last_qr_text is not None
                    and now - self._last_qr_seen > self.qr_repeat_sec):
                self._last_qr_text = None
                self.pub_qr_valid.publish(Bool(data=False))
            return None

        # 同一帧可能有多张码，取面积最大的一张
        results = sorted(results, key=lambda r: r.rect.width * r.rect.height,
                         reverse=True)
        item = results[0]
        self._last_qr_seen = now

        try:
            raw = item.data.decode('utf-8', errors='ignore')
        except Exception:
            raw = str(item.data)

        payload = parse_qr_payload(
            raw,
            required_fields=self.qr_required_fields,
            allow_plain_text=self.qr_allow_plain_text)
        self.pub_qr_valid.publish(Bool(data=bool(payload.valid)))

        if payload.json_text == self._last_qr_text:
            return item.rect
        self._last_qr_text = payload.json_text

        self.pub_qr.publish(String(data=payload.json_text))
        if payload.valid:
            self.get_logger().info('二维码解析：%s' % payload.summary())
        else:
            self.get_logger().warning(
                '二维码缺少字段 %s：%s' % (payload.missing, payload.summary()))
        return item.rect

    def _decode_rotated(self, gray):
        """pyzbar 对斜着的二维码很挑角度：按固定角度补转再试。"""
        if not self.qr_retry_angles:
            return []
        height, width = gray.shape[:2]
        center = (width / 2.0, height / 2.0)
        for angle in self.qr_retry_angles:
            matrix = cv2.getRotationMatrix2D(center, float(angle), 1.0)
            rotated = cv2.warpAffine(
                gray, matrix, (width, height),
                flags=cv2.INTER_LINEAR, borderValue=255)
            try:
                results = pyzbar.decode(rotated)
            except Exception:
                continue
            if results:
                return results
        return []

    # ------------------------------------------------------------------
    # 调试图
    # ------------------------------------------------------------------
    def _publish_debug(self, frame, mask, roi_top_px, result, qr_box):
        now = time.monotonic()
        if now - self._last_debug_time < 1.0 / max(self.process_rate, 1.0):
            return
        self._last_debug_time = now

        height, width = frame.shape[:2]
        canvas = frame.copy()
        cv2.line(canvas, (0, roi_top_px), (width, roi_top_px), (0, 255, 255), 2)

        if result is not None and not result.lost:
            roi_h = height - roi_top_px
            pts = [(int(x), int(roi_top_px + roi_h - y)) for x, y in result.points]
            for x, y in pts:
                cv2.circle(canvas, (x, y), 4, (0, 255, 0), -1)
            if len(pts) >= 2:
                cv2.line(canvas, pts[-1], pts[0], (0, 255, 0), 2)
            bx = int(result.bottom_x)
            cv2.circle(canvas, (bx, height - 12), 10, (0, 255, 0), -1)
            cv2.putText(
                canvas, 'off %.2f slope %.2f' % (result.offset, result.slope),
                (10, roi_top_px + 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 255, 0), 2)
        elif result is not None:
            cv2.putText(canvas, 'LINE LOST', (10, roi_top_px + 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        if qr_box is not None:
            x, y, w, h = qr_box.left, qr_box.top, qr_box.width, qr_box.height
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(canvas, 'QR', (x, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        if mask is not None and mask.size:
            thumb_w = max(80, width // 5)
            thumb_h = max(40, int(mask.shape[0] * thumb_w / max(mask.shape[1], 1)))
            thumb = cv2.resize(mask, (thumb_w, thumb_h))
            thumb = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR)
            y0 = max(0, roi_top_px - thumb_h - 4)
            canvas[y0:y0 + thumb_h, 0:thumb_w] = thumb

        cv2.putText(canvas, '%.1f fps' % self._fps, (width - 120, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        if self._latest_stamp is not None:
            msg.header.stamp = self._latest_stamp
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
