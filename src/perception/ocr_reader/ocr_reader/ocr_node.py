"""OCR 识别节点（离线 tesseract）。

订阅：
    /camera/color/image_raw      sensor_msgs/Image
    /perception/ocr_request      std_msgs/Bool
        True 触发一次识别；trigger_mode=periodic 时也可不用这个话题。

发布：
    /perception/ocr_text         std_msgs/String
        清洗后的识别文本；内容变化时才发布。
    /perception/ocr_status       std_msgs/String
        JSON：文本、置信度、字数、耗时、是否正在识别、错误信息。
    /perception/ocr_debug_image  sensor_msgs/Image
        标出 OCR 区域与识别结果，方便拍视频。

对应功能：⑤ OCR 识别。
tesseract 与 chi_sim 语言包已在车上离线安装；节点运行在独立线程里，
不会阻塞相机与其他话题的回调。
"""

import json
import os
import tempfile
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from ocr_reader.core import (clean_text, parse_tesseract_tsv,
                             preprocess_for_ocr, run_tesseract_tsv,
                             tesseract_available, text_quality)


class OcrReader(Node):
    """周期或按需对画面做文字识别。"""

    def __init__(self):
        super().__init__('ocr_reader')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('trigger_mode', 'both')  # periodic / request / both
        self.declare_parameter('interval', 2.0)
        self.declare_parameter('min_change', 2.0)

        self.declare_parameter('roi_x_min', 0.10)
        self.declare_parameter('roi_x_max', 0.90)
        self.declare_parameter('roi_y_min', 0.10)
        self.declare_parameter('roi_y_max', 0.70)
        self.declare_parameter('scale', 1.5)
        self.declare_parameter('method', 'otsu')

        self.declare_parameter('lang', 'chi_sim+eng')
        self.declare_parameter('psm', 6)
        self.declare_parameter('min_conf', 30.0)
        self.declare_parameter('min_quality', 0.5)
        self.declare_parameter('min_chars', 2)
        self.declare_parameter('timeout', 8.0)
        self.declare_parameter('max_chars', 200)
        self.declare_parameter('debug', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.trigger_mode = str(self.get_parameter('trigger_mode').value).lower()
        self.interval = max(float(self.get_parameter('interval').value), 0.2)
        self.min_change = max(float(self.get_parameter('min_change').value), 0.0)
        self.roi = (
            float(self.get_parameter('roi_x_min').value),
            float(self.get_parameter('roi_y_min').value),
            float(self.get_parameter('roi_x_max').value),
            float(self.get_parameter('roi_y_max').value),
        )
        self.scale = float(self.get_parameter('scale').value)
        self.method = str(self.get_parameter('method').value)
        self.lang = str(self.get_parameter('lang').value)
        self.psm = int(self.get_parameter('psm').value)
        self.min_conf = float(self.get_parameter('min_conf').value)
        self.min_quality = float(self.get_parameter('min_quality').value)
        self.min_chars = int(self.get_parameter('min_chars').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.max_chars = int(self.get_parameter('max_chars').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.bridge = CvBridge()
        self._frame = None
        self._stamp = None
        self._last_gray_small = None
        self._last_run = 0.0
        self._requested = False
        self._busy = False
        self._lock = threading.Lock()

        self._last_text = ''
        self._last_conf = 0.0
        self._last_cost = 0.0
        self._last_error = ''
        self._last_debug = 0.0

        self.pub_text = self.create_publisher(String, '/perception/ocr_text', 10)
        self.pub_status = self.create_publisher(
            String, '/perception/ocr_status', 10)
        self.pub_debug = self.create_publisher(
            Image, '/perception/ocr_debug_image', 10)

        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/perception/ocr_request', self.on_request, 10)
        self.create_timer(0.2, self.tick)

        self._has_tesseract = tesseract_available()
        if not self._has_tesseract:
            self.get_logger().error(
                '找不到 tesseract 可执行文件，OCR 不可用。')
        self.get_logger().info(
            'OCR 节点启动：模式=%s 语言=%s psm=%d 间隔=%.1fs' % (
                self.trigger_mode, self.lang, self.psm, self.interval))

    # ------------------------------------------------------------------
    def on_image(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self._stamp = msg.header.stamp
        except Exception as exc:
            self.get_logger().error('图像转换失败：%s' % exc,
                                    throttle_duration_sec=5.0)

    def on_request(self, msg):
        if msg.data:
            self._requested = True

    def tick(self):
        if self._busy or self._frame is None or not self._has_tesseract:
            return
        now = time.monotonic()

        fire = False
        if self.trigger_mode in ('request', 'both') and self._requested:
            fire = True
        if (self.trigger_mode in ('periodic', 'both')
                and now - self._last_run >= self.interval):
            if self._scene_changed():
                fire = True

        if not fire:
            return
        self._requested = False
        self._last_run = now

        frame = self._frame.copy()
        with self._lock:
            self._busy = True
        thread = threading.Thread(target=self._run_once, args=(frame,),
                                  daemon=True)
        thread.start()

    def _scene_changed(self):
        if self._frame is None:
            return False
        small = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(small, (64, 48), interpolation=cv2.INTER_AREA)
        if self._last_gray_small is None:
            self._last_gray_small = small
            return True
        diff = float(np.mean(cv2.absdiff(small, self._last_gray_small)))
        self._last_gray_small = small
        return diff >= self.min_change

    # ------------------------------------------------------------------
    def _run_once(self, frame):
        t0 = time.monotonic()
        tmp_path = None
        text = ''
        conf = 0.0
        words = 0
        error = ''
        try:
            gray, _ = preprocess_for_ocr(
                frame, roi=self.roi, scale=self.scale, method=self.method)

            fd, tmp_path = tempfile.mkstemp(prefix='ocr_', suffix='.png')
            os.close(fd)
            if not cv2.imwrite(tmp_path, gray):
                raise RuntimeError('临时图片写入失败')

            tsv, error = run_tesseract_tsv(
                tmp_path, lang=self.lang, psm=self.psm,
                timeout=self.timeout)
            if error:
                text = ''
            else:
                text, conf, words = parse_tesseract_tsv(
                    tsv, min_conf=self.min_conf)
                text = clean_text(text, max_chars=self.max_chars)
        except Exception as exc:
            error = str(exc)
            text = ''
        finally:
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

        cost = time.monotonic() - t0
        quality = text_quality(text)
        ok = (not error and len(text) >= self.min_chars
              and quality >= self.min_quality)

        with self._lock:
            self._busy = False
        self._last_cost = cost
        self._last_error = error
        self._last_conf = conf

        if ok:
            if text != self._last_text:
                self._last_text = text
                self.pub_text.publish(String(data=text))
            self.get_logger().info('OCR：%s（置信度 %.0f%%，%.0f ms）' % (
                text, conf, cost * 1000.0))
        elif error:
            self.get_logger().warning('OCR 失败：%s' % error,
                                      throttle_duration_sec=10.0)

        self._publish_status(ok)
        if self.debug:
            self._publish_debug(frame, text if ok else '', ok)

    def _publish_status(self, ok):
        payload = {
            'ok': bool(ok),
            'text': self._last_text,
            'confidence': round(float(self._last_conf), 1),
            'cost_ms': round(float(self._last_cost) * 1000.0, 1),
            'busy': bool(self._busy),
            'error': self._last_error,
        }
        self.pub_status.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_debug(self, frame, text, ok):
        now = time.monotonic()
        if now - self._last_debug < 0.3:
            return
        self._last_debug = now

        canvas = frame.copy()
        height, width = canvas.shape[:2]
        x0 = int(self.roi[0] * width)
        y0 = int(self.roi[1] * height)
        x1 = int(self.roi[2] * width)
        y1 = int(self.roi[3] * height)
        color = (0, 255, 0) if ok else (0, 165, 255)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), color, 2)

        label = (text.encode('ascii', 'replace').decode('ascii')
                 if text else ('OCR RUNNING' if self._busy else 'NO TEXT'))
        cv2.rectangle(canvas, (0, 0), (width, 34), (0, 0, 0), -1)
        cv2.putText(canvas, label[:70], (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        if self._stamp is not None:
            msg.header.stamp = self._stamp
        msg.header.frame_id = 'camera_link'
        self.pub_debug.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OcrReader()
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
