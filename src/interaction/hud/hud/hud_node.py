"""演示用 HUD：把感知/控制/任务信息叠加到相机画面。

订阅（全部可选，缺哪个就显示 --）：
    /camera/color/image_raw    图像底图
    /perception/line           巡线偏移
    /perception/qr_payload     二维码
    /perception/ocr_text       OCR 文本
    /camera/brightness         光照档位
    /lighting/status           曝光详情
    /control/avoid_state       避障状态
    /control/arbiter_state     仲裁/速度
    /control/parking_state     泊车状态
    /mission/nav_status        导航状态
    /mission/state             任务状态
    /cmd_vel                   最终速度
    /odom                      里程计速度

发布：
    /hud/image                 sensor_msgs/Image

查看方式（车上 web_video_server 已装）：
    ros2 run web_video_server web_video_server
    浏览器打开 http://<车IP>:8080/stream?topic=/hud/image

界面文字全部使用 ASCII，避免 OpenCV 不支持中文字体的问题；
二维码/OCR 里的中文会显示成 ?，视频剪辑时可加中文字幕。
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int8, String

W, H = 960, 540
PANEL = (0, 0, 0)


def _ascii(text, limit=48):
    text = '' if text is None else str(text)
    text = text.replace('\n', ' ')
    if len(text) > limit:
        text = text[:limit - 1] + '~'
    return text.encode('ascii', 'replace').decode('ascii')


def _json(text):
    try:
        data = json.loads(text)
        return data if isinstance(data, dict) else {}
    except (ValueError, TypeError):
        return {}


class Hud(Node):
    def __init__(self):
        super().__init__('hud')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/hud/image')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('width', W)
        self.declare_parameter('height', H)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.bridge = CvBridge()
        self._frame = None
        self._stamp = None

        self._data = {}
        self._times = {}
        self._start = time.monotonic()

        # 必须用 RELIABLE（默认 10）：web_video_server 用 RELIABLE 订阅，
        # 发布端若是 sensor-data 的 BEST_EFFORT，QoS 不兼容，一帧都到不了
        # 浏览器 —— 表现为网页一直转圈
        self.pub = self.create_publisher(
            Image, str(self.get_parameter('output_topic').value), 10)

        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self.on_image, qos_profile_sensor_data)

        def sub(topic, msg_type, key, qos=10):
            self.create_subscription(
                msg_type, topic,
                lambda msg, k=key: self._store(k, msg), qos)

        sub('/perception/line', Float32, 'line')
        sub('/perception/qr_payload', String, 'qr')
        sub('/perception/ocr_text', String, 'ocr')
        sub('/camera/brightness', Int8, 'brightness')
        sub('/lighting/status', String, 'lighting')
        sub('/control/avoid_state', String, 'avoid')
        sub('/control/arbiter_state', String, 'arbiter')
        sub('/control/parking_state', String, 'parking')
        sub('/mission/nav_status', String, 'nav')
        sub('/mission/state', String, 'mission')
        sub('/cmd_vel', Twist, 'cmd')
        sub('/odom', Odometry, 'odom')

        rate = max(float(self.get_parameter('rate').value), 1.0)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            'HUD 启动：%dx%d -> %s' % (
                self.width, self.height,
                self.get_parameter('output_topic').value))

    # ------------------------------------------------------------------
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('图像转换失败：%s' % exc,
                                    throttle_duration_sec=5.0)
            return
        self._frame = frame
        self._stamp = msg.header.stamp

    def _store(self, key, msg):
        self._data[key] = msg
        self._times[key] = time.monotonic()

    def _fresh(self, key, timeout=2.0):
        stamp = self._times.get(key)
        return stamp is not None and time.monotonic() - stamp <= timeout

    # ------------------------------------------------------------------
    def tick(self):
        frame = self._frame
        if frame is None:
            canvas = np.full((self.height, self.width, 3), 28, np.uint8)
            cv2.putText(canvas, 'WAITING FOR CAMERA...', (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
            self._publish(canvas, None)
            return

        canvas = cv2.resize(frame, (self.width, self.height),
                            interpolation=cv2.INTER_LINEAR)
        self._draw(canvas)
        self._publish(canvas, self._stamp)

    def _draw(self, canvas):
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (self.width, 74), PANEL, -1)
        cv2.rectangle(overlay, (0, self.height - 128),
                      (self.width, self.height), PANEL, -1)
        cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas)

        mission = self._data.get('mission')
        mission_text = _ascii(mission.data if mission else 'IDLE', 30)
        cv2.putText(canvas, 'XUNFEI CUP 2026  SMART CAR', (18, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2)
        cv2.putText(canvas, 'MISSION: ' + mission_text, (18, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        elapsed = time.monotonic() - self._start
        cv2.putText(canvas, 'T+%05.1fs' % elapsed,
                    (self.width - 180, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)

        y = 100
        line_text, line_value = self._line_info()
        self._row(canvas, y, 'LINE', line_text, line_value)
        y += 28
        self._row(canvas, y, 'QR', self._qr_info(), None)
        y += 28
        self._row(canvas, y, 'OCR', self._ocr_info(), None)
        y += 28
        light = self._light_info()
        self._row(canvas, y, 'LIGHT', light[0], light[1])
        y += 28
        avoid = self._avoid_info()
        self._row(canvas, y, 'OBSTACLE', avoid[0], avoid[1])
        y += 28
        nav = self._nav_info()
        self._row(canvas, y, 'NAV', nav[0], nav[1])
        y += 28
        park = self._park_info()
        self._row(canvas, y, 'PARK', park[0], park[1])

        # 底部：速度 + 偏移条 + 障碍条
        v, w = self._speed_info()
        base_y = self.height - 96
        cv2.putText(canvas, 'V %+.2f m/s' % v, (18, base_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        cv2.putText(canvas, 'W %+.2f rad/s' % w, (200, base_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cfg = self._data.get('arbiter')
        if cfg is not None and self._fresh('arbiter'):
            cfg_data = _json(cfg.data)
            cv2.putText(canvas, 'MODE %s' % _ascii(cfg_data.get('mode', '-')),
                        (400, base_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 220, 0), 2)

        self._bar(canvas, 18, base_y + 16, 360, 12, line_value,
                  'LINE OFFSET')
        distance = self._obstacle_distance()
        if math.isfinite(distance):
            ratio = max(0.0, min(1.0, distance / 1.5))
            self._bar(canvas, 400, base_y + 16, 260, 12, ratio,
                      'CLEARANCE %.2fm' % distance, color=(0, 255, 0))
        else:
            cv2.putText(canvas, 'CLEARANCE --', (400, base_y + 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 2)

        badges = ['LINE', 'QR', 'OCR', 'LIGHT', 'AVOID', 'NAV', 'PARK', 'SIM']
        active = {
            'LINE': self._fresh('line'),
            'QR': self._fresh('qr', 5.0),
            'OCR': self._fresh('ocr', 5.0),
            'LIGHT': self._fresh('lighting'),
            'AVOID': self._fresh('avoid'),
            'NAV': self._fresh('nav', 3.0),
            'PARK': self._fresh('parking', 3.0),
            'SIM': True,
        }
        x = 18
        for name in badges:
            color = (0, 200, 0) if active[name] else (100, 100, 100)
            cv2.rectangle(canvas, (x, self.height - 40),
                          (x + 100, self.height - 12), color, 1)
            cv2.putText(canvas, name, (x + 10, self.height - 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
            x += 112

    def _row(self, canvas, y, label, text, value):
        cv2.putText(canvas, label, (18, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 200, 255), 1)
        cv2.putText(canvas, _ascii(text, 64), (120, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        if value is not None:
            # cv2.putText 只接受 str：LINE 传的是 float，直接塞进去会抛
            # "Can't convert object to 'str' for 'text'"
            if isinstance(value, str):
                value_text = value
            elif isinstance(value, float):
                value_text = '%+.2f' % value
            else:
                value_text = str(value)
            cv2.putText(canvas, value_text, (self.width - 120, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

    def _line_info(self):
        msg = self._data.get('line')
        if msg is None or not self._fresh('line'):
            return '--', 0.0
        value = float(msg.data)
        if not math.isfinite(value):
            return 'LOST', 0.0
        side = 'RIGHT' if value > 0.02 else ('LEFT' if value < -0.02 else 'CENTER')
        return 'off %+.2f  %s' % (value, side), value

    def _qr_info(self):
        msg = self._data.get('qr')
        if msg is None or not self._fresh('qr', 5.0):
            return '--'
        data = _json(msg.data)
        dest = data.get('destination') or data.get('raw') or '?'
        order = data.get('order_id') or data.get('order') or '?'
        return 'order %s -> %s' % (_ascii(order, 16), _ascii(dest, 24))

    def _ocr_info(self):
        msg = self._data.get('ocr')
        if msg is None or not self._fresh('ocr', 5.0):
            return '--'
        return _ascii(msg.data, 56)

    def _light_info(self):
        msg = self._data.get('lighting')
        if msg is None or not self._fresh('lighting'):
            level = self._data.get('brightness')
            if level is not None:
                return 'level %d' % int(level.data), 'exposure --'
            return '--', 'exposure --'
        data = _json(msg.data)
        name = data.get('level_name', '?')
        luma = data.get('luma', '-')
        exposure = data.get('exposure', '-')
        return '%s luma %s' % (name, luma), 'exp %s' % exposure

    def _avoid_info(self):
        msg = self._data.get('avoid')
        if msg is None or not self._fresh('avoid'):
            return '--', '--'
        data = _json(msg.data)
        state = data.get('state', '?')
        distance = data.get('distance', -1.0)
        source = data.get('source', '?')
        return '%s %s' % (state, source), (
            'dist %.2fm' % distance if isinstance(distance, (int, float))
            and distance >= 0 else 'dist --')

    def _obstacle_distance(self):
        msg = self._data.get('avoid')
        if msg is None or not self._fresh('avoid'):
            return math.inf
        data = _json(msg.data)
        value = data.get('distance', -1.0)
        try:
            value = float(value)
        except (TypeError, ValueError):
            return math.inf
        return value if value > 0 else math.inf

    def _nav_info(self):
        msg = self._data.get('nav')
        if msg is None or not self._fresh('nav', 3.0):
            return '--', '--'
        data = _json(msg.data)
        state = data.get('state', '?')
        name = _ascii(data.get('name') or data.get('route') or '-', 20)
        distance = data.get('distance_remaining', -1.0)
        text = '%s %s' % (state, name)
        if isinstance(distance, (int, float)) and distance >= 0:
            return text, '%.2fm left' % distance
        return text, ''

    def _park_info(self):
        msg = self._data.get('parking')
        if msg is None or not self._fresh('parking', 3.0):
            return '--', '--'
        data = _json(msg.data)
        state = data.get('state', '?')
        progress = data.get('progress', 0.0)
        distance = data.get('distance', -1.0)
        text = '%s %d%%' % (state, int(float(progress) * 100))
        if isinstance(distance, (int, float)) and distance > 0:
            return text, '%.2fm' % distance
        return text, ''

    def _speed_info(self):
        cmd = self._data.get('cmd')
        if cmd is not None and self._fresh('cmd'):
            return float(cmd.linear.x), float(cmd.angular.z)
        odom = self._data.get('odom')
        if odom is not None and self._fresh('odom'):
            return (float(odom.twist.twist.linear.x),
                    float(odom.twist.twist.angular.z))
        return 0.0, 0.0

    @staticmethod
    def _bar(canvas, x, y, width, height, ratio, label,
             color=(0, 220, 255)):
        cv2.rectangle(canvas, (x, y), (x + width, y + height),
                      (80, 80, 80), 1)
        center = x + width // 2
        cv2.line(canvas, (center, y - 3), (center, y + height + 3),
                 (255, 255, 255), 1)
        pos = int(center + max(-1.0, min(1.0, ratio)) * (width // 2 - 4))
        cv2.circle(canvas, (pos, y + height // 2), 6, color, -1)
        cv2.putText(canvas, label, (x, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    def _publish(self, canvas, stamp):
        msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = 'hud'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Hud()
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
