"""光照强弱识别 + 摄像机曝光自适应。

订阅：
    /camera/color/image_raw   sensor_msgs/Image
        WHEELTEC 出厂 Astra 彩色话题。

发布：
    /camera/brightness        std_msgs/Int8
        光照档位：0=暗，1=中，2=亮（接口文档约定）。
    /camera/set_exposure      std_msgs/Int32
        曝光目标值；backend=topic 时用于驱动 camera_driver。
    /lighting/status          std_msgs/String
        JSON：亮度均值、档位、当前曝光、后端、是否出错，供 HUD 显示。

曝光后端（参数 exposure_backend）：
    v4l2  直接调用 v4l2-ctl 控制 UVC 彩色相机（Astra Pro 默认走这条）
    topic 只发 /camera/set_exposure，适合自研 camera_driver
    none  只看亮度，不改曝光（调试用）

对应功能：⑥ 自动识别光照强弱、⑦ 自适应调节摄像机曝光。
"""

import json
import os
import re
import subprocess
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Int32, String

from lighting_adaptor.core import BrightnessClassifier, ExposureController

_CTRL_RE = re.compile(
    r'^(?P<name>\S+).*?\((?:int|menu|bool)\)\s*:\s*(?P<attrs>.*)$')


def _run(cmd, timeout=2.0):
    """执行命令，返回 (returncode, stdout+stderr)。"""
    try:
        proc = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout, check=False)
        return proc.returncode, proc.stdout.decode('utf-8', errors='ignore')
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, str(exc)


class LightingAdaptor(Node):
    """把画面亮度映射成光照档位，并闭环调节曝光。"""

    def __init__(self):
        super().__init__('lighting_adaptor')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('process_rate', 5.0)
        self.declare_parameter('roi_x_min', 0.15)
        self.declare_parameter('roi_x_max', 0.85)
        self.declare_parameter('roi_y_min', 0.25)
        self.declare_parameter('roi_y_max', 0.75)

        self.declare_parameter('dark_threshold', 60.0)
        self.declare_parameter('bright_threshold', 170.0)
        self.declare_parameter('hysteresis', 8.0)
        self.declare_parameter('smooth_alpha', 0.3)
        self.declare_parameter('initial_level', 1)

        self.declare_parameter('target_luma', 110.0)
        self.declare_parameter('deadband', 8.0)
        self.declare_parameter('gain', 2.0)
        self.declare_parameter('max_step', 60)
        self.declare_parameter('update_interval', 0.8)
        self.declare_parameter('invert_exposure', False)

        self.declare_parameter('exposure_backend', 'v4l2')
        self.declare_parameter('v4l2_device', 'auto')
        self.declare_parameter('exposure_control', 'exposure_absolute')
        self.declare_parameter('auto_control', 'auto_exposure')
        self.declare_parameter('auto_off_value', 1)
        self.declare_parameter('exposure_min', 1)
        self.declare_parameter('exposure_max', 5000)
        self.declare_parameter('initial_exposure', 120)
        self.declare_parameter('restore_on_exit', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.rate = max(float(self.get_parameter('process_rate').value), 0.5)
        self.roi = (
            float(self.get_parameter('roi_x_min').value),
            float(self.get_parameter('roi_x_max').value),
            float(self.get_parameter('roi_y_min').value),
            float(self.get_parameter('roi_y_max').value),
        )

        self.classifier = BrightnessClassifier(
            dark_threshold=float(self.get_parameter('dark_threshold').value),
            bright_threshold=float(self.get_parameter('bright_threshold').value),
            hysteresis=float(self.get_parameter('hysteresis').value),
            alpha=float(self.get_parameter('smooth_alpha').value),
            initial_level=int(self.get_parameter('initial_level').value))

        self.controller = ExposureController(
            target_luma=float(self.get_parameter('target_luma').value),
            deadband=float(self.get_parameter('deadband').value),
            gain=float(self.get_parameter('gain').value),
            min_exposure=int(self.get_parameter('exposure_min').value),
            max_exposure=int(self.get_parameter('exposure_max').value),
            max_step=int(self.get_parameter('max_step').value),
            initial_exposure=int(self.get_parameter('initial_exposure').value),
            invert=bool(self.get_parameter('invert_exposure').value))

        self.backend = str(self.get_parameter('exposure_backend').value).lower()
        self.exposure_control = str(
            self.get_parameter('exposure_control').value)
        self.auto_control = str(self.get_parameter('auto_control').value)
        self.auto_off_value = int(self.get_parameter('auto_off_value').value)
        self.update_interval = max(
            float(self.get_parameter('update_interval').value), 0.0)
        self.restore_on_exit = bool(self.get_parameter('restore_on_exit').value)

        self.device = ''
        self._orig_auto = None
        self._orig_exposure = None
        self._v4l2_ready = False
        self._last_apply = 0.0
        self._last_status = 0.0
        self._last_luma = None
        self._last_level = None
        self._error = ''

        self.bridge = CvBridge()
        self._latest_frame = None

        self.pub_level = self.create_publisher(Int8, '/camera/brightness', 10)
        self.pub_exposure = self.create_publisher(
            Int32, '/camera/set_exposure', 10)
        self.pub_status = self.create_publisher(String, '/lighting/status', 10)

        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.process)

        if self.backend == 'v4l2':
            self._setup_v4l2()
        elif self.backend not in ('topic', 'none'):
            self.get_logger().warning(
                '未知 exposure_backend=%s，按 none 处理' % self.backend)
            self.backend = 'none'

        self.get_logger().info(
            '光照/曝光节点启动：后端=%s 设备=%s 曝光范围=[%d,%d]' % (
                self.backend, self.device or '-',
                self.controller.min_exposure, self.controller.max_exposure))

    # ------------------------------------------------------------------
    # v4l2 初始化 / 写入
    # ------------------------------------------------------------------
    def _parse_controls(self, text):
        """把 v4l2-ctl --list-ctrls 输出解析成 {name: attrs_dict}。"""
        controls = {}
        for line in text.splitlines():
            match = _CTRL_RE.match(line.strip())
            if not match:
                continue
            attrs = {}
            for key in ('min', 'max', 'step', 'default', 'value'):
                found = re.search(r'%s=(-?\d+)' % key, match.group('attrs'))
                if found:
                    attrs[key] = int(found.group(1))
            controls[match.group('name')] = attrs
        return controls

    def _list_controls(self, device):
        code, out = _run(['v4l2-ctl', '-d', device, '--list-ctrls'])
        if code != 0:
            return {}
        return self._parse_controls(out)

    def _find_device(self):
        wanted = str(self.get_parameter('v4l2_device').value)
        if wanted and wanted.lower() != 'auto':
            return wanted
        for index in range(8):
            dev = '/dev/video%d' % index
            if not os.path.exists(dev):
                continue
            controls = self._list_controls(dev)
            if self.exposure_control in controls:
                self.get_logger().info('自动选中曝光设备 %s' % dev)
                return dev
        return '/dev/video0'

    def _setup_v4l2(self):
        self.device = self._find_device()
        controls = self._list_controls(self.device)
        if not controls:
            self._error = 'v4l2-ctl 不可用或设备无控件'
            self.get_logger().warning(
                '无法读取 %s 的控件（%s）。⑦ 曝光将只记录、不下发。'
                % (self.device, self._error))
            return

        exposure = controls.get(self.exposure_control)
        if exposure is None:
            self._error = '设备没有 %s 控件' % self.exposure_control
            self.get_logger().warning(
                '%s 没有 %s 控件，⑦ 无法主动调曝光。可用 v4l2-ctl --list-ctrls '
                '查看真实控件名，并修改 exposure_control 参数。' % (
                    self.device, self.exposure_control))
            return

        lower = int(exposure.get('min', self.controller.min_exposure))
        upper = int(exposure.get('max', self.controller.max_exposure))
        current = int(exposure.get('value',
                                   self.controller.exposure))
        lower = max(0, lower)
        if upper <= lower:
            upper = lower + 1
        self.controller.set_range(lower, upper)
        self.controller.exposure = int(
            max(lower, min(upper, current)))
        self._orig_exposure = current

        auto = controls.get(self.auto_control)
        if auto is not None:
            self._orig_auto = int(auto.get('value', self.auto_off_value))
            self._set_control(self.auto_control, self.auto_off_value)

        self._v4l2_ready = self._set_control(
            self.exposure_control, self.controller.exposure)
        if self._v4l2_ready:
            self.get_logger().info(
                '曝光控制已接管：%s=%d（范围 %d~%d）' % (
                    self.exposure_control, self.controller.exposure,
                    lower, upper))
        else:
            self._error = '写入曝光失败'

    def _set_control(self, name, value):
        code, out = _run([
            'v4l2-ctl', '-d', self.device,
            '-c', '%s=%d' % (name, int(value))])
        if code != 0:
            self._error = out.strip()[:120]
            self.get_logger().warning(
                'v4l2 设置 %s=%s 失败：%s' % (name, value, self._error),
                throttle_duration_sec=10.0)
            return False
        return True

    def _restore_v4l2(self):
        if not self._v4l2_ready or not self.restore_on_exit:
            return
        if self._orig_exposure is not None:
            self._set_control(self.exposure_control, self._orig_exposure)
        if self._orig_auto is not None:
            self._set_control(self.auto_control, self._orig_auto)

    # ------------------------------------------------------------------
    # 图像与闭环
    # ------------------------------------------------------------------
    def on_image(self, msg):
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error('图像转换失败：%s' % exc,
                                    throttle_duration_sec=5.0)

    def _mean_luma(self, frame):
        height, width = frame.shape[:2]
        x0 = int(max(0.0, min(1.0, self.roi[0])) * width)
        x1 = int(max(0.0, min(1.0, self.roi[1])) * width)
        y0 = int(max(0.0, min(1.0, self.roi[2])) * height)
        y1 = int(max(0.0, min(1.0, self.roi[3])) * height)
        if x1 <= x0:
            x0, x1 = 0, width
        if y1 <= y0:
            y0, y1 = 0, height
        roi = frame[y0:y1, x0:x1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        return float(gray.mean())

    def process(self):
        frame = self._latest_frame
        if frame is None:
            return

        luma = self._mean_luma(frame)
        level, smoothed = self.classifier.update(luma)
        self._last_luma = smoothed
        self._last_level = level
        self.pub_level.publish(Int8(data=int(level)))

        now = time.monotonic()
        if now - self._last_apply >= self.update_interval:
            target = self.controller.update(smoothed)
            if target is not None:
                self._last_apply = now
                self.apply_exposure(target)

        if now - self._last_status >= 0.5:
            self._last_status = now
            self.publish_status()

    def apply_exposure(self, value):
        value = int(value)
        if self.backend == 'v4l2':
            if self._v4l2_ready:
                self._set_control(self.exposure_control, value)
        elif self.backend == 'topic':
            self.pub_exposure.publish(Int32(data=value))
        # none：只维护内部状态，供 HUD 显示“将要设置的值”

    def publish_status(self):
        payload = {
            'level': int(self._last_level if self._last_level is not None else 0),
            'level_name': BrightnessClassifier.level_name(
                self._last_level if self._last_level is not None else 0),
            'luma': round(float(self._last_luma or 0.0), 1),
            'exposure': int(self.controller.exposure),
            'backend': self.backend,
            'device': self.device,
            'applied': bool(self._v4l2_ready or self.backend == 'topic'),
            'error': self._error,
        }
        self.pub_status.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def destroy_node(self):
        if self.backend == 'v4l2':
            self._restore_v4l2()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LightingAdaptor()
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
