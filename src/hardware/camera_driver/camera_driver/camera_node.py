"""相机驱动节点。

两种数据来源，靠参数 source 切换：
    source:=device   真实 USB 摄像头（/dev/video0）
    source:=video    视频文件回放，供无车阶段开发使用

发布：
    /camera/image_raw     sensor_msgs/Image (bgr8)

订阅：
    /camera/set_exposure  std_msgs/Int32
        运行中动态调整曝光值，对应功能⑦「自适应调节摄像机曝光」。

重要：
    CAP_PROP_EXPOSURE 的量纲与取值范围因相机型号而异，务必先执行
        v4l2-ctl --list-ctrls
    确认后再写入 params.yaml。若相机只支持自动曝光，功能⑦无法实现。
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class CameraNode(Node):
    """把视频源的帧发布成 ROS2 图像话题。"""

    def __init__(self):
        super().__init__('camera_driver')

        self.declare_parameter('source', 'device')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('video_path', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('auto_exposure', False)
        self.declare_parameter('exposure_absolute', 120)
        self.declare_parameter('gain', 0)
        self.declare_parameter('frame_id', 'camera_link')

        self.source = self.get_parameter('source').value
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Image, '/camera/image_raw', qos_profile_sensor_data)
        self.create_subscription(
            Int32, '/camera/set_exposure', self.on_set_exposure, 10)

        self.cap = self._open_capture()
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError(
                '无法打开视频源，请检查 device / video_path 参数是否填写正确')

        self._apply_camera_settings()

        self._frame_count = 0
        self._last_report = time.monotonic()
        self.create_timer(1.0 / self.fps, self.tick)

        self.get_logger().info(
            '相机已启动 source=%s fps=%.1f 分辨率=%dx%d' % (
                self.source, self.fps,
                int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------
    def _open_capture(self):
        if self.source == 'video':
            path = self.get_parameter('video_path').value
            if not path:
                raise RuntimeError('source=video 时必须指定 video_path 参数')
            return cv2.VideoCapture(path)

        device = self.get_parameter('device').value
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            # 某些平台不支持 CAP_V4L2 后端，退回默认后端
            cap = cv2.VideoCapture(device)
        return cap

    def _apply_camera_settings(self):
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if self.source == 'device':
            self.apply_exposure(int(self.get_parameter('exposure_absolute').value))
            self.cap.set(cv2.CAP_PROP_GAIN, int(self.get_parameter('gain').value))

    # ------------------------------------------------------------------
    # 曝光控制（功能⑦）
    # ------------------------------------------------------------------
    def apply_exposure(self, value):
        """主动设置曝光值。auto_exposure=true 时不做任何事。"""
        if self.get_parameter('auto_exposure').value:
            self.get_logger().debug('auto_exposure=true，跳过手动曝光设置')
            return

        # OpenCV + V4L2：1 = 手动曝光，3 = 自动曝光
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        ok = self.cap.set(cv2.CAP_PROP_EXPOSURE, float(value))
        if ok:
            self.get_logger().info('曝光已设为 %s' % value)
        else:
            self.get_logger().warning(
                '设置曝光值 %s 失败。请执行 v4l2-ctl --list-ctrls 确认取值范围，'
                '或检查该相机是否支持手动曝光' % value)

    def on_set_exposure(self, msg):
        self.apply_exposure(int(msg.data))

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            if self.source == 'video' and self.get_parameter('loop').value:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
            if not ok or frame is None:
                self.get_logger().warning(
                    '读帧失败，请检查设备连接或视频文件路径', throttle_duration_sec=5.0)
                return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._last_report
        if elapsed >= 5.0:
            self.get_logger().info(
                '发布帧率 %.1f Hz' % (self._frame_count / elapsed))
            self._frame_count = 0
            self._last_report = now

    def destroy_node(self):
        if getattr(self, 'cap', None) is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        rclpy.logging.get_logger('camera_driver').fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
