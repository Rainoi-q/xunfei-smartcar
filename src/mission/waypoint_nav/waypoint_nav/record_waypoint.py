"""把车当前位置录成航点，写入 waypoints.yaml。

使用：
    1. 启动 nav2 并在 RViz 里用“2D Pose Estimate”把车定位好；
    2. 手动遥控车到目标位置；
    3. 另开终端：
         ros2 run waypoint_nav record_waypoint \
             --ros-args -p waypoints_file:=$HOME/smartcar_ws/src/smartcar_bringup/config/waypoints.yaml
    4. 每到一个点发布一次名字：
         ros2 topic pub --once /mission/record_waypoint std_msgs/String \
             "{data: door_3}"

位置来源优先 /amcl_pose，2 秒内没有就退回 TF(map→base_link)。
"""

import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from waypoint_nav.core import quaternion_to_yaw


class RecordWaypoint(Node):
    def __init__(self):
        super().__init__('record_waypoint')

        default_file = os.path.expanduser(
            '~/smartcar_ws/src/smartcar_bringup/config/waypoints.yaml')
        self.declare_parameter('waypoints_file', default_file)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.path = os.path.expanduser(
            str(self.get_parameter('waypoints_file').value))
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self._pose = None
        self._pose_time = 0.0

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(
            String, '/mission/record_waypoint', self.on_record, 10)

        self.get_logger().info(
            '航点录制器就绪：%s\n'
            '发布名字即记录，例如：ros2 topic pub --once /mission/record_waypoint '
            'std_msgs/String "{data: door_3}"' % self.path)

    def on_amcl(self, msg):
        pose = msg.pose.pose
        self._pose = (pose.position.x, pose.position.y,
                      quaternion_to_yaw(pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w))
        self._pose_time = time.monotonic()

    def _current_pose(self):
        if self._pose is not None and time.monotonic() - self._pose_time < 2.0:
            return self._pose
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().error('TF 查询失败：%s' % exc)
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (translation.x, translation.y,
                quaternion_to_yaw(rotation.x, rotation.y,
                                  rotation.z, rotation.w))

    def on_record(self, msg):
        name = (msg.data or '').strip()
        if not name:
            return
        pose = self._current_pose()
        if pose is None:
            return
        x, y, yaw = pose

        data = {'frame_id': self.map_frame, 'waypoints': {}, 'routes': {}}
        if os.path.exists(self.path):
            try:
                with open(self.path, 'r', encoding='utf-8') as handle:
                    loaded = yaml.safe_load(handle) or {}
                if isinstance(loaded, dict):
                    data.update(loaded)
                    data.setdefault('waypoints', {})
                    data.setdefault('routes', {})
            except Exception as exc:
                self.get_logger().warning('读取原文件失败，将新建：%s' % exc)

        data['waypoints'][name] = {
            'x': round(float(x), 3),
            'y': round(float(y), 3),
            'yaw': round(float(yaw), 3),
        }

        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, 'w', encoding='utf-8') as handle:
                yaml.safe_dump(data, handle, allow_unicode=True,
                               sort_keys=False, default_flow_style=None)
        except Exception as exc:
            self.get_logger().error('写入失败：%s' % exc)
            return

        self.get_logger().info(
            '已记录航点 %s = (%.3f, %.3f, yaw %.3f)，共 %d 个点' % (
                name, x, y, yaw, len(data['waypoints'])))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RecordWaypoint()
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
