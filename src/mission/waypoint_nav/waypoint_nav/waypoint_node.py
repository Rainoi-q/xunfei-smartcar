"""nav2 航点导航：按路线依次发送 NavigateToPose 目标。

订阅：
    /mission/nav_start   std_msgs/String  路线名或单个航点名
    /mission/stop        std_msgs/Bool    true 取消当前导航

发布：
    /mission/nav_status  std_msgs/String  JSON：状态、当前航点、剩余距离
    /mission/nav_done    std_msgs/Bool    true=全部到达，false=失败/取消
    /mission/nav_feedback std_msgs/String JSON：剩余距离、当前序号

依赖：
    先启动厂家的 nav2（定位 + 地图 + 规划器），本节点只负责“按顺序发目标”。
    地图用车上已建好的 WHEELTEC 地图。

对应功能：⑬ 自主路径规划（使用 nav2 全局/局部规划）。
"""

import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waypoint_nav.core import load_config, resolve_route, yaw_to_quaternion


class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        default_file = os.path.expanduser(
            '~/smartcar_ws/src/smartcar_bringup/config/waypoints.yaml')
        self.declare_parameter('waypoints_file', default_file)
        self.declare_parameter('frame_id', '')  # 空=用文件里的 frame_id
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('retry_limit', 1)
        self.declare_parameter('status_rate', 2.0)

        self.waypoints_file = os.path.expanduser(
            str(self.get_parameter('waypoints_file').value))
        self.frame_override = str(self.get_parameter('frame_id').value)
        self.goal_timeout = max(
            float(self.get_parameter('goal_timeout').value), 5.0)
        self.retry_limit = max(int(self.get_parameter('retry_limit').value), 0)
        self.status_rate = max(
            float(self.get_parameter('status_rate').value), 0.5)

        self.frame_id = 'map'
        self.waypoints = {}
        self.routes = {}
        self._load_file()

        self._goals = []
        self._index = 0
        self._retry = 0
        self._route_name = ''
        self._running = False
        self._goal_handle = None
        self._goal_sent_at = 0.0
        self._distance_remaining = -1.0
        self._cancel_requested = False
        self._last_status = 0.0

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.pub_status = self.create_publisher(
            String, '/mission/nav_status', 10)
        self.pub_done = self.create_publisher(Bool, '/mission/nav_done', 10)
        self.pub_feedback = self.create_publisher(
            String, '/mission/nav_feedback', 10)

        self.create_subscription(String, '/mission/nav_start',
                                 self.on_start, 10)
        self.create_subscription(Bool, '/mission/stop', self.on_stop, 10)
        self.create_timer(1.0, self.watchdog)
        self.create_timer(1.0 / self.status_rate, self.publish_status)

        self.get_logger().info(
            '航点导航启动：文件=%s 航点=%d 路线=%d frame=%s' % (
                self.waypoints_file, len(self.waypoints),
                len(self.routes), self.frame_id))

    # ------------------------------------------------------------------
    def _load_file(self):
        if not os.path.exists(self.waypoints_file):
            self.get_logger().error(
                '航点文件不存在：%s。可用 record_waypoint 录制。'
                % self.waypoints_file)
            return
        try:
            frame_id, waypoints, routes = load_config(self.waypoints_file)
        except Exception as exc:
            self.get_logger().error('航点文件解析失败：%s' % exc)
            return
        self.frame_id = self.frame_override or frame_id
        self.waypoints = waypoints
        self.routes = routes

    def reload(self):
        self._load_file()
        self.get_logger().info('航点已重新加载：%d 个点' % len(self.waypoints))

    # ------------------------------------------------------------------
    def on_start(self, msg):
        name = (msg.data or '').strip()
        if not name:
            return
        if self._running:
            self.get_logger().warning('正在执行 %s，忽略新请求 %s' % (
                self._route_name, name))
            return
        self.reload()
        try:
            goals = resolve_route(name, self.waypoints, self.routes)
        except Exception as exc:
            self.get_logger().error('无法开始导航：%s' % exc)
            self.pub_done.publish(Bool(data=False))
            self._publish_status('FAILED', name, 0, len(self._goals),
                                 str(exc))
            return

        self._route_name = name
        self._goals = goals
        self._index = 0
        self._retry = 0
        self._running = True
        self._cancel_requested = False
        self._distance_remaining = -1.0
        self.get_logger().info('开始路线 %s：%s' % (
            name, ' -> '.join(point.name for point in goals)))

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                '找不到 navigate_to_pose action，请先启动 nav2')
            self._finish(False, 'nav2 not available')
            return

        self._send_current()

    def on_stop(self, msg):
        if msg.data and self._running:
            self.get_logger().warning('收到停止指令，取消导航')
            self._cancel_requested = True
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._finish(False, 'stopped by user')

    # ------------------------------------------------------------------
    def _send_current(self):
        if not self._running:
            return
        if self._index >= len(self._goals):
            self._finish(True, 'all waypoints reached')
            return

        point = self._goals[self._index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point.x
        goal.pose.pose.position.y = point.y
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            '发送航点 %d/%d：%s (%.2f, %.2f, yaw %.2f)' % (
                self._index + 1, len(self._goals), point.name,
                point.x, point.y, point.yaw))
        self._goal_sent_at = time.monotonic()
        self._goal_handle = None
        self._distance_remaining = -1.0

        future = self.client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error('发送导航目标失败：%s' % exc)
            self._retry_or_fail('send exception')
            return
        if not goal_handle.accepted:
            self.get_logger().error('nav2 拒绝了目标')
            self._retry_or_fail('goal rejected')
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        try:
            feedback = feedback_msg.feedback
            distance = float(feedback.distance_remaining)
            self._distance_remaining = distance
            self.pub_feedback.publish(String(data=json.dumps({
                'index': self._index,
                'total': len(self._goals),
                'name': self._goals[self._index].name,
                'distance_remaining': round(distance, 2),
            }, ensure_ascii=False)))
        except Exception:
            pass

    def on_result(self, future):
        self._goal_handle = None
        if not self._running:
            return
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self.get_logger().error('获取导航结果失败：%s' % exc)
            status = GoalStatus.STATUS_ABORTED

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._index += 1
            self._retry = 0
            self._send_current()
        else:
            self._retry_or_fail('status=%d' % status)

    def _retry_or_fail(self, reason):
        if self._cancel_requested:
            self._finish(False, reason)
            return
        if self._retry < self.retry_limit:
            self._retry += 1
            self.get_logger().warning(
                '航点失败（%s），重试 %d/%d' % (
                    reason, self._retry, self.retry_limit))
            self._send_current()
        else:
            self._finish(False, reason)

    def watchdog(self):
        if not self._running:
            return
        if self._index >= len(self._goals):
            return
        elapsed = time.monotonic() - self._goal_sent_at
        if elapsed > self.goal_timeout:
            self.get_logger().error(
                '航点超时（%.0fs），取消' % elapsed)
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._finish(False, 'timeout')

    def _finish(self, ok, reason):
        if not self._running and ok:
            return
        self._running = False
        self._goals = []
        self._goal_handle = None
        self._publish_status('DONE' if ok else 'FAILED',
                             self._route_name, self._index, 0, reason)
        self.pub_done.publish(Bool(data=bool(ok)))
        self.get_logger().info('导航结束：%s（%s）' % (
            '成功' if ok else '失败', reason))

    def publish_status(self):
        self._publish_status(
            'RUNNING' if self._running else 'IDLE',
            self._route_name, self._index, len(self._goals), '')

    def _publish_status(self, state, route, index, total, message):
        payload = {
            'state': state,
            'route': route,
            'index': int(index),
            'total': int(total),
            'name': (self._goals[index].name
                     if 0 <= index < len(self._goals) else ''),
            'distance_remaining': round(float(self._distance_remaining), 2),
            'message': message,
        }
        self.pub_status.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def destroy_node(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointNav()
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
