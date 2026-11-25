"""任务状态机：二维码下单 → nav2 导航 → 二维码泊车投递。

订阅：
    /perception/qr_payload   std_msgs/String   二维码 JSON（自动下单）
    /mission/start           std_msgs/String   手动开始：路线名，或 JSON
                                              {"destination":"door_3"}
    /mission/reset           std_msgs/Bool     true 回到 IDLE
    /mission/nav_done        std_msgs/Bool     航点导航结果
    /mission/parked          std_msgs/Bool     泊车结果

发布：
    /mission/nav_start       std_msgs/String   发给 waypoint_nav 的路线名
    /mission/stop            std_msgs/Bool     取消导航
    /control/parking_enable  std_msgs/Bool     启动/停止泊车
    /control/line_enable     std_msgs/Bool     任务期间暂停/恢复巡线
    /control/nav_active      std_msgs/Bool     导航阶段让 nav2 独占 /cmd_vel
    /mission/state           std_msgs/String   一行状态，HUD/视频用
    /mission/status          std_msgs/String   JSON 详细状态
    /mission/event           std_msgs/String   状态迁移事件（含订单信息）

对应功能：④⑨⑬ 的串联；① ② ③ 语音/多指令/语义按本次范围不实现。
"""

import json
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mission_manager.core import IDLE, MissionMachine


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        default_alias = os.path.expanduser(
            '~/smartcar_ws/src/smartcar_bringup/config/destinations.yaml')
        self.declare_parameter('auto_start_from_qr', True)
        self.declare_parameter('destinations_file', default_alias)
        self.declare_parameter('mission_timeout', 240.0)
        self.declare_parameter('status_rate', 1.0)
        # 仿真/局部演示没有 nav2 时置 true：二维码下单后直接进入泊车
        self.declare_parameter('skip_navigation', False)

        self.auto_start = bool(
            self.get_parameter('auto_start_from_qr').value)
        self.skip_navigation = bool(
            self.get_parameter('skip_navigation').value)
        self.alias_file = os.path.expanduser(
            str(self.get_parameter('destinations_file').value))
        self.machine = MissionMachine(
            timeout=float(self.get_parameter('mission_timeout').value))
        self.machine.state_since = time.monotonic()
        self.aliases = self._load_aliases()

        self._last_state = None
        self._last_status = 0.0
        self._initial_published = False

        self.pub_nav_start = self.create_publisher(
            String, '/mission/nav_start', 10)
        self.pub_stop = self.create_publisher(Bool, '/mission/stop', 10)
        self.pub_parking = self.create_publisher(
            Bool, '/control/parking_enable', 10)
        self.pub_line = self.create_publisher(
            Bool, '/control/line_enable', 10)
        self.pub_nav_active = self.create_publisher(
            Bool, '/control/nav_active', 10)
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_status = self.create_publisher(String, '/mission/status', 10)
        self.pub_event = self.create_publisher(String, '/mission/event', 10)

        self.create_subscription(
            String, '/perception/qr_payload', self.on_qr, 10)
        self.create_subscription(String, '/mission/start', self.on_start, 10)
        self.create_subscription(Bool, '/mission/reset', self.on_reset, 10)
        self.create_subscription(
            Bool, '/mission/nav_done', self.on_nav_done, 10)
        self.create_subscription(
            Bool, '/mission/parked', self.on_parked, 10)

        rate = max(float(self.get_parameter('status_rate').value), 0.2)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            '任务管理器启动：二维码自动下单=%s 超时=%.0fs 目的地别名=%d 条' % (
                self.auto_start, self.machine.timeout, len(self.aliases)))

    # ------------------------------------------------------------------
    def _load_aliases(self):
        if not os.path.exists(self.alias_file):
            return {}
        try:
            with open(self.alias_file, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            aliases = data.get('aliases') or {}
            return {str(k): str(v) for k, v in aliases.items()}
        except Exception as exc:
            self.get_logger().warning('目的地别名文件解析失败：%s' % exc)
            return {}

    def _resolve_route(self, destination):
        return self.aliases.get(destination, destination)

    def _publish_gates(self, line=None, nav=None):
        """line/nav 传 None 表示本次不改。"""
        if line is not None:
            self.pub_line.publish(Bool(data=bool(line)))
        if nav is not None:
            self.pub_nav_active.publish(Bool(data=bool(nav)))

    # ------------------------------------------------------------------
    def on_qr(self, msg):
        if not self.auto_start:
            return
        if self.machine.state != IDLE:
            self.get_logger().info('已有任务在执行，忽略新二维码')
            return

        raw = (msg.data or '').strip()
        if not raw:
            return
        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                data = {'raw': raw}
        except (ValueError, TypeError):
            data = {'raw': raw}

        destination = str(data.get('destination') or data.get('raw') or '')
        destination = destination.strip()
        if not destination:
            self.get_logger().warning('二维码没有目的地字段：%s' % raw)
            return

        self._begin(destination, data, source='qr')

    def on_start(self, msg):
        text = (msg.data or '').strip()
        if not text:
            return
        if self.machine.state != IDLE:
            self.get_logger().warning(
                '当前状态 %s，先发 /mission/reset 再开始' % self.machine.state)
            return

        order = {}
        destination = text
        if text.startswith('{'):
            try:
                data = json.loads(text)
                if isinstance(data, dict):
                    order = data
                    destination = str(
                        data.get('destination') or data.get('route') or '')
            except (ValueError, TypeError):
                destination = text
        if not destination:
            self.get_logger().warning('手动任务缺少目的地')
            return
        self._begin(destination, order, source='manual')

    def _begin(self, destination, order, source):
        route = self._resolve_route(destination)
        now = time.monotonic()
        if not self.machine.start(destination, route, order, now):
            return
        self._publish_gates(line=False, nav=not self.skip_navigation)
        self.pub_nav_start.publish(String(data=route))
        self._emit_state(force=True)
        self._emit_event('start', source=source)
        self.get_logger().info(
            '任务开始：%s -> 路线 %s（来源 %s）' % (
                destination, route, source))

        if self.skip_navigation:
            # 仿真/无 nav2 场景：跳过导航，直接开始泊车
            self.machine.nav_done(True, now)
            self.pub_parking.publish(Bool(data=True))
            self._emit_state(force=True)
            self._emit_event('nav_skipped')
            self.get_logger().info('skip_navigation=true，直接进入泊车')
        else:
            self.pub_parking.publish(Bool(data=False))

    # ------------------------------------------------------------------
    def on_nav_done(self, msg):
        now = time.monotonic()
        ok = bool(msg.data)
        if not self.machine.nav_done(ok, now):
            return
        self._publish_gates(nav=False)
        if ok:
            self.pub_parking.publish(Bool(data=True))
            self.get_logger().info('导航完成，开始泊车投递')
        else:
            self.pub_parking.publish(Bool(data=False))
            self.get_logger().error('导航失败')
        self._emit_state(force=True)
        self._emit_event('nav_done', ok=ok)

    def on_parked(self, msg):
        now = time.monotonic()
        ok = bool(msg.data)
        if not self.machine.parked(ok, now):
            return
        self.pub_parking.publish(Bool(data=False))
        self._publish_gates(nav=False)
        self._emit_state(force=True)
        self._emit_event('parked', ok=ok)
        if ok:
            self.get_logger().info('任务完成：已到达投递点')
        else:
            self.get_logger().error('泊车失败，任务终止')

    def on_reset(self, msg):
        if not msg.data:
            return
        self.pub_stop.publish(Bool(data=True))
        self.pub_parking.publish(Bool(data=False))
        # 复位=停车：巡线不自动恢复，防止车在投递点自己又动起来；
        # 要重新巡线请显式发 /control/line_enable true
        self._publish_gates(line=False, nav=False)
        self.machine.reset(time.monotonic())
        self._emit_state(force=True)
        self._emit_event('reset')
        self.get_logger().info('任务状态已复位')

    def tick(self):
        now = time.monotonic()
        if not self._initial_published:
            # 启动时把开关状态广播一次，晚启动的节点也能收到
            self._initial_published = True
            self._publish_gates(line=True, nav=False)

        if self.machine.tick(now):
            self.pub_stop.publish(Bool(data=True))
            self.pub_parking.publish(Bool(data=False))
            self._publish_gates(line=False, nav=False)
            self._emit_state(force=True)
            self._emit_event('timeout')
            self.get_logger().error('任务超时，已请求停车')

        self._emit_state(force=False)
        if now - self._last_status >= 1.0:
            self._last_status = now
            self._publish_status()

    # ------------------------------------------------------------------
    def _emit_state(self, force=False):
        state = self.machine.state
        if not force and state == self._last_state:
            return
        self._last_state = state
        mission = self.machine.current
        text = state
        if mission.route:
            text += ' ' + mission.route
        self.pub_state.publish(String(data=text))

    def _publish_status(self):
        mission = self.machine.current
        payload = {
            'state': self.machine.state,
            'destination': mission.destination,
            'route': mission.route,
            'message': mission.message,
            'order': mission.order,
            'elapsed': round(time.monotonic() - mission.started_at, 1)
            if mission.started_at else 0.0,
        }
        self.pub_status.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def _emit_event(self, kind, **extra):
        mission = self.machine.current
        payload = {
            'event': kind,
            'state': self.machine.state,
            'destination': mission.destination,
            'route': mission.route,
            'order': mission.order,
            'message': mission.message,
        }
        payload.update(extra)
        self.pub_event.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionManager()
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
