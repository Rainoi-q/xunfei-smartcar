"""任务状态机（不依赖 ROS）。

IDLE --start--> NAVIGATING --nav_done--> DOCKING --parked--> DONE
                     |                      |
                     +------ 失败/超时 ------+--> FAILED
任意状态 --reset--> IDLE

本次不做语音与多指令，因此一个任务 = 一个目的地（来自二维码或手动话题）。
"""

from dataclasses import dataclass, field

IDLE = 'IDLE'
NAVIGATING = 'NAVIGATING'
DOCKING = 'DOCKING'
DONE = 'DONE'
FAILED = 'FAILED'


@dataclass
class Mission:
    destination: str = ''
    route: str = ''
    order: dict = field(default_factory=dict)
    started_at: float = 0.0
    message: str = ''


class MissionMachine:
    def __init__(self, timeout=180.0):
        self.timeout = max(10.0, float(timeout))
        self.state = IDLE
        self.current = Mission()
        self.state_since = 0.0

    def start(self, destination, route, order, now):
        if self.state in (NAVIGATING, DOCKING):
            return False
        self.current = Mission(destination=str(destination),
                               route=str(route),
                               order=dict(order or {}),
                               started_at=float(now),
                               message='started')
        self.state = NAVIGATING
        self.state_since = float(now)
        return True

    def nav_done(self, ok, now):
        if self.state != NAVIGATING:
            return False
        if ok:
            self.state = DOCKING
            self.current.message = 'arrived at dock area'
        else:
            self.state = FAILED
            self.current.message = 'navigation failed'
        self.state_since = float(now)
        return True

    def parked(self, ok, now):
        if self.state != DOCKING:
            return False
        self.state = DONE if ok else FAILED
        self.current.message = 'delivered' if ok else 'docking failed'
        self.state_since = float(now)
        return True

    def tick(self, now):
        """超时检查，返回是否发生超时。"""
        if self.state not in (NAVIGATING, DOCKING):
            return False
        if now - self.current.started_at > self.timeout:
            self.state = FAILED
            self.current.message = 'mission timeout'
            self.state_since = float(now)
            return True
        return False

    def reset(self, now):
        self.state = IDLE
        self.current = Mission()
        self.state_since = float(now)
