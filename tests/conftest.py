"""让测试可以直接 import 各 ROS 包的“纯逻辑”模块（core.py）。

本地电脑（Windows / 无 ROS）上运行：
    python -m pytest tests -q

测试只覆盖不依赖 rclpy / cv2 的 core 模块；ROS 节点请上车用
    ros2 launch smartcar_bringup sim.launch.py
做集成验证。
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PACKAGE_DIRS = [
    os.path.join(ROOT, 'src', 'hardware', 'camera_driver'),
    os.path.join(ROOT, 'src', 'perception', 'vision_detect'),
    os.path.join(ROOT, 'src', 'perception', 'lighting_adaptor'),
    os.path.join(ROOT, 'src', 'perception', 'ocr_reader'),
    os.path.join(ROOT, 'src', 'control', 'line_follower'),
    os.path.join(ROOT, 'src', 'control', 'obstacle_avoider'),
    os.path.join(ROOT, 'src', 'control', 'cmd_vel_arbiter'),
    os.path.join(ROOT, 'src', 'control', 'parking'),
    os.path.join(ROOT, 'src', 'mission', 'waypoint_nav'),
    os.path.join(ROOT, 'src', 'mission', 'mission_manager'),
    os.path.join(ROOT, 'src', 'interaction', 'hud'),
    os.path.join(ROOT, 'src', 'simulation', 'virtual_sim'),
]

for path in PACKAGE_DIRS:
    if path not in sys.path:
        sys.path.insert(0, path)
