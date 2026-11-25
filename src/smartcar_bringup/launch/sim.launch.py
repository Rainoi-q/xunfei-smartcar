"""虚拟仿真一键启动：没有场地/没上电也能跑通整条链路。

和实车的唯一区别：
    - 用 virtual_sim 代替“厂家底盘 + 相机 + 雷达”，话题名完全相同；
    - 没有 nav2，所以 mission_manager 跳过导航直接进入泊车；
    - 泊车在仿真里用深度测距（正交视角下二维码高度占比不随距离变化）。

用法：
    ros2 launch smartcar_bringup sim.launch.py
    ros2 launch smartcar_bringup sim.launch.py start_web_video:=true
    ros2 launch smartcar_bringup sim.launch.py enable_mission:=false

查看：
    浏览器 http://<车IP>:8080/stream?topic=/hud/image   （start_web_video:=true）
    ros2 topic echo /mission/state
    ros2 topic echo /perception/qr_payload
    ros2 topic echo /control/parking_state
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    params_sim_file = LaunchConfiguration('params_sim_file')
    destinations_file = LaunchConfiguration('destinations_file')

    enable_line = LaunchConfiguration('enable_line')
    enable_parking = LaunchConfiguration('enable_parking')
    enable_mission = LaunchConfiguration('enable_mission')
    enable_hud = LaunchConfiguration('enable_hud')
    start_web_video = LaunchConfiguration('start_web_video')

    config_dir = PathJoinSubstitution(
        [FindPackageShare('smartcar_bringup'), 'config'])

    declared = [
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution(
                [config_dir, 'params.yaml']),
            description='全局参数文件'),
        DeclareLaunchArgument(
            'params_sim_file', default_value=PathJoinSubstitution(
                [config_dir, 'params_sim.yaml']),
            description='仿真专用覆盖（具名段，优先级高于通配覆盖）'),
        DeclareLaunchArgument(
            'destinations_file', default_value=PathJoinSubstitution(
                [config_dir, 'destinations.yaml']),
            description='二维码 destination → 路线名 的别名表'),
        DeclareLaunchArgument('enable_line', default_value='true'),
        DeclareLaunchArgument('enable_parking', default_value='true'),
        DeclareLaunchArgument('enable_mission', default_value='true'),
        DeclareLaunchArgument('enable_hud', default_value='true'),
        DeclareLaunchArgument('start_web_video', default_value='false'),
    ]

    sim = [
        Node(package='virtual_sim', executable='sim_node',
             name='virtual_sim', output='screen',
             # auto_loop 等演示参数在 params_sim.yaml 的 virtual_sim 段
             parameters=[params_file, params_sim_file]),
    ]

    perception = [
        Node(package='vision_detect', executable='vision_node',
             name='vision_detect', output='screen',
             parameters=[params_file]),
    ]

    control = [
        Node(package='obstacle_avoider', executable='obstacle_node',
             name='obstacle_avoider', output='screen',
             parameters=[params_file]),
        Node(package='cmd_vel_arbiter', executable='arbiter_node',
             name='cmd_vel_arbiter', output='screen',
             parameters=[params_file]),
        Node(package='line_follower', executable='line_follower_node',
             name='line_follower', output='screen',
             parameters=[params_file],
             condition=IfCondition(enable_line)),
        Node(package='parking', executable='parking_node',
             name='parking', output='screen',
             # 仿真专用标定（深度测距）写在 params_sim.yaml 的 parking 段：
             # launch dict 会变成 /** 通配段，压不过 params.yaml 的具名段
             parameters=[params_file, params_sim_file],
             condition=IfCondition(enable_parking)),
    ]

    mission = [
        Node(package='mission_manager', executable='mission_node',
             name='mission_manager', output='screen',
             # skip_navigation 在 params_sim.yaml 的 mission_manager 段
             parameters=[params_file, params_sim_file, {
                 'destinations_file': destinations_file,
             }],
             condition=IfCondition(enable_mission)),
    ]

    interaction = [
        Node(package='hud', executable='hud_node',
             name='hud', output='screen',
             parameters=[params_file],
             condition=IfCondition(enable_hud)),
        Node(package='web_video_server', executable='web_video_server',
             name='web_video_server', output='screen',
             condition=IfCondition(start_web_video)),
    ]

    return LaunchDescription(
        declared
        + [LogInfo(msg=(
            '[sim] 仿真启动：巡线 → 二维码下单 → 泊车 全自动。'
            '开浏览器看 /hud/image（start_web_video:=true）。'))]
        + sim + perception + control + mission + interaction
    )
