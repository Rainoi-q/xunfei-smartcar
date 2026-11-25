"""实车一键启动：感知 + 控制仲裁 + 任务，全流程。

前置条件（WHEELTEC 出厂栈，先另开终端启动）：
    # 1) 底盘/雷达/相机：具体 launch 名以车上 ~/wheeltec_ros2 为准，例如
    #    ls ~/wheeltec_ros2/src  →  找到厂家 turn_on_* / bringup 包
    #    ros2 launch <厂家包> <厂家launch>.launch.py
    # 2) 需要导航时再启动厂家 nav2（地图已建好）：
    #    ros2 launch <nav2包> <nav2launch>.launch.py
    #    启动后确认：ros2 action list | grep navigate_to_pose
    # 注意：导航阶段 nav2 自己发 /cmd_vel，本工程的仲裁器会自动暂停输出。

本文件只启动“上层”节点，不重复启动底盘/相机驱动。

常用：
    ros2 launch smartcar_bringup bringup.launch.py
    ros2 launch smartcar_bringup bringup.launch.py enable_ocr:=false
    ros2 launch smartcar_bringup bringup.launch.py start_web_video:=true

启动后：
    # 看 HUD（需要 web_video_server 与上面话题都在线）
    ros2 topic hz /hud/image
    # 手动发任务（二维码也能自动触发）
    ros2 topic pub --once /mission/start std_msgs/msg/String "{data: door_3}"
    ros2 topic pub --once /mission/reset std_msgs/msg/Bool "{data: true}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    destinations_file = LaunchConfiguration('destinations_file')

    enable_line = LaunchConfiguration('enable_line')
    enable_parking = LaunchConfiguration('enable_parking')
    enable_ocr = LaunchConfiguration('enable_ocr')
    enable_lighting = LaunchConfiguration('enable_lighting')
    enable_hud = LaunchConfiguration('enable_hud')
    enable_mission = LaunchConfiguration('enable_mission')
    start_web_video = LaunchConfiguration('start_web_video')
    web_video_port = LaunchConfiguration('web_video_port')

    config_dir = PathJoinSubstitution(
        [FindPackageShare('smartcar_bringup'), 'config'])

    declared = [
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution(
                [config_dir, 'params.yaml']),
            description='全局参数文件'),
        DeclareLaunchArgument(
            'waypoints_file', default_value=PathJoinSubstitution(
                [config_dir, 'waypoints.yaml']),
            description='航点/路线文件'),
        DeclareLaunchArgument(
            'destinations_file', default_value=PathJoinSubstitution(
                [config_dir, 'destinations.yaml']),
            description='二维码 destination → 路线名 的别名表'),
        DeclareLaunchArgument('enable_line', default_value='true',
                              description='巡线控制'),
        DeclareLaunchArgument('enable_parking', default_value='true',
                              description='二维码泊车'),
        DeclareLaunchArgument('enable_ocr', default_value='true',
                              description='OCR 文字识别'),
        DeclareLaunchArgument('enable_lighting', default_value='true',
                              description='光照识别与曝光自适应'),
        DeclareLaunchArgument('enable_hud', default_value='true',
                              description='HUD 可视化'),
        DeclareLaunchArgument('enable_mission', default_value='true',
                              description='任务状态机'),
        DeclareLaunchArgument('start_web_video', default_value='false',
                              description='顺便启动 web_video_server'),
        DeclareLaunchArgument('web_video_port', default_value='8080',
                              description='web_video_server 端口'),
    ]

    # 感知：始终启动视觉（巡线偏移 + 二维码），其余可关
    perception = [
        Node(package='vision_detect', executable='vision_node',
             name='vision_detect', output='screen',
             parameters=[params_file]),
        Node(package='lighting_adaptor', executable='lighting_node',
             name='lighting_adaptor', output='screen',
             parameters=[params_file],
             condition=IfCondition(enable_lighting)),
        Node(package='ocr_reader', executable='ocr_node',
             name='ocr_reader', output='screen',
             parameters=[params_file],
             condition=IfCondition(enable_ocr)),
    ]

    # 控制：仲裁器必须启动，否则没有节点发布 /cmd_vel
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
             parameters=[params_file, {'auto_enable': False}],
             condition=IfCondition(enable_parking)),
    ]

    # 任务：航点导航 + 状态机
    mission = [
        Node(package='waypoint_nav', executable='waypoint_node',
             name='waypoint_nav', output='screen',
             parameters=[params_file, {'waypoints_file': waypoints_file}]),
        Node(package='mission_manager', executable='mission_node',
             name='mission_manager', output='screen',
             parameters=[params_file, {
                 'destinations_file': destinations_file,
                 'skip_navigation': False,
             }],
             condition=IfCondition(enable_mission)),
    ]

    # 交互：HUD + 可选网页视频
    interaction = [
        Node(package='hud', executable='hud_node',
             name='hud', output='screen',
             parameters=[params_file],
             condition=IfCondition(enable_hud)),
        Node(package='web_video_server', executable='web_video_server',
             name='web_video_server', output='screen',
             parameters=[{'port': LaunchConfiguration('web_video_port')}],
             condition=IfCondition(start_web_video)),
    ]

    return LaunchDescription(
        declared
        + [LogInfo(msg=(
            '[smartcar_bringup] 上层节点启动。请确认厂家底盘/相机/雷达已单独启动，'
            '并且 nav2 在导航时在线；通过 /mission/start 或二维码发起任务。'))]
        + perception + control + mission + interaction
    )
