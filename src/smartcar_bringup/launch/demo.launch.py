"""无车阶段联调：视频回放 → 视觉感知 → 巡线控制 → 仲裁输出。

用一段道路视频验证整条链路是否打通；话题名与实车/仿真完全一致，
换成真实相机只需把 camera_driver 换成厂家相机 launch。

用法：
    ros2 launch smartcar_bringup demo.launch.py video_path:=/home/xxx/line.mp4

另开终端观察：
    ros2 topic echo /perception/line
    ros2 topic echo /perception/qr_payload
    ros2 topic echo /control/arbiter_state
    ros2 run rqt_image_view rqt_image_view /perception/debug_image
    ros2 run rqt_image_view rqt_image_view /hud/image
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_path = LaunchConfiguration('video_path')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_hud = LaunchConfiguration('use_hud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path', default_value='',
            description='用于回放的视频文件绝对路径'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('smartcar_bringup'), 'config', 'params.yaml']),
            description='全局参数文件'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否启动 RViz2（一般看 debug_image 更方便）'),
        DeclareLaunchArgument(
            'use_hud', default_value='true',
            description='是否启动 HUD'),

        LogInfo(msg='[demo] 视频回放 + 视觉感知 + 巡线控制 + 速度仲裁'),

        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera_driver',
            output='screen',
            parameters=[params_file, {
                'source': 'video',
                'video_path': video_path,
                'image_topic': '/camera/color/image_raw',
            }],
        ),

        Node(
            package='vision_detect',
            executable='vision_node',
            name='vision_detect',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='line_follower',
            executable='line_follower_node',
            name='line_follower',
            output='screen',
            parameters=[params_file],
        ),

        # 没有仲裁器时 /cmd_vel 不会有任何输出，这是刻意的安全设计
        Node(
            package='cmd_vel_arbiter',
            executable='arbiter_node',
            name='cmd_vel_arbiter',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='hud',
            executable='hud_node',
            name='hud',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_hud),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
