"""无车阶段联调：视频回放 → 视觉感知 → 巡线控制。

适用于还没有拿到车的阶段，用一段道路视频验证整条链路是否打通。

用法：
    ros2 launch smartcar_bringup demo.launch.py video_path:=/home/xxx/line.mp4

另开终端观察：
    ros2 topic echo /perception/line
    ros2 topic echo /perception/traffic_light
    ros2 topic hz /camera/image_raw
    ros2 run rqt_image_view rqt_image_view /perception/debug_image
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

        LogInfo(msg='[demo] 启动视频回放 + 视觉感知 + 巡线控制'),

        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera_driver',
            output='screen',
            parameters=[params_file, {
                'source': 'video',
                'video_path': video_path,
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

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
