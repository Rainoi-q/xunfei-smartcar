"""智能车一键启动（骨架）。

启动策略：
  - 默认只打印启动信息，不拉起任何节点，保证空仓库也能启动成功。
  - 各成员完成自己的节点后，把对应的 Node(...) 取消注释即可。
  - 用 use_hardware 区分「仿真」与「真车」，用 use_debug 控制调试可视化。

用法：
  ros2 launch smartcar_bringup bringup.launch.py
  ros2 launch smartcar_bringup bringup.launch.py use_hardware:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    use_debug = LaunchConfiguration('use_debug')
    params_file = LaunchConfiguration('params_file')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_hardware', default_value='false',
            description='true=连接真车硬件，false=使用 Gazebo 仿真'),
        DeclareLaunchArgument(
            'use_debug', default_value='true',
            description='是否启动 RViz2 与调试 HUD'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('smartcar_bringup'), 'config', 'params.yaml']),
            description='全局参数文件'),
    ]

    # ---------------------------------------------------------------
    # 硬件层：选车后由 A/C 实现后取消注释，并按实际包名/可执行文件修改
    # ---------------------------------------------------------------
    hardware_nodes = [
        # Node(package='camera_driver', executable='camera_driver_node',
        #      name='camera_driver', output='screen',
        #      parameters=[params_file], condition=IfCondition(use_hardware)),
        # Node(package='base_driver', executable='base_driver_node',
        #      name='base_driver', output='screen',
        #      parameters=[params_file], condition=IfCondition(use_hardware)),
        # Node(package='range_driver', executable='range_driver_node',
        #      name='range_driver', output='screen',
        #      parameters=[params_file], condition=IfCondition(use_hardware)),
    ]

    # ---------------------------------------------------------------
    # 感知层：B 负责
    # ---------------------------------------------------------------
    perception_nodes = [
        # Node(package='vision_detect', executable='vision_detect_node',
        #      name='vision_detect', output='screen', parameters=[params_file]),
        # Node(package='lighting_adaptor', executable='lighting_adaptor_node',
        #      name='lighting_adaptor', output='screen', parameters=[params_file]),
    ]

    # ---------------------------------------------------------------
    # 控制层：C 负责
    # ---------------------------------------------------------------
    control_nodes = [
        # Node(package='line_follower', executable='line_follower_node',
        #      name='line_follower', output='screen', parameters=[params_file]),
        # Node(package='obstacle_avoider', executable='obstacle_avoider_node',
        #      name='obstacle_avoider', output='screen', parameters=[params_file]),
        # Node(package='parking', executable='parking_node',
        #      name='parking', output='screen', parameters=[params_file]),
    ]

    # ---------------------------------------------------------------
    # 交互层：D 负责
    # ---------------------------------------------------------------
    interaction_nodes = [
        # Node(package='voice_input', executable='voice_input_node',
        #      name='voice_input', output='screen', parameters=[params_file]),
        # Node(package='nlp_planner', executable='nlp_planner_node',
        #      name='nlp_planner', output='screen', parameters=[params_file]),
    ]

    # ---------------------------------------------------------------
    # 任务层：A 负责
    # ---------------------------------------------------------------
    mission_nodes = [
        # Node(package='mission_manager', executable='mission_manager_node',
        #      name='mission_manager', output='screen', parameters=[params_file]),
    ]

    # 延迟 3 秒启动上层节点，给驱动层留出设备初始化时间
    delayed = TimerAction(period=3.0, actions=(
        perception_nodes + control_nodes + interaction_nodes + mission_nodes))

    return LaunchDescription(
        declared_arguments
        + [LogInfo(msg='[smartcar_bringup] 骨架启动：当前未启用任何节点，请在 launch 中取消注释。')]
        + hardware_nodes
        + [delayed]
    )
