"""Astra Pro 彩色流兜底：用 usb_cam 直接读 UVC 彩色设备。

背景：出厂 `wheeltec_sensors.launch.py` 里相机用的是 Astra_S 配置，
本车（Astra Pro，彩色是独立的 UVC 设备 2bc5:0501）经常拿不到彩色帧，
但 UVC 设备本身是好的（`v4l2-ctl -d /dev/video2 --stream-mmap` 能出帧）。
本 launch 用 usb_cam + yuyv 补一路 `/camera/color/image_raw`，
深度仍由出厂 astra 节点提供，下游节点（vision_detect/hud/parking/ocr/
lighting）完全不用改。

用法：
    ros2 launch smartcar_bringup color_usbcam.launch.py
    ros2 launch smartcar_bringup color_usbcam.launch.py video_device:=/dev/video2

设备号确认：
    v4l2-ctl --list-devices        # 找 "Astra Pro HD Camera" 对应的 /dev/videoN
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('pixel_format')

    declared = [
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video2',
            description='Astra 彩色 UVC 设备（v4l2-ctl --list-devices 查）'),
        DeclareLaunchArgument(
            'pixel_format', default_value='yuyv',
            description='yuyv（本车实测可用）/ mjpeg2rgb（本车会 abort，别用）'),
    ]

    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': pixel_format,
        }],
        remappings=[('image_raw', '/camera/color/image_raw')])

    return LaunchDescription(declared + [camera])
