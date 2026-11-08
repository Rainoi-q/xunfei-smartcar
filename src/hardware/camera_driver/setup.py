from setuptools import setup

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SmartCar Team',
    maintainer_email='1830702190@qq.com',
    description='相机驱动节点：USB 摄像头 / 视频回放，支持动态曝光',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = camera_driver.camera_node:main',
        ],
    },
)
