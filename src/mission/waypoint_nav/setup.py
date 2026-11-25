from setuptools import setup

package_name = 'waypoint_nav'

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
    description='nav2 航点导航与路线执行',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_node = waypoint_nav.waypoint_node:main',
            'record_waypoint = waypoint_nav.record_waypoint:main',
        ],
    },
)
