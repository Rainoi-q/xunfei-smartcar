from setuptools import setup

package_name = 'cmd_vel_arbiter'

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
    description='速度指令仲裁：安全优先、巡线/泊车切换',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arbiter_node = cmd_vel_arbiter.arbiter_node:main',
        ],
    },
)
