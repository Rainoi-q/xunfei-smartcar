from setuptools import setup

package_name = 'vision_detect'

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
    description='视觉感知：巡线偏移、红绿灯、二维码',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_node = vision_detect.vision_node:main',
        ],
    },
)
