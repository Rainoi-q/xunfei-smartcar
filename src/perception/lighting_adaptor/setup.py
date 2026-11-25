from setuptools import setup

package_name = 'lighting_adaptor'

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
    description='光照强弱识别与摄像机曝光自适应',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lighting_node = lighting_adaptor.lighting_node:main',
        ],
    },
)
