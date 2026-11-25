from setuptools import setup

package_name = 'virtual_sim'

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
    description='二维虚拟仿真：合成相机、雷达、里程计，可跑巡线/避障',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_node = virtual_sim.sim_node:main',
        ],
    },
)
