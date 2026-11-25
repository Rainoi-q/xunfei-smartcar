from setuptools import setup

package_name = 'parking'

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
    description='二维码引导的定点泊车与精准停靠',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'parking_node = parking.parking_node:main',
        ],
    },
)
