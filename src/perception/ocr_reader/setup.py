from setuptools import setup

package_name = 'ocr_reader'

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
    description='OCR 文字识别：预处理 + tesseract 离线识别',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ocr_node = ocr_reader.ocr_node:main',
        ],
    },
)
