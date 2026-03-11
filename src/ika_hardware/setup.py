from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ika_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Add additional directories
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cagan',
    maintainer_email='emincaganapaydin@gmail.com',
    description='Hardware drivers: STM32 UART bridge, cameras, IMU, LiDAR, laser gimbal',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge_node = ika_hardware.stm32_bridge_node:main',
            'fake_sensors_node = ika_hardware.fake_sensors_node:main',
            'fake_stm32_node = ika_hardware.fake_stm32_node:main',
        ],
    },
)