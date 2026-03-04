from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ika_teleop'

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
    description='Teleoperation: joystick and keyboard control, cmd_vel mux',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_joy_node = ika_teleop.teleop_joy_node:main',
            'teleop_keyboard_node = ika_teleop.teleop_keyboard_node:main',
            'cmd_vel_mux_node = ika_teleop.cmd_vel_mux_node:main',
        ],
    },
)
