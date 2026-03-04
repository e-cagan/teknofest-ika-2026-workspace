from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ika_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cagan',
    maintainer_email='emincaganapaydin@gmail.com',
    description='Safety: heartbeat monitor, e-stop relay, speed limiter, system health',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_monitor_node = ika_safety.heartbeat_monitor_node:main',
            'estop_relay_node = ika_safety.estop_relay_node:main',
            'speed_limiter_node = ika_safety.speed_limiter_node:main',
            'system_health_node = ika_safety.system_health_node:main',
        ],
    },
)
