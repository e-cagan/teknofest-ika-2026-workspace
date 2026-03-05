from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ika_navigation'

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
    description='Autonomous navigation: barrier following, cone avoidance, slope control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_follower_node = ika_navigation.path_follower_node:main',
            'cone_avoidance_node = ika_navigation.cone_avoidance_node:main',
            'sliding_obstacle_planner_node = ika_navigation.sliding_obstacle_planner_node:main',
            'slope_controller_node = ika_navigation.slope_controller_node:main',
            'speed_controller_node = ika_navigation.speed_controller_node:main',
        ],
    },
)
