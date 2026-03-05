from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ika_perception'

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
    description='Perception: stage sign, cone, barrier, target, sliding obstacle detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stage_sign_detector_node = ika_perception.stage_sign_detector_node:main',
            'cone_detector_node = ika_perception.cone_detector_node:main',
            'barrier_detector_node = ika_perception.barrier_detector_node:main',
            'sliding_obstacle_detector_node = ika_perception.sliding_obstacle_detector_node:main',
            'target_detector_node = ika_perception.target_detector_node:main',
        ],
    },
)
