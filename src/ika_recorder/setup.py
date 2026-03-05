from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ika_recorder'

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
    description='Data recording: rosbag2 recorder, MP4 video recorder',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_recorder_node = ika_recorder.video_recorder_node:main',
            'bag_recorder_node = ika_recorder.bag_recorder_node:main',
        ],
    },
)
