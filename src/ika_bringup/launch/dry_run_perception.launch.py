"""
Dry Run: Perception Test

Fake kameralar + fake LiDAR + tüm perception node'ları.
Algılama pipeline'ının çalıştığını doğrula.

Kullanım:
  ros2 launch ika_bringup dry_run_perception.launch.py
  Foxglove: ws://localhost:9090
  
  Kontrol et:
    ros2 topic echo /perception/stage_info
    ros2 topic echo /perception/cones
    ros2 topic echo /perception/lane_center
    ros2 topic echo /perception/target
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share = get_package_share_directory('ika_description')
    bringup_share = get_package_share_directory('ika_bringup')
    perception_share = get_package_share_directory('ika_perception')

    dry_config = os.path.join(bringup_share, 'config', 'dry_run_params.yaml')
    perc_config = os.path.join(perception_share, 'config', 'perception_params.yaml')

    # Robot description
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'description.launch.py')
        )
    )

    # Fake sensörler
    fake_sensors = Node(
        package='ika_hardware', executable='fake_sensors_node',
        name='fake_sensors_node',
        parameters=[dry_config, {'sim_cone_count': 5, 'sim_sliding_obstacle': True}],
        output='screen',
    )

    # Perception node'ları
    perception_nodes = [
        Node(package='ika_perception', executable=exe, name=exe,
             parameters=[perc_config], output='screen')
        for exe in [
            'stage_sign_detector_node',
            'cone_detector_node',
            'barrier_detector_node',
            'sliding_obstacle_detector_node',
            'target_detector_node',
        ]
    ]

    # Foxglove
    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')],
        output='log',
    )

    return LaunchDescription([
        description,
        fake_sensors,
        *perception_nodes,
        foxglove,
    ])
