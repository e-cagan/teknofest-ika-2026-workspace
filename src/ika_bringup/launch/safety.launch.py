"""
Güvenlik katmanı — her zaman çalışmalı.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    safety_share = get_package_share_directory('ika_safety')
    config = os.path.join(safety_share, 'config', 'safety_params.yaml')

    return LaunchDescription([
        Node(
            package='ika_safety',
            executable='heartbeat_monitor_node',
            name='heartbeat_monitor_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='ika_safety',
            executable='estop_relay_node',
            name='estop_relay_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='ika_safety',
            executable='speed_limiter_node',
            name='speed_limiter_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='ika_safety',
            executable='system_health_node',
            name='system_health_node',
            parameters=[config],
            output='screen',
        ),
    ])
