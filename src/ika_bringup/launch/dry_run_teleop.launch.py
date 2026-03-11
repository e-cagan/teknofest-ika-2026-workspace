"""
Dry Run: Teleop Test

Donanım yok — fake sensörler + fake STM32 + klavye kontrol + Foxglove.
Klavye ile sürüş yap, Foxglove'da kameraları ve LiDAR'ı gör.

Kullanım:
  Terminal 1: ros2 launch ika_bringup dry_run_teleop.launch.py
  Terminal 2: ros2 run ika_teleop teleop_keyboard_node
  Foxglove:   ws://localhost:9090
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
    teleop_share = get_package_share_directory('ika_teleop')
    safety_share = get_package_share_directory('ika_safety')

    dry_config = os.path.join(bringup_share, 'config', 'dry_run_params.yaml')
    teleop_config = os.path.join(teleop_share, 'config', 'teleop_params.yaml')
    safety_config = os.path.join(safety_share, 'config', 'safety_params.yaml')

    # Robot description (TF)
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'description.launch.py')
        )
    )

    # Fake donanım
    fake_sensors = Node(
        package='ika_hardware', executable='fake_sensors_node',
        name='fake_sensors_node', parameters=[dry_config], output='screen',
    )
    fake_stm32 = Node(
        package='ika_hardware', executable='fake_stm32_node',
        name='fake_stm32_node', parameters=[dry_config], output='screen',
    )

    # Safety
    heartbeat = Node(
        package='ika_safety', executable='heartbeat_monitor_node',
        name='heartbeat_monitor_node', parameters=[safety_config], output='screen',
    )
    estop = Node(
        package='ika_safety', executable='estop_relay_node',
        name='estop_relay_node', parameters=[safety_config], output='screen',
    )
    speed_limiter = Node(
        package='ika_safety', executable='speed_limiter_node',
        name='speed_limiter_node', parameters=[safety_config], output='screen',
    )

    # Teleop (cmd_vel_mux)
    cmd_vel_mux = Node(
        package='ika_teleop', executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node', parameters=[teleop_config], output='screen',
    )

    # Foxglove bridge
    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')],
        output='log',
    )

    return LaunchDescription([
        description,
        fake_sensors,
        fake_stm32,
        heartbeat,
        estop,
        speed_limiter,
        cmd_vel_mux,
        foxglove,
    ])
