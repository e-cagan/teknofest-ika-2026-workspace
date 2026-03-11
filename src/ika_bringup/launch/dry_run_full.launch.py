"""
Dry Run: Tam Sistem

Tüm node'lar fake donanım ile. Otonom pipeline dahil.
Runtime'da mod geçişi yapılabilir.

Kullanım:
  Terminal 1: ros2 launch ika_bringup dry_run_full.launch.py
  Terminal 2: ros2 run ika_teleop teleop_keyboard_node
  Foxglove:   ws://localhost:9090

  Mod geçişi:
    ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 1}"  # MANUAL
    ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 2}"  # AUTO
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
    perception_share = get_package_share_directory('ika_perception')
    navigation_share = get_package_share_directory('ika_navigation')
    autonomy_share = get_package_share_directory('ika_autonomy')
    targeting_share = get_package_share_directory('ika_targeting')

    dry_config = os.path.join(bringup_share, 'config', 'dry_run_params.yaml')

    # Robot description
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'description.launch.py')
        )
    )

    # Fake donanım
    fake_sensors = Node(
        package='ika_hardware', executable='fake_sensors_node',
        name='fake_sensors_node',
        parameters=[dry_config, {'sim_cone_count': 5, 'sim_sliding_obstacle': True}],
        output='screen',
    )
    fake_stm32 = Node(
        package='ika_hardware', executable='fake_stm32_node',
        name='fake_stm32_node', parameters=[dry_config], output='screen',
    )

    # Safety
    safety_config = os.path.join(safety_share, 'config', 'safety_params.yaml')
    safety_nodes = [
        Node(package='ika_safety', executable=exe, name=exe,
             parameters=[safety_config], output='screen')
        for exe in [
            'heartbeat_monitor_node', 'estop_relay_node',
            'speed_limiter_node', 'system_health_node',
        ]
    ]

    # Teleop + mux
    teleop_config = os.path.join(teleop_share, 'config', 'teleop_params.yaml')
    cmd_vel_mux = Node(
        package='ika_teleop', executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node', parameters=[teleop_config], output='screen',
    )

    # Perception
    perc_config = os.path.join(perception_share, 'config', 'perception_params.yaml')
    perception_nodes = [
        Node(package='ika_perception', executable=exe, name=exe,
             parameters=[perc_config], output='screen')
        for exe in [
            'stage_sign_detector_node', 'cone_detector_node',
            'barrier_detector_node', 'sliding_obstacle_detector_node',
            'target_detector_node',
        ]
    ]

    # Navigation
    nav_config = os.path.join(navigation_share, 'config', 'navigation_params.yaml')
    navigation_nodes = [
        Node(package='ika_navigation', executable=exe, name=exe,
             parameters=[nav_config], output='screen')
        for exe in [
            'path_follower_node', 'cone_avoidance_node',
            'sliding_obstacle_planner_node', 'slope_controller_node',
            'speed_controller_node',
        ]
    ]

    # Autonomy
    auto_config = os.path.join(autonomy_share, 'config', 'autonomy_params.yaml')
    autonomy_nodes = [
        Node(package='ika_autonomy', executable=exe, name=exe,
             parameters=[auto_config], output='screen')
        for exe in [
            'mission_controller_node', 'stage_manager_node',
            'behavior_executor_node',
        ]
    ]

    # Targeting
    target_config = os.path.join(targeting_share, 'config', 'targeting_params.yaml')
    targeting_nodes = [
        Node(package='ika_targeting', executable=exe, name=exe,
             parameters=[target_config], output='screen')
        for exe in ['auto_targeting_node', 'targeting_sequencer_node']
    ]

    # Foxglove
    foxglove = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')],
        output='log',
    )

    return LaunchDescription([
        description,
        fake_sensors,
        fake_stm32,
        *safety_nodes,
        cmd_vel_mux,
        *perception_nodes,
        *navigation_nodes,
        *autonomy_nodes,
        *targeting_nodes,
        foxglove,
    ])
