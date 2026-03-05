"""
Tam Sistem — Manuel + Otonom tüm node'lar.
Runtime'da SetMode servisi ile mod geçişi yapılır.

Kullanım:
  ros2 launch ika_bringup full_system.launch.py

  # Sonra Foxglove'dan veya terminalden:
  ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 1}"  # MANUAL
  ros2 service call /system/set_mode ika_msgs/srv/SetMode "{requested_mode: 2}"  # AUTONOMOUS
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('ika_bringup')
    teleop_share = get_package_share_directory('ika_teleop')
    perception_share = get_package_share_directory('ika_perception')
    navigation_share = get_package_share_directory('ika_navigation')
    autonomy_share = get_package_share_directory('ika_autonomy')
    targeting_share = get_package_share_directory('ika_targeting')
    recorder_share = get_package_share_directory('ika_recorder')

    teleop_config = os.path.join(teleop_share, 'config', 'teleop_params.yaml')
    perception_config = os.path.join(perception_share, 'config', 'perception_params.yaml')
    navigation_config = os.path.join(navigation_share, 'config', 'navigation_params.yaml')
    autonomy_config = os.path.join(autonomy_share, 'config', 'autonomy_params.yaml')
    targeting_config = os.path.join(targeting_share, 'config', 'targeting_params.yaml')
    recorder_config = os.path.join(recorder_share, 'config', 'recorder_params.yaml')

    # ── Hardware + Safety ──
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'hardware.launch.py')
        )
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'safety.launch.py')
        )
    )

    # ── Teleop (manuel koşu için) ──
    joy_driver = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 20.0}],
        output='log',
    )
    teleop_joy = Node(
        package='ika_teleop',
        executable='teleop_joy_node',
        name='teleop_joy_node',
        parameters=[teleop_config],
        output='screen',
    )
    cmd_vel_mux = Node(
        package='ika_teleop',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        parameters=[teleop_config],
        output='screen',
    )

    # ── Perception ──
    perception_nodes = [
        Node(package='ika_perception', executable=exe, name=exe,
             parameters=[perception_config], output='screen')
        for exe in [
            'stage_sign_detector_node',
            'cone_detector_node',
            'barrier_detector_node',
            'sliding_obstacle_detector_node',
            'target_detector_node',
        ]
    ]

    # ── Navigation ──
    navigation_nodes = [
        Node(package='ika_navigation', executable=exe, name=exe,
             parameters=[navigation_config], output='screen')
        for exe in [
            'path_follower_node',
            'cone_avoidance_node',
            'sliding_obstacle_planner_node',
            'slope_controller_node',
            'speed_controller_node',
        ]
    ]

    # ── Autonomy ──
    autonomy_nodes = [
        Node(package='ika_autonomy', executable=exe, name=exe,
             parameters=[autonomy_config], output='screen')
        for exe in [
            'mission_controller_node',
            'stage_manager_node',
            'behavior_executor_node',
        ]
    ]

    # ── Targeting ──
    targeting_nodes = [
        Node(package='ika_targeting', executable=exe, name=exe,
             parameters=[targeting_config], output='screen')
        for exe in [
            'auto_targeting_node',
            'targeting_sequencer_node',
        ]
    ]

    # ── Recorder ──
    video_recorder = Node(
        package='ika_recorder', executable='video_recorder_node',
        name='video_recorder_node', parameters=[recorder_config], output='screen',
    )
    bag_recorder = Node(
        package='ika_recorder', executable='bag_recorder_node',
        name='bag_recorder_node', parameters=[recorder_config], output='screen',
    )

    # ── Foxglove ──
    foxglove_bridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')],
        output='log',
    )

    return LaunchDescription([
        hardware,
        safety,
        joy_driver,
        teleop_joy,
        cmd_vel_mux,
        *perception_nodes,
        *navigation_nodes,
        *autonomy_nodes,
        *targeting_nodes,
        video_recorder,
        bag_recorder,
        foxglove_bridge,
    ])
