"""
Manuel Koşu (1. Koşu) — Uzaktan Kontrollü

hardware + teleop + targeting_manual + safety + recorder + foxglove

Kullanım:
  ros2 launch ika_bringup manual_run.launch.py
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
    recorder_share = get_package_share_directory('ika_recorder')
    autonomy_share = get_package_share_directory('ika_autonomy')

    # ── Hardware ──
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'hardware.launch.py')
        )
    )

    # ── Safety ──
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'safety.launch.py')
        )
    )

    # ── Teleop ──
    teleop_config = os.path.join(teleop_share, 'config', 'teleop_params.yaml')

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

    # ── Mission Controller (mod yönetimi, timer, pas hakkı) ──
    autonomy_config = os.path.join(autonomy_share, 'config', 'autonomy_params.yaml')

    mission_controller = Node(
        package='ika_autonomy',
        executable='mission_controller_node',
        name='mission_controller_node',
        parameters=[autonomy_config],
        output='screen',
    )

    # ── Targeting Sequencer (manuel atış sekansı) ──
    targeting_share = get_package_share_directory('ika_targeting')
    targeting_config = os.path.join(targeting_share, 'config', 'targeting_params.yaml')

    targeting_sequencer = Node(
        package='ika_targeting',
        executable='targeting_sequencer_node',
        name='targeting_sequencer_node',
        parameters=[targeting_config],
        output='screen',
    )

    # ── Recorder ──
    recorder_config = os.path.join(recorder_share, 'config', 'recorder_params.yaml')

    video_recorder = Node(
        package='ika_recorder',
        executable='video_recorder_node',
        name='video_recorder_node',
        parameters=[recorder_config],
        output='screen',
    )

    bag_recorder = Node(
        package='ika_recorder',
        executable='bag_recorder_node',
        name='bag_recorder_node',
        parameters=[recorder_config],
        output='screen',
    )

    # ── Foxglove Bridge ──
    foxglove_bridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='foxglove_bridge',
        parameters=[
            os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')
        ],
        output='log',
    )

    return LaunchDescription([
        hardware,
        safety,
        joy_driver,
        teleop_joy,
        cmd_vel_mux,
        mission_controller,
        targeting_sequencer,
        video_recorder,
        bag_recorder,
        foxglove_bridge,
    ])
