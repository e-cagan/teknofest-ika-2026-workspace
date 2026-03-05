"""
Otonom Koşu (2. Koşu) — Tam Otonom Sürüş + Otonom Atış

hardware + perception + navigation + autonomy + targeting + safety + recorder + foxglove

Kullanım:
  ros2 launch ika_bringup autonomous_run.launch.py
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

    perception_config = os.path.join(perception_share, 'config', 'perception_params.yaml')
    navigation_config = os.path.join(navigation_share, 'config', 'navigation_params.yaml')
    autonomy_config = os.path.join(autonomy_share, 'config', 'autonomy_params.yaml')
    targeting_config = os.path.join(targeting_share, 'config', 'targeting_params.yaml')
    teleop_config = os.path.join(teleop_share, 'config', 'teleop_params.yaml')
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

    # ── cmd_vel_mux (otonom modda /cmd_vel_nav'ı yönlendirir) ──
    cmd_vel_mux = Node(
        package='ika_teleop',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        parameters=[teleop_config],
        output='screen',
    )

    # ── Perception ──
    stage_sign_detector = Node(
        package='ika_perception',
        executable='stage_sign_detector_node',
        name='stage_sign_detector_node',
        parameters=[perception_config],
        output='screen',
    )
    cone_detector = Node(
        package='ika_perception',
        executable='cone_detector_node',
        name='cone_detector_node',
        parameters=[perception_config],
        output='screen',
    )
    barrier_detector = Node(
        package='ika_perception',
        executable='barrier_detector_node',
        name='barrier_detector_node',
        parameters=[perception_config],
        output='screen',
    )
    sliding_obstacle_detector = Node(
        package='ika_perception',
        executable='sliding_obstacle_detector_node',
        name='sliding_obstacle_detector_node',
        parameters=[perception_config],
        output='screen',
    )
    target_detector = Node(
        package='ika_perception',
        executable='target_detector_node',
        name='target_detector_node',
        parameters=[perception_config],
        output='screen',
    )

    # ── Navigation ──
    path_follower = Node(
        package='ika_navigation',
        executable='path_follower_node',
        name='path_follower_node',
        parameters=[navigation_config],
        output='screen',
    )
    cone_avoidance = Node(
        package='ika_navigation',
        executable='cone_avoidance_node',
        name='cone_avoidance_node',
        parameters=[navigation_config],
        output='screen',
    )
    sliding_obstacle_planner = Node(
        package='ika_navigation',
        executable='sliding_obstacle_planner_node',
        name='sliding_obstacle_planner_node',
        parameters=[navigation_config],
        output='screen',
    )
    slope_controller = Node(
        package='ika_navigation',
        executable='slope_controller_node',
        name='slope_controller_node',
        parameters=[navigation_config],
        output='screen',
    )
    speed_controller = Node(
        package='ika_navigation',
        executable='speed_controller_node',
        name='speed_controller_node',
        parameters=[navigation_config],
        output='screen',
    )

    # ── Autonomy ──
    mission_controller = Node(
        package='ika_autonomy',
        executable='mission_controller_node',
        name='mission_controller_node',
        parameters=[autonomy_config],
        output='screen',
    )
    stage_manager = Node(
        package='ika_autonomy',
        executable='stage_manager_node',
        name='stage_manager_node',
        parameters=[autonomy_config],
        output='screen',
    )
    behavior_executor = Node(
        package='ika_autonomy',
        executable='behavior_executor_node',
        name='behavior_executor_node',
        parameters=[autonomy_config],
        output='screen',
    )

    # ── Targeting (otonom) ──
    auto_targeting = Node(
        package='ika_targeting',
        executable='auto_targeting_node',
        name='auto_targeting_node',
        parameters=[targeting_config],
        output='screen',
    )
    targeting_sequencer = Node(
        package='ika_targeting',
        executable='targeting_sequencer_node',
        name='targeting_sequencer_node',
        parameters=[targeting_config],
        output='screen',
    )

    # ── Recorder ──
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
        name='rosbridge_websocket',
        parameters=[
            os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')
        ],
        output='log',
    )

    return LaunchDescription([
        # Temel
        hardware,
        safety,
        cmd_vel_mux,
        # Algılama
        stage_sign_detector,
        cone_detector,
        barrier_detector,
        sliding_obstacle_detector,
        target_detector,
        # Navigasyon
        path_follower,
        cone_avoidance,
        sliding_obstacle_planner,
        slope_controller,
        speed_controller,
        # Otonomi
        mission_controller,
        stage_manager,
        behavior_executor,
        # Nişan
        auto_targeting,
        targeting_sequencer,
        # Kayıt
        video_recorder,
        bag_recorder,
        # UI
        foxglove_bridge,
    ])
