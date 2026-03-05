"""
Donanım Testi — sadece hardware + teleop + safety + foxglove.
Algılama/navigasyon/otonomi yok. Joystick ile sürüş testi için.

Kullanım:
  ros2 launch ika_bringup test_hardware.launch.py
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

    teleop_config = os.path.join(teleop_share, 'config', 'teleop_params.yaml')

    joy_driver = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 20.0}],
        output='log',
    )
    teleop_joy = Node(
        package='ika_teleop', executable='teleop_joy_node',
        name='teleop_joy_node', parameters=[teleop_config], output='screen',
    )
    cmd_vel_mux = Node(
        package='ika_teleop', executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node', parameters=[teleop_config], output='screen',
    )

    foxglove_bridge = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[os.path.join(bringup_share, 'config', 'foxglove_bridge.yaml')],
        output='log',
    )

    return LaunchDescription([
        hardware, safety,
        joy_driver, teleop_joy, cmd_vel_mux,
        foxglove_bridge,
    ])
