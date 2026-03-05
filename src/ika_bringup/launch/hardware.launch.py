import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share = get_package_share_directory('ika_description')
    hardware_share = get_package_share_directory('ika_hardware')

    # ── Launch Arguments ──
    use_cameras_arg = DeclareLaunchArgument('use_cameras', default_value='false')
    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='false')
    use_stm32_arg = DeclareLaunchArgument('use_stm32', default_value='false')

    use_cameras = LaunchConfiguration('use_cameras')
    use_lidar = LaunchConfiguration('use_lidar')
    use_stm32 = LaunchConfiguration('use_stm32')

    # ── Robot Description (TF) — her zaman ──
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'description.launch.py')
        )
    )

    # ── STM32 Bridge — opsiyonel ──
    stm32_bridge = Node(
        package='ika_hardware',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        parameters=[
            os.path.join(hardware_share, 'config', 'stm32_params.yaml')
        ],
        output='screen',
        condition=IfCondition(use_stm32),
    )

    # ── Kameralar — opsiyonel ──
    front_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='front_camera',
        namespace='camera/front',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'front_camera_optical_link',
        }],
        remappings=[
            ('image_raw', '/camera/front/image_raw'),
            ('image_raw/compressed', '/camera/front/compressed'),
        ],
        output='log',
        condition=IfCondition(use_cameras),
    )

    rear_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='rear_camera',
        namespace='camera/rear',
        parameters=[{
            'video_device': '/dev/video2',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'rear_camera_optical_link',
        }],
        remappings=[
            ('image_raw', '/camera/rear/image_raw'),
            ('image_raw/compressed', '/camera/rear/compressed'),
        ],
        output='log',
        condition=IfCondition(use_cameras),
    )

    targeting_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='targeting_camera',
        namespace='camera/targeting',
        parameters=[{
            'video_device': '/dev/video4',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'targeting_camera_optical_link',
        }],
        remappings=[
            ('image_raw', '/camera/targeting/image_raw'),
            ('image_raw/compressed', '/camera/targeting/compressed'),
        ],
        output='log',
        condition=IfCondition(use_cameras),
    )

    # ── LiDAR — opsiyonel ──
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='log',
        condition=IfCondition(use_lidar),
    )

    return LaunchDescription([
        use_cameras_arg,
        use_lidar_arg,
        use_stm32_arg,
        description_launch,
        stm32_bridge,
        front_camera,
        rear_camera,
        targeting_camera,
        lidar,
    ])
