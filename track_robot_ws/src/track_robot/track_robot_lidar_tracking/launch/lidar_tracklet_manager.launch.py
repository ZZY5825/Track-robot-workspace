from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    manager = Node(
        package='track_robot_lidar_tracking',
        executable='lidar_tracklet_manager_node',
        name='lidar_tracklet_manager_node',
        output='screen',
        parameters=[config_file, {
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'lidar_qos_reliability': LaunchConfiguration('lidar_qos_reliability'),
            'tracking_frame': LaunchConfiguration('tracking_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_lidar_tracking'),
                'config',
                'lidar_tracklets.yaml',
            ])),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('lidar_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('tracking_frame', default_value='base_link'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        manager,
    ])
