from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    manager = Node(
        package='track_robot_lidar_tracking',
        executable='lidar_tracklet_manager_node',
        name='semantic_memory_lidar_tracklet_manager',
        output='screen',
        parameters=[config_file, {
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'tracking_frame': LaunchConfiguration('tracking_frame'),
            'source_epoch_seed': ParameterValue(
                LaunchConfiguration('source_epoch_seed'), value_type=int),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_lidar_tracking'),
                'config',
                'semantic_memory_lidar_tracklets.yaml',
            ])),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('tracking_frame', default_value='base_link'),
        DeclareLaunchArgument('source_epoch_seed', default_value='0'),
        manager,
    ])
