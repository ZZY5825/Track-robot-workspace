from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ground_segment = Node(
        package='track_robot_perception',
        executable='lidar_ground_segment_node',
        name='lidar_ground_segment_node',
        output='screen',
        parameters=[{
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'output_topic': LaunchConfiguration('ground_topic'),
            'method': 'ransac_plane',
            'ground_fit_max_range': LaunchConfiguration('max_range'),
            'ransac_distance_threshold': LaunchConfiguration(
                'ground_distance_threshold'),
            'process_every_n_clouds': 1,
            'debug_timing': LaunchConfiguration('debug_timing'),
        }],
    )

    human_segment = Node(
        package='track_robot_perception',
        executable='lidar_human_segment_node',
        name='lidar_human_segment_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('ground_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'max_sample_points': LaunchConfiguration('max_sample_points'),
            'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
            'cluster_min_samples': LaunchConfiguration('cluster_min_samples'),
            'min_cluster_points': LaunchConfiguration('min_cluster_points'),
            'min_human_height': LaunchConfiguration('min_human_height'),
            'max_human_height': LaunchConfiguration('max_human_height'),
            'max_human_width': LaunchConfiguration('max_human_width'),
            'min_verticality': LaunchConfiguration('min_verticality'),
            'human_color': LaunchConfiguration('human_color'),
            'process_every_n_clouds': LaunchConfiguration(
                'process_every_n_clouds'),
            'debug_timing': LaunchConfiguration('debug_timing'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument(
            'ground_topic', default_value='/lidar_ground_segmented_points'),
        DeclareLaunchArgument(
            'output_topic', default_value='/lidar_human_segmented_points'),
        DeclareLaunchArgument('min_range', default_value='0.5'),
        DeclareLaunchArgument('max_range', default_value='20.0'),
        DeclareLaunchArgument(
            'ground_distance_threshold', default_value='0.18'),
        DeclareLaunchArgument('voxel_size', default_value='0.08'),
        DeclareLaunchArgument('max_sample_points', default_value='25000'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.35'),
        DeclareLaunchArgument('cluster_min_samples', default_value='4'),
        DeclareLaunchArgument('min_cluster_points', default_value='8'),
        DeclareLaunchArgument('min_human_height', default_value='0.7'),
        DeclareLaunchArgument('max_human_height', default_value='2.4'),
        DeclareLaunchArgument('max_human_width', default_value='1.2'),
        DeclareLaunchArgument('min_verticality', default_value='0.55'),
        DeclareLaunchArgument('human_color', default_value='255,30,30'),
        DeclareLaunchArgument('process_every_n_clouds', default_value='1'),
        DeclareLaunchArgument('debug_timing', default_value='true'),
        ground_segment,
        human_segment,
    ])
