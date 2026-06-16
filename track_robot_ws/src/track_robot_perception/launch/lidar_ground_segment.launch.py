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
            'output_topic': LaunchConfiguration('output_topic'),
            'method': LaunchConfiguration('method'),
            'ground_z_threshold': LaunchConfiguration('ground_z_threshold'),
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
            'ground_fit_max_range': LaunchConfiguration('ground_fit_max_range'),
            'seed_grid_size': LaunchConfiguration('seed_grid_size'),
            'ransac_distance_threshold': LaunchConfiguration(
                'ransac_distance_threshold'),
            'ransac_max_iterations': LaunchConfiguration('ransac_max_iterations'),
            'ransac_min_seed_points': LaunchConfiguration(
                'ransac_min_seed_points'),
            'max_ground_tilt_deg': LaunchConfiguration('max_ground_tilt_deg'),
            'ground_color': LaunchConfiguration('ground_color'),
            'process_every_n_clouds': LaunchConfiguration('process_every_n_clouds'),
            'debug_timing': LaunchConfiguration('debug_timing'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument(
            'output_topic', default_value='/lidar_ground_segmented_points'),
        DeclareLaunchArgument('method', default_value='ransac_plane'),
        DeclareLaunchArgument('ground_z_threshold', default_value='-0.7'),
        DeclareLaunchArgument('min_range', default_value='0.0'),
        DeclareLaunchArgument('max_range', default_value='200.0'),
        DeclareLaunchArgument('ground_fit_max_range', default_value='20.0'),
        DeclareLaunchArgument('seed_grid_size', default_value='0.5'),
        DeclareLaunchArgument('ransac_distance_threshold', default_value='0.18'),
        DeclareLaunchArgument('ransac_max_iterations', default_value='120'),
        DeclareLaunchArgument('ransac_min_seed_points', default_value='30'),
        DeclareLaunchArgument('max_ground_tilt_deg', default_value='45.0'),
        DeclareLaunchArgument('ground_color', default_value='35,255,80'),
        DeclareLaunchArgument('process_every_n_clouds', default_value='1'),
        DeclareLaunchArgument('debug_timing', default_value='true'),
        ground_segment,
    ])
