from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    clustering = Node(
        package='track_robot_perception',
        executable='lidar_cluster_baseline_node',
        name='lidar_cluster_baseline_node',
        output='screen',
        parameters=[{
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'fixed_frame': LaunchConfiguration('fixed_frame'),
            'method': LaunchConfiguration('method'),
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
            'ground_z_threshold': LaunchConfiguration('ground_z_threshold'),
            'dbscan_eps': LaunchConfiguration('dbscan_eps'),
            'dbscan_min_samples': LaunchConfiguration('dbscan_min_samples'),
            'euclidean_tolerance': LaunchConfiguration('euclidean_tolerance'),
            'min_cluster_points': LaunchConfiguration('min_cluster_points'),
            'max_cluster_points': LaunchConfiguration('max_cluster_points'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'process_every_n_clouds': LaunchConfiguration('process_every_n_clouds'),
            'publish_clustered_cloud': LaunchConfiguration('publish_clustered_cloud'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('fixed_frame', default_value='rslidar'),
        DeclareLaunchArgument('method', default_value='dbscan'),
        DeclareLaunchArgument('min_range', default_value='0.5'),
        DeclareLaunchArgument('max_range', default_value='15.0'),
        DeclareLaunchArgument('ground_z_threshold', default_value='-0.7'),
        DeclareLaunchArgument('dbscan_eps', default_value='0.35'),
        DeclareLaunchArgument('dbscan_min_samples', default_value='8'),
        DeclareLaunchArgument('euclidean_tolerance', default_value='0.35'),
        DeclareLaunchArgument('min_cluster_points', default_value='20'),
        DeclareLaunchArgument('max_cluster_points', default_value='5000'),
        DeclareLaunchArgument('voxel_size', default_value='0.05'),
        DeclareLaunchArgument('process_every_n_clouds', default_value='2'),
        DeclareLaunchArgument('publish_clustered_cloud', default_value='false'),
        clustering,
    ])
