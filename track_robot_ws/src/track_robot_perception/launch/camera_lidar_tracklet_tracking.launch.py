from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tracklet_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('track_robot_lidar_tracking'),
                'launch',
                'lidar_tracklet_manager.launch.py',
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('tracklet_config_file'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'lidar_qos_reliability': LaunchConfiguration('lidar_qos_reliability'),
            'tracking_frame': LaunchConfiguration('tracking_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
        }.items(),
    )

    association = Node(
        package='track_robot_lidar_tracking',
        executable='selected_human_target_tracker_node',
        name='selected_human_target_tracker_node',
        output='screen',
        parameters=[LaunchConfiguration('association_config_file'), {
            'camera_target_topic': LaunchConfiguration('camera_target_topic'),
            'lidar_tracklets_topic': '/human_tracking/lidar_tracklets',
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'candidate_clusters_topic': '/human_tracking/lidar_candidate_clusters',
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'tracking_frame': LaunchConfiguration('tracking_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('lidar_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('camera_target_topic', default_value='/human_tracking/camera_target'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('camera_frame', default_value='zed_left_camera_optical_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('tracking_frame', default_value='track_robot_center'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument(
            'tracklet_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_lidar_tracking'),
                'config',
                'lidar_tracklets.yaml',
            ])),
        DeclareLaunchArgument(
            'association_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_lidar_tracking'),
                'config',
                'selected_target_tracker.yaml',
            ])),
        tracklet_manager,
        association,
    ])
