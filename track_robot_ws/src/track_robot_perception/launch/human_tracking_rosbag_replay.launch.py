from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_config = PathJoinSubstitution([
        FindPackageShare('track_robot_perception'), 'config', 'human_tracking.yaml'])
    tracklet_config = PathJoinSubstitution([
        FindPackageShare('track_robot_lidar_tracking'), 'config', 'lidar_tracklets.yaml'])
    selected_config = PathJoinSubstitution([
        FindPackageShare('track_robot_lidar_tracking'), 'config', 'selected_target_tracker.yaml'])

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'launch',
                'human_camera_tracking.launch.py',
            ])
        ]),
        launch_arguments={
            'camera_config_file': camera_config,
            'image_topic': '/zed/zed_node/left/image_rect_color',
            'tracker_backend': 'bytetrack',
            'model_path': '/home/track-robot/track_robot_ws/models/human_tracking/yolov8n-pose.pt',
            'resize_width': '960',
        }.items(),
    )

    tracklets = Node(
        package='track_robot_lidar_tracking',
        executable='lidar_tracklet_manager_node',
        name='lidar_tracklet_manager_node',
        output='screen',
        parameters=[{
            'lidar_topic': '/rslidar_points',
            'lidar_qos_reliability': 'reliable',
            'tracking_frame_override': 'base_link',
            'base_frame': 'base_link',
        }, tracklet_config],
    )

    selected_target = Node(
        package='track_robot_lidar_tracking',
        executable='selected_human_target_tracker_node',
        name='selected_human_target_tracker_node',
        output='screen',
        parameters=[{
            'camera_target_topic': '/human_tracking/camera_target',
            'lidar_tracklets_topic': '/human_tracking/lidar_tracklets',
            'lidar_topic': '/rslidar_points',
            'candidate_clusters_topic': '/human_tracking/lidar_candidate_clusters',
            'camera_info_topic': '/zed/zed_node/left/camera_info',
            'camera_frame': 'zed_left_camera_optical_frame',
            'tracking_frame_override': 'base_link',
            'base_frame': 'base_link',
        }, selected_config],
    )

    return LaunchDescription([camera, tracklets, selected_target])
