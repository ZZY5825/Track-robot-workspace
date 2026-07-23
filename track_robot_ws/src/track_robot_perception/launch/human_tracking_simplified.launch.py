from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package_name, launch_name, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(package_name), 'launch', launch_name])
        ]),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    lidar_topic = LaunchConfiguration('lidar_topic')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')

    camera = include('track_robot_perception', 'human_camera_tracking.launch.py', {
        'image_topic': image_topic,
        'tracker_backend': LaunchConfiguration('tracker_backend'),
        'model_path': LaunchConfiguration('model_path'),
        'resize_width': LaunchConfiguration('resize_width'),
        'camera_config_file': LaunchConfiguration('camera_config_file'),
    })

    lidar_fusion = include('track_robot_perception', 'camera_lidar_tracklet_tracking.launch.py', {
        'lidar_topic': lidar_topic,
        'lidar_qos_reliability': LaunchConfiguration('lidar_qos_reliability'),
        'camera_target_topic': '/human_tracking/camera_target',
        'camera_info_topic': camera_info_topic,
        'camera_frame': camera_frame,
        'base_frame': base_frame,
        'tracking_frame': LaunchConfiguration('tracking_frame'),
        'map_frame': map_frame,
        'tracklet_config_file': LaunchConfiguration('tracklet_config_file'),
        'association_config_file': LaunchConfiguration('association_config_file'),
    })

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('camera_frame', default_value='zed_left_camera_optical_frame'),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('lidar_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('tracking_frame', default_value='track_robot_center'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('tracker_backend', default_value='bytetrack'),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'human_tracking.yaml',
            ])),
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
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/track-robot/track_robot_ws/models/human_tracking/yolov8n-pose.pt'),
        DeclareLaunchArgument('resize_width', default_value='960'),
        camera,
        lidar_fusion,
    ])
