from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    fusion = Node(
        package='track_robot_perception',
        executable='target_fusion_node',
        name='target_fusion_node',
        output='screen',
        parameters=[config_file, {
            'camera_target_topic': '/human_tracking/camera_target',
            'lidar_candidates_topic': '/human_tracking/lidar_human_candidates',
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
            'output_topic': '/human_tracking/target_state',
            'marker_topic': '/human_tracking/target_marker',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'target_fusion.yaml',
            ])),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('camera_frame', default_value='zed_left_camera_optical_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        fusion,
    ])
