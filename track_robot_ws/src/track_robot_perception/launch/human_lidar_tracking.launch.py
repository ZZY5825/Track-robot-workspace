from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('lidar_config_file')
    lidar_node = Node(
        package='track_robot_perception',
        executable='lidar_human_cluster_node',
        name='lidar_human_cluster_node',
        output='screen',
        parameters=[config_file, {
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'lidar_qos_reliability': LaunchConfiguration('lidar_qos_reliability'),
            'camera_target_topic': LaunchConfiguration('camera_target_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'fixed_frame': LaunchConfiguration('lidar_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
            'use_base_lidar_extrinsic_fallback': LaunchConfiguration(
                'use_base_lidar_extrinsic_fallback'),
            'base_lidar_extrinsic_parent_frame': LaunchConfiguration(
                'base_lidar_extrinsic_parent_frame'),
            'base_lidar_extrinsic_child_frame': LaunchConfiguration(
                'base_lidar_extrinsic_child_frame'),
            'base_lidar_extrinsic_x': LaunchConfiguration('base_lidar_extrinsic_x'),
            'base_lidar_extrinsic_y': LaunchConfiguration('base_lidar_extrinsic_y'),
            'base_lidar_extrinsic_z': LaunchConfiguration('base_lidar_extrinsic_z'),
            'base_lidar_extrinsic_yaw': LaunchConfiguration('base_lidar_extrinsic_yaw'),
            'base_lidar_extrinsic_pitch': LaunchConfiguration('base_lidar_extrinsic_pitch'),
            'base_lidar_extrinsic_roll': LaunchConfiguration('base_lidar_extrinsic_roll'),
            'use_camera_lidar_extrinsic_fallback': LaunchConfiguration(
                'use_camera_lidar_extrinsic_fallback'),
            'prefer_camera_lidar_extrinsic': LaunchConfiguration(
                'prefer_camera_lidar_extrinsic'),
            'camera_lidar_extrinsic_parent_frame': LaunchConfiguration(
                'camera_lidar_extrinsic_parent_frame'),
            'camera_lidar_extrinsic_child_frame': LaunchConfiguration(
                'camera_lidar_extrinsic_child_frame'),
            'camera_lidar_extrinsic_x': LaunchConfiguration('camera_lidar_extrinsic_x'),
            'camera_lidar_extrinsic_y': LaunchConfiguration('camera_lidar_extrinsic_y'),
            'camera_lidar_extrinsic_z': LaunchConfiguration('camera_lidar_extrinsic_z'),
            'camera_lidar_extrinsic_yaw': LaunchConfiguration('camera_lidar_extrinsic_yaw'),
            'camera_lidar_extrinsic_pitch': LaunchConfiguration('camera_lidar_extrinsic_pitch'),
            'camera_lidar_extrinsic_roll': LaunchConfiguration('camera_lidar_extrinsic_roll'),
            'output_topic': '/human_tracking/fused_target_state',
            'compat_output_topic': '/human_tracking/target_state',
            'target_points_topic': '/human_tracking/target_lidar_points',
            'search_gate_marker_topic': '/human_tracking/target_search_gate_marker',
            'fused_marker_topic': '/human_tracking/fused_target_marker',
            'debug_topic': '/human_tracking/lidar_target_debug',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'lidar_human_candidates.yaml',
            ])),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('lidar_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('camera_target_topic', default_value='/human_tracking/camera_target'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('lidar_frame', default_value='rslidar'),
        DeclareLaunchArgument('camera_frame', default_value='zed_left_camera_optical_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('use_base_lidar_extrinsic_fallback', default_value='true'),
        DeclareLaunchArgument('base_lidar_extrinsic_parent_frame', default_value='base_link'),
        DeclareLaunchArgument('base_lidar_extrinsic_child_frame', default_value='rslidar'),
        DeclareLaunchArgument('base_lidar_extrinsic_x', default_value='0.0'),
        DeclareLaunchArgument('base_lidar_extrinsic_y', default_value='0.0'),
        DeclareLaunchArgument('base_lidar_extrinsic_z', default_value='0.70'),
        DeclareLaunchArgument('base_lidar_extrinsic_yaw', default_value='0.0'),
        DeclareLaunchArgument('base_lidar_extrinsic_pitch', default_value='0.0'),
        DeclareLaunchArgument('base_lidar_extrinsic_roll', default_value='0.0'),
        DeclareLaunchArgument('use_camera_lidar_extrinsic_fallback', default_value='true'),
        DeclareLaunchArgument('prefer_camera_lidar_extrinsic', default_value='true'),
        DeclareLaunchArgument('camera_lidar_extrinsic_parent_frame', default_value='zed_camera_link'),
        DeclareLaunchArgument('camera_lidar_extrinsic_child_frame', default_value='rslidar'),
        DeclareLaunchArgument('camera_lidar_extrinsic_x', default_value='-0.27'),
        DeclareLaunchArgument('camera_lidar_extrinsic_y', default_value='0.0'),
        DeclareLaunchArgument('camera_lidar_extrinsic_z', default_value='0.08'),
        DeclareLaunchArgument('camera_lidar_extrinsic_yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_lidar_extrinsic_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_lidar_extrinsic_roll', default_value='0.0'),
        lidar_node,
    ])
