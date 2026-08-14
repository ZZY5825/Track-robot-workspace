from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _profile_parameters(context, overrides):
    parameters = []
    profile_config = LaunchConfiguration('profile_config').perform(context)
    if profile_config:
        parameters.append(profile_config)
    parameters.append(overrides)
    return parameters


def _launch_nodes(context):
    controller = Node(
        package='track_robot_control',
        executable='target_follow_controller_node',
        name='target_follow_controller_node',
        output='screen',
        parameters=_profile_parameters(context, {
            'decision_topic': LaunchConfiguration('decision_topic'),
            'rc_state_topic': LaunchConfiguration('rc_state_topic'),
            'debug_cmd_vel_topic': LaunchConfiguration('debug_cmd_vel_topic'),
            'planned_cmd_vel_topic': LaunchConfiguration('planned_cmd_vel_topic'),
            'debug_text_topic': LaunchConfiguration('debug_text_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'marker_frame': LaunchConfiguration('marker_frame'),
            'output_topic': LaunchConfiguration('output_topic'),
            'enable_service': LaunchConfiguration('enable_service'),
            'disable_service': LaunchConfiguration('disable_service'),
            'enable_cmd_vel': LaunchConfiguration('enable_cmd_vel'),
            'follow_distance': LaunchConfiguration('follow_distance'),
            'deadband_distance': LaunchConfiguration('deadband_distance'),
            'max_linear_x': LaunchConfiguration('max_linear_x'),
            'max_angular_z': LaunchConfiguration('max_angular_z'),
            'linear_gain': LaunchConfiguration('linear_gain'),
            'angular_gain': LaunchConfiguration('angular_gain'),
            'min_confidence': LaunchConfiguration('min_confidence'),
            'target_timeout_sec': LaunchConfiguration('target_timeout_sec'),
            'front_cone_rad': LaunchConfiguration('front_cone_rad'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'linear_accel_limit': LaunchConfiguration('linear_accel_limit'),
            'angular_accel_limit': LaunchConfiguration('angular_accel_limit'),
            'lidar_only_no_motion_distance': LaunchConfiguration('lidar_only_no_motion_distance'),
            'rc_override_deadband': LaunchConfiguration('rc_override_deadband'),
            'require_gesture_relock_after_rc_override': LaunchConfiguration(
                'require_gesture_relock_after_rc_override'),
            'allow_lidar_only_forward_motion': LaunchConfiguration(
                'allow_lidar_only_forward_motion'),
        }),
    )
    return [controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('decision_topic', default_value='/follow/decision'),
        DeclareLaunchArgument('rc_state_topic', default_value='/bunker_rc_state'),
        DeclareLaunchArgument('debug_cmd_vel_topic', default_value='/follow/cmd_vel_debug'),
        DeclareLaunchArgument('planned_cmd_vel_topic', default_value='/follow/cmd_vel_planned'),
        DeclareLaunchArgument('debug_text_topic', default_value='/follow/controller_debug'),
        DeclareLaunchArgument('marker_topic', default_value='/follow/controller_markers'),
        DeclareLaunchArgument('marker_frame', default_value='base_link'),
        DeclareLaunchArgument('output_topic', default_value='/follow/cmd_vel'),
        DeclareLaunchArgument('enable_service', default_value='/follow/enable_cmd_vel'),
        DeclareLaunchArgument('disable_service', default_value='/follow/disable_cmd_vel'),
        DeclareLaunchArgument('enable_cmd_vel', default_value='false'),
        DeclareLaunchArgument('follow_distance', default_value='1.8'),
        DeclareLaunchArgument('deadband_distance', default_value='0.25'),
        DeclareLaunchArgument('max_linear_x', default_value='0.15'),
        DeclareLaunchArgument('max_angular_z', default_value='0.35'),
        DeclareLaunchArgument('linear_gain', default_value='0.35'),
        DeclareLaunchArgument('angular_gain', default_value='0.9'),
        DeclareLaunchArgument('min_confidence', default_value='0.35'),
        DeclareLaunchArgument('target_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('front_cone_rad', default_value='0.26'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('linear_accel_limit', default_value='0.10'),
        DeclareLaunchArgument('angular_accel_limit', default_value='0.25'),
        DeclareLaunchArgument('lidar_only_no_motion_distance', default_value='1.8'),
        DeclareLaunchArgument('rc_override_deadband', default_value='10'),
        DeclareLaunchArgument('require_gesture_relock_after_rc_override', default_value='true'),
        DeclareLaunchArgument('allow_lidar_only_forward_motion', default_value='false'),
        DeclareLaunchArgument('profile_config', default_value=''),
        OpaqueFunction(function=_launch_nodes),
    ])
