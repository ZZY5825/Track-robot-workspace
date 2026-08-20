from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in (
        '1', 'true', 'yes', 'on')


def _launch_feature(context):
    if _is_true(context, 'start_cmd_vel_gate'):
        raise RuntimeError(
            'start_cmd_vel_gate cannot enable motion; use '
            'runtime_mode:=active motion_confirmed:=true')

    runtime_mode = LaunchConfiguration('runtime_mode').perform(context)
    motion_confirmed = _is_true(context, 'motion_confirmed')
    if runtime_mode not in ('shadow', 'active'):
        raise RuntimeError("runtime_mode must be 'shadow' or 'active'")
    if runtime_mode == 'active' and not motion_confirmed:
        raise RuntimeError(
            'active human following requires motion_confirmed:=true')

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'launch',
            'human_following_live.launch.py',
        ])),
        launch_arguments={
            'runtime_mode': LaunchConfiguration('runtime_mode'),
            'motion_confirmed': LaunchConfiguration('motion_confirmed'),
            'profile_config': LaunchConfiguration('profile_config'),
            'start_description': 'false',
            'start_camera': 'false',
            'start_lidar': 'false',
            'start_base': 'false',
            'start_imu': 'false',
            'start_rviz': 'false',
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_qos_reliability': LaunchConfiguration(
                'lidar_qos_reliability'),
            'allow_latest_tf_fallback': LaunchConfiguration(
                'allow_latest_tf_fallback'),
            'enable_avoidance': LaunchConfiguration('enable_avoidance'),
            'require_bunker_status': LaunchConfiguration(
                'require_bunker_status'),
            'require_rc_state': LaunchConfiguration('require_rc_state'),
            'require_decision_health': LaunchConfiguration(
                'require_decision_health'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('runtime_mode', default_value='shadow'),
        DeclareLaunchArgument('motion_confirmed', default_value='false'),
        DeclareLaunchArgument(
            'profile_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_bringup'),
                'config',
                'human_following_shadow.yaml',
            ]),
        ),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'lidar_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument(
            'allow_latest_tf_fallback', default_value='false'),
        DeclareLaunchArgument('enable_avoidance', default_value='true'),
        DeclareLaunchArgument(
            'require_bunker_status', default_value='true'),
        DeclareLaunchArgument('require_rc_state', default_value='true'),
        DeclareLaunchArgument(
            'start_cmd_vel_gate', default_value='false'),
        DeclareLaunchArgument(
            'require_decision_health', default_value='true'),
        OpaqueFunction(function=_launch_feature),
    ])
