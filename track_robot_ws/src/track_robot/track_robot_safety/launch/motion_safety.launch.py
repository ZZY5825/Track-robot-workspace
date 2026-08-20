from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_EXPLICIT_ARGUMENT_NAMES = '_motion_safety_explicit_argument_names'


def _capture_explicit_launch_arguments(context):
    context.launch_configurations[_EXPLICIT_ARGUMENT_NAMES] = ','.join(
        sorted(context.launch_configurations))


def _profile_parameters(
        context, base_config, overrides, profile_owned_arguments=()):
    parameters = [base_config]
    profile_config = LaunchConfiguration('profile_config').perform(context)
    if profile_config:
        parameters.append(profile_config)
        explicit_arguments = {
            name for name in context.launch_configurations.get(
                _EXPLICIT_ARGUMENT_NAMES, '').split(',')
            if name
        }
        overrides = {
            name: value for name, value in overrides.items()
            if name not in profile_owned_arguments or name in explicit_arguments
        }
    parameters.append(overrides)
    return parameters


def _launch_nodes(context):
    obstacle_map = Node(
        package='track_robot_safety',
        executable='local_obstacle_map_node',
        name='local_obstacle_map_node',
        output='screen',
        parameters=_profile_parameters(
            context, LaunchConfiguration('obstacle_map_config'), {
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_qos_reliability': LaunchConfiguration('lidar_qos_reliability'),
            'allow_latest_tf_fallback': LaunchConfiguration('allow_latest_tf_fallback'),
            }),
    )

    planner = Node(
        package='track_robot_safety',
        executable='local_trajectory_planner_node',
        name='local_trajectory_planner_node',
        output='screen',
        parameters=_profile_parameters(
            context, LaunchConfiguration('planner_config'), {
            'enable_avoidance': LaunchConfiguration('enable_avoidance'),
            'base_frame': LaunchConfiguration('base_frame'),
            }),
    )

    supervisor = Node(
        package='track_robot_safety',
        executable='motion_safety_supervisor_node',
        name='motion_safety_supervisor_node',
        output='screen',
        parameters=_profile_parameters(
            context, LaunchConfiguration('supervisor_config'), {
            'require_bunker_status': LaunchConfiguration('require_bunker_status'),
            'require_rc_state': LaunchConfiguration('require_rc_state'),
            }),
    )
    return [obstacle_map, planner, supervisor]


def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            scoped=True,
            actions=[
                OpaqueFunction(function=_capture_explicit_launch_arguments),
                DeclareLaunchArgument(
                    'obstacle_map_config',
                    default_value=PathJoinSubstitution([
                        FindPackageShare('track_robot_safety'),
                        'config', 'local_obstacle_map.yaml'])),
                DeclareLaunchArgument(
                    'planner_config',
                    default_value=PathJoinSubstitution([
                        FindPackageShare('track_robot_safety'),
                        'config', 'local_trajectory_planner.yaml'])),
                DeclareLaunchArgument(
                    'supervisor_config',
                    default_value=PathJoinSubstitution([
                        FindPackageShare('track_robot_safety'),
                        'config', 'motion_safety_supervisor.yaml'])),
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
                DeclareLaunchArgument('profile_config', default_value=''),
                OpaqueFunction(function=_launch_nodes),
            ],
        ),
    ])
