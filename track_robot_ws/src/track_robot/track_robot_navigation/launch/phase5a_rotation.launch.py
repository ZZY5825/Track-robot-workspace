"""Fail-closed, rotation-only Nav2 bringup for Phase 5A."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from track_robot_navigation.runtime_modes import (
    RuntimeMode,
    validate_mode_request,
)


TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
NAV2_CMD_REMAPPINGS = TF_REMAPPINGS + [
    ('cmd_vel', '/nav2/cmd_vel_raw'),
]


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _runtime_nodes(context):
    mode = RuntimeMode.parse(
        LaunchConfiguration('runtime_mode').perform(context))
    rotation_enabled = _as_bool(
        LaunchConfiguration('enable_rotation_execution').perform(context))
    validate_mode_request(
        mode,
        enable_semantic_execution=False,
        enable_rotation_execution=rotation_enabled,
    )
    if mode is RuntimeMode.PLANNING_ONLY:
        return []
    if mode is not RuntimeMode.ROTATION_ONLY_ACTIVE:
        raise ValueError(
            'phase5a_rotation supports PLANNING_ONLY or '
            'ROTATION_ONLY_ACTIVE')

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
        convert_types=True,
    )
    start_obstacle_map = _as_bool(
        LaunchConfiguration('start_obstacle_map').perform(context))
    actions = []
    if start_obstacle_map:
        actions.append(Node(
            package='track_robot_safety',
            executable='local_obstacle_map_node',
            name='local_obstacle_map_node',
            output='screen',
            parameters=[LaunchConfiguration('obstacle_map_config')],
        ))
    actions.extend([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=NAV2_CMD_REMAPPINGS,
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=NAV2_CMD_REMAPPINGS,
        ),
        Node(
            package='track_robot_safety',
            executable='motion_safety_supervisor_node',
            name='motion_safety_supervisor_node',
            output='screen',
            parameters=[LaunchConfiguration('safety_supervisor_config')],
        ),
        Node(
            package='track_robot_core',
            executable='cmd_vel_gate',
            name='cmd_vel_gate',
            output='screen',
            parameters=[LaunchConfiguration('cmd_vel_gate_config')],
        ),
        Node(
            package='track_robot_navigation',
            executable='search_motion_adapter',
            name='search_motion_adapter',
            output='screen',
            parameters=[LaunchConfiguration('search_motion_config')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_phase5a_rotation',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'node_names': ['controller_server', 'recoveries_server'],
            }],
        ),
    ])
    return actions


def generate_launch_description():
    navigation_share = get_package_share_directory(
        'track_robot_navigation')
    safety_share = get_package_share_directory('track_robot_safety')
    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_mode', default_value='PLANNING_ONLY'),
        DeclareLaunchArgument(
            'enable_rotation_execution', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_obstacle_map', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=(
                navigation_share + '/config/nav2_phase5a.yaml')),
        DeclareLaunchArgument(
            'search_motion_config',
            default_value=(
                navigation_share + '/config/active_search_motion.yaml')),
        DeclareLaunchArgument(
            'obstacle_map_config',
            default_value=(
                safety_share + '/config/local_obstacle_map.yaml')),
        DeclareLaunchArgument(
            'safety_supervisor_config',
            default_value=(
                safety_share
                + '/config/motion_safety_supervisor_nav2.yaml')),
        DeclareLaunchArgument(
            'cmd_vel_gate_config',
            default_value=(
                navigation_share + '/config/cmd_vel_gate_nav2.yaml')),
        OpaqueFunction(function=_runtime_nodes),
    ])
