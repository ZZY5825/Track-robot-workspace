"""Mode-gated Nav2 bringup for Phase 4B."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from track_robot_navigation.runtime_modes import (
    RuntimeMode,
    mode_spec,
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
    semantic_enabled = _as_bool(
        LaunchConfiguration('enable_semantic_execution').perform(context))
    validate_mode_request(mode, semantic_enabled)
    spec = mode_spec(mode)

    if spec.semantic_adapter:
        raise RuntimeError(
            '{} requires the semantic supervisor delivered in the next '
            'regression-gated stage'.format(mode.value))

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'default_bt_xml_filename':
                LaunchConfiguration('default_bt_xml_filename'),
        },
        convert_types=True,
    )
    actions = []
    lifecycle_nodes = []

    if spec.planner:
        actions.append(Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=TF_REMAPPINGS,
        ))
        lifecycle_nodes.append('planner_server')

    if spec.controller:
        actions.append(Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=NAV2_CMD_REMAPPINGS,
        ))
        lifecycle_nodes.append('controller_server')

    if spec.recoveries:
        actions.append(Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=NAV2_CMD_REMAPPINGS,
        ))
        lifecycle_nodes.append('recoveries_server')

    if spec.bt_navigator:
        actions.append(Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=TF_REMAPPINGS,
        ))
        lifecycle_nodes.append('bt_navigator')

    if spec.safety_chain:
        actions.extend([
            Node(
                package='track_robot_safety',
                executable='local_obstacle_map_node',
                name='local_obstacle_map_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('obstacle_map_config'),
                ],
            ),
            Node(
                package='track_robot_safety',
                executable='motion_safety_supervisor_node',
                name='motion_safety_supervisor_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('safety_supervisor_config'),
                ],
            ),
            Node(
                package='track_robot_core',
                executable='cmd_vel_gate',
                name='cmd_vel_gate',
                output='screen',
                parameters=[
                    LaunchConfiguration('cmd_vel_gate_config'),
                ],
            ),
        ])

    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': lifecycle_nodes,
        }],
    ))
    return actions


def generate_launch_description():
    navigation_share = get_package_share_directory('track_robot_navigation')
    safety_share = get_package_share_directory('track_robot_safety')

    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_mode',
            default_value=RuntimeMode.PLANNING_ONLY.value,
        ),
        DeclareLaunchArgument(
            'enable_semantic_execution',
            default_value='false',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=(
                navigation_share + '/config/nav2_phase4b.yaml'),
        ),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=(
                navigation_share
                + '/behavior_trees/navigate_supervised.xml'),
        ),
        DeclareLaunchArgument(
            'obstacle_map_config',
            default_value=safety_share + '/config/local_obstacle_map.yaml',
        ),
        DeclareLaunchArgument(
            'safety_supervisor_config',
            default_value=(
                safety_share
                + '/config/motion_safety_supervisor_nav2.yaml'),
        ),
        DeclareLaunchArgument(
            'cmd_vel_gate_config',
            default_value=(
                navigation_share + '/config/cmd_vel_gate_nav2.yaml'),
        ),
        OpaqueFunction(function=_runtime_nodes),
    ])
