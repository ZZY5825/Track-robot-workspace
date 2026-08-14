from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_EXPLICIT_ARGUMENT_NAMES = '_outdoor_follow_decision_explicit_argument_names'


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
    config = LaunchConfiguration('config')
    health = Node(
        package='track_robot_decision',
        executable='perception_health_monitor_node',
        name='perception_health_monitor_node',
        output='screen',
        parameters=_profile_parameters(context, config, {
            'image_topic': LaunchConfiguration('image_topic'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'command_topic': LaunchConfiguration('command_topic'),
        }),
    )
    decision = Node(
        package='track_robot_decision',
        executable='follow_behavior_tree_node',
        name='follow_behavior_tree_node',
        output='screen',
        parameters=_profile_parameters(context, config, {
            'require_health_override': ParameterValue(
                LaunchConfiguration('require_health'), value_type=str),
            'require_avoidance_feedback_override': ParameterValue(
                LaunchConfiguration('require_avoidance_feedback'), value_type=str),
            'require_safety_feedback_override': ParameterValue(
                LaunchConfiguration('require_safety_feedback'), value_type=str),
        }),
    )
    return [health, decision]


def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            scoped=True,
            actions=[
                OpaqueFunction(function=_capture_explicit_launch_arguments),
                DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
                    FindPackageShare('track_robot_decision'),
                    'config', 'outdoor_decision.yaml'])),
                DeclareLaunchArgument(
                    'image_topic',
                    default_value='/zed/zed_node/left/image_rect_color'),
                DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
                DeclareLaunchArgument(
                    'imu_topic', default_value='/zed/zed_node/imu/data'),
                DeclareLaunchArgument('odometry_topic', default_value='/odom'),
                DeclareLaunchArgument(
                    'command_topic', default_value='/follow/cmd_vel_safe'),
                DeclareLaunchArgument('require_health', default_value='true'),
                DeclareLaunchArgument(
                    'require_avoidance_feedback', default_value='false'),
                DeclareLaunchArgument(
                    'require_safety_feedback', default_value='false'),
                DeclareLaunchArgument('profile_config', default_value=''),
                OpaqueFunction(function=_launch_nodes),
            ],
        ),
    ])
