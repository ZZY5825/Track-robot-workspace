from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    health = Node(
        package='track_robot_decision',
        executable='perception_health_monitor_node',
        name='perception_health_monitor_node',
        output='screen',
        parameters=[config, {
            'image_topic': LaunchConfiguration('image_topic'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'command_topic': LaunchConfiguration('command_topic'),
        }],
    )
    decision = Node(
        package='track_robot_decision',
        executable='follow_behavior_tree_node',
        name='follow_behavior_tree_node',
        output='screen',
        parameters=[config, {
            'require_health_override': ParameterValue(
                LaunchConfiguration('require_health'), value_type=str),
            'require_avoidance_feedback_override': ParameterValue(
                LaunchConfiguration('require_avoidance_feedback'), value_type=str),
            'require_safety_feedback_override': ParameterValue(
                LaunchConfiguration('require_safety_feedback'), value_type=str),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('track_robot_decision'), 'config', 'outdoor_decision.yaml'])),
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('odometry_topic', default_value='/odom'),
        DeclareLaunchArgument('command_topic', default_value='/follow/cmd_vel_safe'),
        DeclareLaunchArgument('require_health', default_value='true'),
        DeclareLaunchArgument('require_avoidance_feedback', default_value='false'),
        DeclareLaunchArgument('require_safety_feedback', default_value='false'),
        health,
        decision,
    ])
