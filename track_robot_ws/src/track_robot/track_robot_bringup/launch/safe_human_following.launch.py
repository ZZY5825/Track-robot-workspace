from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    decision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('track_robot_decision'),
                'launch', 'outdoor_follow_decision.launch.py'])]),
        launch_arguments={
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'require_health': LaunchConfiguration('require_decision_health'),
            'require_avoidance_feedback': 'true',
            'require_safety_feedback': 'true',
        }.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('track_robot_control'),
                'launch', 'target_follow_controller.launch.py'])]),
        launch_arguments={
            'enable_cmd_vel': 'false',
            'planned_cmd_vel_topic': '/follow/cmd_vel_planned',
        }.items(),
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('track_robot_safety'),
                'launch', 'motion_safety.launch.py'])]),
        launch_arguments={
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_qos_reliability': LaunchConfiguration('lidar_qos_reliability'),
            'allow_latest_tf_fallback': LaunchConfiguration('allow_latest_tf_fallback'),
            'enable_avoidance': LaunchConfiguration('enable_avoidance'),
            'require_bunker_status': LaunchConfiguration('require_bunker_status'),
            'require_rc_state': LaunchConfiguration('require_rc_state'),
        }.items(),
    )

    gate = Node(
        package='track_robot_core',
        executable='cmd_vel_gate',
        name='cmd_vel_gate',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_cmd_vel_gate')),
        parameters=[PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'config', 'cmd_vel_gate_follow.yaml'])],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('allow_latest_tf_fallback', default_value='false'),
        DeclareLaunchArgument('enable_avoidance', default_value='true'),
        DeclareLaunchArgument('require_bunker_status', default_value='true'),
        DeclareLaunchArgument('require_rc_state', default_value='true'),
        DeclareLaunchArgument('start_cmd_vel_gate', default_value='true'),
        DeclareLaunchArgument('require_decision_health', default_value='true'),
        decision,
        controller,
        safety,
        gate,
    ])
