from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_evaluator = LaunchConfiguration('start_evaluator')
    config_file = LaunchConfiguration('config_file')
    manifest_path = LaunchConfiguration('manifest_path')
    output_path = LaunchConfiguration('output_path')
    tegrastats_path = LaunchConfiguration('tegrastats_path')
    duration_sec = LaunchConfiguration('duration_sec')
    run_id = LaunchConfiguration('run_id')
    replay_rate = LaunchConfiguration('replay_rate')
    software_revision = LaunchConfiguration('software_revision')
    freshness_time_base = LaunchConfiguration('freshness_time_base')
    timing_policy = LaunchConfiguration('timing_policy')

    localization = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_localization_health',
        name='semantic_search_localization_health',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'freshness_time_base': freshness_time_base,
            },
        ],
    )
    evaluator = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_evaluator',
        name='semantic_search_evaluator',
        output='screen',
        condition=IfCondition(start_evaluator),
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'manifest_path': manifest_path,
                'output_path': output_path,
                'tegrastats_path': tegrastats_path,
                'duration_sec': duration_sec,
                'run_id': run_id,
                'replay_rate': replay_rate,
                'software_revision': software_revision,
                'config_path': config_file,
                'freshness_time_base': freshness_time_base,
                'timing_policy': timing_policy,
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_evaluator', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_semantic_search'),
                'config',
                'semantic_search_phase0.yaml',
            ])),
        DeclareLaunchArgument('manifest_path', default_value=''),
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/semantic_search_phase0_report.json'),
        DeclareLaunchArgument('tegrastats_path', default_value=''),
        DeclareLaunchArgument('duration_sec', default_value='30.0'),
        DeclareLaunchArgument('run_id', default_value='phase0'),
        DeclareLaunchArgument('replay_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'software_revision', default_value='unversioned'),
        DeclareLaunchArgument(
            'freshness_time_base', default_value='source_clock'),
        DeclareLaunchArgument(
            'timing_policy', default_value='online_source_time'),
        localization,
        evaluator,
    ])
