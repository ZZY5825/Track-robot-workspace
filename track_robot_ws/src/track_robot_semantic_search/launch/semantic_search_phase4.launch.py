from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planner = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_phase4_planner',
        name='phase4_approach_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='semantic_search_phase4_rviz',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('track_robot_bringup'),
                'rviz',
                'semantic_search_phase4.rviz',
            ]),
        ],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_semantic_search'),
                'config',
                'semantic_search_phase4.yaml',
            ])),
        planner,
        rviz,
    ])
