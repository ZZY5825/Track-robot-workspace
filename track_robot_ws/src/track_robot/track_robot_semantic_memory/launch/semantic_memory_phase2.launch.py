from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    semantic_memory = Node(
        package='track_robot_semantic_memory',
        executable='semantic_memory_node',
        name='semantic_memory',
        output='screen',
        parameters=[config_file],
    )
    semantic_memory_visualizer = Node(
        package='track_robot_semantic_memory',
        executable='semantic_memory_visualizer_node',
        name='semantic_memory_visualizer',
        output='screen',
        parameters=[config_file],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_semantic_memory'),
                'config',
                'semantic_memory.yaml',
            ])),
        semantic_memory,
        semantic_memory_visualizer,
    ])
