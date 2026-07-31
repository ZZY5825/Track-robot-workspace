from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    semantic_memory = Node(
        package='track_robot_semantic_memory',
        executable='semantic_memory_node',
        name='semantic_memory',
        output='screen',
        parameters=[
            config_file,
            {
                'enable_test_camera_attachment': ParameterValue(
                    LaunchConfiguration('enable_test_camera_attachment'),
                    value_type=bool),
                'allow_degraded_calibration': ParameterValue(
                    LaunchConfiguration('allow_degraded_calibration'),
                    value_type=bool),
            },
        ],
    )
    semantic_memory_visualizer = Node(
        package='track_robot_semantic_memory',
        executable='semantic_memory_visualizer_node',
        name='semantic_memory_visualizer',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('start_visualizer')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('enable_test_camera_attachment',
                              default_value='false'),
        DeclareLaunchArgument('allow_degraded_calibration',
                              default_value='false'),
        DeclareLaunchArgument('start_visualizer', default_value='true'),
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
