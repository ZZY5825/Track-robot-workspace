from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    calibrate_on_start = LaunchConfiguration('calibrate_on_start')

    debug_node = Node(
        package='track_robot_perception',
        executable='imu_lio_debug_node',
        name='imu_lio_debug',
        output='screen',
        parameters=[config_file, {
            'calibrate_on_start': calibrate_on_start,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'imu_lio_debug.yaml',
            ]),
        ),
        DeclareLaunchArgument('calibrate_on_start', default_value='false'),
        debug_node,
    ])
