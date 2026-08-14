from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'launch',
            'track_robot_hardware.launch.py',
        ])),
        launch_arguments={
            'start_description': 'false',
            'start_camera': 'false',
            'start_lidar': 'false',
            'start_base': LaunchConfiguration('start_base'),
            'start_imu': LaunchConfiguration('start_imu'),
            'base_frame': LaunchConfiguration('base_frame'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        hardware,
    ])
