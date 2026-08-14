from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bunker_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bunker_base'),
            'launch',
            'bunker_base.launch.py',
        ])),
        launch_arguments={
            'base_frame': LaunchConfiguration('base_frame'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_base')),
    )
    phidget_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_perception'),
            'launch',
            'phidget_imu.launch.py',
        ])),
        launch_arguments={
            'config_path': PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'phidget_imu.yaml',
            ]),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_imu')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        bunker_base,
        phidget_imu,
    ])
