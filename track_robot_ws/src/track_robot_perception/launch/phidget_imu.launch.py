from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')

    imu_node = Node(
        package='track_robot_perception',
        executable='phidget_spatial_imu_node',
        name='phidget_spatial_imu',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'phidget_imu.yaml',
            ]),
        ),
        imu_node,
    ])
