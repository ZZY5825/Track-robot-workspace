from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="track_robot_teleop",
            executable="teleop_backend_node",
            name="teleop_backend_node",
            output="screen",
        )
    ])
