from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="track_robot_teleop",
            executable="gui_input_node",
            name="gui_input_node",
            output="screen",
        )
    ])
