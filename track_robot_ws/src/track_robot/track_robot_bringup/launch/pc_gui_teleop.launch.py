import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("track_robot_bringup")
    gui_input_config = os.path.join(bringup_dir, "config", "gui_input.yaml")
    teleop_backend_config = os.path.join(bringup_dir, "config", "teleop_backend.yaml")

    return LaunchDescription([
        Node(
            package="track_robot_teleop",
            executable="gui_input_node",
            name="gui_input_node",
            output="screen",
            parameters=[gui_input_config],
        ),
        Node(
            package="track_robot_teleop",
            executable="teleop_backend_node",
            name="teleop_backend_node",
            output="screen",
            parameters=[teleop_backend_config],
        ),
    ])
