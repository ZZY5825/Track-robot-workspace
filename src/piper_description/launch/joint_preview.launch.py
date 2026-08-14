"""Launch the motion-free, draggable PiPER STL preview model."""

import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare(package="piper_description").find("piper_description")
    urdf_path = os.path.join(package_share, "urdf", "piper_description.xacro")
    rviz_path = os.path.join(package_share, "rviz", "joint_preview.rviz")
    with open(urdf_path, encoding="utf-8") as description_file:
        robot_description = description_file.read()
    preview_root = ET.fromstring(robot_description)
    preview_root.attrib["name"] = "piper_gui_preview"
    for link in preview_root.findall("link"):
        link.attrib["name"] = "preview_" + link.attrib["name"]
    for joint in preview_root.findall("joint"):
        joint.find("parent").attrib["link"] = (
            "preview_" + joint.find("parent").attrib["link"])
        joint.find("child").attrib["link"] = (
            "preview_" + joint.find("child").attrib["link"])
    preview_robot_description = ET.tostring(preview_root, encoding="unicode")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="piper_gui_preview",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": preview_robot_description,
            }],
            remappings=[("joint_states", "/piper_gui/preview_joint_states")],
        ),
        Node(
            package="piper_description",
            executable="piper_joint_preview_node.py",
            name="piper_gui_joint_editor",
            output="screen",
            parameters=[{
                "urdf_path": urdf_path,
                "frame_prefix": "preview_",
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="piper_joint_preview_rviz",
            output="screen",
            arguments=["-d", rviz_path],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
