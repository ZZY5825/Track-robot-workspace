#!/usr/bin/env python3
"""Motion-free RViz joint editor for a prefixed PiPER preview model."""

import math
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer


JOINT_NAMES = ["joint%d" % index for index in range(1, 7)]


def clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


def normalize_angle(value):
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_inverse(value):
    x, y, z, w = value
    norm = x * x + y * y + z * z + w * w
    return (-x / norm, -y / norm, -z / norm, w / norm)


def rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def z_rotation_quaternion(angle):
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def relative_z_angle(fixed_orientation, marker_orientation):
    relative = quaternion_multiply(quaternion_inverse(fixed_orientation), marker_orientation)
    return normalize_angle(2.0 * math.atan2(relative[2], relative[3]))


def assign_quaternion(message, values):
    message.x, message.y, message.z, message.w = values


class PiperJointPreview(Node):
    def __init__(self):
        super().__init__("piper_gui_joint_editor")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("frame_prefix", "preview_")
        urdf_path = self.get_parameter("urdf_path").value
        self.frame_prefix = self.get_parameter("frame_prefix").value
        if not urdf_path:
            raise ValueError("urdf_path is required")

        self.specs = self.load_joint_specs(urdf_path)
        self.positions = [0.0] * 8
        self.initialized = False
        self.preview_pub = self.create_publisher(
            JointState, "/piper_gui/preview_joint_states", 10)
        self.create_subscription(
            JointState, "/piper_gui/preview_set", self.preview_set_callback, 10)
        self.create_subscription(
            JointState, "/joint_states_single", self.live_feedback_callback, 10)
        self.server = InteractiveMarkerServer(self, "piper_gui_joint_editor")
        self.create_timer(0.5, self.publish_preview)
        self.rebuild_markers()

    @staticmethod
    def load_joint_specs(path):
        root = ET.parse(path).getroot()
        by_name = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        specs = []
        for name in JOINT_NAMES:
            joint = by_name[name]
            origin = joint.find("origin")
            xyz = tuple(float(value) for value in origin.attrib["xyz"].split())
            rpy = tuple(float(value) for value in origin.attrib["rpy"].split())
            limit = joint.find("limit")
            specs.append({
                "name": name,
                "parent": joint.find("parent").attrib["link"],
                "xyz": xyz,
                "fixed_orientation": rpy_quaternion(*rpy),
                "lower": float(limit.attrib["lower"]),
                "upper": float(limit.attrib["upper"]),
            })
        return specs

    def live_feedback_callback(self, message):
        if self.initialized or len(message.position) < 6:
            return
        self.set_positions(message)

    def preview_set_callback(self, message):
        if len(message.position) < 6:
            self.get_logger().warning("preview_set ignored: fewer than six joint positions")
            return
        self.set_positions(message)

    def set_positions(self, message):
        values = list(message.position)
        if message.name:
            named = dict(zip(message.name, values))
            for index, spec in enumerate(self.specs):
                if spec["name"] in named:
                    self.positions[index] = clamp(
                        named[spec["name"]], spec["lower"], spec["upper"])
        else:
            for index, spec in enumerate(self.specs):
                self.positions[index] = clamp(values[index], spec["lower"], spec["upper"])
        for index in range(6, min(8, len(values))):
            self.positions[index] = float(values[index])
        self.initialized = True
        self.rebuild_markers()
        self.publish_preview()

    def marker_pose(self, spec, angle):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_prefix + spec["parent"]
        marker.name = spec["name"]
        marker.description = "%s  %.3f rad" % (spec["name"], angle)
        # Keep the control outside the STL bodywork so RViz can render and pick it.
        # The previous 0.13 m control was mostly hidden inside the joint meshes.
        marker.scale = 0.30
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec["xyz"]
        orientation = quaternion_multiply(
            spec["fixed_orientation"], z_rotation_quaternion(angle))
        assign_quaternion(marker.pose.orientation, orientation)

        rotate = InteractiveMarkerControl()
        rotate.name = "rotate_" + spec["name"]
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate.orientation_mode = InteractiveMarkerControl.INHERIT
        # ROTATE_AXIS uses the control's X axis. Rotate X onto the marker's Z axis.
        assign_quaternion(rotate.orientation, (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)))
        rotate.always_visible = True

        center = Marker()
        center.type = Marker.SPHERE
        center.scale.x = center.scale.y = center.scale.z = 0.055
        center.color.r = 1.0
        center.color.g = 0.55
        center.color.b = 0.02
        center.color.a = 1.0
        rotate.markers.append(center)

        # RViz's generated ROTATE_AXIS ring can be occluded by the robot mesh.
        # This explicit circle belongs to the same interactive control, so the
        # clearly visible orange ring is also a large, reliable click target.
        ring = Marker()
        ring.type = Marker.LINE_STRIP
        ring.scale.x = 0.014
        ring.color.r = 1.0
        ring.color.g = 0.55
        ring.color.b = 0.02
        ring.color.a = 1.0
        ring_radius = 0.115
        for step in range(49):
            phase = 2.0 * math.pi * step / 48.0
            ring.points.append(Point(
                x=ring_radius * math.cos(phase),
                y=ring_radius * math.sin(phase),
                z=0.0,
            ))
        rotate.markers.append(ring)
        marker.controls.append(rotate)
        return marker

    def rebuild_markers(self):
        self.server.clear()
        for index, spec in enumerate(self.specs):
            marker = self.marker_pose(spec, self.positions[index])
            self.server.insert(
                marker,
                feedback_callback=lambda feedback, joint_index=index: self.marker_feedback(
                    joint_index, feedback))
        self.server.applyChanges()

    def marker_feedback(self, index, feedback):
        if feedback.event_type not in (
                InteractiveMarkerFeedback.POSE_UPDATE,
                InteractiveMarkerFeedback.MOUSE_UP):
            return
        spec = self.specs[index]
        orientation = (
            feedback.pose.orientation.x,
            feedback.pose.orientation.y,
            feedback.pose.orientation.z,
            feedback.pose.orientation.w,
        )
        angle = relative_z_angle(spec["fixed_orientation"], orientation)
        self.positions[index] = clamp(angle, spec["lower"], spec["upper"])
        self.initialized = True
        # Snap the marker back onto its one valid revolute axis and enforce limits.
        self.server.setPose(spec["name"], self.marker_pose(spec, self.positions[index]).pose)
        self.server.applyChanges()
        self.publish_preview()

    def publish_preview(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "piper_gui_joint_preview"
        message.name = ["joint%d" % index for index in range(1, 9)]
        message.position = list(self.positions)
        self.preview_pub.publish(message)


def main():
    rclpy.init()
    node = PiperJointPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.server.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
