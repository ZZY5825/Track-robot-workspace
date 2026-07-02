#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from track_robot_interfaces.msg import LidarCluster, LidarClusterArray, TargetState
from visualization_msgs.msg import Marker, MarkerArray

from track_robot_perception.lidar_camera_colorizer import quaternion_to_rotation_matrix


def point_to_array(point: Point) -> np.ndarray:
    return np.array([point.x, point.y, point.z], dtype=np.float32)


def array_to_point(values: np.ndarray) -> Point:
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    t = transform.transform.translation
    q = transform.transform.rotation
    rotation = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    translation = np.array([t.x, t.y, t.z], dtype=np.float32)
    return points @ rotation.T + translation


class TargetFusionNode(Node):
    def __init__(self):
        super().__init__('target_fusion_node')

        self.camera_target_topic = self.declare_parameter(
            'camera_target_topic', '/human_tracking/camera_target').value
        self.lidar_candidates_topic = self.declare_parameter(
            'lidar_candidates_topic', '/human_tracking/lidar_human_candidates').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/zed/zed_node/left/camera_info').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/human_tracking/target_state').value
        self.marker_topic = self.declare_parameter(
            'marker_topic', '/human_tracking/target_marker').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'zed_left_camera_optical_frame').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.min_bbox_iou = float(self.declare_parameter('min_bbox_iou', 0.01).value)
        self.max_lidar_recovery_distance = float(
            self.declare_parameter('max_lidar_recovery_distance', 1.5).value)
        self.max_target_age_sec = float(self.declare_parameter('max_target_age_sec', 2.0).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 15.0).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info: Optional[CameraInfo] = None
        self.camera_target: Optional[TargetState] = None
        self.latest_clusters: Optional[LidarClusterArray] = None
        self.camera_target_received_time: Optional[Time] = None
        self.latest_clusters_received_time: Optional[Time] = None
        self.last_position_base: Optional[np.ndarray] = None
        self.last_velocity_base = np.zeros(3, dtype=np.float32)
        self.last_target_time: Optional[Time] = None

        self.target_pub = self.create_publisher(TargetState, self.output_topic, 5)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 5)
        self.create_subscription(TargetState, self.camera_target_topic, self.camera_target_callback, 5)
        self.create_subscription(
            LidarClusterArray, self.lidar_candidates_topic, self.lidar_clusters_callback, 5)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 5)
        self.create_timer(max(0.02, 1.0 / max(self.publish_rate, 1.0)), self.publish_target)

        self.get_logger().info(
            f'target_fusion_node publishing {self.output_topic}; '
            f'base_frame={self.base_frame}; map_frame={self.map_frame}')

    def camera_target_callback(self, msg: TargetState):
        self.camera_target = msg
        self.camera_target_received_time = self.get_clock().now()

    def lidar_clusters_callback(self, msg: LidarClusterArray):
        self.latest_clusters = msg
        self.latest_clusters_received_time = self.get_clock().now()

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def publish_target(self):
        if self.camera_target is None:
            return
        if self.input_is_stale(self.camera_target_received_time):
            state = self.no_target_state()
            self.target_pub.publish(state)
            self.marker_pub.publish(self.make_marker(state))
            return

        state = TargetState()
        state.header = self.camera_target.header
        state.target_id = self.camera_target.target_id
        state.lock_state = self.camera_target.lock_state
        state.bbox = self.camera_target.bbox
        state.confidence = self.camera_target.confidence
        state.camera_visible = self.camera_target.camera_visible

        matched_cluster = None
        if (
                self.camera_target.camera_visible and
                self.latest_clusters is not None and
                not self.input_is_stale(self.latest_clusters_received_time) and
                self.camera_info is not None):
            matched_cluster = self.match_camera_bbox_to_cluster(
                self.camera_target.bbox, self.latest_clusters)

        if matched_cluster is not None:
            if self.fill_from_cluster(state, matched_cluster, self.latest_clusters.header.frame_id):
                state.source_state = TargetState.SOURCE_CAMERA_LIDAR
                state.lidar_visible = True
            else:
                state.source_state = TargetState.SOURCE_CAMERA_ONLY
        elif self.camera_target.camera_visible:
            state.source_state = TargetState.SOURCE_CAMERA_ONLY
        else:
            recovered = self.recover_lidar_cluster()
            if recovered is not None and self.latest_clusters is not None:
                if self.fill_from_cluster(state, recovered, self.latest_clusters.header.frame_id):
                    state.source_state = TargetState.SOURCE_LIDAR_ONLY
                    state.lidar_visible = True
                    state.lock_state = TargetState.LOCK_TARGET_LOCKED
                    state.confidence = min(0.8, max(state.confidence, recovered.confidence))
                else:
                    state.source_state = TargetState.SOURCE_NONE
            else:
                state.source_state = TargetState.SOURCE_NONE

        self.target_pub.publish(state)
        self.marker_pub.publish(self.make_marker(state))

    def match_camera_bbox_to_cluster(
            self, bbox: List[float], clusters_msg: LidarClusterArray) -> Optional[LidarCluster]:
        best_cluster = None
        best_iou = 0.0
        for cluster in clusters_msg.clusters:
            projected = self.project_cluster_bbox(cluster, clusters_msg.header.frame_id)
            if projected is None:
                continue
            overlap = self.iou(bbox, projected)
            if overlap > best_iou:
                best_iou = overlap
                best_cluster = cluster
        return best_cluster if best_iou >= self.min_bbox_iou else None

    def project_cluster_bbox(self, cluster: LidarCluster, source_frame: str) -> Optional[List[float]]:
        if self.camera_info is None:
            return None
        corners = self.cluster_corners(cluster)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, source_frame, Time(), timeout=Duration(seconds=0.03))
        except TransformException:
            return None
        camera_points = transform_points(corners, transform)
        z = camera_points[:, 2]
        valid = z > 0.05
        if not np.any(valid):
            return None
        camera_points = camera_points[valid]
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        u = fx * camera_points[:, 0] / camera_points[:, 2] + cx
        v = fy * camera_points[:, 1] / camera_points[:, 2] + cy
        return [float(np.min(u)), float(np.min(v)), float(np.max(u)), float(np.max(v))]

    @staticmethod
    def cluster_corners(cluster: LidarCluster) -> np.ndarray:
        mn = point_to_array(cluster.minimum)
        mx = point_to_array(cluster.maximum)
        return np.array([
            [mn[0], mn[1], mn[2]], [mn[0], mn[1], mx[2]],
            [mn[0], mx[1], mn[2]], [mn[0], mx[1], mx[2]],
            [mx[0], mn[1], mn[2]], [mx[0], mn[1], mx[2]],
            [mx[0], mx[1], mn[2]], [mx[0], mx[1], mx[2]],
        ], dtype=np.float32)

    def recover_lidar_cluster(self) -> Optional[LidarCluster]:
        if (
                self.latest_clusters is None or
                self.input_is_stale(self.latest_clusters_received_time) or
                len(self.latest_clusters.clusters) == 0):
            return None
        if self.last_position_base is None:
            return None

        best_cluster = None
        best_distance = float('inf')
        for cluster in self.latest_clusters.clusters:
            position = self.transform_point(
                point_to_array(cluster.centroid),
                self.latest_clusters.header.frame_id,
                self.base_frame)
            if position is None:
                continue
            distance = float(np.linalg.norm(position - self.last_position_base))
            if distance < best_distance:
                best_distance = distance
                best_cluster = cluster
        if best_cluster is None or best_distance > self.max_lidar_recovery_distance:
            return None
        return best_cluster

    def fill_from_cluster(self, state: TargetState, cluster: LidarCluster, source_frame: str) -> bool:
        position_base = self.transform_point(point_to_array(cluster.centroid), source_frame, self.base_frame)
        if position_base is None:
            return False
        now = self.get_clock().now()
        if self.last_position_base is not None and self.last_target_time is not None:
            dt = (now - self.last_target_time).nanoseconds * 1e-9
            if dt > 1e-3:
                self.last_velocity_base = (position_base - self.last_position_base) / dt
        self.last_position_base = position_base
        self.last_target_time = now

        state.position_base = array_to_point(position_base)
        state.velocity = Vector3(
            x=float(self.last_velocity_base[0]),
            y=float(self.last_velocity_base[1]),
            z=float(self.last_velocity_base[2]))
        state.distance = float(math.hypot(position_base[0], position_base[1]))
        state.bearing = float(math.atan2(position_base[1], position_base[0]))
        state.confidence = max(float(state.confidence), float(cluster.confidence))

        position_map = self.transform_point(point_to_array(cluster.centroid), source_frame, self.map_frame)
        if position_map is not None:
            state.position_map_valid = True
            state.position_map = array_to_point(position_map)
        return True

    def input_is_stale(self, received_time: Optional[Time]) -> bool:
        if received_time is None:
            return True
        age = (self.get_clock().now() - received_time).nanoseconds * 1e-9
        return age > self.max_target_age_sec

    def no_target_state(self) -> TargetState:
        state = TargetState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.base_frame
        state.target_id = -1
        state.lock_state = TargetState.LOCK_TARGET_LOST
        state.source_state = TargetState.SOURCE_NONE
        state.confidence = 0.0
        return state

    def transform_point(self, point: np.ndarray, source_frame: str, target_frame: str) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.03))
        except TransformException:
            return None
        return transform_points(point.reshape(1, 3), transform)[0]

    @staticmethod
    def iou(a, b) -> float:
        x1 = max(float(a[0]), float(b[0]))
        y1 = max(float(a[1]), float(b[1]))
        x2 = min(float(a[2]), float(b[2]))
        y2 = min(float(a[3]), float(b[3]))
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area_a = max(0.0, float(a[2]) - float(a[0])) * max(0.0, float(a[3]) - float(a[1]))
        area_b = max(0.0, float(b[2]) - float(b[0])) * max(0.0, float(b[3]) - float(b[1]))
        denom = area_a + area_b - inter
        return inter / denom if denom > 0.0 else 0.0

    def make_marker(self, state: TargetState) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header = state.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if state.source_state not in (
                TargetState.SOURCE_CAMERA_LIDAR,
                TargetState.SOURCE_LIDAR_ONLY):
            return markers
        marker = Marker()
        marker.header = state.header
        marker.header.frame_id = self.base_frame
        marker.ns = 'human_target'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = state.position_base
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
        marker.lifetime = DurationMsg(sec=0, nanosec=300000000)
        markers.markers.append(marker)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = TargetFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
