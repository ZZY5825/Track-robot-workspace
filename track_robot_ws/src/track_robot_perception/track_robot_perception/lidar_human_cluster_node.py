#!/usr/bin/env python3

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from track_robot_interfaces.msg import LidarCluster, LidarClusterArray, TargetState
from visualization_msgs.msg import Marker, MarkerArray

from track_robot_perception.lidar_cluster_baseline_node import (
    cloud_xyz_intensity,
    rgb_to_pcl_float,
)
from track_robot_perception.lidar_camera_colorizer import quaternion_to_rotation_matrix


TARGET_POINT_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('intensity', '<f4'),
    ('rgb', '<f4'),
])


@dataclass
class RangeProfile:
    search_radius: float
    min_points: int
    cluster_tolerance: float
    confidence_decay: float


@dataclass
class LocalCandidate:
    points: np.ndarray
    centroid: np.ndarray
    minimum: np.ndarray
    maximum: np.ndarray
    score: float
    point_count: int
    distance_to_prediction: float
    bearing_error: float


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    t = transform.transform.translation
    q = transform.transform.rotation
    rotation = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    translation = np.array([t.x, t.y, t.z], dtype=np.float32)
    return points @ rotation.T + translation


def ypr_to_rotation_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float32)


def array_to_point(values: np.ndarray) -> Point:
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def make_xyzrgb_cloud(
        points: np.ndarray,
        intensity: np.ndarray,
        color: Tuple[int, int, int],
        header: Header) -> PointCloud2:
    data = np.empty(points.shape[0], dtype=TARGET_POINT_DTYPE)
    if points.shape[0] > 0:
        data['x'] = points[:, 0]
        data['y'] = points[:, 1]
        data['z'] = points[:, 2]
        data['intensity'] = intensity[:points.shape[0]]
        colors = np.repeat(np.array([color], dtype=np.uint8), points.shape[0], axis=0)
        data['rgb'] = rgb_to_pcl_float(colors)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = TARGET_POINT_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = False
    msg.data = data.tobytes()
    return msg


class LidarHumanClusterNode(Node):
    """Camera-initialized LiDAR target tracker.

    The executable name is preserved for compatibility with existing launch
    files, but this node no longer performs global human classification.
    """

    def __init__(self):
        super().__init__('lidar_human_cluster_node')

        self.lidar_topic = self.declare_parameter('lidar_topic', '/rslidar_points').value
        self.lidar_qos_reliability = str(
            self.declare_parameter('lidar_qos_reliability', 'reliable').value).lower()
        self.camera_target_topic = self.declare_parameter(
            'camera_target_topic', '/human_tracking/camera_target').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/zed/zed_node/left/camera_info').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/human_tracking/fused_target_state').value
        self.compat_output_topic = self.declare_parameter(
            'compat_output_topic', '/human_tracking/target_state').value
        self.target_points_topic = self.declare_parameter(
            'target_points_topic', '/human_tracking/target_lidar_points').value
        self.search_gate_marker_topic = self.declare_parameter(
            'search_gate_marker_topic', '/human_tracking/target_search_gate_marker').value
        self.fused_marker_topic = self.declare_parameter(
            'fused_marker_topic', '/human_tracking/fused_target_marker').value
        self.debug_topic = self.declare_parameter(
            'debug_topic', '/human_tracking/lidar_target_debug').value
        self.legacy_clusters_topic = self.declare_parameter(
            'human_clusters_topic', '/human_tracking/lidar_clusters').value
        self.legacy_candidates_topic = self.declare_parameter(
            'human_candidates_topic', '/human_tracking/lidar_human_candidates').value

        self.lidar_frame = self.declare_parameter('fixed_frame', 'rslidar').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'zed_left_camera_optical_frame').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.use_base_lidar_extrinsic_fallback = parse_bool(
            self.declare_parameter('use_base_lidar_extrinsic_fallback', True).value)
        self.base_lidar_extrinsic_parent_frame = self.declare_parameter(
            'base_lidar_extrinsic_parent_frame', self.base_frame).value
        self.base_lidar_extrinsic_child_frame = self.declare_parameter(
            'base_lidar_extrinsic_child_frame', self.lidar_frame).value
        self.base_lidar_extrinsic_translation = np.array([
            float(self.declare_parameter('base_lidar_extrinsic_x', 0.0).value),
            float(self.declare_parameter('base_lidar_extrinsic_y', 0.0).value),
            float(self.declare_parameter('base_lidar_extrinsic_z', 0.70).value),
        ], dtype=np.float32)
        self.base_lidar_extrinsic_rotation_parent_child = ypr_to_rotation_matrix(
            float(self.declare_parameter('base_lidar_extrinsic_yaw', 0.0).value),
            float(self.declare_parameter('base_lidar_extrinsic_pitch', 0.0).value),
            float(self.declare_parameter('base_lidar_extrinsic_roll', 0.0).value),
        )
        self.use_camera_lidar_extrinsic_fallback = parse_bool(
            self.declare_parameter('use_camera_lidar_extrinsic_fallback', True).value)
        self.prefer_camera_lidar_extrinsic = parse_bool(
            self.declare_parameter('prefer_camera_lidar_extrinsic', True).value)
        self.extrinsic_parent_frame = self.declare_parameter(
            'camera_lidar_extrinsic_parent_frame', 'zed_camera_link').value
        self.extrinsic_child_frame = self.declare_parameter(
            'camera_lidar_extrinsic_child_frame', self.lidar_frame).value
        self.extrinsic_translation = np.array([
            float(self.declare_parameter('camera_lidar_extrinsic_x', -0.27).value),
            float(self.declare_parameter('camera_lidar_extrinsic_y', 0.0).value),
            float(self.declare_parameter('camera_lidar_extrinsic_z', 0.08).value),
        ], dtype=np.float32)
        self.extrinsic_rotation_parent_child = ypr_to_rotation_matrix(
            float(self.declare_parameter('camera_lidar_extrinsic_yaw', 0.0).value),
            float(self.declare_parameter('camera_lidar_extrinsic_pitch', 0.0).value),
            float(self.declare_parameter('camera_lidar_extrinsic_roll', 0.0).value),
        )

        self.min_range = float(self.declare_parameter('min_range', 0.25).value)
        self.max_range = float(self.declare_parameter('max_range', 20.0).value)
        self.min_z_base = float(self.declare_parameter('min_z_base', -0.35).value)
        self.max_z_base = float(self.declare_parameter('max_z_base', 2.4).value)
        self.bbox_padding_px = float(self.declare_parameter('bbox_padding_px', 40.0).value)
        self.min_projected_points = max(
            1, int(self.declare_parameter('min_projected_points', 5).value))
        self.camera_roi_depth_filter = parse_bool(
            self.declare_parameter('camera_roi_depth_filter', True).value)
        self.camera_roi_depth_percentile = float(
            self.declare_parameter('camera_roi_depth_percentile', 20.0).value)
        self.camera_roi_depth_window_m = float(
            self.declare_parameter('camera_roi_depth_window_m', 1.0).value)
        self.camera_roi_cluster = parse_bool(
            self.declare_parameter('camera_roi_cluster', True).value)
        self.camera_roi_cluster_tolerance = float(
            self.declare_parameter('camera_roi_cluster_tolerance', 0.45).value)
        self.target_point_percentile_low = float(
            self.declare_parameter('target_point_percentile_low', 10.0).value)
        self.target_point_percentile_high = float(
            self.declare_parameter('target_point_percentile_high', 90.0).value)
        self.process_every_n_clouds = max(
            1, int(self.declare_parameter('process_every_n_clouds', 1).value))

        self.alpha_position = float(self.declare_parameter('alpha_position', 0.55).value)
        self.beta_velocity = float(self.declare_parameter('beta_velocity', 0.25).value)
        self.lidar_only_alpha_position = float(
            self.declare_parameter('lidar_only_alpha_position', 0.35).value)
        self.lidar_only_beta_velocity = float(
            self.declare_parameter('lidar_only_beta_velocity', 0.08).value)
        self.max_target_speed_mps = float(
            self.declare_parameter('max_target_speed_mps', 2.0).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 15.0).value)
        self.max_prediction_only_sec = float(
            self.declare_parameter('max_prediction_only_sec', 1.5).value)
        self.max_lidar_only_sec = float(
            self.declare_parameter('max_lidar_only_sec', 3.0).value)
        self.camera_target_timeout_sec = float(
            self.declare_parameter('camera_target_timeout_sec', 1.0).value)

        self.near_range_m = float(self.declare_parameter('near_range_m', 2.0).value)
        self.far_range_m = float(self.declare_parameter('far_range_m', 6.0).value)
        self.near_search_radius = float(
            self.declare_parameter('near_search_radius', 0.75).value)
        self.mid_search_radius = float(
            self.declare_parameter('mid_search_radius', 1.1).value)
        self.far_search_radius = float(
            self.declare_parameter('far_search_radius', 1.8).value)
        self.near_min_points = max(1, int(self.declare_parameter('near_min_points', 3).value))
        self.mid_min_points = max(1, int(self.declare_parameter('mid_min_points', 8).value))
        self.far_min_points = max(1, int(self.declare_parameter('far_min_points', 3).value))
        self.local_cluster_tolerance_near = float(
            self.declare_parameter('local_cluster_tolerance_near', 0.28).value)
        self.local_cluster_tolerance_mid = float(
            self.declare_parameter('local_cluster_tolerance_mid', 0.38).value)
        self.local_cluster_tolerance_far = float(
            self.declare_parameter('local_cluster_tolerance_far', 0.60).value)

        self.max_cluster_width = float(self.declare_parameter('max_cluster_width', 1.6).value)
        self.max_cluster_depth = float(self.declare_parameter('max_cluster_depth', 1.6).value)
        self.max_cluster_height = float(self.declare_parameter('max_cluster_height', 2.4).value)
        self.lidar_only_min_score = float(
            self.declare_parameter('lidar_only_min_score', 0.32).value)
        self.lidar_only_max_jump_m = float(
            self.declare_parameter('lidar_only_max_jump_m', 0.85).value)
        self.lidar_only_z_margin = float(
            self.declare_parameter('lidar_only_z_margin', 0.45).value)
        self.lidar_only_keep_last_points = parse_bool(
            self.declare_parameter('lidar_only_keep_last_points', True).value)
        self.confidence_camera_lidar = float(
            self.declare_parameter('confidence_camera_lidar', 0.9).value)
        self.confidence_lidar_only_decay_per_sec = float(
            self.declare_parameter('confidence_lidar_only_decay_per_sec', 0.25).value)
        self.confidence_prediction_decay_per_sec = float(
            self.declare_parameter('confidence_prediction_decay_per_sec', 0.55).value)

        self.enable_static_background_filter = parse_bool(
            self.declare_parameter('enable_static_background_filter', False).value)
        self.background_voxel_size = max(
            0.01, float(self.declare_parameter('background_voxel_size', 0.15).value))
        self.background_min_observations = max(
            1, int(self.declare_parameter('background_min_observations', 20).value))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info: Optional[CameraInfo] = None
        self.camera_target: Optional[TargetState] = None
        self.camera_target_received_time: Optional[Time] = None
        self.cloud_count = 0

        self.target_id = -1
        self.position_base: Optional[np.ndarray] = None
        self.velocity_base = np.zeros(3, dtype=np.float32)
        self.predicted_base: Optional[np.ndarray] = None
        self.confidence = 0.0
        self.track_state = TargetState.TRACK_NO_TARGET
        self.source_state = TargetState.SOURCE_NONE
        self.last_filter_time: Optional[Time] = None
        self.last_camera_seen_time: Optional[Time] = None
        self.last_lidar_seen_time: Optional[Time] = None
        self.latest_state = TargetState()
        self.latest_target_points_base = np.empty((0, 3), dtype=np.float32)
        self.latest_target_intensity = np.empty(0, dtype=np.float32)
        self.last_target_minimum: Optional[np.ndarray] = None
        self.last_target_maximum: Optional[np.ndarray] = None
        self.last_target_size: Optional[np.ndarray] = None
        self.last_measurement_base: Optional[np.ndarray] = None
        self.background_counts = {}
        self.camera_init_status = 'no_camera_target'
        self.projection_status = 'not_run'
        self.base_transform_status = 'not_run'
        self.projection_in_front_points = 0
        self.projection_roi_points_raw = 0
        self.projection_roi_points_depth_filtered = 0
        self.projection_uv_range = None
        self.lidar_only_candidate_count = 0
        self.lidar_only_selected_score = 0.0
        self.lidar_only_reject_reason = 'not_run'

        self.target_pub = self.create_publisher(TargetState, self.output_topic, 5)
        self.compat_target_pub = self.create_publisher(TargetState, self.compat_output_topic, 5)
        self.target_points_pub = self.create_publisher(PointCloud2, self.target_points_topic, 5)
        self.search_gate_marker_pub = self.create_publisher(
            MarkerArray, self.search_gate_marker_topic, 5)
        self.fused_marker_pub = self.create_publisher(MarkerArray, self.fused_marker_topic, 5)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 5)
        self.legacy_clusters_pub = self.create_publisher(
            LidarClusterArray, self.legacy_clusters_topic, 5)
        self.legacy_candidates_pub = self.create_publisher(
            LidarClusterArray, self.legacy_candidates_topic, 5)

        self.create_subscription(
            PointCloud2, self.lidar_topic, self.cloud_callback, self.lidar_qos_profile())
        self.create_subscription(TargetState, self.camera_target_topic, self.camera_target_callback, 5)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 5)
        self.create_timer(max(0.02, 1.0 / max(self.publish_rate, 1.0)), self.publish_latest_state)

        self.get_logger().info(
            f'Camera-initialized LiDAR target tracker: lidar={self.lidar_topic}, '
            f'camera_target={self.camera_target_topic}, output={self.output_topic}, '
            f'lidar_qos={self.lidar_qos_reliability}')

    def lidar_qos_profile(self):
        if self.lidar_qos_reliability in ('best_effort', 'besteffort', 'sensor_data'):
            return qos_profile_sensor_data
        return QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

    def camera_target_callback(self, msg: TargetState):
        self.camera_target = msg
        self.camera_target_received_time = self.get_clock().now()
        if msg.target_id < 0 or msg.lock_state in (
                TargetState.LOCK_NO_TARGET, TargetState.LOCK_CANDIDATE_VISIBLE):
            self.reset_target()
            return
        if msg.target_id != self.target_id:
            self.reset_target()
            self.target_id = int(msg.target_id)
            self.track_state = TargetState.TRACK_CAMERA_LOCKED
            self.source_state = TargetState.SOURCE_CAMERA_ONLY
            self.confidence = float(msg.confidence)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def cloud_callback(self, cloud: PointCloud2):
        self.cloud_count += 1
        if self.cloud_count % self.process_every_n_clouds != 0:
            return
        start = time.monotonic()

        try:
            lidar_points, intensity = cloud_xyz_intensity(cloud)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        source_frame = cloud.header.frame_id or self.lidar_frame
        now = self.get_clock().now()
        self.predict_to(now)

        base_points = self.transform_lidar_to_base(lidar_points, source_frame)
        if base_points is None:
            self.publish_debug(start, 'missing_base_tf', 0, 0, 0, None)
            return
        valid_base = self.valid_base_mask(base_points)
        base_points = base_points[valid_base]
        lidar_points = lidar_points[valid_base]
        intensity = intensity[valid_base]

        if self.enable_static_background_filter:
            self.update_background(base_points)

        associated_points = np.empty((0, 3), dtype=np.float32)
        associated_intensity = np.empty(0, dtype=np.float32)
        measurement = None
        measurement_source = 'none'
        self.lidar_only_candidate_count = 0
        self.lidar_only_selected_score = 0.0
        self.lidar_only_reject_reason = 'not_run'

        camera_points_count = 0
        camera_ready = self.camera_can_initialize()
        if camera_ready:
            selected = self.points_in_camera_bbox(lidar_points, source_frame)
            camera_points_count = int(np.count_nonzero(selected))
            if camera_points_count >= self.min_projected_points:
                associated_points = base_points[selected]
                associated_intensity = intensity[selected]
                associated_points, associated_intensity = self.refine_camera_association(
                    associated_points, associated_intensity)
                if associated_points.shape[0] >= self.min_projected_points:
                    measurement = self.robust_position(associated_points)
                    measurement_source = 'camera_lidar'

        if measurement is None and self.position_base is not None:
            candidate = self.local_lidar_association(base_points, intensity, now)
            if candidate is not None:
                associated_points = candidate.points
                associated_intensity = np.zeros(candidate.points.shape[0], dtype=np.float32)
                measurement = candidate.centroid
                measurement_source = 'lidar_only'

        if measurement is not None:
            time_since_previous_lidar = self.dt_since(self.last_lidar_seen_time, now)
            if measurement_source == 'lidar_only':
                self.correct_with_measurement(
                    measurement,
                    now,
                    self.lidar_only_alpha_position,
                    self.lidar_only_beta_velocity)
            else:
                self.correct_with_measurement(
                    measurement,
                    now,
                    self.alpha_position,
                    self.beta_velocity)
            self.latest_target_points_base = associated_points.astype(np.float32)
            self.latest_target_intensity = associated_intensity.astype(np.float32)
            self.update_target_signature(self.latest_target_points_base, measurement)
            self.last_lidar_seen_time = now
            if measurement_source == 'camera_lidar':
                self.last_camera_seen_time = now
                self.confidence = max(self.confidence, self.confidence_camera_lidar)
                self.track_state = TargetState.TRACK_CAMERA_LIDAR_TRACKED
                self.source_state = TargetState.SOURCE_CAMERA_LIDAR
            else:
                self.track_state = TargetState.TRACK_LIDAR_ONLY_TRACKING
                self.source_state = TargetState.SOURCE_LIDAR_ONLY
                self.confidence = max(
                    0.0,
                    self.confidence -
                    self.confidence_lidar_only_decay_per_sec * time_since_previous_lidar)
        else:
            if not self.lidar_only_keep_last_points:
                self.latest_target_points_base = np.empty((0, 3), dtype=np.float32)
                self.latest_target_intensity = np.empty(0, dtype=np.float32)
            self.update_prediction_state(now)

        self.publish_state(now, cloud.header)
        self.publish_target_points(cloud.header)
        self.publish_markers(cloud.header)
        self.publish_legacy_clusters(cloud.header)
        self.publish_debug(
            start, measurement_source, lidar_points.shape[0], camera_points_count,
            self.latest_target_points_base.shape[0], measurement)

    def camera_can_initialize(self) -> bool:
        self.projection_status = 'not_run'
        self.projection_in_front_points = 0
        self.projection_roi_points_raw = 0
        self.projection_roi_points_depth_filtered = 0
        self.projection_uv_range = None
        if self.camera_target is None:
            self.camera_init_status = 'no_camera_target'
            return False
        if self.camera_info is None:
            self.camera_init_status = 'no_camera_info'
            return False
        if self.camera_target.target_id < 0:
            self.camera_init_status = 'no_locked_target_id'
            return False
        if self.camera_target.lock_state != TargetState.LOCK_TARGET_LOCKED:
            self.camera_init_status = f'camera_lock_state_{int(self.camera_target.lock_state)}'
            return False
        if not self.camera_target.camera_visible:
            self.camera_init_status = 'camera_target_not_visible'
            return False
        if self.camera_target.bbox[2] <= self.camera_target.bbox[0]:
            self.camera_init_status = 'invalid_camera_bbox'
            return False
        if self.input_is_stale(self.camera_target_received_time, self.camera_target_timeout_sec):
            self.camera_init_status = 'stale_camera_target'
            return False
        self.camera_init_status = 'ready'
        return True

    def points_in_camera_bbox(self, lidar_points: np.ndarray, source_frame: str) -> np.ndarray:
        self.projection_in_front_points = 0
        self.projection_roi_points_raw = 0
        self.projection_roi_points_depth_filtered = 0
        self.projection_uv_range = None
        if self.camera_info is None or self.camera_target is None or lidar_points.shape[0] == 0:
            self.projection_status = 'missing_inputs'
            return np.zeros(lidar_points.shape[0], dtype=bool)

        camera_points = None
        if self.prefer_camera_lidar_extrinsic:
            camera_points = self.transform_lidar_to_camera_with_extrinsic_fallback(
                lidar_points, source_frame)
        if camera_points is None:
            camera_points = self.transform_array(lidar_points, source_frame, self.camera_frame)
            if camera_points is not None:
                self.projection_status = 'tf'
        if camera_points is None and not self.prefer_camera_lidar_extrinsic:
            camera_points = self.transform_lidar_to_camera_with_extrinsic_fallback(
                lidar_points, source_frame)
        if camera_points is None:
            self.projection_status = f'missing_tf_{source_frame}_to_{self.camera_frame}'
            return np.zeros(lidar_points.shape[0], dtype=bool)

        fx, fy, cx, cy = self.get_intrinsics(self.camera_info)
        z = camera_points[:, 2]
        finite = np.isfinite(camera_points).all(axis=1)
        in_front = z > 0.05
        u = fx * camera_points[:, 0] / np.maximum(z, 1e-6) + cx
        v = fy * camera_points[:, 1] / np.maximum(z, 1e-6) + cy
        projectable = finite & in_front
        self.projection_in_front_points = int(np.count_nonzero(projectable))
        if self.projection_in_front_points > 0:
            self.projection_uv_range = {
                'u_min': round(float(np.min(u[projectable])), 1),
                'u_max': round(float(np.max(u[projectable])), 1),
                'v_min': round(float(np.min(v[projectable])), 1),
                'v_max': round(float(np.max(v[projectable])), 1),
            }
        x1, y1, x2, y2 = [float(value) for value in self.camera_target.bbox]
        pad = self.bbox_padding_px
        selected = (
            projectable &
            (u >= x1 - pad) & (u <= x2 + pad) &
            (v >= y1 - pad) & (v <= y2 + pad))
        self.projection_roi_points_raw = int(np.count_nonzero(selected))
        if self.camera_roi_depth_filter and self.projection_roi_points_raw > 0:
            selected = self.filter_camera_roi_by_depth(selected, z)
        self.projection_roi_points_depth_filtered = int(np.count_nonzero(selected))

        if self.projection_in_front_points == 0:
            self.projection_status = f'{self.projection_status}_no_lidar_points_in_front_of_camera'
        elif np.count_nonzero(selected) == 0:
            self.projection_status = f'{self.projection_status}_projected_points_outside_bbox'
        else:
            self.projection_status = f'{self.projection_status}_projected_points_in_bbox'
        return selected

    def filter_camera_roi_by_depth(self, selected: np.ndarray, depth: np.ndarray) -> np.ndarray:
        roi_depth = depth[selected]
        if roi_depth.shape[0] < self.min_projected_points:
            return selected
        anchor = np.percentile(
            roi_depth,
            max(0.0, min(100.0, self.camera_roi_depth_percentile)))
        max_depth = float(anchor) + max(0.05, self.camera_roi_depth_window_m)
        filtered = selected.copy()
        filtered[selected] = roi_depth <= max_depth
        if np.count_nonzero(filtered) < self.min_projected_points:
            return selected
        return filtered

    def refine_camera_association(
            self,
            points: np.ndarray,
            intensity: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if (not self.camera_roi_cluster or
                points.shape[0] < max(self.min_projected_points * 2, 3)):
            return points, intensity
        labels = self.local_clusters(
            points,
            self.camera_roi_cluster_tolerance,
            self.min_projected_points)
        valid_labels = np.unique(labels[labels >= 0])
        if valid_labels.shape[0] == 0:
            return points, intensity

        best_label = None
        best_score = -float('inf')
        for label in valid_labels:
            cluster_points = points[labels == label]
            size = cluster_points.max(axis=0) - cluster_points.min(axis=0)
            compact_penalty = float(max(size[0], size[1]))
            score = float(cluster_points.shape[0]) - 5.0 * compact_penalty
            if score > best_score:
                best_score = score
                best_label = label
        cluster_mask = labels == best_label
        return points[cluster_mask], intensity[cluster_mask]

    def local_lidar_association(
            self, base_points: np.ndarray, intensity: np.ndarray, now: Time) -> Optional[LocalCandidate]:
        if self.predicted_base is None or base_points.shape[0] == 0:
            self.lidar_only_reject_reason = 'missing_prediction_or_points'
            return None
        profile = self.range_profile(self.predicted_base)
        delta = base_points[:, :2] - self.predicted_base[:2]
        distance_xy = np.linalg.norm(delta, axis=1)
        local_mask = distance_xy <= profile.search_radius
        if self.last_target_minimum is not None and self.last_target_maximum is not None:
            z_min = float(self.last_target_minimum[2] - self.lidar_only_z_margin)
            z_max = float(self.last_target_maximum[2] + self.lidar_only_z_margin)
            local_mask &= (base_points[:, 2] >= z_min) & (base_points[:, 2] <= z_max)
        local_points = base_points[local_mask]
        if self.enable_static_background_filter and local_points.shape[0] > 0:
            dynamic = ~self.background_static_mask(local_points)
            local_points = local_points[dynamic]
        if local_points.shape[0] < profile.min_points:
            self.lidar_only_reject_reason = f'not_enough_local_points_{local_points.shape[0]}'
            return None

        labels = self.local_clusters(local_points, profile.cluster_tolerance, profile.min_points)
        candidates = []
        for label in np.unique(labels[labels >= 0]):
            points = local_points[labels == label]
            if points.shape[0] < profile.min_points:
                continue
            candidate = self.describe_candidate(points, profile, now)
            if candidate is not None:
                candidates.append(candidate)
        self.lidar_only_candidate_count = len(candidates)
        if not candidates:
            if local_points.shape[0] >= profile.min_points:
                candidate = self.describe_candidate(local_points, profile, now)
                if candidate is not None:
                    self.lidar_only_candidate_count = 1
                    self.lidar_only_selected_score = float(candidate.score)
                    self.lidar_only_reject_reason = 'accepted_unclustered_local_points'
                    return candidate
            self.lidar_only_reject_reason = 'no_scored_candidates'
            return None
        best = max(candidates, key=lambda item: item.score)
        self.lidar_only_selected_score = float(best.score)
        self.lidar_only_reject_reason = 'accepted'
        return best

    def describe_candidate(
            self,
            points: np.ndarray,
            profile: RangeProfile,
            now: Time) -> Optional[LocalCandidate]:
        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        size = maximum - minimum
        width = float(size[0])
        depth = float(size[1])
        height = float(size[2])
        if width > self.max_cluster_width or depth > self.max_cluster_depth or height > self.max_cluster_height:
            self.lidar_only_reject_reason = 'size_too_large'
            return None
        centroid = self.robust_position(points)
        pred = self.predicted_base if self.predicted_base is not None else centroid
        distance_to_prediction = float(np.linalg.norm(centroid[:2] - pred[:2]))
        distance_to_last = 0.0
        if self.last_measurement_base is not None:
            distance_to_last = float(np.linalg.norm(centroid[:2] - self.last_measurement_base[:2]))
            if distance_to_last > self.allowed_lidar_only_jump(now):
                self.lidar_only_reject_reason = 'jump_too_large'
                return None
        prediction_bearing = math.atan2(float(pred[1]), float(pred[0]))
        candidate_bearing = math.atan2(float(centroid[1]), float(centroid[0]))
        bearing_error = abs(math.atan2(
            math.sin(candidate_bearing - prediction_bearing),
            math.cos(candidate_bearing - prediction_bearing)))
        distance_score = max(0.0, 1.0 - distance_to_prediction / max(profile.search_radius, 1e-3))
        last_score = 1.0
        if self.last_measurement_base is not None:
            last_score = max(0.0, 1.0 - distance_to_last / max(self.allowed_lidar_only_jump(now), 1e-3))
        bearing_score = max(0.0, 1.0 - bearing_error / max(0.8, 1e-3))
        point_score = min(1.0, points.shape[0] / max(profile.min_points * 3.0, 1.0))
        compact_score = max(0.0, 1.0 - max(width, depth) / max(self.max_cluster_width, self.max_cluster_depth, 1e-3))
        size_score = self.target_size_score(size)
        score = (
            0.34 * distance_score +
            0.24 * last_score +
            0.16 * bearing_score +
            0.12 * point_score +
            0.08 * compact_score +
            0.06 * size_score)
        if score < self.lidar_only_min_score:
            self.lidar_only_reject_reason = 'score_too_low'
            return None
        return LocalCandidate(
            points=points,
            centroid=centroid,
            minimum=minimum,
            maximum=maximum,
            score=score,
            point_count=int(points.shape[0]),
            distance_to_prediction=distance_to_prediction,
            bearing_error=bearing_error,
        )

    def allowed_lidar_only_jump(self, now: Time) -> float:
        dt = self.dt_since(self.last_lidar_seen_time, now)
        return max(self.lidar_only_max_jump_m, self.max_target_speed_mps * max(dt, 0.05) * 1.5)

    def target_size_score(self, size: np.ndarray) -> float:
        if self.last_target_size is None:
            return 0.5
        last_xy = np.maximum(self.last_target_size[:2], 0.05)
        current_xy = np.maximum(size[:2], 0.05)
        ratio_x = min(float(last_xy[0]), float(current_xy[0])) / max(float(last_xy[0]), float(current_xy[0]))
        ratio_y = min(float(last_xy[1]), float(current_xy[1])) / max(float(last_xy[1]), float(current_xy[1]))
        return max(0.0, min(1.0, 0.5 * (ratio_x + ratio_y)))

    @staticmethod
    def local_clusters(points: np.ndarray, tolerance: float, min_samples: int) -> np.ndarray:
        tree = cKDTree(points[:, :2])
        labels = np.full(points.shape[0], -1, dtype=np.int32)
        cluster_id = 0
        for seed_index in range(points.shape[0]):
            if labels[seed_index] >= 0:
                continue
            neighbors = tree.query_ball_point(points[seed_index, :2], tolerance)
            if len(neighbors) < min_samples:
                continue
            labels[seed_index] = cluster_id
            queue = deque(neighbors)
            while queue:
                index = queue.popleft()
                if labels[index] >= 0:
                    continue
                labels[index] = cluster_id
                more = tree.query_ball_point(points[index, :2], tolerance)
                if len(more) >= min_samples:
                    queue.extend(more)
            cluster_id += 1
        return labels

    def robust_position(self, points: np.ndarray) -> np.ndarray:
        if points.shape[0] == 0:
            return np.zeros(3, dtype=np.float32)
        low = np.percentile(points, self.target_point_percentile_low, axis=0)
        high = np.percentile(points, self.target_point_percentile_high, axis=0)
        mask = np.all((points >= low) & (points <= high), axis=1)
        trimmed = points[mask] if np.any(mask) else points
        return np.median(trimmed, axis=0).astype(np.float32)

    def predict_to(self, now: Time):
        if self.position_base is None:
            self.predicted_base = None
            self.last_filter_time = now
            return
        if self.last_filter_time is None:
            self.last_filter_time = now
            self.predicted_base = self.position_base.copy()
            return
        dt = max(0.0, (now - self.last_filter_time).nanoseconds * 1e-9)
        self.velocity_base = self.clamp_velocity(self.velocity_base)
        self.predicted_base = self.position_base + self.velocity_base * dt

    def correct_with_measurement(
            self,
            measurement: np.ndarray,
            now: Time,
            alpha_position: float,
            beta_velocity: float):
        if self.position_base is None or self.last_filter_time is None:
            self.position_base = measurement.astype(np.float32)
            self.predicted_base = self.position_base.copy()
            self.velocity_base = np.zeros(3, dtype=np.float32)
            self.last_filter_time = now
            return
        dt = max(1e-3, (now - self.last_filter_time).nanoseconds * 1e-9)
        predicted = self.predicted_base if self.predicted_base is not None else self.position_base
        residual = measurement - predicted
        self.position_base = predicted + alpha_position * residual
        self.velocity_base = self.velocity_base + (beta_velocity / dt) * residual
        self.velocity_base = self.clamp_velocity(self.velocity_base)
        self.predicted_base = self.position_base.copy()
        self.last_filter_time = now

    def clamp_velocity(self, velocity: np.ndarray) -> np.ndarray:
        speed_xy = float(np.linalg.norm(velocity[:2]))
        if speed_xy <= self.max_target_speed_mps or speed_xy <= 1e-6:
            return velocity.astype(np.float32)
        output = velocity.copy()
        output[:2] *= self.max_target_speed_mps / speed_xy
        return output.astype(np.float32)

    def update_target_signature(self, points: np.ndarray, measurement: np.ndarray):
        self.last_measurement_base = measurement.astype(np.float32)
        if points.shape[0] == 0:
            return
        self.last_target_minimum = points.min(axis=0).astype(np.float32)
        self.last_target_maximum = points.max(axis=0).astype(np.float32)
        self.last_target_size = (self.last_target_maximum - self.last_target_minimum).astype(np.float32)

    def update_prediction_state(self, now: Time):
        if self.position_base is None:
            self.track_state = TargetState.TRACK_CAMERA_LOCKED if self.target_id >= 0 else TargetState.TRACK_NO_TARGET
            self.source_state = TargetState.SOURCE_CAMERA_ONLY if self.target_id >= 0 else TargetState.SOURCE_NONE
            return
        time_since_lidar = self.time_since(self.last_lidar_seen_time, now)
        if time_since_lidar <= self.max_prediction_only_sec:
            self.track_state = TargetState.TRACK_PREDICTION_ONLY
            self.source_state = TargetState.SOURCE_PREDICTION_ONLY
            self.confidence = max(
                0.0,
                self.confidence - self.confidence_prediction_decay_per_sec * 0.05)
        elif time_since_lidar <= self.max_lidar_only_sec:
            self.track_state = TargetState.TRACK_PREDICTION_ONLY
            self.source_state = TargetState.SOURCE_PREDICTION_ONLY
            self.confidence = max(
                0.0,
                self.confidence - self.confidence_prediction_decay_per_sec * 0.10)
        else:
            self.track_state = TargetState.TRACK_TARGET_LOST
            self.source_state = TargetState.SOURCE_NONE
            self.confidence = 0.0

    def publish_state(self, now: Time, cloud_header: Header):
        state = TargetState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = self.base_frame
        state.target_id = int(self.target_id)
        state.lock_state = (
            TargetState.LOCK_TARGET_LOCKED
            if self.track_state in (
                TargetState.TRACK_CAMERA_LOCKED,
                TargetState.TRACK_CAMERA_LIDAR_TRACKED,
                TargetState.TRACK_LIDAR_ONLY_TRACKING,
                TargetState.TRACK_PREDICTION_ONLY)
            else TargetState.LOCK_TARGET_LOST)
        if self.target_id < 0:
            state.lock_state = TargetState.LOCK_NO_TARGET
        state.source_state = int(self.source_state)
        state.track_state = int(self.track_state)
        if self.camera_target is not None:
            state.bbox = self.camera_target.bbox
            state.camera_visible = bool(self.camera_target.camera_visible)
        state.lidar_visible = self.track_state in (
            TargetState.TRACK_CAMERA_LIDAR_TRACKED,
            TargetState.TRACK_LIDAR_ONLY_TRACKING)
        position = self.position_base if self.position_base is not None else self.predicted_base
        if position is not None:
            state.position_base = array_to_point(position)
            state.distance = float(math.hypot(position[0], position[1]))
            state.bearing = float(math.atan2(position[1], position[0]))
            state.velocity = Vector3(
                x=float(self.velocity_base[0]),
                y=float(self.velocity_base[1]),
                z=float(self.velocity_base[2]))
            map_position = self.transform_single(position, self.base_frame, self.map_frame)
            if map_position is not None:
                state.position_map_valid = True
                state.position_map = array_to_point(map_position)
        state.confidence = float(max(0.0, min(1.0, self.confidence)))
        state.time_since_camera_seen = float(self.time_since(self.last_camera_seen_time, now))
        state.time_since_lidar_seen = float(self.time_since(self.last_lidar_seen_time, now))
        self.latest_state = state
        self.target_pub.publish(state)
        self.compat_target_pub.publish(state)

    def publish_latest_state(self):
        if self.latest_state.header.stamp.sec == 0 and self.latest_state.header.stamp.nanosec == 0:
            return
        self.target_pub.publish(self.latest_state)
        self.compat_target_pub.publish(self.latest_state)

    def publish_target_points(self, cloud_header: Header):
        header = Header()
        header.stamp = cloud_header.stamp
        header.frame_id = self.base_frame
        self.target_points_pub.publish(make_xyzrgb_cloud(
            self.latest_target_points_base,
            self.latest_target_intensity,
            (255, 60, 30),
            header))

    def publish_markers(self, cloud_header: Header):
        now_header = Header()
        now_header.stamp = cloud_header.stamp
        now_header.frame_id = self.base_frame
        self.search_gate_marker_pub.publish(self.make_search_gate_marker(now_header))
        self.fused_marker_pub.publish(self.make_target_marker(now_header))

    def make_search_gate_marker(self, header: Header) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if self.predicted_base is None:
            return markers
        profile = self.range_profile(self.predicted_base)
        marker = Marker()
        marker.header = header
        marker.ns = 'target_search_gate'
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = array_to_point(self.predicted_base)
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.x = profile.search_radius * 2.0
        marker.scale.y = profile.search_radius * 2.0
        marker.scale.z = 1.8
        marker.color = ColorRGBA(r=0.1, g=0.55, b=1.0, a=0.22)
        marker.lifetime = DurationMsg(sec=0, nanosec=300000000)
        markers.markers.append(marker)
        return markers

    def make_target_marker(self, header: Header) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        position = self.position_base if self.position_base is not None else self.predicted_base
        if position is None or self.track_state in (TargetState.TRACK_NO_TARGET, TargetState.TRACK_TARGET_LOST):
            return markers
        sphere = Marker()
        sphere.header = header
        sphere.ns = 'fused_target'
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = array_to_point(position)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.35
        sphere.scale.y = 0.35
        sphere.scale.z = 0.35
        if self.track_state == TargetState.TRACK_PREDICTION_ONLY:
            sphere.color = ColorRGBA(r=1.0, g=0.8, b=0.1, a=0.9)
        elif self.track_state == TargetState.TRACK_LIDAR_ONLY_TRACKING:
            sphere.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=1.0)
        else:
            sphere.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0)
        sphere.lifetime = DurationMsg(sec=0, nanosec=300000000)
        markers.markers.append(sphere)

        text = Marker()
        text.header = header
        text.ns = 'fused_target_label'
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = array_to_point(position + np.array([0.0, 0.0, 0.45], dtype=np.float32))
        text.pose.orientation.w = 1.0
        text.scale.z = 0.18
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f'id={self.target_id} state={self.track_state} conf={self.confidence:.2f}'
        text.lifetime = DurationMsg(sec=0, nanosec=300000000)
        markers.markers.append(text)
        return markers

    def publish_legacy_clusters(self, cloud_header: Header):
        msg = LidarClusterArray()
        msg.header = cloud_header
        msg.header.frame_id = self.base_frame
        if self.latest_target_points_base.shape[0] > 0:
            points = self.latest_target_points_base
            minimum = points.min(axis=0)
            maximum = points.max(axis=0)
            size = maximum - minimum
            centroid = self.robust_position(points)
            cluster = LidarCluster()
            cluster.cluster_id = int(self.target_id)
            cluster.centroid = array_to_point(centroid)
            cluster.minimum = array_to_point(minimum)
            cluster.maximum = array_to_point(maximum)
            cluster.size = Vector3(
                x=float(size[0]),
                y=float(size[1]),
                z=float(size[2]))
            cluster.point_count = int(points.shape[0])
            cluster.distance = float(math.hypot(centroid[0], centroid[1]))
            cluster.confidence = float(max(0.0, min(1.0, self.confidence)))
            cluster.human_candidate = True
            msg.clusters.append(cluster)
        self.legacy_clusters_pub.publish(msg)
        self.legacy_candidates_pub.publish(msg)

    def publish_debug(
            self,
            start_time: float,
            source: str,
            valid_lidar_points: int,
            camera_projected_points: int,
            associated_points: int,
            measurement: Optional[np.ndarray]):
        now = self.get_clock().now()
        data = {
            'target_id': int(self.target_id),
            'track_state': int(self.track_state),
            'source_state': int(self.source_state),
            'source': source,
            'confidence': round(float(self.confidence), 3),
            'valid_lidar_points': int(valid_lidar_points),
            'camera_projected_points': int(camera_projected_points),
            'associated_points': int(associated_points),
            'camera_init_status': self.camera_init_status,
            'projection_status': self.projection_status,
            'base_transform_status': self.base_transform_status,
            'projection_in_front_points': int(self.projection_in_front_points),
            'projection_roi_points_raw': int(self.projection_roi_points_raw),
            'projection_roi_points_depth_filtered': int(
                self.projection_roi_points_depth_filtered),
            'lidar_only_candidate_count': int(self.lidar_only_candidate_count),
            'lidar_only_selected_score': round(float(self.lidar_only_selected_score), 3),
            'lidar_only_reject_reason': self.lidar_only_reject_reason,
            'allowed_lidar_only_jump': round(float(self.allowed_lidar_only_jump(now)), 3),
            'time_since_camera_seen': round(self.time_since(self.last_camera_seen_time, now), 3),
            'time_since_lidar_seen': round(self.time_since(self.last_lidar_seen_time, now), 3),
            'processing_ms': round((time.monotonic() - start_time) * 1000.0, 3),
        }
        if self.camera_target is not None:
            data['camera_lock_state'] = int(self.camera_target.lock_state)
            data['camera_visible'] = bool(self.camera_target.camera_visible)
            data['camera_bbox'] = [round(float(v), 1) for v in self.camera_target.bbox]
        if self.projection_uv_range is not None:
            data['projected_uv_range'] = self.projection_uv_range
        if self.predicted_base is not None:
            data['predicted_base'] = [round(float(v), 3) for v in self.predicted_base]
        if self.last_measurement_base is not None:
            data['last_measurement_base'] = [
                round(float(v), 3) for v in self.last_measurement_base]
        if self.last_target_minimum is not None and self.last_target_maximum is not None:
            data['last_target_z_range'] = [
                round(float(self.last_target_minimum[2]), 3),
                round(float(self.last_target_maximum[2]), 3)]
        if measurement is not None:
            data['measurement_base'] = [round(float(v), 3) for v in measurement]
        self.debug_pub.publish(String(data=json.dumps(data)))

    def range_profile(self, position: np.ndarray) -> RangeProfile:
        distance = float(math.hypot(position[0], position[1]))
        if distance < self.near_range_m:
            return RangeProfile(
                self.near_search_radius,
                self.near_min_points,
                self.local_cluster_tolerance_near,
                self.confidence_lidar_only_decay_per_sec)
        if distance > self.far_range_m:
            return RangeProfile(
                self.far_search_radius,
                self.far_min_points,
                self.local_cluster_tolerance_far,
                self.confidence_lidar_only_decay_per_sec * 1.5)
        return RangeProfile(
            self.mid_search_radius,
            self.mid_min_points,
            self.local_cluster_tolerance_mid,
            self.confidence_lidar_only_decay_per_sec)

    def valid_base_mask(self, base_points: np.ndarray) -> np.ndarray:
        finite = np.isfinite(base_points).all(axis=1)
        distance = np.linalg.norm(base_points[:, :2], axis=1)
        return (
            finite &
            (distance >= self.min_range) &
            (distance <= self.max_range) &
            (base_points[:, 2] >= self.min_z_base) &
            (base_points[:, 2] <= self.max_z_base))

    def update_background(self, base_points: np.ndarray):
        if base_points.shape[0] == 0:
            return
        keys = np.floor(base_points / self.background_voxel_size).astype(np.int32)
        unique = np.unique(keys, axis=0)
        for key in unique:
            self.background_counts[tuple(int(v) for v in key)] = (
                self.background_counts.get(tuple(int(v) for v in key), 0) + 1)

    def background_static_mask(self, base_points: np.ndarray) -> np.ndarray:
        keys = np.floor(base_points / self.background_voxel_size).astype(np.int32)
        return np.array([
            self.background_counts.get(tuple(int(v) for v in key), 0) >= self.background_min_observations
            for key in keys
        ], dtype=bool)

    def reset_target(self):
        self.target_id = -1
        self.position_base = None
        self.predicted_base = None
        self.velocity_base = np.zeros(3, dtype=np.float32)
        self.confidence = 0.0
        self.track_state = TargetState.TRACK_NO_TARGET
        self.source_state = TargetState.SOURCE_NONE
        self.last_filter_time = None
        self.last_camera_seen_time = None
        self.last_lidar_seen_time = None
        self.latest_target_points_base = np.empty((0, 3), dtype=np.float32)
        self.latest_target_intensity = np.empty(0, dtype=np.float32)
        self.last_target_minimum = None
        self.last_target_maximum = None
        self.last_target_size = None
        self.last_measurement_base = None
        self.lidar_only_candidate_count = 0
        self.lidar_only_selected_score = 0.0
        self.lidar_only_reject_reason = 'reset'

    def transform_array(self, points: np.ndarray, source_frame: str, target_frame: str) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.03))
        except TransformException:
            return None
        return transform_points(points, transform)

    @staticmethod
    def static_child_to_parent_transform(
            points: np.ndarray,
            translation_parent_child: np.ndarray,
            rotation_parent_child: np.ndarray) -> np.ndarray:
        return points @ rotation_parent_child.T + translation_parent_child.reshape(1, 3)

    def transform_lidar_to_base(
            self, points: np.ndarray, source_frame: str) -> Optional[np.ndarray]:
        self.base_transform_status = 'not_run'
        base_points = self.transform_array(points, source_frame, self.base_frame)
        if base_points is not None:
            self.base_transform_status = 'tf'
            return base_points
        if not self.use_base_lidar_extrinsic_fallback:
            self.base_transform_status = f'missing_tf_{source_frame}_to_{self.base_frame}'
            return None
        normalized_source = source_frame.lstrip('/')
        normalized_child = self.base_lidar_extrinsic_child_frame.lstrip('/')
        normalized_parent = self.base_lidar_extrinsic_parent_frame.lstrip('/')
        normalized_base = self.base_frame.lstrip('/')
        if normalized_source != normalized_child or normalized_parent != normalized_base:
            self.base_transform_status = (
                f'base_extrinsic_mismatch_source={source_frame}_'
                f'parent={self.base_lidar_extrinsic_parent_frame}')
            return None
        self.base_transform_status = 'base_extrinsic_fallback'
        return self.static_child_to_parent_transform(
            points,
            self.base_lidar_extrinsic_translation,
            self.base_lidar_extrinsic_rotation_parent_child)

    def transform_lidar_to_camera_with_extrinsic_fallback(
            self, points: np.ndarray, source_frame: str) -> Optional[np.ndarray]:
        if not self.use_camera_lidar_extrinsic_fallback:
            return None
        normalized_source = source_frame.lstrip('/')
        normalized_child = self.extrinsic_child_frame.lstrip('/')
        if normalized_source != normalized_child:
            self.projection_status = f'extrinsic_source_mismatch_{source_frame}'
            return None

        # Parameters follow tf2_ros static_transform_publisher order:
        # x y z yaw pitch roll parent_frame child_frame. The translation is the
        # LiDAR origin expressed in the parent frame, so child/LiDAR points move
        # into the parent frame as R * point + t.
        parent_points = self.static_child_to_parent_transform(
            points,
            self.extrinsic_translation,
            self.extrinsic_rotation_parent_child)
        if self.extrinsic_parent_frame == self.camera_frame:
            self.projection_status = 'extrinsic_fallback'
            return parent_points
        camera_points = self.transform_array(
            parent_points, self.extrinsic_parent_frame, self.camera_frame)
        if camera_points is None:
            self.projection_status = (
                f'extrinsic_missing_tf_{self.extrinsic_parent_frame}_to_{self.camera_frame}')
            return None
        self.projection_status = 'extrinsic_fallback'
        return camera_points

    def transform_single(self, point: np.ndarray, source_frame: str, target_frame: str) -> Optional[np.ndarray]:
        transformed = self.transform_array(point.reshape(1, 3), source_frame, target_frame)
        if transformed is None:
            return None
        return transformed[0]

    @staticmethod
    def get_intrinsics(info: CameraInfo):
        if len(info.p) >= 12 and info.p[0] != 0.0 and info.p[5] != 0.0:
            return float(info.p[0]), float(info.p[5]), float(info.p[2]), float(info.p[6])
        return float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])

    def input_is_stale(self, received_time: Optional[Time], timeout_sec: float) -> bool:
        if received_time is None:
            return True
        return (self.get_clock().now() - received_time).nanoseconds * 1e-9 > timeout_sec

    @staticmethod
    def time_since(stamp: Optional[Time], now: Time) -> float:
        if stamp is None:
            return float('inf')
        return max(0.0, (now - stamp).nanoseconds * 1e-9)

    @staticmethod
    def dt_since(stamp: Optional[Time], now: Time) -> float:
        if stamp is None:
            return 0.0
        return max(0.0, (now - stamp).nanoseconds * 1e-9)


def main(args=None):
    rclpy.init(args=args)
    node = LidarHumanClusterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
