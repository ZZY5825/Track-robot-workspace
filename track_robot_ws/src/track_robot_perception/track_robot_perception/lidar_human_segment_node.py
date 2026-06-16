#!/usr/bin/env python3

import json
import struct
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from track_robot_perception.lidar_ground_segment_node import (
    cloud_field_array,
    contiguous_point_bytes,
    field_by_name,
    intensity_grayscale,
    pack_rgb,
    parse_bool,
)


@dataclass
class HumanCandidate:
    candidate_id: int
    sample_indices: np.ndarray
    minimum: np.ndarray
    maximum: np.ndarray
    centroid: np.ndarray
    point_count: int
    verticality: float
    aspect_ratio: float
    ground_clearance: float

    @property
    def size(self):
        return self.maximum - self.minimum


def dbscan(
        points: np.ndarray,
        eps: float,
        min_samples: int) -> np.ndarray:
    if points.shape[0] == 0:
        return np.empty(0, dtype=np.int32)

    tree = cKDTree(points)
    unvisited = -2
    noise = -1
    labels = np.full(points.shape[0], unvisited, dtype=np.int32)
    queued = np.zeros(points.shape[0], dtype=bool)
    cluster_id = 0

    for seed_index in range(points.shape[0]):
        if labels[seed_index] != unvisited:
            continue
        neighbors = tree.query_ball_point(points[seed_index], eps)
        if len(neighbors) < min_samples:
            labels[seed_index] = noise
            continue

        labels[seed_index] = cluster_id
        queue = deque()
        for neighbor in neighbors:
            if neighbor != seed_index and not queued[neighbor]:
                queue.append(neighbor)
                queued[neighbor] = True

        while queue:
            point_index = queue.popleft()
            if labels[point_index] == noise:
                labels[point_index] = cluster_id
            if labels[point_index] != unvisited:
                continue

            labels[point_index] = cluster_id
            point_neighbors = tree.query_ball_point(points[point_index], eps)
            if len(point_neighbors) >= min_samples:
                for neighbor in point_neighbors:
                    if labels[neighbor] <= noise and not queued[neighbor]:
                        queue.append(neighbor)
                        queued[neighbor] = True
        cluster_id += 1
    return labels


def make_human_colored_cloud(
        cloud: PointCloud2,
        human_ids: np.ndarray,
        human_color) -> PointCloud2:
    point_count = cloud.width * cloud.height
    existing_rgb = field_by_name(cloud, 'rgb')
    append_rgb = (
        existing_rgb is None or
        existing_rgb.datatype not in (PointField.FLOAT32, PointField.UINT32))

    rgb_offset = cloud.point_step if append_rgb else existing_rgb.offset
    appended_base = cloud.point_step + (4 if append_rgb else 0)
    human_flag_offset = appended_base
    human_id_offset = appended_base + 4
    output_point_step = appended_base + 8

    input_points = contiguous_point_bytes(cloud)
    output = bytearray(point_count * output_point_step)
    endian = '>' if cloud.is_bigendian else '<'
    human_mask = human_ids >= 0

    if append_rgb:
        intensity = cloud_field_array(cloud, 'intensity')
        colors = intensity_grayscale(intensity, point_count)
        packed_colors = pack_rgb(colors)
    else:
        packed_colors = None

    packed_human_color = int(pack_rgb(
        np.asarray([human_color], dtype=np.uint8))[0])

    for index in range(point_count):
        input_start = index * cloud.point_step
        output_start = index * output_point_step
        output[output_start:output_start + cloud.point_step] = (
            input_points[input_start:input_start + cloud.point_step])

        if append_rgb:
            struct.pack_into(
                endian + 'I',
                output,
                output_start + rgb_offset,
                int(packed_colors[index]))
        if human_mask[index]:
            struct.pack_into(
                endian + 'I',
                output,
                output_start + rgb_offset,
                packed_human_color)
            output[output_start + human_flag_offset] = 1
        struct.pack_into(
            endian + 'i',
            output,
            output_start + human_id_offset,
            int(human_ids[index]))

    msg = PointCloud2()
    msg.header = cloud.header
    msg.height = cloud.height
    msg.width = cloud.width
    msg.fields = list(cloud.fields)
    if append_rgb:
        msg.fields.append(PointField(
            name='rgb', offset=rgb_offset, datatype=PointField.FLOAT32, count=1))
    msg.fields.append(PointField(
        name='is_human',
        offset=human_flag_offset,
        datatype=PointField.UINT8,
        count=1))
    msg.fields.append(PointField(
        name='human_cluster_id',
        offset=human_id_offset,
        datatype=PointField.INT32,
        count=1))
    msg.is_bigendian = cloud.is_bigendian
    msg.point_step = output_point_step
    msg.row_step = output_point_step * cloud.width
    msg.is_dense = cloud.is_dense
    msg.data = bytes(output)
    return msg


class LidarHumanSegmentNode(Node):
    def __init__(self):
        super().__init__('lidar_human_segment_node')

        self.input_topic = self.declare_parameter(
            'input_topic', '/lidar_ground_segmented_points').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/lidar_human_segmented_points').value
        self.debug_topic = self.declare_parameter(
            'debug_topic', '/lidar_human_candidates_debug').value

        self.min_range = float(self.declare_parameter('min_range', 0.5).value)
        self.max_range = float(self.declare_parameter('max_range', 20.0).value)
        self.min_z = float(self.declare_parameter('min_z', -3.0).value)
        self.max_z = float(self.declare_parameter('max_z', 3.5).value)
        self.voxel_size = max(
            0.0, float(self.declare_parameter('voxel_size', 0.08).value))
        self.max_sample_points = max(
            100, int(self.declare_parameter('max_sample_points', 25000).value))
        self.cluster_tolerance = max(
            0.05, float(self.declare_parameter('cluster_tolerance', 0.35).value))
        self.cluster_min_samples = max(
            1, int(self.declare_parameter('cluster_min_samples', 4).value))
        self.use_height = parse_bool(
            self.declare_parameter('use_height', True).value)

        self.min_cluster_points = max(
            3, int(self.declare_parameter('min_cluster_points', 8).value))
        self.max_cluster_points = max(
            self.min_cluster_points,
            int(self.declare_parameter('max_cluster_points', 2000).value))
        self.min_human_height = float(
            self.declare_parameter('min_human_height', 0.7).value)
        self.max_human_height = float(
            self.declare_parameter('max_human_height', 2.4).value)
        self.min_human_width = float(
            self.declare_parameter('min_human_width', 0.12).value)
        self.max_human_width = float(
            self.declare_parameter('max_human_width', 1.2).value)
        self.max_footprint_area = float(
            self.declare_parameter('max_footprint_area', 1.0).value)
        self.min_aspect_ratio = float(
            self.declare_parameter('min_aspect_ratio', 1.0).value)
        self.min_verticality = float(
            self.declare_parameter('min_verticality', 0.55).value)
        self.local_ground_radius = float(
            self.declare_parameter('local_ground_radius', 1.0).value)
        self.min_ground_clearance = float(
            self.declare_parameter('min_ground_clearance', -0.25).value)
        self.max_ground_clearance = float(
            self.declare_parameter('max_ground_clearance', 0.45).value)
        self.human_color = self.parse_color(
            self.declare_parameter('human_color', [255, 30, 30]).value)
        self.process_every_n_clouds = max(
            1, int(self.declare_parameter('process_every_n_clouds', 1).value))
        self.debug_timing = parse_bool(
            self.declare_parameter('debug_timing', True).value)

        self.cloud_count = 0
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 5)
        self.debug_publisher = self.create_publisher(String, self.debug_topic, 5)
        self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Human candidate segmentation {self.input_topic} -> {self.output_topic}; '
            f'eps={self.cluster_tolerance:.2f}m; '
            f'height={self.min_human_height:.2f}-{self.max_human_height:.2f}m')

    @staticmethod
    def parse_color(value):
        if isinstance(value, str):
            parts = [int(part.strip()) for part in value.split(',')]
        else:
            parts = [int(part) for part in value]
        if len(parts) != 3:
            raise ValueError('human_color must contain three RGB values')
        return tuple(max(0, min(255, part)) for part in parts)

    def cloud_callback(self, cloud: PointCloud2):
        self.cloud_count += 1
        if self.cloud_count % self.process_every_n_clouds != 0:
            return

        start = time.monotonic()
        x = cloud_field_array(cloud, 'x')
        y = cloud_field_array(cloud, 'y')
        z = cloud_field_array(cloud, 'z')
        is_ground = cloud_field_array(cloud, 'is_ground')
        if x is None or y is None or z is None:
            self.get_logger().warn('Input PointCloud2 must contain x, y, z fields')
            return
        if is_ground is None:
            self.get_logger().warn(
                'Input PointCloud2 must contain is_ground; run '
                'lidar_ground_segment_node before this node')
            return

        points = np.column_stack((x, y, z)).astype(np.float32)
        finite = np.isfinite(points).all(axis=1)
        distance = np.linalg.norm(points, axis=1)
        roi = (
            finite &
            (distance >= self.min_range) &
            (distance <= self.max_range) &
            (points[:, 2] >= self.min_z) &
            (points[:, 2] <= self.max_z))
        ground_mask = roi & (is_ground.astype(np.uint8) != 0)
        foreground_mask = roi & ~ground_mask

        human_ids, candidates, sample_count = self.detect_candidates(
            points, ground_mask, foreground_mask)
        self.publisher.publish(make_human_colored_cloud(
            cloud, human_ids, self.human_color))

        elapsed_ms = (time.monotonic() - start) * 1000.0
        debug = {
            'input_points': int(points.shape[0]),
            'ground_points': int(np.count_nonzero(ground_mask)),
            'foreground_points': int(np.count_nonzero(foreground_mask)),
            'sample_points': sample_count,
            'human_candidate_count': len(candidates),
            'human_points': int(np.count_nonzero(human_ids >= 0)),
            'processing_ms': round(elapsed_ms, 3),
            'candidates': [self.candidate_to_dict(item) for item in candidates],
        }
        self.debug_publisher.publish(String(data=json.dumps(debug)))

        if self.debug_timing:
            self.get_logger().info(
                f'Human candidates {elapsed_ms:.1f} ms; '
                f'foreground={debug["foreground_points"]}; '
                f'samples={sample_count}; candidates={len(candidates)}; '
                f'human_points={debug["human_points"]}')

    def detect_candidates(
            self,
            points: np.ndarray,
            ground_mask: np.ndarray,
            foreground_mask: np.ndarray):
        human_ids = np.full(points.shape[0], -1, dtype=np.int32)
        foreground_indices = np.flatnonzero(foreground_mask)
        if foreground_indices.size == 0:
            return human_ids, [], 0

        sample_indices = self.voxel_sample_indices(
            points, foreground_indices, self.voxel_size)
        if sample_indices.size > self.max_sample_points:
            keep = np.linspace(
                0,
                sample_indices.size - 1,
                self.max_sample_points,
                dtype=np.int64)
            sample_indices = sample_indices[keep]
        sample_points = points[sample_indices]
        cluster_coordinates = (
            sample_points if self.use_height else sample_points[:, :2])
        labels = dbscan(
            cluster_coordinates,
            self.cluster_tolerance,
            self.cluster_min_samples)

        ground_points = points[ground_mask]
        ground_tree = (
            cKDTree(ground_points[:, :2]) if ground_points.shape[0] else None)
        ground_normal = self.estimate_ground_normal(ground_points)
        candidates = self.filter_human_candidates(
            sample_points,
            labels,
            ground_points,
            ground_tree,
            ground_normal)

        sample_human_ids = np.full(sample_points.shape[0], -1, dtype=np.int32)
        for candidate in candidates:
            sample_human_ids[candidate.sample_indices] = candidate.candidate_id

        sample_tree = cKDTree(cluster_coordinates)
        foreground_coordinates = (
            points[foreground_indices] if self.use_height
            else points[foreground_indices, :2])
        nearest_distance, nearest_index = sample_tree.query(
            foreground_coordinates, k=1)
        assigned = nearest_distance <= max(
            self.cluster_tolerance, self.voxel_size * 2.0)
        assigned_ids = sample_human_ids[nearest_index]
        valid_assignment = assigned & (assigned_ids >= 0)
        human_ids[foreground_indices[valid_assignment]] = (
            assigned_ids[valid_assignment])
        return human_ids, candidates, int(sample_points.shape[0])

    @staticmethod
    def voxel_sample_indices(
            points: np.ndarray,
            source_indices: np.ndarray,
            voxel_size: float) -> np.ndarray:
        if voxel_size <= 0.0 or source_indices.size == 0:
            return source_indices
        voxel_keys = np.floor(
            points[source_indices] / voxel_size).astype(np.int32)
        _, first = np.unique(voxel_keys, axis=0, return_index=True)
        return source_indices[np.sort(first)]

    @staticmethod
    def estimate_ground_normal(ground_points: np.ndarray) -> np.ndarray:
        if ground_points.shape[0] < 3:
            return np.array([0.0, 0.0, 1.0], dtype=np.float32)
        if ground_points.shape[0] > 5000:
            indices = np.linspace(
                0, ground_points.shape[0] - 1, 5000, dtype=np.int64)
            ground_points = ground_points[indices]
        centered = ground_points - ground_points.mean(axis=0)
        covariance = centered.T @ centered
        _, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, 0]
        if normal[2] < 0.0:
            normal = -normal
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            return np.array([0.0, 0.0, 1.0], dtype=np.float32)
        return (normal / norm).astype(np.float32)

    def filter_human_candidates(
            self,
            sample_points: np.ndarray,
            labels: np.ndarray,
            ground_points: np.ndarray,
            ground_tree: Optional[cKDTree],
            ground_normal: np.ndarray) -> List[HumanCandidate]:
        candidates = []
        for raw_label in np.unique(labels[labels >= 0]):
            indices = np.flatnonzero(labels == raw_label)
            if not (
                    self.min_cluster_points <= indices.size <=
                    self.max_cluster_points):
                continue

            cluster = sample_points[indices]
            minimum = cluster.min(axis=0)
            maximum = cluster.max(axis=0)
            size = maximum - minimum
            height = float(size[2])
            width = max(float(size[0]), float(size[1]))
            footprint_area = float(size[0] * size[1])
            if not (
                    self.min_human_height <= height <= self.max_human_height and
                    self.min_human_width <= width <= self.max_human_width and
                    footprint_area <= self.max_footprint_area):
                continue

            aspect_ratio = height / max(width, 0.05)
            if aspect_ratio < self.min_aspect_ratio:
                continue

            centered = cluster - cluster.mean(axis=0)
            covariance = centered.T @ centered
            _, eigenvectors = np.linalg.eigh(covariance)
            main_axis = eigenvectors[:, -1]
            verticality = abs(float(np.dot(main_axis, ground_normal)))
            if verticality < self.min_verticality:
                continue

            centroid = cluster.mean(axis=0)
            local_ground_z = self.local_ground_height(
                centroid[:2], ground_points, ground_tree)
            if local_ground_z is None:
                continue
            ground_clearance = float(minimum[2] - local_ground_z)
            if not (
                    self.min_ground_clearance <= ground_clearance <=
                    self.max_ground_clearance):
                continue

            candidates.append(HumanCandidate(
                candidate_id=len(candidates),
                sample_indices=indices,
                minimum=minimum,
                maximum=maximum,
                centroid=centroid,
                point_count=int(indices.size),
                verticality=verticality,
                aspect_ratio=aspect_ratio,
                ground_clearance=ground_clearance,
            ))
        return candidates

    def local_ground_height(
            self,
            xy: np.ndarray,
            ground_points: np.ndarray,
            ground_tree: Optional[cKDTree]) -> Optional[float]:
        if ground_tree is None:
            return None
        local_indices = ground_tree.query_ball_point(
            xy, self.local_ground_radius)
        if not local_indices:
            return None
        return float(np.median(ground_points[local_indices, 2]))

    @staticmethod
    def candidate_to_dict(candidate: HumanCandidate):
        return {
            'candidate_id': candidate.candidate_id,
            'centroid': [
                round(float(value), 3) for value in candidate.centroid],
            'size': [round(float(value), 3) for value in candidate.size],
            'sample_point_count': candidate.point_count,
            'verticality': round(candidate.verticality, 3),
            'aspect_ratio': round(candidate.aspect_ratio, 3),
            'ground_clearance': round(candidate.ground_clearance, 3),
        }


def main(args=None):
    rclpy.init(args=args)
    node = LidarHumanSegmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
