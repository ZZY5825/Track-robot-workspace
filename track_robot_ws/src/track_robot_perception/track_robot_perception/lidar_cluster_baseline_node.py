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
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray


POINT_FIELD_TO_DTYPE = {
    PointField.INT8: 'i1',
    PointField.UINT8: 'u1',
    PointField.INT16: 'i2',
    PointField.UINT16: 'u2',
    PointField.INT32: 'i4',
    PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
}

CLUSTER_POINT_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('intensity', '<f4'),
    ('rgb', '<f4'),
    ('cluster_id', '<i4'),
])


@dataclass
class ClusterInfo:
    cluster_id: int
    indices: np.ndarray
    centroid: np.ndarray
    minimum: np.ndarray
    maximum: np.ndarray
    size: np.ndarray
    point_count: int
    distance: float
    density: float


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def field_by_name(cloud: PointCloud2, name: str) -> Optional[PointField]:
    for field in cloud.fields:
        if field.name == name:
            return field
    return None


def cloud_field_array(cloud: PointCloud2, name: str) -> Optional[np.ndarray]:
    field = field_by_name(cloud, name)
    if field is None or field.datatype not in POINT_FIELD_TO_DTYPE:
        return None

    endian = '>' if cloud.is_bigendian else '<'
    dtype = np.dtype({
        'names': [name],
        'formats': [endian + POINT_FIELD_TO_DTYPE[field.datatype]],
        'offsets': [field.offset],
        'itemsize': cloud.point_step,
    })
    return np.frombuffer(
        cloud.data,
        dtype=dtype,
        count=cloud.width * cloud.height)[name]


def cloud_xyz_intensity(cloud: PointCloud2) -> Tuple[np.ndarray, np.ndarray]:
    fields = [field_by_name(cloud, name) for name in ('x', 'y', 'z')]
    if any(field is None for field in fields):
        raise ValueError('PointCloud2 must contain x, y, z fields')
    if any(field.datatype != PointField.FLOAT32 for field in fields):
        raise ValueError('PointCloud2 x, y, z fields must be FLOAT32')

    endian = '>' if cloud.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4', endian + 'f4', endian + 'f4'],
        'offsets': [field.offset for field in fields],
        'itemsize': cloud.point_step,
    })
    values = np.frombuffer(
        cloud.data, dtype=dtype, count=cloud.width * cloud.height)
    points = np.column_stack(
        (values['x'], values['y'], values['z'])).astype(np.float32)

    intensity_field = cloud_field_array(cloud, 'intensity')
    if intensity_field is None:
        intensity = np.zeros(points.shape[0], dtype=np.float32)
    else:
        intensity = intensity_field.astype(np.float32)
    return points, intensity


def cluster_rgb(cluster_ids: np.ndarray) -> np.ndarray:
    colors = np.full((cluster_ids.shape[0], 3), 80, dtype=np.uint8)
    valid = cluster_ids >= 0
    if not np.any(valid):
        return colors

    ids = cluster_ids[valid].astype(np.uint32) + np.uint32(1)
    colors[valid, 0] = ((ids * 97) % 190 + 50).astype(np.uint8)
    colors[valid, 1] = ((ids * 57) % 190 + 50).astype(np.uint8)
    colors[valid, 2] = ((ids * 137) % 190 + 50).astype(np.uint8)
    return colors


def rgb_to_pcl_float(colors: np.ndarray) -> np.ndarray:
    packed = (
        colors[:, 0].astype(np.uint32) << 16 |
        colors[:, 1].astype(np.uint32) << 8 |
        colors[:, 2].astype(np.uint32)
    )
    return packed.astype('<u4').view('<f4')


def make_clustered_cloud(
        points: np.ndarray,
        intensity: np.ndarray,
        cluster_ids: np.ndarray,
        header: Header) -> PointCloud2:
    data = np.empty(points.shape[0], dtype=CLUSTER_POINT_DTYPE)
    data['x'] = points[:, 0]
    data['y'] = points[:, 1]
    data['z'] = points[:, 2]
    data['intensity'] = intensity
    data['rgb'] = rgb_to_pcl_float(cluster_rgb(cluster_ids))
    data['cluster_id'] = cluster_ids

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
        PointField(name='cluster_id', offset=20, datatype=PointField.INT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = CLUSTER_POINT_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = False
    msg.data = data.tobytes()
    return msg


class LidarClusterBaselineNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_baseline_node')

        self.lidar_topic = self.declare_parameter('lidar_topic', '/rslidar_points').value
        self.marker_topic = self.declare_parameter(
            'marker_topic', '/lidar_cluster_markers').value
        self.debug_topic = self.declare_parameter(
            'debug_topic', '/lidar_clusters_debug').value
        self.clustered_cloud_topic = self.declare_parameter(
            'clustered_cloud_topic', '/lidar_clustered_points').value
        self.fixed_frame = self.declare_parameter('fixed_frame', 'rslidar').value

        self.min_range = float(self.declare_parameter('min_range', 0.5).value)
        self.max_range = float(self.declare_parameter('max_range', 15.0).value)
        self.min_x = float(self.declare_parameter('min_x', -15.0).value)
        self.max_x = float(self.declare_parameter('max_x', 15.0).value)
        self.min_y = float(self.declare_parameter('min_y', -8.0).value)
        self.max_y = float(self.declare_parameter('max_y', 8.0).value)
        self.min_z = float(self.declare_parameter('min_z', -2.0).value)
        self.max_z = float(self.declare_parameter('max_z', 3.0).value)
        self.ground_z_threshold = float(
            self.declare_parameter('ground_z_threshold', -0.7).value)

        self.method = str(self.declare_parameter('method', 'dbscan').value).lower()
        if self.method not in ('dbscan', 'euclidean'):
            raise ValueError('method must be dbscan or euclidean')
        self.dbscan_eps = float(self.declare_parameter('dbscan_eps', 0.35).value)
        self.dbscan_min_samples = max(
            1, int(self.declare_parameter('dbscan_min_samples', 8).value))
        self.euclidean_tolerance = float(
            self.declare_parameter('euclidean_tolerance', 0.35).value)
        self.min_cluster_points = max(
            1, int(self.declare_parameter('min_cluster_points', 20).value))
        self.max_cluster_points = max(
            self.min_cluster_points,
            int(self.declare_parameter('max_cluster_points', 5000).value))

        self.voxel_size = max(0.0, float(self.declare_parameter('voxel_size', 0.05).value))
        self.max_points_before_clustering = max(
            1, int(self.declare_parameter('max_points_before_clustering', 30000).value))
        self.process_every_n_clouds = max(
            1, int(self.declare_parameter('process_every_n_clouds', 2).value))

        self.min_cluster_height = float(
            self.declare_parameter('min_cluster_height', 0.05).value)
        self.max_cluster_height = float(
            self.declare_parameter('max_cluster_height', 3.0).value)
        self.min_cluster_width = float(
            self.declare_parameter('min_cluster_width', 0.05).value)
        self.max_cluster_width = float(
            self.declare_parameter('max_cluster_width', 4.0).value)

        self.debug_timing = parse_bool(
            self.declare_parameter('debug_timing', True).value)
        self.publish_clustered_cloud = parse_bool(
            self.declare_parameter('publish_clustered_cloud', False).value)
        self.publish_markers = parse_bool(
            self.declare_parameter('publish_markers', True).value)

        self.cloud_count = 0
        self.warned_frame_mismatch = False
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 5)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 5)
        self.clustered_cloud_pub = None
        if self.publish_clustered_cloud:
            self.clustered_cloud_pub = self.create_publisher(
                PointCloud2, self.clustered_cloud_topic, 5)
        self.create_subscription(
            PointCloud2, self.lidar_topic, self.cloud_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'LiDAR clustering {self.lidar_topic} with method={self.method}; '
            f'markers={self.marker_topic}; clustered_cloud={self.publish_clustered_cloud}')

    def cloud_callback(self, cloud: PointCloud2):
        self.cloud_count += 1
        if self.cloud_count % self.process_every_n_clouds != 0:
            return

        start = time.monotonic()
        try:
            input_points, input_intensity = cloud_xyz_intensity(cloud)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        points, intensity, filter_counts = self.filter_points(
            input_points, input_intensity)
        points, intensity = self.voxel_downsample(points, intensity)
        if points.shape[0] > self.max_points_before_clustering:
            indices = np.linspace(
                0,
                points.shape[0] - 1,
                self.max_points_before_clustering,
                dtype=np.int64)
            points = points[indices]
            intensity = intensity[indices]

        if points.shape[0] == 0:
            labels = np.empty(0, dtype=np.int32)
        elif self.method == 'dbscan':
            labels = self.dbscan(points, self.dbscan_eps, self.dbscan_min_samples)
        else:
            labels = self.euclidean_clusters(points, self.euclidean_tolerance)

        accepted_labels, clusters = self.filter_and_describe_clusters(points, labels)
        frame_id = self.fixed_frame if self.fixed_frame else cloud.header.frame_id
        if (
                self.fixed_frame and cloud.header.frame_id and
                self.fixed_frame != cloud.header.frame_id and
                not self.warned_frame_mismatch):
            self.get_logger().warn(
                f'fixed_frame={self.fixed_frame} differs from cloud frame '
                f'{cloud.header.frame_id}; no coordinate transform is applied')
            self.warned_frame_mismatch = True

        header = Header()
        header.stamp = cloud.header.stamp
        header.frame_id = frame_id
        if self.publish_markers:
            self.marker_pub.publish(self.make_markers(clusters, header))
        if self.publish_clustered_cloud and self.clustered_cloud_pub is not None:
            labelled = accepted_labels >= 0
            self.clustered_cloud_pub.publish(make_clustered_cloud(
                points[labelled],
                intensity[labelled],
                accepted_labels[labelled],
                header))

        elapsed_ms = (time.monotonic() - start) * 1000.0
        debug = {
            'input_points': int(input_points.shape[0]),
            'finite_range_roi_points': filter_counts['roi'],
            'after_ground_removal': filter_counts['non_ground'],
            'after_downsampling': int(points.shape[0]),
            'noise_points': int(np.count_nonzero(accepted_labels < 0)),
            'cluster_count': len(clusters),
            'processing_ms': round(elapsed_ms, 3),
            'method': self.method,
            'clusters': [self.cluster_to_dict(cluster) for cluster in clusters],
        }
        self.debug_pub.publish(String(data=json.dumps(debug)))

        if self.debug_timing:
            self.get_logger().info(
                f'Clustering {elapsed_ms:.1f} ms; input={input_points.shape[0]}; '
                f'filtered={filter_counts["non_ground"]}; downsampled={points.shape[0]}; '
                f'clusters={len(clusters)}')

    def filter_points(self, points: np.ndarray, intensity: np.ndarray):
        finite = np.isfinite(points).all(axis=1)
        distance = np.linalg.norm(points, axis=1)
        in_range = (distance >= self.min_range) & (distance <= self.max_range)
        in_roi = (
            (points[:, 0] >= self.min_x) & (points[:, 0] <= self.max_x) &
            (points[:, 1] >= self.min_y) & (points[:, 1] <= self.max_y) &
            (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z))
        roi_mask = finite & in_range & in_roi
        non_ground = roi_mask & (points[:, 2] > self.ground_z_threshold)
        return (
            points[non_ground],
            intensity[non_ground],
            {
                'roi': int(np.count_nonzero(roi_mask)),
                'non_ground': int(np.count_nonzero(non_ground)),
            })

    def voxel_downsample(self, points: np.ndarray, intensity: np.ndarray):
        if self.voxel_size <= 0.0 or points.shape[0] == 0:
            return points, intensity
        voxel_keys = np.floor(points / self.voxel_size).astype(np.int32)
        _, first_indices = np.unique(voxel_keys, axis=0, return_index=True)
        first_indices.sort()
        return points[first_indices], intensity[first_indices]

    @staticmethod
    def dbscan(points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
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

    @staticmethod
    def euclidean_clusters(points: np.ndarray, tolerance: float) -> np.ndarray:
        tree = cKDTree(points)
        labels = np.full(points.shape[0], -1, dtype=np.int32)
        cluster_id = 0

        for seed_index in range(points.shape[0]):
            if labels[seed_index] >= 0:
                continue
            labels[seed_index] = cluster_id
            queue = deque([seed_index])
            while queue:
                point_index = queue.popleft()
                neighbors = tree.query_ball_point(points[point_index], tolerance)
                for neighbor in neighbors:
                    if labels[neighbor] < 0:
                        labels[neighbor] = cluster_id
                        queue.append(neighbor)
            cluster_id += 1
        return labels

    def filter_and_describe_clusters(self, points: np.ndarray, labels: np.ndarray):
        accepted_labels = np.full(labels.shape[0], -1, dtype=np.int32)
        clusters: List[ClusterInfo] = []
        raw_cluster_ids = np.unique(labels[labels >= 0])

        for raw_id in raw_cluster_ids:
            indices = np.flatnonzero(labels == raw_id)
            point_count = indices.size
            if (
                    point_count < self.min_cluster_points or
                    point_count > self.max_cluster_points):
                continue

            cluster_points = points[indices]
            minimum = cluster_points.min(axis=0)
            maximum = cluster_points.max(axis=0)
            size = maximum - minimum
            horizontal_width = max(float(size[0]), float(size[1]))
            height = float(size[2])
            if not (
                    self.min_cluster_height <= height <= self.max_cluster_height and
                    self.min_cluster_width <= horizontal_width <= self.max_cluster_width):
                continue

            cluster_id = len(clusters)
            accepted_labels[indices] = cluster_id
            centroid = cluster_points.mean(axis=0)
            volume = max(float(np.prod(np.maximum(size, 0.05))), 1e-6)
            clusters.append(ClusterInfo(
                cluster_id=cluster_id,
                indices=indices,
                centroid=centroid,
                minimum=minimum,
                maximum=maximum,
                size=size,
                point_count=point_count,
                distance=float(np.linalg.norm(centroid)),
                density=float(point_count / volume),
            ))
        return accepted_labels, clusters

    def make_markers(self, clusters: List[ClusterInfo], header: Header) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for cluster in clusters:
            color = self.marker_color(cluster.cluster_id)
            base_id = cluster.cluster_id * 3

            box = Marker()
            box.header = header
            box.ns = 'lidar_cluster_boxes'
            box.id = base_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float((cluster.minimum[0] + cluster.maximum[0]) * 0.5)
            box.pose.position.y = float((cluster.minimum[1] + cluster.maximum[1]) * 0.5)
            box.pose.position.z = float((cluster.minimum[2] + cluster.maximum[2]) * 0.5)
            box.pose.orientation.w = 1.0
            box.scale.x = max(float(cluster.size[0]), 0.05)
            box.scale.y = max(float(cluster.size[1]), 0.05)
            box.scale.z = max(float(cluster.size[2]), 0.05)
            box.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.35)
            box.lifetime = DurationMsg(sec=0, nanosec=500000000)
            markers.markers.append(box)

            centroid = Marker()
            centroid.header = header
            centroid.ns = 'lidar_cluster_centroids'
            centroid.id = base_id + 1
            centroid.type = Marker.SPHERE
            centroid.action = Marker.ADD
            centroid.pose.position = Point(
                x=float(cluster.centroid[0]),
                y=float(cluster.centroid[1]),
                z=float(cluster.centroid[2]))
            centroid.pose.orientation.w = 1.0
            centroid.scale.x = 0.18
            centroid.scale.y = 0.18
            centroid.scale.z = 0.18
            centroid.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            centroid.lifetime = DurationMsg(sec=0, nanosec=500000000)
            markers.markers.append(centroid)

            text = Marker()
            text.header = header
            text.ns = 'lidar_cluster_labels'
            text.id = base_id + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(cluster.centroid[0])
            text.pose.position.y = float(cluster.centroid[1])
            text.pose.position.z = float(cluster.maximum[2] + 0.25)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = (
                f'id={cluster.cluster_id} n={cluster.point_count} '
                f'd={cluster.distance:.1f}m\n'
                f'{cluster.size[0]:.1f}x{cluster.size[1]:.1f}x{cluster.size[2]:.1f}m')
            text.lifetime = DurationMsg(sec=0, nanosec=500000000)
            markers.markers.append(text)
        return markers

    @staticmethod
    def marker_color(cluster_id: int):
        rgb = cluster_rgb(np.array([cluster_id], dtype=np.int32))[0]
        return tuple(float(channel) / 255.0 for channel in rgb)

    @staticmethod
    def cluster_to_dict(cluster: ClusterInfo):
        return {
            'cluster_id': cluster.cluster_id,
            'centroid': [round(float(value), 3) for value in cluster.centroid],
            'minimum': [round(float(value), 3) for value in cluster.minimum],
            'maximum': [round(float(value), 3) for value in cluster.maximum],
            'size': [round(float(value), 3) for value in cluster.size],
            'point_count': cluster.point_count,
            'distance': round(cluster.distance, 3),
            'density': round(cluster.density, 3),
        }


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterBaselineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
