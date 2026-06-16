#!/usr/bin/env python3

import math
import struct
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


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
    raw_points = contiguous_point_bytes(cloud)
    return np.frombuffer(raw_points, dtype=dtype, count=cloud.width * cloud.height)[name]


def contiguous_point_bytes(cloud: PointCloud2) -> bytes:
    expected_row_step = cloud.width * cloud.point_step
    if cloud.row_step == expected_row_step:
        return bytes(cloud.data[:cloud.height * cloud.row_step])

    output = bytearray(cloud.height * expected_row_step)
    for row in range(cloud.height):
        input_start = row * cloud.row_step
        output_start = row * expected_row_step
        output[output_start:output_start + expected_row_step] = (
            cloud.data[input_start:input_start + expected_row_step])
    return bytes(output)


def intensity_grayscale(intensity: Optional[np.ndarray], point_count: int) -> np.ndarray:
    if intensity is None or intensity.size == 0:
        return np.full((point_count, 3), 170, dtype=np.uint8)

    values = intensity.astype(np.float32)
    finite = np.isfinite(values)
    grayscale = np.full(point_count, 120, dtype=np.uint8)
    if not np.any(finite):
        return np.repeat(grayscale[:, None], 3, axis=1)

    low, high = np.percentile(values[finite], [5.0, 95.0])
    if high <= low:
        grayscale[finite] = 170
    else:
        normalized = np.clip((values[finite] - low) / (high - low), 0.0, 1.0)
        grayscale[finite] = (70.0 + normalized * 170.0).astype(np.uint8)
    return np.repeat(grayscale[:, None], 3, axis=1)


def pack_rgb(colors: np.ndarray) -> np.ndarray:
    return (
        colors[:, 0].astype(np.uint32) << 16 |
        colors[:, 1].astype(np.uint32) << 8 |
        colors[:, 2].astype(np.uint32))


def make_ground_colored_cloud(
        cloud: PointCloud2,
        ground_mask: np.ndarray,
        ground_color,
        non_ground_colors: np.ndarray) -> PointCloud2:
    point_count = cloud.width * cloud.height
    existing_rgb = field_by_name(cloud, 'rgb')
    append_rgb = (
        existing_rgb is None or
        existing_rgb.datatype not in (PointField.FLOAT32, PointField.UINT32))

    rgb_offset = cloud.point_step if append_rgb else existing_rgb.offset
    ground_offset = cloud.point_step + (4 if append_rgb else 0)
    output_point_step = ground_offset + 4

    output = bytearray(point_count * output_point_step)
    input_points = contiguous_point_bytes(cloud)
    colors = non_ground_colors.copy()
    colors[ground_mask] = np.asarray(ground_color, dtype=np.uint8)
    packed_colors = pack_rgb(colors)
    endian = '>' if cloud.is_bigendian else '<'

    for index in range(point_count):
        input_start = index * cloud.point_step
        output_start = index * output_point_step
        output[output_start:output_start + cloud.point_step] = (
            input_points[input_start:input_start + cloud.point_step])
        struct.pack_into(
            endian + 'I', output, output_start + rgb_offset, int(packed_colors[index]))
        output[output_start + ground_offset] = 1 if ground_mask[index] else 0

    msg = PointCloud2()
    msg.header = cloud.header
    msg.height = cloud.height
    msg.width = cloud.width
    msg.fields = list(cloud.fields)
    if append_rgb:
        msg.fields.append(PointField(
            name='rgb', offset=rgb_offset, datatype=PointField.FLOAT32, count=1))
    msg.fields.append(PointField(
        name='is_ground', offset=ground_offset, datatype=PointField.UINT8, count=1))
    msg.is_bigendian = cloud.is_bigendian
    msg.point_step = output_point_step
    msg.row_step = output_point_step * cloud.width
    msg.is_dense = cloud.is_dense
    msg.data = bytes(output)
    return msg


class LidarGroundSegmentNode(Node):
    def __init__(self):
        super().__init__('lidar_ground_segment_node')

        self.lidar_topic = self.declare_parameter('lidar_topic', '/rslidar_points').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/lidar_ground_segmented_points').value
        self.method = str(
            self.declare_parameter('method', 'ransac_plane').value).lower()
        if self.method not in ('ransac_plane', 'height'):
            raise ValueError('method must be ransac_plane or height')
        self.ground_z_threshold = float(
            self.declare_parameter('ground_z_threshold', -0.7).value)
        self.min_range = float(self.declare_parameter('min_range', 0.0).value)
        self.max_range = float(self.declare_parameter('max_range', 200.0).value)
        self.ground_fit_max_range = float(
            self.declare_parameter('ground_fit_max_range', 20.0).value)
        self.seed_grid_size = max(
            0.1, float(self.declare_parameter('seed_grid_size', 0.5).value))
        self.ransac_distance_threshold = max(
            0.01,
            float(self.declare_parameter('ransac_distance_threshold', 0.18).value))
        self.ransac_max_iterations = max(
            10, int(self.declare_parameter('ransac_max_iterations', 120).value))
        self.ransac_min_seed_points = max(
            3, int(self.declare_parameter('ransac_min_seed_points', 30).value))
        self.max_ground_tilt_deg = float(
            self.declare_parameter('max_ground_tilt_deg', 45.0).value)
        self.ground_color = self.parse_color(
            self.declare_parameter('ground_color', [35, 255, 80]).value)
        self.process_every_n_clouds = max(
            1, int(self.declare_parameter('process_every_n_clouds', 1).value))
        self.debug_timing = parse_bool(
            self.declare_parameter('debug_timing', True).value)

        self.cloud_count = 0
        self.random_generator = np.random.default_rng(7)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 5)
        self.create_subscription(
            PointCloud2, self.lidar_topic, self.cloud_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Ground highlighting {self.lidar_topic} -> {self.output_topic}; '
            f'method={self.method}; plane_distance={self.ransac_distance_threshold:.3f}m')

    @staticmethod
    def parse_color(value):
        if isinstance(value, str):
            parts = [int(part.strip()) for part in value.split(',')]
        else:
            parts = [int(part) for part in value]
        if len(parts) != 3:
            raise ValueError('ground_color must contain three RGB values')
        return tuple(max(0, min(255, part)) for part in parts)

    def cloud_callback(self, cloud: PointCloud2):
        self.cloud_count += 1
        if self.cloud_count % self.process_every_n_clouds != 0:
            return

        start = time.monotonic()
        x = cloud_field_array(cloud, 'x')
        y = cloud_field_array(cloud, 'y')
        z = cloud_field_array(cloud, 'z')
        if x is None or y is None or z is None:
            self.get_logger().warn('PointCloud2 must contain readable x, y, z fields')
            return

        xyz = np.column_stack((x, y, z)).astype(np.float32)
        finite = np.isfinite(xyz).all(axis=1)
        distance = np.linalg.norm(xyz, axis=1)
        in_range = (distance >= self.min_range) & (distance <= self.max_range)
        valid = finite & in_range

        plane = None
        if self.method == 'ransac_plane':
            fit_mask = valid & (distance <= self.ground_fit_max_range)
            plane = self.fit_ground_plane(xyz[fit_mask])

        if plane is None:
            ground_mask = valid & (xyz[:, 2] <= self.ground_z_threshold)
        else:
            normal, offset = plane
            plane_distance = np.abs(xyz @ normal + offset)
            ground_mask = valid & (plane_distance <= self.ransac_distance_threshold)

        intensity = cloud_field_array(cloud, 'intensity')
        non_ground_colors = intensity_grayscale(intensity, xyz.shape[0])
        output = make_ground_colored_cloud(
            cloud, ground_mask, self.ground_color, non_ground_colors)
        self.publisher.publish(output)

        if self.debug_timing:
            elapsed_ms = (time.monotonic() - start) * 1000.0
            plane_text = 'height fallback'
            if plane is not None:
                normal, offset = plane
                plane_text = (
                    f'normal=[{normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f}] '
                    f'd={offset:.3f}')
            self.get_logger().info(
                f'Ground segmentation {elapsed_ms:.1f} ms; points={xyz.shape[0]}; '
                f'ground={np.count_nonzero(ground_mask)}; '
                f'non_ground={xyz.shape[0] - np.count_nonzero(ground_mask)}; '
                f'{plane_text}')

    def fit_ground_plane(
            self, points: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
        seeds = self.lowest_grid_seeds(points)
        if seeds.shape[0] < self.ransac_min_seed_points:
            return None

        best_inliers = None
        best_count = 0
        best_error = float('inf')
        minimum_normal_z = math.cos(math.radians(self.max_ground_tilt_deg))

        for _ in range(self.ransac_max_iterations):
            sample_indices = self.random_generator.choice(
                seeds.shape[0], size=3, replace=False)
            p0, p1, p2 = seeds[sample_indices]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm
            if normal[2] < 0.0:
                normal = -normal
            if normal[2] < minimum_normal_z:
                continue

            offset = -float(np.dot(normal, p0))
            distances = np.abs(seeds @ normal + offset)
            inliers = distances <= self.ransac_distance_threshold
            count = int(np.count_nonzero(inliers))
            if count < 3:
                continue
            error = float(np.mean(distances[inliers]))
            if count > best_count or (count == best_count and error < best_error):
                best_inliers = inliers
                best_count = count
                best_error = error

        if best_inliers is None or best_count < self.ransac_min_seed_points:
            return None

        inlier_points = seeds[best_inliers]
        for _ in range(3):
            normal, offset = self.fit_plane_svd(inlier_points)
            if normal[2] < minimum_normal_z:
                return None
            distances = np.abs(seeds @ normal + offset)
            refined = distances <= self.ransac_distance_threshold
            if np.count_nonzero(refined) < self.ransac_min_seed_points:
                break
            inlier_points = seeds[refined]
        return normal.astype(np.float32), float(offset)

    def lowest_grid_seeds(self, points: np.ndarray) -> np.ndarray:
        if points.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        grid_x = np.floor(points[:, 0] / self.seed_grid_size).astype(np.int32)
        grid_y = np.floor(points[:, 1] / self.seed_grid_size).astype(np.int32)
        order = np.lexsort((points[:, 2], grid_y, grid_x))
        sorted_x = grid_x[order]
        sorted_y = grid_y[order]
        first = np.ones(order.shape[0], dtype=bool)
        first[1:] = (
            (sorted_x[1:] != sorted_x[:-1]) |
            (sorted_y[1:] != sorted_y[:-1]))
        return points[order[first]]

    @staticmethod
    def fit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
        centroid = points.mean(axis=0)
        covariance = (points - centroid).T @ (points - centroid)
        _, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, 0]
        if normal[2] < 0.0:
            normal = -normal
        normal = normal / np.linalg.norm(normal)
        offset = -float(np.dot(normal, centroid))
        return normal, offset


def main(args=None):
    rclpy.init(args=args)
    node = LidarGroundSegmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
