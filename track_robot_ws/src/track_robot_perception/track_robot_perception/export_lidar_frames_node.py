#!/usr/bin/env python3

import json
from pathlib import Path
from typing import Dict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from track_robot_perception.lidar_ground_segment_node import cloud_field_array


POINT_FIELD_TYPES = {
    PointField.INT8: 'INT8',
    PointField.UINT8: 'UINT8',
    PointField.INT16: 'INT16',
    PointField.UINT16: 'UINT16',
    PointField.INT32: 'INT32',
    PointField.UINT32: 'UINT32',
    PointField.FLOAT32: 'FLOAT32',
    PointField.FLOAT64: 'FLOAT64',
}


def point_field_metadata(field: PointField) -> Dict:
    return {
        'name': field.name,
        'offset': int(field.offset),
        'datatype': POINT_FIELD_TYPES.get(
            field.datatype, f'UNKNOWN_{field.datatype}'),
        'datatype_id': int(field.datatype),
        'count': int(field.count),
    }


def cloud_to_xyzi(cloud: PointCloud2) -> np.ndarray:
    coordinates = [cloud_field_array(cloud, name) for name in ('x', 'y', 'z')]
    if any(value is None for value in coordinates):
        raise ValueError('PointCloud2 must contain readable x, y, z fields')

    intensity = cloud_field_array(cloud, 'intensity')
    if intensity is None:
        intensity = np.zeros(coordinates[0].shape[0], dtype=np.float32)

    points = np.column_stack(
        (coordinates[0], coordinates[1], coordinates[2], intensity))
    return points.astype(np.float32, copy=False)


def frame_metadata(
        cloud: PointCloud2,
        points: np.ndarray,
        frame_index: int) -> Dict:
    finite_xyz = np.isfinite(points[:, :3]).all(axis=1)
    finite_points = points[finite_xyz, :3]
    if finite_points.shape[0]:
        minimum = finite_points.min(axis=0).tolist()
        maximum = finite_points.max(axis=0).tolist()
    else:
        minimum = [None, None, None]
        maximum = [None, None, None]

    timestamp_ns = (
        int(cloud.header.stamp.sec) * 1_000_000_000 +
        int(cloud.header.stamp.nanosec))
    return {
        'frame_index': frame_index,
        'timestamp': {
            'sec': int(cloud.header.stamp.sec),
            'nanosec': int(cloud.header.stamp.nanosec),
            'nanoseconds': timestamp_ns,
        },
        'frame_id': cloud.header.frame_id,
        'point_count': int(points.shape[0]),
        'finite_xyz_point_count': int(np.count_nonzero(finite_xyz)),
        'organized_shape': {
            'height': int(cloud.height),
            'width': int(cloud.width),
        },
        'point_step': int(cloud.point_step),
        'row_step': int(cloud.row_step),
        'is_bigendian': bool(cloud.is_bigendian),
        'is_dense': bool(cloud.is_dense),
        'fields': [point_field_metadata(field) for field in cloud.fields],
        'xyz_min': minimum,
        'xyz_max': maximum,
        'saved_layout': ['x', 'y', 'z', 'intensity'],
        'saved_dtype': 'float32',
    }


def save_frame(
        output_dir: Path,
        cloud: PointCloud2,
        points: np.ndarray,
        frame_index: int) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp_ns = (
        int(cloud.header.stamp.sec) * 1_000_000_000 +
        int(cloud.header.stamp.nanosec))
    stem = f'frame_{frame_index:06d}_{timestamp_ns}'
    base_path = output_dir / stem

    np.save(str(base_path) + '.npy', points)
    points.astype('<f4', copy=False).tofile(str(base_path) + '.bin')
    with open(str(base_path) + '.json', 'w', encoding='utf-8') as stream:
        json.dump(
            frame_metadata(cloud, points, frame_index),
            stream,
            indent=2)
    return base_path


class ExportLidarFramesNode(Node):
    def __init__(self):
        super().__init__('export_lidar_frames_node')

        self.lidar_topic = self.declare_parameter(
            'lidar_topic', '/rslidar_points').value
        self.output_dir = Path(str(self.declare_parameter(
            'output_dir',
            '~/track_robot_ws/lidar_pretrained_test_frames').value)).expanduser()
        self.save_every_n_frames = max(
            1, int(self.declare_parameter('save_every_n_frames', 10).value))
        self.max_saved_frames = max(
            1, int(self.declare_parameter('max_saved_frames', 20).value))

        self.received_frames = 0
        self.saved_frames = 0
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.cloud_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'Exporting {self.lidar_topic} every {self.save_every_n_frames} frames '
            f'to {self.output_dir}; maximum={self.max_saved_frames}')

    def cloud_callback(self, cloud: PointCloud2):
        self.received_frames += 1
        if self.saved_frames >= self.max_saved_frames:
            return
        if self.received_frames % self.save_every_n_frames != 0:
            return

        try:
            points = cloud_to_xyzi(cloud)
            base_path = save_frame(
                self.output_dir, cloud, points, self.saved_frames)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Failed to export LiDAR frame: {exc}')
            return

        self.saved_frames += 1
        self.get_logger().info(
            f'Saved {base_path.name}.npy/.bin/.json; '
            f'points={points.shape[0]}; '
            f'{self.saved_frames}/{self.max_saved_frames}')
        if self.saved_frames == self.max_saved_frames:
            self.get_logger().info(
                'Maximum saved frame count reached; no more frames will be written')


def main(args=None):
    rclpy.init(args=args)
    node = ExportLidarFramesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
