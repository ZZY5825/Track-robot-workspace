#!/usr/bin/env python3

import struct
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from track_robot_perception.lidar_ground_segment_node import (
    cloud_field_array,
    contiguous_point_bytes,
    field_by_name,
    parse_bool,
)


def ros_stamp_sec(cloud: PointCloud2) -> float:
    return float(cloud.header.stamp.sec) + float(cloud.header.stamp.nanosec) * 1e-9


def time_offsets_from_timestamp(
        cloud: PointCloud2,
        timestamp: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if timestamp is None or timestamp.size == 0:
        return None

    values = timestamp.astype(np.float64)
    finite = np.isfinite(values)
    if not np.any(finite):
        return None

    header_time = ros_stamp_sec(cloud)
    finite_values = values[finite]
    median_value = float(np.median(finite_values))

    if median_value > 1.0e12:
        absolute_sec = values * 1.0e-9
    elif median_value > 1.0e9:
        absolute_sec = values * 1.0e-6
    elif median_value > 1.0e6:
        absolute_sec = values * 1.0e-3
    elif abs(median_value - header_time) < 10.0:
        absolute_sec = values
    else:
        relative = values - float(np.nanmin(finite_values))
        if float(np.nanmax(relative[finite])) > 100.0:
            relative = relative * 1.0e-3
        return relative.astype(np.float32)

    offsets = absolute_sec - header_time
    if np.nanmin(offsets[finite]) < -0.5 or np.nanmax(offsets[finite]) > 0.5:
        offsets = absolute_sec - float(np.nanmin(absolute_sec[finite]))
    return offsets.astype(np.float32)


def make_point_lio_cloud(
        cloud: PointCloud2,
        output_frame_id: str,
        timestamp_field: str,
        output_time_field: str,
        keep_original_timestamp: bool) -> PointCloud2:
    point_count = cloud.width * cloud.height
    existing_time = field_by_name(cloud, output_time_field)
    if existing_time is not None:
        msg = PointCloud2()
        msg.header = cloud.header
        if output_frame_id:
            msg.header.frame_id = output_frame_id
        msg.height = cloud.height
        msg.width = cloud.width
        msg.fields = list(cloud.fields)
        msg.is_bigendian = cloud.is_bigendian
        msg.point_step = cloud.point_step
        msg.row_step = cloud.row_step
        msg.is_dense = cloud.is_dense
        msg.data = bytes(cloud.data)
        return msg

    timestamp = cloud_field_array(cloud, timestamp_field)
    time_offsets = time_offsets_from_timestamp(cloud, timestamp)
    if time_offsets is None:
        time_offsets = np.zeros(point_count, dtype=np.float32)

    input_points = contiguous_point_bytes(cloud)
    time_offset = cloud.point_step
    output_point_step = cloud.point_step + 4
    output = bytearray(point_count * output_point_step)
    endian = '>' if cloud.is_bigendian else '<'

    for index in range(point_count):
        input_start = index * cloud.point_step
        output_start = index * output_point_step
        output[output_start:output_start + cloud.point_step] = (
            input_points[input_start:input_start + cloud.point_step])
        struct.pack_into(
            endian + 'f',
            output,
            output_start + time_offset,
            float(time_offsets[index]))

    fields = list(cloud.fields)
    if not keep_original_timestamp:
        fields = [field for field in fields if field.name != timestamp_field]
    fields.append(PointField(
        name=output_time_field,
        offset=time_offset,
        datatype=PointField.FLOAT32,
        count=1))

    msg = PointCloud2()
    msg.header = cloud.header
    if output_frame_id:
        msg.header.frame_id = output_frame_id
    msg.height = cloud.height
    msg.width = cloud.width
    msg.fields = fields
    msg.is_bigendian = cloud.is_bigendian
    msg.point_step = output_point_step
    msg.row_step = output_point_step * cloud.width
    msg.is_dense = cloud.is_dense
    msg.data = bytes(output)
    return msg


class RslidarPointLioAdapterNode(Node):
    def __init__(self):
        super().__init__('rslidar_point_lio_adapter_node')

        self.input_topic = self.declare_parameter(
            'input_topic', '/rslidar_points').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/rslidar_points_point_lio').value
        self.output_frame_id = self.declare_parameter(
            'output_frame_id', 'rslidar').value
        self.timestamp_field = self.declare_parameter(
            'timestamp_field', 'timestamp').value
        self.output_time_field = self.declare_parameter(
            'output_time_field', 'time').value
        self.keep_original_timestamp = parse_bool(self.declare_parameter(
            'keep_original_timestamp', True).value)

        self.warned_missing_ring = False
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 5)
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'Adapting {self.input_topic} -> {self.output_topic}; '
            f'{self.timestamp_field} -> {self.output_time_field}')

    def cloud_callback(self, cloud: PointCloud2):
        x = cloud_field_array(cloud, 'x')
        y = cloud_field_array(cloud, 'y')
        z = cloud_field_array(cloud, 'z')
        ring = field_by_name(cloud, 'ring')
        if x is None or y is None or z is None:
            self.get_logger().warn('Input cloud must contain readable x, y, z fields')
            return
        if ring is None and not self.warned_missing_ring:
            self.get_logger().warn(
                'Input cloud has no ring field; Point-LIO mechanical LiDAR '
                'preprocess may reject or poorly deskew this cloud')
            self.warned_missing_ring = True

        output = make_point_lio_cloud(
            cloud,
            self.output_frame_id,
            self.timestamp_field,
            self.output_time_field,
            self.keep_original_timestamp)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RslidarPointLioAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
