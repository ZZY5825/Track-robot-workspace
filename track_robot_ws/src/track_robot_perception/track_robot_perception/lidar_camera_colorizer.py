#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


POINT_FIELD_TO_DTYPE = {
    PointField.INT8: ('i1', 1),
    PointField.UINT8: ('u1', 1),
    PointField.INT16: ('i2', 2),
    PointField.UINT16: ('u2', 2),
    PointField.INT32: ('i4', 4),
    PointField.UINT32: ('u4', 4),
    PointField.FLOAT32: ('f4', 4),
    PointField.FLOAT64: ('f8', 8),
}


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def field_offset(cloud: PointCloud2, name: str) -> Optional[int]:
    for field in cloud.fields:
        if field.name == name:
            return field.offset
    return None


def cloud_xyz_array(cloud: PointCloud2) -> np.ndarray:
    x_offset = field_offset(cloud, 'x')
    y_offset = field_offset(cloud, 'y')
    z_offset = field_offset(cloud, 'z')
    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError('PointCloud2 must contain x, y, z fields')

    endian = '>' if cloud.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4', endian + 'f4', endian + 'f4'],
        'offsets': [x_offset, y_offset, z_offset],
        'itemsize': cloud.point_step,
    })
    points = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width * cloud.height)
    xyz = np.empty((points.shape[0], 3), dtype=np.float32)
    xyz[:, 0] = points['x']
    xyz[:, 1] = points['y']
    xyz[:, 2] = points['z']
    return xyz


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    # TF gives target_from_source. The camera image is only used for color; LiDAR geometry is
    # transformed into the camera optical frame temporarily for projection.
    t = transform.transform.translation
    q = transform.transform.rotation
    rotation = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    translation = np.array([t.x, t.y, t.z], dtype=np.float32)
    return points @ rotation.T + translation


def rgb_to_pcl_float(colors: np.ndarray) -> np.ndarray:
    rgb_uint = (
        colors[:, 0].astype(np.uint32) << 16 |
        colors[:, 1].astype(np.uint32) << 8 |
        colors[:, 2].astype(np.uint32)
    )
    return rgb_uint.astype('<u4').view('<f4')


def make_colored_cloud(points: np.ndarray, colors: np.ndarray, header: Header) -> PointCloud2:
    dtype = np.dtype([
        ('x', '<f4'),
        ('y', '<f4'),
        ('z', '<f4'),
        ('rgb', '<f4'),
    ])
    data = np.empty(points.shape[0], dtype=dtype)
    data['x'] = points[:, 0].astype(np.float32)
    data['y'] = points[:, 1].astype(np.float32)
    data['z'] = points[:, 2].astype(np.float32)
    data['rgb'] = rgb_to_pcl_float(colors.astype(np.uint8))

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = dtype.itemsize
    msg.row_step = dtype.itemsize * points.shape[0]
    msg.is_dense = False
    msg.data = data.tobytes()
    return msg


class LidarCameraColorizer(Node):
    def __init__(self):
        super().__init__('lidar_camera_colorizer_node')

        self.lidar_topic = self.declare_parameter('lidar_topic', '/rslidar_points').value
        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/zed/zed_node/left/camera_info').value
        self.output_topic = self.declare_parameter('output_topic', '/lidar_colored_points').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'rslidar').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'zed_left_camera_optical_frame').value
        self.output_frame = self.declare_parameter('output_frame', '').value
        self.max_projection_distance = float(
            self.declare_parameter('max_projection_distance', 80.0).value)
        self.min_projection_distance = float(
            self.declare_parameter('min_projection_distance', 0.2).value)
        self.use_approximate_sync = bool(
            self.declare_parameter('use_approximate_sync', False).value)
        self.max_image_age_sec = float(self.declare_parameter('max_image_age_sec', 0.25).value)
        self.publish_uncolored_points = bool(
            self.declare_parameter('publish_uncolored_points', True).value)
        self.default_color_for_unprojected_points = self._parse_color_parameter(
            self.declare_parameter('default_color_for_unprojected_points', [120, 120, 120]).value)

        if self.use_approximate_sync:
            self.get_logger().warn(
                'message_filters sync is not used in this Foxy prototype; using latest image '
                'and camera_info buffers instead.')

        self.bridge = CvBridge()
        self.latest_image: Optional[Tuple[Image, np.ndarray]] = None
        self.latest_camera_info: Optional[CameraInfo] = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 5)
        self.create_subscription(PointCloud2, self.lidar_topic, self.cloud_callback, 5)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 5)

        self.get_logger().info(
            f'Colorizing {self.lidar_topic} using {self.image_topic} + '
            f'{self.camera_info_topic}; projecting into {self.camera_frame}; '
            f'publishing {self.output_topic}')

    def _parse_color_parameter(self, value) -> np.ndarray:
        if isinstance(value, str):
            parts = [int(p.strip()) for p in value.split(',')]
        else:
            parts = [int(v) for v in value]
        if len(parts) != 3:
            raise ValueError('default_color_for_unprojected_points must contain 3 RGB values')
        return np.array([max(0, min(255, p)) for p in parts], dtype=np.uint8)

    def image_callback(self, msg: Image):
        try:
            # The RGB image is used only for color. The LiDAR point cloud remains the source of 3D
            # geometry, and CameraInfo supplies the pinhole intrinsics for projection.
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image to rgb8: {exc}')
            return
        self.latest_image = (msg, image)

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def cloud_callback(self, cloud: PointCloud2):
        if self.latest_image is None or self.latest_camera_info is None:
            self.get_logger().warn('Waiting for image and camera_info before colorizing cloud')
            return

        image_msg, image = self.latest_image
        info = self.latest_camera_info
        dt = abs(stamp_to_seconds(cloud.header.stamp) - stamp_to_seconds(image_msg.header.stamp))
        if dt > self.max_image_age_sec:
            self.get_logger().warn(
                f'Image/cloud timestamps differ by {dt:.3f}s; colors may be misaligned')

        source_frame = cloud.header.frame_id or self.lidar_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, source_frame, Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warn(
                f'No TF from {source_frame} to {self.camera_frame}: {exc}')
            return

        try:
            lidar_points = cloud_xyz_array(cloud)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        camera_points = transform_points(lidar_points, transform)
        colors, keep_mask, colored_count = self.colorize_points(camera_points, lidar_points, image, info)

        output_points = lidar_points[keep_mask]
        output_colors = colors[keep_mask]
        header = Header()
        header.stamp = cloud.header.stamp
        header.frame_id = self.output_frame if self.output_frame else source_frame
        self.publisher.publish(make_colored_cloud(output_points, output_colors, header))

        self.get_logger().debug(
            f'Published {output_points.shape[0]} points, {colored_count} projected into image')

    def colorize_points(self, camera_points: np.ndarray, lidar_points: np.ndarray,
                        image: np.ndarray, info: CameraInfo):
        fx, fy, cx, cy = self.get_intrinsics(info)
        height, width = image.shape[:2]

        finite = np.isfinite(camera_points).all(axis=1) & np.isfinite(lidar_points).all(axis=1)
        z = camera_points[:, 2]
        distance = np.linalg.norm(camera_points, axis=1)
        in_front = z > 0.0
        in_range = (
            (distance >= self.min_projection_distance) &
            (distance <= self.max_projection_distance))

        u = fx * camera_points[:, 0] / z + cx
        v = fy * camera_points[:, 1] / z + cy
        u_int = np.rint(u).astype(np.int32)
        v_int = np.rint(v).astype(np.int32)
        in_image = (
            (u_int >= 0) & (u_int < width) &
            (v_int >= 0) & (v_int < height))
        projected = finite & in_front & in_range & in_image

        colors = np.repeat(
            self.default_color_for_unprojected_points.reshape(1, 3),
            camera_points.shape[0],
            axis=0)
        colors[projected] = image[v_int[projected], u_int[projected], :3]

        if self.publish_uncolored_points:
            keep_mask = finite
        else:
            keep_mask = projected
        return colors, keep_mask, int(np.count_nonzero(projected))

    def get_intrinsics(self, info: CameraInfo):
        # Prefer P for rectified images; fall back to K if P is empty.
        if len(info.p) >= 12 and info.p[0] != 0.0 and info.p[5] != 0.0:
            return float(info.p[0]), float(info.p[5]), float(info.p[2]), float(info.p[6])
        return float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraColorizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
