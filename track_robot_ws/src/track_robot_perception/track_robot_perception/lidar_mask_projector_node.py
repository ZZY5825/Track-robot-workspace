#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Optional

import torch
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_MODEL_CONFIG = 'COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml'

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

SEMANTIC_POINT_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('intensity', '<f4'),
    ('rgb', '<f4'),
    ('class_id', '<i4'),
    ('instance_id', '<i4'),
    ('confidence', '<f4'),
])


@dataclass
class MaskResult:
    masks: np.ndarray
    classes: np.ndarray
    scores: np.ndarray
    stamp_sec: float
    received_monotonic: float
    height: int
    width: int


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_is_zero(stamp) -> bool:
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0


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
    values = np.frombuffer(
        cloud.data, dtype=dtype, count=cloud.width * cloud.height)[name]
    return values


def cloud_xyz_array(cloud: PointCloud2) -> np.ndarray:
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
    xyz = np.empty((values.shape[0], 3), dtype=np.float32)
    xyz[:, 0] = values['x']
    xyz[:, 1] = values['y']
    xyz[:, 2] = values['z']
    return xyz


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    translation_msg = transform.transform.translation
    rotation_msg = transform.transform.rotation
    rotation = quaternion_to_rotation_matrix(
        rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w)
    translation = np.array([
        translation_msg.x,
        translation_msg.y,
        translation_msg.z,
    ], dtype=np.float32)
    return points @ rotation.T + translation


def rgb_to_pcl_float(colors: np.ndarray) -> np.ndarray:
    rgb_uint = (
        colors[:, 0].astype(np.uint32) << 16 |
        colors[:, 1].astype(np.uint32) << 8 |
        colors[:, 2].astype(np.uint32)
    )
    return rgb_uint.astype('<u4').view('<f4')


def semantic_colors(class_ids: np.ndarray, instance_ids: np.ndarray) -> np.ndarray:
    colors = np.full((class_ids.shape[0], 3), 90, dtype=np.uint8)
    labelled = class_ids >= 0
    if not np.any(labelled):
        return colors

    class_values = class_ids[labelled].astype(np.int32)
    instance_values = np.maximum(instance_ids[labelled], 0).astype(np.int32)
    hue = ((class_values * 37 + instance_values * 53) % 180).astype(np.uint8)
    hsv = np.empty((hue.shape[0], 1, 3), dtype=np.uint8)
    hsv[:, 0, 0] = hue
    hsv[:, 0, 1] = 220
    hsv[:, 0, 2] = 255
    colors[labelled] = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[:, 0, :]
    return colors


def make_semantic_cloud(
        points: np.ndarray,
        intensity: np.ndarray,
        class_ids: np.ndarray,
        instance_ids: np.ndarray,
        confidence: np.ndarray,
        header: Header) -> PointCloud2:
    data = np.empty(points.shape[0], dtype=SEMANTIC_POINT_DTYPE)
    data['x'] = points[:, 0]
    data['y'] = points[:, 1]
    data['z'] = points[:, 2]
    data['intensity'] = intensity
    data['rgb'] = rgb_to_pcl_float(semantic_colors(class_ids, instance_ids))
    data['class_id'] = class_ids
    data['instance_id'] = instance_ids
    data['confidence'] = confidence

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
        PointField(name='class_id', offset=20, datatype=PointField.INT32, count=1),
        PointField(name='instance_id', offset=24, datatype=PointField.INT32, count=1),
        PointField(name='confidence', offset=28, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = SEMANTIC_POINT_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = False
    msg.data = data.tobytes()
    return msg


class LidarMaskProjectorNode(Node):
    def __init__(self):
        super().__init__('lidar_mask_projector_node')

        default_device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.lidar_topic = self.declare_parameter('lidar_topic', '/rslidar_points').value
        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/zed/zed_node/left/camera_info').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/lidar_semantic_points').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'rslidar').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'zed_left_camera_optical_frame').value
        self.output_frame = self.declare_parameter('output_frame', '').value
        self.model_config = self.declare_parameter(
            'model_config', DEFAULT_MODEL_CONFIG).value
        self.score_threshold = float(self.declare_parameter('score_threshold', 0.5).value)
        device_value = str(self.declare_parameter('device', default_device).value)
        self.device = default_device if device_value.lower() == 'auto' else device_value
        self.run_inference_every_n_images = max(
            1, int(self.declare_parameter('run_inference_every_n_images', 1).value))
        self.project_every_n_clouds = max(
            1, int(self.declare_parameter('project_every_n_clouds', 1).value))
        self.max_instances = max(1, int(self.declare_parameter('max_instances', 20).value))
        self.min_projection_distance = float(
            self.declare_parameter('min_projection_distance', 0.3).value)
        self.max_projection_distance = float(
            self.declare_parameter('max_projection_distance', 20.0).value)
        self.publish_unknown_points = parse_bool(
            self.declare_parameter('publish_unknown_points', True).value)
        self.publish_only_labelled_points = parse_bool(
            self.declare_parameter('publish_only_labelled_points', False).value)
        self.keep_intensity = parse_bool(
            self.declare_parameter('keep_intensity', True).value)
        self.default_class_id = int(self.declare_parameter('default_class_id', -1).value)
        self.default_instance_id = int(self.declare_parameter('default_instance_id', -1).value)
        self.max_mask_age_sec = float(
            self.declare_parameter('max_mask_age_sec', 0.3).value)
        self.timestamp_mode = str(
            self.declare_parameter('timestamp_mode', 'auto').value).lower()
        if self.timestamp_mode not in ('auto', 'header', 'receipt'):
            raise ValueError('timestamp_mode must be auto, header, or receipt')
        self.debug_timing = parse_bool(
            self.declare_parameter('debug_timing', True).value)
        self.resize_width = max(0, int(self.declare_parameter('resize_width', 0).value))

        self.bridge = CvBridge()
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_masks: Optional[MaskResult] = None
        self.image_count = 0
        self.cloud_count = 0
        self.last_wait_warning = 0.0
        self.reported_timestamp_fallback = False

        self.predictor = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.lidar_topic, self.cloud_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 5)

        self.get_logger().info(
            f'Projecting Detectron2 masks from {self.image_topic} onto {self.lidar_topic}; '
            f'camera_frame={self.camera_frame}; output={self.output_topic}; '
            f'model={self.model_config}; device={self.device}')

    def _create_predictor(self, image_shape):
        from detectron2 import model_zoo
        from detectron2.config import get_cfg
        from detectron2.engine import DefaultPredictor

        height, width = image_shape
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file(self.model_config))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = self.score_threshold
        cfg.MODEL.DEVICE = self.device
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(self.model_config)
        # DefaultPredictor otherwise enlarges small inputs to the model-zoo 800/1333 test size.
        cfg.INPUT.MIN_SIZE_TEST = min(height, width)
        cfg.INPUT.MAX_SIZE_TEST = max(height, width)
        return DefaultPredictor(cfg)

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def image_callback(self, msg: Image):
        self.image_count += 1
        if self.image_count % self.run_inference_every_n_images != 0:
            return

        start = time.monotonic()
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            inference_image = self.resize_image(image_bgr)
            if self.predictor is None:
                self.predictor = self._create_predictor(inference_image.shape[:2])
            outputs = self.predictor(inference_image)
            instances = outputs['instances'].to('cpu')
            instances = self.limit_instances(instances)
            masks = self.restore_masks(instances, image_bgr.shape[:2])
            classes = (
                instances.pred_classes.numpy().astype(np.int32)
                if instances.has('pred_classes') else np.empty(0, dtype=np.int32))
            scores = (
                instances.scores.numpy().astype(np.float32)
                if instances.has('scores') else np.empty(0, dtype=np.float32))
        except Exception as exc:
            self.get_logger().error(f'Detectron2 mask inference failed: {exc}')
            return

        self.latest_masks = MaskResult(
            masks=masks,
            classes=classes,
            scores=scores,
            stamp_sec=stamp_to_seconds(msg.header.stamp),
            received_monotonic=time.monotonic(),
            height=image_bgr.shape[0],
            width=image_bgr.shape[1])

        if self.debug_timing:
            elapsed_ms = (time.monotonic() - start) * 1000.0
            self.get_logger().info(
                f'Mask inference {elapsed_ms:.1f} ms; instances={len(scores)}')

    def resize_image(self, image_bgr: np.ndarray) -> np.ndarray:
        if self.resize_width <= 0 or image_bgr.shape[1] <= self.resize_width:
            return image_bgr
        scale = float(self.resize_width) / float(image_bgr.shape[1])
        height = max(1, int(round(image_bgr.shape[0] * scale)))
        return cv2.resize(
            image_bgr, (self.resize_width, height), interpolation=cv2.INTER_AREA)

    def limit_instances(self, instances):
        if len(instances) <= self.max_instances:
            return instances
        keep = instances.scores.argsort(descending=True)[:self.max_instances]
        return instances[keep]

    def restore_masks(self, instances, original_shape) -> np.ndarray:
        original_height, original_width = original_shape
        if not instances.has('pred_masks') or len(instances) == 0:
            return np.empty((0, original_height, original_width), dtype=bool)

        masks = instances.pred_masks.numpy()
        if masks.shape[1:] == (original_height, original_width):
            return masks.astype(bool, copy=False)

        restored = np.empty(
            (masks.shape[0], original_height, original_width), dtype=bool)
        for index, mask in enumerate(masks):
            restored[index] = cv2.resize(
                mask.astype(np.uint8),
                (original_width, original_height),
                interpolation=cv2.INTER_NEAREST).astype(bool)
        return restored

    def cloud_callback(self, cloud: PointCloud2):
        self.cloud_count += 1
        if self.cloud_count % self.project_every_n_clouds != 0:
            return

        masks = self.latest_masks
        camera_info = self.latest_camera_info
        if masks is None or camera_info is None:
            self.warn_throttled('Waiting for camera_info and Detectron2 masks')
            return

        mask_age, use_latest_tf = self.mask_age_and_tf_mode(cloud, masks)
        if mask_age > self.max_mask_age_sec:
            self.warn_throttled(
                f'Latest mask age is {mask_age:.3f}s; '
                f'max_mask_age_sec={self.max_mask_age_sec:.3f}')
            return

        source_frame = cloud.header.frame_id or self.lidar_frame
        tf_time = Time() if use_latest_tf else Time.from_msg(cloud.header.stamp)
        try:
            transform, tf_time = self.lookup_transform_with_latest_fallback(
                self.camera_frame, source_frame, tf_time)
        except TransformException as exc:
            self.warn_throttled(
                f'No TF from {source_frame} to {self.camera_frame}: {exc}')
            return

        start = time.monotonic()
        try:
            lidar_points = cloud_xyz_array(cloud)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        intensity = np.zeros(lidar_points.shape[0], dtype=np.float32)
        if self.keep_intensity:
            input_intensity = cloud_field_array(cloud, 'intensity')
            if input_intensity is not None:
                intensity = input_intensity.astype(np.float32)

        camera_points = transform_points(lidar_points, transform)
        class_ids, instance_ids, confidence, projected_count = self.assign_labels(
            lidar_points, camera_points, masks, camera_info)

        finite = np.isfinite(lidar_points).all(axis=1)
        labelled = class_ids != self.default_class_id
        if self.publish_only_labelled_points or not self.publish_unknown_points:
            keep = finite & labelled
        else:
            keep = finite

        output_points = lidar_points
        output_frame = self.output_frame if self.output_frame else source_frame
        if output_frame != source_frame:
            try:
                output_transform, _ = self.lookup_transform_with_latest_fallback(
                    output_frame, source_frame, tf_time)
                output_points = transform_points(lidar_points, output_transform)
            except TransformException as exc:
                self.warn_throttled(
                    f'No TF from {source_frame} to output frame {output_frame}: {exc}')
                return

        header = Header()
        header.stamp = (
            self.get_clock().now().to_msg()
            if stamp_is_zero(cloud.header.stamp) else cloud.header.stamp)
        header.frame_id = output_frame
        output = make_semantic_cloud(
            output_points[keep],
            intensity[keep],
            class_ids[keep],
            instance_ids[keep],
            confidence[keep],
            header)
        self.publisher.publish(output)

        if self.debug_timing:
            elapsed_ms = (time.monotonic() - start) * 1000.0
            self.get_logger().info(
                f'Projection {elapsed_ms:.1f} ms; input={len(lidar_points)}; '
                f'projected={projected_count}; labelled={np.count_nonzero(labelled)}; '
                f'published={np.count_nonzero(keep)}; mask_age={mask_age:.3f}s')

    def assign_labels(
            self,
            lidar_points: np.ndarray,
            camera_points: np.ndarray,
            masks: MaskResult,
            camera_info: CameraInfo):
        count = lidar_points.shape[0]
        class_ids = np.full(count, self.default_class_id, dtype=np.int32)
        instance_ids = np.full(count, self.default_instance_id, dtype=np.int32)
        confidence = np.zeros(count, dtype=np.float32)

        finite = (
            np.isfinite(lidar_points).all(axis=1) &
            np.isfinite(camera_points).all(axis=1))
        z = camera_points[:, 2]
        distance = np.linalg.norm(camera_points, axis=1)
        valid = (
            finite &
            (z > 0.0) &
            (distance >= self.min_projection_distance) &
            (distance <= self.max_projection_distance))

        fx, fy, cx, cy = self.get_intrinsics(camera_info)
        safe_z = np.where(valid, z, 1.0)
        u = np.rint(fx * camera_points[:, 0] / safe_z + cx).astype(np.int32)
        v = np.rint(fy * camera_points[:, 1] / safe_z + cy).astype(np.int32)
        in_image = (
            valid &
            (u >= 0) & (u < masks.width) &
            (v >= 0) & (v < masks.height))
        projected_indices = np.flatnonzero(in_image)
        if projected_indices.size == 0 or masks.masks.shape[0] == 0:
            return class_ids, instance_ids, confidence, int(projected_indices.size)

        projected_u = u[projected_indices]
        projected_v = v[projected_indices]
        for instance_index in np.argsort(masks.scores):
            score = float(masks.scores[instance_index])
            hits = masks.masks[instance_index, projected_v, projected_u]
            if not np.any(hits):
                continue
            point_indices = projected_indices[hits]
            class_ids[point_indices] = int(masks.classes[instance_index])
            instance_ids[point_indices] = int(instance_index)
            confidence[point_indices] = score

        return class_ids, instance_ids, confidence, int(projected_indices.size)

    def mask_age_and_tf_mode(self, cloud: PointCloud2, masks: MaskResult):
        receipt_age = max(0.0, time.monotonic() - masks.received_monotonic)
        header_age = abs(stamp_to_seconds(cloud.header.stamp) - masks.stamp_sec)
        invalid_header_time = (
            stamp_is_zero(cloud.header.stamp) or
            masks.stamp_sec <= 0.0 or
            header_age > 60.0)

        if self.timestamp_mode == 'receipt':
            return receipt_age, True
        if self.timestamp_mode == 'header':
            return header_age, stamp_is_zero(cloud.header.stamp)
        if invalid_header_time:
            if not self.reported_timestamp_fallback:
                self.get_logger().warn(
                    'LiDAR and image timestamps are zero or use different clock domains; '
                    'falling back to local receipt-time synchronization and latest TF')
                self.reported_timestamp_fallback = True
            return receipt_age, True
        return header_age, False

    def lookup_transform_with_latest_fallback(
            self, target_frame: str, source_frame: str, requested_time: Time):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                requested_time,
                timeout=Duration(seconds=0.05))
            return transform, requested_time
        except TransformException as exact_error:
            if requested_time.nanoseconds == 0:
                raise exact_error
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.05))
                self.warn_throttled(
                    f'TF at the LiDAR timestamp is unavailable; using latest TF for '
                    f'{source_frame} -> {target_frame}')
                return transform, Time()
            except TransformException:
                raise exact_error

    @staticmethod
    def get_intrinsics(info: CameraInfo):
        return float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])

    def warn_throttled(self, message: str):
        now = time.monotonic()
        if now - self.last_wait_warning >= 2.0:
            self.get_logger().warn(message)
            self.last_wait_warning = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarMaskProjectorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
