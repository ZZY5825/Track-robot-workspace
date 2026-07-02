#!/usr/bin/env python3

import json
import time
from typing import List, Optional, Tuple

import torch  # Import before OpenCV/cv_bridge on Jetson to avoid libgomp TLS errors.
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from track_robot_interfaces.msg import HumanDetection2D, HumanDetection2DArray


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class HumanImageTrackerNode(Node):
    def __init__(self):
        super().__init__('human_image_tracker_node')

        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/human_tracking/detections').value
        self.annotated_image_topic = self.declare_parameter(
            'annotated_image_topic', '/human_tracking/annotated_image').value
        self.debug_topic = self.declare_parameter(
            'debug_topic', '/human_tracking/tracker_debug').value
        self.model_path = self.declare_parameter('model_path', 'yolov8n.pt').value
        self.tracker_backend = str(
            self.declare_parameter('tracker_backend', 'bytetrack').value).lower()
        self.conf_threshold = float(self.declare_parameter('conf_threshold', 0.35).value)
        self.iou_threshold = float(self.declare_parameter('iou_threshold', 0.5).value)
        self.person_class_id = int(self.declare_parameter('person_class_id', 0).value)
        self.device = str(self.declare_parameter('device', 'auto').value)
        self.resize_width = max(0, int(self.declare_parameter('resize_width', 0).value))
        self.run_every_n_frames = max(
            1, int(self.declare_parameter('run_every_n_frames', 1).value))
        self.publish_annotated_image = parse_bool(
            self.declare_parameter('publish_annotated_image', True).value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.processed_count = 0
        self.last_stats_time = time.monotonic()
        self.model = self.load_model()

        self.detections_pub = self.create_publisher(
            HumanDetection2DArray, self.detections_topic, 5)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 5)
        self.annotated_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(Image, self.annotated_image_topic, 5)
        self.create_subscription(Image, self.image_topic, self.image_callback, 5)

        self.get_logger().info(
            f'human_image_tracker_node subscribed to {self.image_topic}; '
            f'publishing {self.detections_topic}; backend={self.tracker_backend}; '
            f'model={self.model_path}')

    def load_model(self):
        try:
            from ultralytics import YOLO
        except Exception as exc:
            self.get_logger().error(
                'Ultralytics is not importable. Install a Jetson-compatible YOLO runtime '
                f'or use another detector backend. Import error: {exc}')
            return None
        try:
            return YOLO(self.model_path)
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model {self.model_path}: {exc}')
            return None

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.run_every_n_frames != 0:
            return

        if self.model is None:
            self.publish_empty(msg, 'model_not_loaded')
            return

        try:
            image_bgr = self.ros_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            return

        inference_image, scale_x, scale_y = self.resize_for_inference(image_bgr)
        tracker_cfg = 'botsort.yaml' if self.tracker_backend == 'botsort' else 'bytetrack.yaml'
        kwargs = {
            'persist': True,
            'tracker': tracker_cfg,
            'conf': self.conf_threshold,
            'iou': self.iou_threshold,
            'classes': [self.person_class_id],
            'verbose': False,
        }
        if self.device.lower() != 'auto':
            kwargs['device'] = self.device

        start = time.monotonic()
        try:
            results = self.model.track(inference_image, **kwargs)
        except Exception as exc:
            self.get_logger().error(f'YOLO tracking failed: {exc}')
            self.publish_empty(msg, 'tracking_failed')
            return
        elapsed_ms = (time.monotonic() - start) * 1000.0
        self.processed_count += 1

        result = results[0] if results else None
        detections = self.extract_detections(result, msg, scale_x, scale_y)
        self.detections_pub.publish(detections)
        self.publish_debug(msg, detections, elapsed_ms)
        if self.annotated_pub is not None:
            self.publish_annotated(msg, image_bgr, result)
        self.log_stats(elapsed_ms, len(detections.detections))

    def publish_empty(self, msg: Image, status: str):
        output = HumanDetection2DArray()
        output.header = msg.header
        self.detections_pub.publish(output)
        self.debug_pub.publish(String(data=json.dumps({'status': status, 'count': 0})))

    def ros_image_to_bgr(self, msg: Image) -> np.ndarray:
        if msg.encoding == 'bgr8':
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if msg.encoding == 'rgb8':
            image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def resize_for_inference(self, image_bgr: np.ndarray) -> Tuple[np.ndarray, float, float]:
        if self.resize_width <= 0 or image_bgr.shape[1] <= self.resize_width:
            return image_bgr, 1.0, 1.0
        scale = float(self.resize_width) / float(image_bgr.shape[1])
        new_height = max(1, int(round(image_bgr.shape[0] * scale)))
        resized = cv2.resize(
            image_bgr, (self.resize_width, new_height), interpolation=cv2.INTER_AREA)
        return resized, 1.0 / scale, 1.0 / scale

    def extract_detections(
            self, result, msg: Image, scale_x: float, scale_y: float) -> HumanDetection2DArray:
        output = HumanDetection2DArray()
        output.header = msg.header
        if result is None or getattr(result, 'boxes', None) is None:
            return output

        boxes = result.boxes
        xyxy = self.tensor_to_numpy(getattr(boxes, 'xyxy', None))
        conf = self.tensor_to_numpy(getattr(boxes, 'conf', None))
        cls = self.tensor_to_numpy(getattr(boxes, 'cls', None))
        ids = self.tensor_to_numpy(getattr(boxes, 'id', None))
        keypoints = self.result_keypoints(result)

        if xyxy is None:
            return output

        for index, box in enumerate(xyxy):
            class_id = int(cls[index]) if cls is not None and len(cls) > index else -1
            if class_id != self.person_class_id:
                continue
            detection = HumanDetection2D()
            detection.header = msg.header
            detection.track_id = self.track_id_at(ids, index)
            detection.class_name = 'person'
            detection.score = float(conf[index]) if conf is not None and len(conf) > index else 0.0
            detection.bbox = [
                float(box[0] * scale_x),
                float(box[1] * scale_y),
                float(box[2] * scale_x),
                float(box[3] * scale_y),
            ]
            if keypoints is not None and len(keypoints) > index:
                detection.keypoints = [
                    float(value)
                    for point in keypoints[index]
                    for value in (point[0] * scale_x, point[1] * scale_y, point[2])
                ]
            output.detections.append(detection)
        return output

    @staticmethod
    def track_id_at(ids: Optional[np.ndarray], index: int) -> int:
        if ids is None or len(ids) <= index or ids[index] is None:
            return -1
        value = float(ids[index])
        if not np.isfinite(value):
            return -1
        return int(value)

    @staticmethod
    def tensor_to_numpy(value) -> Optional[np.ndarray]:
        if value is None:
            return None
        if hasattr(value, 'detach'):
            return value.detach().cpu().numpy()
        if hasattr(value, 'cpu'):
            return value.cpu().numpy()
        return np.asarray(value)

    def result_keypoints(self, result):
        keypoints = getattr(result, 'keypoints', None)
        if keypoints is None:
            return None
        data = getattr(keypoints, 'data', None)
        return self.tensor_to_numpy(data)

    def publish_annotated(self, msg: Image, original_bgr: np.ndarray, result):
        try:
            if result is not None:
                annotated = result.plot()
                if annotated.shape[:2] != original_bgr.shape[:2]:
                    annotated = cv2.resize(
                        annotated, (original_bgr.shape[1], original_bgr.shape[0]),
                        interpolation=cv2.INTER_LINEAR)
            else:
                annotated = original_bgr
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to publish annotated image: {exc}')

    def publish_debug(self, msg: Image, detections: HumanDetection2DArray, elapsed_ms: float):
        payload = {
            'stamp': {'sec': int(msg.header.stamp.sec), 'nanosec': int(msg.header.stamp.nanosec)},
            'count': len(detections.detections),
            'processing_ms': round(elapsed_ms, 3),
            'tracker_backend': self.tracker_backend,
            'track_ids': [int(detection.track_id) for detection in detections.detections],
        }
        self.debug_pub.publish(String(data=json.dumps(payload)))

    def log_stats(self, elapsed_ms: float, detection_count: int):
        now = time.monotonic()
        if now - self.last_stats_time < 5.0:
            return
        self.last_stats_time = now
        self.get_logger().info(
            f'YOLO tracking {elapsed_ms:.1f} ms; detections={detection_count}; '
            f'processed={self.processed_count}')


def main(args=None):
    rclpy.init(args=args)
    node = HumanImageTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
