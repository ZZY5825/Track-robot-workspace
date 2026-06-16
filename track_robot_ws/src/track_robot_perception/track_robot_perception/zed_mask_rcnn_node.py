#!/usr/bin/env python3

import json
import time
from typing import List

import torch
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


DEFAULT_MODEL_CONFIG = 'COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml'
KEYPOINT_SCORE_THRESHOLD = 0.05


def torch_cuda_available() -> bool:
    return bool(torch.cuda.is_available())


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class ZedMaskRcnnNode(Node):
    def __init__(self):
        super().__init__('zed_mask_rcnn_node')

        default_device = 'cuda' if torch_cuda_available() else 'cpu'
        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.output_image_topic = self.declare_parameter(
            'output_image_topic', '/mask_rcnn/annotated_image').value
        self.output_text_topic = self.declare_parameter(
            'output_text_topic', '/mask_rcnn/detections_text').value
        self.model_config = self.declare_parameter(
            'model_config', DEFAULT_MODEL_CONFIG).value
        self.score_threshold = float(self.declare_parameter('score_threshold', 0.5).value)
        device_param = self.declare_parameter('device', default_device).value
        self.device = default_device if str(device_param).lower() == 'auto' else str(device_param)
        self.publish_annotated_image = parse_bool(
            self.declare_parameter('publish_annotated_image', True).value)
        self.publish_text = parse_bool(self.declare_parameter('publish_text', True).value)
        self.run_every_n_frames = max(
            1, int(self.declare_parameter('run_every_n_frames', 1).value))
        self.resize_width = max(0, int(self.declare_parameter('resize_width', 0).value))
        self.max_detections = max(1, int(self.declare_parameter('max_detections', 20).value))

        self.bridge = CvBridge()
        self.frame_count = 0
        self.processed_count = 0
        self.last_stats_log_time = time.monotonic()
        self.stats_window_start_time = self.last_stats_log_time

        self.predictor = None
        self.metadata = None
        self.class_names: List[str] = []
        self.keypoint_names: List[str] = []
        self._load_detectron2()

        self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.annotated_pub = None
        self.text_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(Image, self.output_image_topic, 5)
        if self.publish_text:
            self.text_pub = self.create_publisher(String, self.output_text_topic, 5)

        self.get_logger().info(
            f'Running Detectron2 model on {self.image_topic}; '
            f'annotated={self.output_image_topic}; detections={self.output_text_topic}; '
            f'model={self.model_config}; threshold={self.score_threshold:.2f}; '
            f'device={self.device}; every_n={self.run_every_n_frames}; '
            f'resize_width={self.resize_width}')

    def _load_detectron2(self):
        try:
            from detectron2 import model_zoo
            from detectron2.config import get_cfg
            from detectron2.data import MetadataCatalog
            from detectron2.engine import DefaultPredictor
        except Exception as exc:
            self.get_logger().error(
                'Detectron2 is not importable. Install a PyTorch/CUDA-compatible Detectron2 '
                f'build before running this node. Import error: {exc}')
            raise

        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file(self.model_config))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = self.score_threshold
        cfg.MODEL.DEVICE = self.device
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(self.model_config)

        self.predictor = DefaultPredictor(cfg)
        dataset_names = cfg.DATASETS.TRAIN
        self.metadata = MetadataCatalog.get(dataset_names[0]) if dataset_names else None
        self.class_names = list(getattr(self.metadata, 'thing_classes', []))
        self.keypoint_names = list(getattr(self.metadata, 'keypoint_names', []))

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.run_every_n_frames != 0:
            return

        try:
            image_bgr = self.ros_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')
            return

        inference_image, scale_x, scale_y = self.resize_for_inference(image_bgr)

        start_time = time.monotonic()
        try:
            outputs = self.predictor(inference_image)
        except Exception as exc:
            self.get_logger().error(f'Mask R-CNN inference failed on this frame: {exc}')
            return
        elapsed_ms = (time.monotonic() - start_time) * 1000.0
        self.processed_count += 1

        instances = outputs['instances'].to('cpu')
        instances = self.limit_instances(instances)

        if self.publish_text and self.text_pub is not None:
            self.publish_detection_text(instances, msg.header.stamp, scale_x, scale_y)

        if self.publish_annotated_image and self.annotated_pub is not None:
            try:
                annotated = self.draw_instances(inference_image, instances)
                if self.resize_width > 0:
                    annotated = cv2.resize(
                        annotated, (image_bgr.shape[1], image_bgr.shape[0]),
                        interpolation=cv2.INTER_LINEAR)
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)
            except Exception as exc:
                self.get_logger().warn(f'Failed to publish annotated image: {exc}')

        self.log_stats(elapsed_ms, len(instances))

    def ros_image_to_bgr(self, msg: Image) -> np.ndarray:
        if msg.encoding == 'bgr8':
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if msg.encoding == 'rgb8':
            image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def resize_for_inference(self, image_bgr: np.ndarray):
        if self.resize_width <= 0 or image_bgr.shape[1] <= self.resize_width:
            return image_bgr, 1.0, 1.0

        scale = float(self.resize_width) / float(image_bgr.shape[1])
        new_height = max(1, int(round(image_bgr.shape[0] * scale)))
        resized = cv2.resize(
            image_bgr, (self.resize_width, new_height), interpolation=cv2.INTER_AREA)
        return resized, 1.0 / scale, 1.0 / scale

    def limit_instances(self, instances):
        if len(instances) <= self.max_detections:
            return instances
        scores = instances.scores
        keep = scores.argsort(descending=True)[:self.max_detections]
        return instances[keep]

    def draw_instances(self, image_bgr: np.ndarray, instances) -> np.ndarray:
        from detectron2.utils.visualizer import Visualizer

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        visualizer = Visualizer(image_rgb, metadata=self.metadata, scale=1.0)
        annotated_rgb = visualizer.draw_instance_predictions(instances).get_image()
        return cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)

    def publish_detection_text(self, instances, stamp, scale_x: float, scale_y: float):
        boxes = instances.pred_boxes.tensor.numpy() if instances.has('pred_boxes') else []
        scores = instances.scores.numpy() if instances.has('scores') else []
        classes = instances.pred_classes.numpy() if instances.has('pred_classes') else []
        masks = instances.pred_masks.numpy() if instances.has('pred_masks') else []
        keypoints = instances.pred_keypoints.numpy() if instances.has('pred_keypoints') else []

        lines = []
        records = []
        for index in range(len(instances)):
            class_id = int(classes[index]) if len(classes) > index else -1
            class_name = self.class_name(class_id)
            score = float(scores[index]) if len(scores) > index else 0.0
            bbox = self.scale_box(boxes[index], scale_x, scale_y) if len(boxes) > index else []
            mask_area = 0
            if len(masks) > index:
                mask_area = int(round(np.count_nonzero(masks[index]) * scale_x * scale_y))
            scaled_keypoints = (
                self.scale_keypoints(keypoints[index], scale_x, scale_y)
                if len(keypoints) > index else [])
            visible_keypoints = sum(
                1 for keypoint in scaled_keypoints
                if keypoint['score'] >= KEYPOINT_SCORE_THRESHOLD)

            lines.append(
                f'class={class_name}, score={score:.2f}, bbox={bbox}, '
                f'mask_area={mask_area}, keypoints={visible_keypoints}')
            records.append({
                'class': class_name,
                'class_id': class_id,
                'score': round(score, 4),
                'bbox': bbox,
                'mask_area': mask_area,
                'keypoints': scaled_keypoints,
            })

        payload = {
            'stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},
            'count': len(records),
            'detections': records,
            'summary': lines,
        }
        self.text_pub.publish(String(data=json.dumps(payload)))

    def class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f'class_{class_id}'

    def scale_box(self, box, scale_x: float, scale_y: float):
        x1, y1, x2, y2 = box.tolist()
        return [
            int(round(x1 * scale_x)),
            int(round(y1 * scale_y)),
            int(round(x2 * scale_x)),
            int(round(y2 * scale_y)),
        ]

    def scale_keypoints(self, keypoints, scale_x: float, scale_y: float):
        scaled = []
        for index, keypoint in enumerate(keypoints):
            name = self.keypoint_names[index] if index < len(self.keypoint_names) else f'kp_{index}'
            x, y, score = keypoint.tolist()
            scaled.append({
                'name': name,
                'x': int(round(x * scale_x)),
                'y': int(round(y * scale_y)),
                'score': round(float(score), 4),
            })
        return scaled

    def log_stats(self, elapsed_ms: float, detection_count: int):
        now = time.monotonic()
        if now - self.last_stats_log_time < 5.0:
            return

        window_sec = max(1e-6, now - self.stats_window_start_time)
        fps = float(self.processed_count) / window_sec
        self.get_logger().info(
            f'Detectron2 inference {elapsed_ms:.1f} ms; detections={detection_count}; '
            f'processed_fps={fps:.2f}')
        self.last_stats_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedMaskRcnnNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
