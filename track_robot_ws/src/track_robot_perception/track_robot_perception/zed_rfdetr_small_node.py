#!/usr/bin/env python3

import json
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class ZedRfDetrSmallNode(Node):
    def __init__(self):
        super().__init__('zed_rfdetr_small_node')

        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.output_image_topic = self.declare_parameter(
            'output_image_topic', '/rfdetr/annotated_image').value
        self.output_text_topic = self.declare_parameter(
            'output_text_topic', '/rfdetr/detections_text').value
        self.score_threshold = float(
            self.declare_parameter('score_threshold', 0.5).value)
        self.device = str(self.declare_parameter('device', 'cuda').value)
        self.weights_path = os.path.expanduser(str(
            self.declare_parameter('weights_path', '').value))
        self.run_every_n_frames = max(
            1, int(self.declare_parameter('run_every_n_frames', 3).value))
        self.max_detections = max(
            1, int(self.declare_parameter('max_detections', 30).value))
        self.publish_annotated_image = parse_bool(
            self.declare_parameter('publish_annotated_image', True).value)
        self.publish_text = parse_bool(
            self.declare_parameter('publish_text', True).value)
        self.debug_timing = parse_bool(
            self.declare_parameter('debug_timing', True).value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.processed_count = 0
        self.stats_start = time.monotonic()
        self.last_stats_log = self.stats_start

        self.model = self.load_model()
        self.class_names = dict(getattr(self.model, 'class_names', {}))

        self.annotated_pub = None
        self.text_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(
                Image, self.output_image_topic, 5)
        if self.publish_text:
            self.text_pub = self.create_publisher(
                String, self.output_text_topic, 5)
        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            'RF-DETR Small: {} -> {}; detections={}; threshold={:.2f}; '
            'device={}; every_n={}'.format(
                self.image_topic, self.output_image_topic,
                self.output_text_topic, self.score_threshold,
                self.device, self.run_every_n_frames))

    def load_model(self):
        try:
            from rfdetr import RFDETRSmall
        except Exception as exc:
            raise RuntimeError(
                'Roboflow RF-DETR is not available in this runtime. The current '
                'release requires Python >=3.10, PyTorch >=2.2, and torchvision '
                '>=0.17; this ROS Foxy host has Python 3.8 and PyTorch 1.13. '
                'Do not install the unrelated rfdetr 0.0.1 package exposed to '
                'Python 3.8. Run this node in a compatible ROS runtime, or export '
                'RFDETRSmall to ONNX on a newer machine and deploy that artifact '
                'with a Foxy-compatible inference backend. '
                'Import error: {}'.format(exc)) from exc

        kwargs = {'device': self.device}
        if self.weights_path:
            if not os.path.isfile(self.weights_path):
                raise FileNotFoundError(
                    'RF-DETR weights_path does not exist: {}'.format(
                        self.weights_path))
            kwargs['pretrain_weights'] = self.weights_path

        return RFDETRSmall(**kwargs)

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.run_every_n_frames != 0:
            return

        try:
            image_bgr = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        except Exception as exc:
            self.get_logger().warn(
                'Failed to convert ZED image: {}'.format(exc))
            return

        start = time.monotonic()
        try:
            detections = self.model.predict(
                image_rgb, threshold=self.score_threshold)
        except Exception as exc:
            self.get_logger().error(
                'RF-DETR Small inference failed: {}'.format(exc))
            return
        elapsed_ms = (time.monotonic() - start) * 1000.0
        self.processed_count += 1

        boxes, confidences, class_ids = self.detection_arrays(detections)
        if len(confidences) > self.max_detections:
            keep = np.argsort(confidences)[::-1][:self.max_detections]
            boxes = boxes[keep]
            confidences = confidences[keep]
            class_ids = class_ids[keep]

        records = self.make_records(
            boxes, confidences, class_ids,
            image_bgr.shape[1], image_bgr.shape[0])

        if self.publish_text and self.text_pub is not None:
            payload = {
                'stamp': {
                    'sec': int(msg.header.stamp.sec),
                    'nanosec': int(msg.header.stamp.nanosec),
                },
                'model': 'RFDETRSmall',
                'count': len(records),
                'inference_ms': round(elapsed_ms, 2),
                'image_size': [
                    int(image_bgr.shape[1]), int(image_bgr.shape[0])],
                'detections': records,
            }
            self.text_pub.publish(String(data=json.dumps(payload)))

        if self.publish_annotated_image and self.annotated_pub is not None:
            annotated = self.draw_detections(image_bgr, records)
            output_msg = self.bridge.cv2_to_imgmsg(
                annotated, encoding='bgr8')
            output_msg.header = msg.header
            self.annotated_pub.publish(output_msg)

        self.log_stats(elapsed_ms, len(records))

    @staticmethod
    def detection_arrays(detections):
        boxes = np.asarray(
            getattr(detections, 'xyxy', np.empty((0, 4))),
            dtype=np.float32).reshape(-1, 4)
        confidences = np.asarray(
            getattr(detections, 'confidence', np.empty(0)),
            dtype=np.float32).reshape(-1)
        class_ids = np.asarray(
            getattr(detections, 'class_id', np.empty(0)),
            dtype=np.int32).reshape(-1)

        count = min(len(boxes), len(confidences), len(class_ids))
        return boxes[:count], confidences[:count], class_ids[:count]

    def make_records(self, boxes, confidences, class_ids, width, height):
        records = []
        for box, confidence, class_id in zip(
                boxes, confidences, class_ids):
            x1, y1, x2, y2 = box.tolist()
            clipped = [
                int(round(np.clip(x1, 0, max(0, width - 1)))),
                int(round(np.clip(y1, 0, max(0, height - 1)))),
                int(round(np.clip(x2, 0, max(0, width - 1)))),
                int(round(np.clip(y2, 0, max(0, height - 1)))),
            ]
            records.append({
                'class': self.class_name(int(class_id)),
                'class_id': int(class_id),
                'score': round(float(confidence), 4),
                'bbox': clipped,
            })
        return records

    def class_name(self, class_id):
        if class_id in self.class_names:
            return str(self.class_names[class_id])
        if str(class_id) in self.class_names:
            return str(self.class_names[str(class_id)])
        return 'class_{}'.format(class_id)

    def draw_detections(self, image_bgr, records):
        output = image_bgr.copy()
        for record in records:
            x1, y1, x2, y2 = record['bbox']
            color = self.class_color(record['class_id'])
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            label = '{} {:.2f}'.format(
                record['class'], record['score'])
            (label_width, label_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            label_top = max(0, y1 - label_height - baseline - 6)
            cv2.rectangle(
                output, (x1, label_top),
                (min(output.shape[1] - 1, x1 + label_width + 8), y1),
                color, -1)
            cv2.putText(
                output, label, (x1 + 4, max(label_height + 1, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)
        return output

    @staticmethod
    def class_color(class_id):
        hue = (int(class_id) * 37) % 180
        hsv = np.uint8([[[hue, 210, 245]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return tuple(int(channel) for channel in bgr)

    def log_stats(self, elapsed_ms, detection_count):
        if not self.debug_timing:
            return
        now = time.monotonic()
        if now - self.last_stats_log < 5.0:
            return
        fps = self.processed_count / max(1e-6, now - self.stats_start)
        self.get_logger().info(
            'RF-DETR Small inference {:.1f} ms; detections={}; '
            'processed_fps={:.2f}'.format(
                elapsed_ms, detection_count, fps))
        self.last_stats_log = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedRfDetrSmallNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
