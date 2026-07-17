#!/usr/bin/env python3

import json
import os
import time

import torch
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from track_robot_perception.dinov3_runtime import (
    DEFAULT_HF_MODEL_ID,
    extract_features,
    load_model,
    make_feature_heatmap,
    normalize_feature_rows,
    patch_grid,
    preprocess_bgr_aspect_preserving,
)


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class ZedDinov3FeatureNode(Node):
    def __init__(self):
        super().__init__('zed_dinov3_feature_node')

        default_device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/dinov3/debug_image').value
        self.debug_text_topic = self.declare_parameter(
            'debug_text_topic', '/dinov3/feature_debug').value
        self.model_name = self.declare_parameter(
            'model_name', 'dinov3_vits16plus').value
        self.model_source = self.declare_parameter(
            'model_source', 'torch_hub').value
        requested_device = str(self.declare_parameter(
            'device', default_device).value)
        self.device = default_device if requested_device == 'auto' else requested_device
        self.input_size = int(self.declare_parameter('input_size', 512).value)
        self.run_every_n_frames = max(
            1, int(self.declare_parameter('run_every_n_frames', 5).value))
        self.save_features = parse_bool(
            self.declare_parameter('save_features', False).value)
        self.output_dir = os.path.expanduser(str(self.declare_parameter(
            'output_dir',
            '/home/track-robot/track_robot_ws/dinov3_feature_outputs').value))
        self.publish_heatmap = parse_bool(
            self.declare_parameter('publish_heatmap', True).value)
        self.normalize_features = parse_bool(
            self.declare_parameter('normalize_features', True).value)
        self.max_saved_frames = max(
            0, int(self.declare_parameter('max_saved_frames', 100).value))
        self.debug_timing = parse_bool(
            self.declare_parameter('debug_timing', True).value)
        self.local_repo = str(self.declare_parameter('local_repo', '').value)
        self.weights_path = str(self.declare_parameter('weights_path', '').value)
        self.hf_model_id = str(self.declare_parameter(
            'hf_model_id', DEFAULT_HF_MODEL_ID).value)

        if self.device.startswith('cuda') and not torch.cuda.is_available():
            raise RuntimeError('CUDA was requested but torch.cuda.is_available() is false')

        self.model, self.backend = load_model(
            self.model_source,
            self.model_name,
            self.device,
            self.local_repo,
            self.weights_path,
            self.hf_model_id)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self.keys_logged = False
        self.debug_image_pub = self.create_publisher(
            Image, self.debug_image_topic, 5)
        self.debug_text_pub = self.create_publisher(
            String, self.debug_text_topic, 5)
        self.create_subscription(Image, self.image_topic, self.image_callback, 5)

        if self.save_features:
            os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(
            'DINOv3 feature extraction: model={} source={} device={} input={} topic={}'.format(
                self.model_name, self.backend, self.device, self.input_size,
                self.image_topic))

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.run_every_n_frames != 0:
            return

        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            input_tensor, transform = preprocess_bgr_aspect_preserving(
                image_bgr, self.input_size)
            input_tensor = input_tensor.to(self.device)
            start = time.monotonic()
            cls_token, patch_tokens, details = extract_features(
                self.model, input_tensor, self.backend)
            if self.device.startswith('cuda'):
                torch.cuda.synchronize()
            elapsed_ms = (time.monotonic() - start) * 1000.0
            grid = patch_grid(
                patch_tokens, transform.grid_height, transform.grid_width)
        except Exception as exc:
            self.get_logger().error('DINOv3 inference failed: {}'.format(exc))
            return

        if not self.keys_logged and details:
            self.get_logger().info('Model output details: {}'.format(details))
            self.keys_logged = True

        cls_array = cls_token[0].detach().float().cpu().numpy()
        patch_array = grid
        if self.normalize_features:
            cls_array = normalize_feature_rows(cls_array)
            patch_array = normalize_feature_rows(patch_array)

        payload = {
            'stamp': {'sec': int(msg.header.stamp.sec),
                      'nanosec': int(msg.header.stamp.nanosec)},
            'model': self.model_name,
            'source': self.backend,
            'device': self.device,
            'input_size': self.input_size,
            'original_size': [int(image_bgr.shape[1]), int(image_bgr.shape[0])],
            'resized_size': [
                transform.resized_width, transform.resized_height],
            'preprocessing_scale': transform.scale,
            'padding': {
                'left': transform.padding_left,
                'top': transform.padding_top,
                'right': transform.padding_right,
                'bottom': transform.padding_bottom,
            },
            'valid_patch_count': int(transform.valid_patch_mask.sum()),
            'preprocessing_version': 'aspect_pad_v1',
            'cls_token_shape': list(cls_array.shape),
            'patch_tokens_shape': list(patch_array.shape),
            'inference_ms': round(elapsed_ms, 2),
            'heatmap_is_semantic_mask': False,
        }
        payload.update(details)
        self.debug_text_pub.publish(String(data=json.dumps(payload)))

        if self.publish_heatmap:
            debug_image = make_feature_heatmap(image_bgr, grid)
            cv2.putText(
                debug_image,
                'DINOv3 {} {:.1f} ms | feature norm (not segmentation)'.format(
                    self.model_name, elapsed_ms),
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
                cv2.LINE_AA)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

        if self.save_features and self.saved_count < self.max_saved_frames:
            self.save_frame(
                msg, cls_array, patch_array, transform.valid_patch_mask, payload)

        if self.debug_timing:
            self.get_logger().info(
                'DINOv3 inference {:.1f} ms; cls={}; patches={}'.format(
                    elapsed_ms, tuple(cls_array.shape), tuple(patch_array.shape)))

    def save_frame(
            self, msg, cls_token, patch_tokens, valid_patch_mask, metadata):
        stamp = '{}_{:09d}'.format(msg.header.stamp.sec, msg.header.stamp.nanosec)
        frame_dir = os.path.join(self.output_dir, stamp)
        os.makedirs(frame_dir, exist_ok=False)
        np.save(os.path.join(frame_dir, 'cls_token.npy'), cls_token)
        np.save(os.path.join(frame_dir, 'patch_tokens.npy'), patch_tokens)
        np.save(os.path.join(frame_dir, 'valid_patch_mask.npy'), valid_patch_mask)
        with open(os.path.join(frame_dir, 'metadata.json'), 'w') as stream:
            json.dump(metadata, stream, indent=2)
        self.saved_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedDinov3FeatureNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
