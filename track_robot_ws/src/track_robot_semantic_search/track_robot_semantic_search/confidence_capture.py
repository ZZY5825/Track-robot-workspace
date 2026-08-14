"""Subscriber-only capture for the controlled confidence benchmark."""

import argparse
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np

from .confidence_benchmark import append_trial


IMAGE_TOPIC = '/zed/zed_node/left/image_rect_color'
DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'


def _parser():
    parser = argparse.ArgumentParser(
        description='Capture synchronized ZED RGB/depth benchmark samples.')
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--trial-id', required=True)
    parser.add_argument(
        '--kind', required=True,
        choices=('target', 'distractor', 'background'))
    parser.add_argument('--label', required=True)
    parser.add_argument('--distance-m', required=True, type=float)
    parser.add_argument('--samples', type=int, default=10)
    parser.add_argument('--interval-sec', type=float, default=0.5)
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    parser.add_argument('--notes', default='static controlled fixture')
    roi = parser.add_mutually_exclusive_group(required=True)
    roi.add_argument(
        '--roi', nargs=4, type=float, metavar=('X', 'Y', 'W', 'H'))
    roi.add_argument('--interactive-roi', action='store_true')
    return parser


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _git_commit(workspace):
    try:
        value = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=str(workspace),
            stderr=subprocess.DEVNULL, text=True, timeout=5.0).strip()
    except (OSError, subprocess.SubprocessError):
        return 'unknown'
    return value or 'unknown'


def _stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class _SynchronizedZedReceiver:
    def __init__(self, node):
        from cv_bridge import CvBridge
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        from sensor_msgs.msg import Image

        self._bridge = CvBridge()
        self.latest = None
        self._rgb = Subscriber(node, Image, IMAGE_TOPIC)
        self._depth = Subscriber(node, Image, DEPTH_TOPIC)
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb, self._depth], queue_size=8, slop=0.08)
        self._sync.registerCallback(self._callback)

    def _callback(self, image_message, depth_message):
        image = self._bridge.imgmsg_to_cv2(
            image_message, desired_encoding='bgr8')
        depth = self._bridge.imgmsg_to_cv2(
            depth_message, desired_encoding='32FC1')
        image = np.asarray(image, dtype=np.uint8)
        depth = np.asarray(depth, dtype=np.float32)
        if image.ndim != 3 or image.shape[2] != 3 or depth.ndim != 2:
            return
        if image.shape[:2] != depth.shape:
            return
        self.latest = (
            _stamp_ns(image_message), image.copy(), depth.copy(),
            str(image_message.header.frame_id),
            str(depth_message.header.frame_id))


def _wait_for_new_pair(node, receiver, after_stamp_ns, deadline):
    import rclpy

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        value = receiver.latest
        if value is not None and value[0] > after_stamp_ns:
            return value
    raise TimeoutError('timed out waiting for synchronized ZED RGB/depth')


def _select_roi(image, configured_roi, interactive):
    if interactive:
        value = cv2.selectROI(
            'Select the labelled object, then press ENTER', image,
            showCrosshair=True, fromCenter=False)
        cv2.destroyWindow('Select the labelled object, then press ENTER')
        roi = [float(item) for item in value]
    else:
        roi = [float(item) for item in configured_roi]
    x, y, width, height = roi
    if (x < 0.0 or y < 0.0 or width <= 0.0 or height <= 0.0 or
            x + width > image.shape[1] or y + height > image.shape[0]):
        raise ValueError('ROI is empty or outside the ZED image')
    return roi


def _base_document(dataset_path):
    source_location = Path(__file__).resolve().parent
    return {
        'schema_version': 'semantic_confidence_dataset/1.0.0',
        'dataset_id': dataset_path.parent.name,
        'query_text': 'green bottle',
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'provenance': {
            'git_commit': _git_commit(source_location),
            'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', '20')),
            'image_topic': IMAGE_TOPIC,
            'depth_topic': DEPTH_TOPIC,
        },
        'trials': [],
    }


def main(argv=None):
    args = _parser().parse_args(argv)
    if not 1 <= args.samples <= 1000:
        raise SystemExit('--samples must be in [1, 1000]')
    if args.interval_sec < 0.0 or args.timeout_sec <= 0.0:
        raise SystemExit('capture timing arguments are invalid')

    import rclpy

    rclpy.init(args=None)
    node = rclpy.create_node('semantic_search_confidence_capture')
    receiver = _SynchronizedZedReceiver(node)
    deadline = time.monotonic() + args.timeout_sec
    dataset_path = Path(args.dataset).resolve()
    try:
        first = _wait_for_new_pair(node, receiver, -1, deadline)
        roi = _select_roi(first[1], args.roi, args.interactive_roi)
        images_dir = dataset_path.parent / 'images'
        depth_dir = dataset_path.parent / 'depth'
        images_dir.mkdir(parents=True, exist_ok=True)
        depth_dir.mkdir(parents=True, exist_ok=True)
        samples = []
        previous_stamp = -1
        next_capture = time.monotonic()
        for index in range(args.samples):
            while time.monotonic() < next_capture:
                rclpy.spin_once(node, timeout_sec=min(
                    0.1, next_capture - time.monotonic()))
            value = _wait_for_new_pair(
                node, receiver, previous_stamp, deadline)
            stamp_ns, image, depth, image_frame, depth_frame = value
            sample_id = '{}-{:03d}'.format(args.trial_id, index)
            image_path = images_dir / '{}.png'.format(sample_id)
            depth_path = depth_dir / '{}.npy'.format(sample_id)
            if image_path.exists() or depth_path.exists():
                raise FileExistsError(
                    'sample already exists: {}'.format(sample_id))
            if not cv2.imwrite(str(image_path), image):
                raise OSError('unable to write captured ZED image')
            np.save(str(depth_path), depth, allow_pickle=False)
            samples.append({
                'sample_id': sample_id,
                'stamp_ns': stamp_ns,
                'image_relative_path': str(image_path.relative_to(
                    dataset_path.parent)),
                'image_sha256': _sha256(image_path),
                'depth_relative_path': str(depth_path.relative_to(
                    dataset_path.parent)),
                'depth_sha256': _sha256(depth_path),
                'image_width': int(image.shape[1]),
                'image_height': int(image.shape[0]),
                'image_frame_id': image_frame,
                'depth_frame_id': depth_frame,
            })
            previous_stamp = stamp_ns
            next_capture = time.monotonic() + args.interval_sec
            node.get_logger().info(
                'captured {}/{}: {}'.format(
                    index + 1, args.samples, sample_id))
        trial = {
            'trial_id': args.trial_id,
            'ground_truth_kind': args.kind,
            'ground_truth_label': args.label,
            'nominal_distance_m': args.distance_m,
            'ground_truth_bbox_xywh': roi,
            'label_review_status': 'human_authored',
            'notes': args.notes,
            'samples': samples,
        }
        append_trial(dataset_path, _base_document(dataset_path), trial)
        node.get_logger().info(
            'saved controlled trial to {}'.format(dataset_path))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
