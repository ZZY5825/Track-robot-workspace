#!/usr/bin/env python3

import math
import json
from typing import List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from track_robot_interfaces.msg import GestureState, HumanDetection2D, HumanDetection2DArray, TargetState


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class CameraTargetLockNode(Node):
    def __init__(self):
        super().__init__('camera_target_lock_node')

        self.detections_topic = self.declare_parameter(
            'detections_topic', '/human_tracking/detections').value
        self.gesture_topic = self.declare_parameter(
            'gesture_topic', '/human_tracking/gesture_state').value
        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/human_tracking/camera_target').value
        self.overlay_topic = self.declare_parameter(
            'overlay_topic', '/human_tracking/target_overlay').value
        self.debug_topic = self.declare_parameter(
            'debug_topic', '/human_tracking/camera_target_debug').value
        self.max_camera_dropout_sec = float(
            self.declare_parameter('max_camera_dropout_sec', 2.0).value)
        self.target_memory_sec = float(
            self.declare_parameter('target_memory_sec', 8.0).value)
        self.min_lock_confidence = float(
            self.declare_parameter('min_lock_confidence', 0.35).value)
        self.reacquire_iou = float(
            self.declare_parameter('track_id_reacquire_iou', 0.2).value)
        self.reacquire_max_center_distance_px = float(
            self.declare_parameter('reacquire_max_center_distance_px', 160.0).value)
        self.reacquire_min_score = float(
            self.declare_parameter('reacquire_min_score', 0.35).value)
        self.publish_overlay = parse_bool(
            self.declare_parameter('publish_overlay', True).value)

        self.bridge = CvBridge()
        self.logical_target_id = -1
        self.next_logical_target_id = 1
        self.active_track_id = -1
        self.last_bbox = [0.0, 0.0, 0.0, 0.0]
        self.predicted_bbox = [0.0, 0.0, 0.0, 0.0]
        self.last_confidence = 0.0
        self.velocity_px = [0.0, 0.0]
        self.last_seen_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.latest_detections: List[HumanDetection2D] = []
        self.latest_state = TargetState()
        self.locked = False

        self.target_pub = self.create_publisher(TargetState, self.output_topic, 5)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 5)
        self.overlay_pub = None
        if self.publish_overlay:
            self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 5)
            self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.create_subscription(
            HumanDetection2DArray, self.detections_topic, self.detections_callback, 5)
        self.create_subscription(GestureState, self.gesture_topic, self.gesture_callback, 5)

        self.get_logger().info(
            f'camera_target_lock_node publishing {self.output_topic}')

    def gesture_callback(self, msg: GestureState):
        if not msg.trigger_active or msg.track_id < 0:
            return
        if msg.command == 'stop_tracking':
            if self.locked and int(msg.track_id) == self.active_track_id:
                self.get_logger().info(
                    f'Unlocked target logical_id={self.logical_target_id} '
                    f'track_id={self.active_track_id} by stop gesture')
                self.clear_target()
            elif self.locked:
                self.get_logger().info(
                    f'Ignoring stop gesture from non-target track_id={msg.track_id}; '
                    f'current target track_id={self.active_track_id}')
            return
        if msg.command == 'start_tracking':
            if self.locked:
                self.get_logger().info(
                    f'Ignoring start gesture from track_id={msg.track_id}; '
                    f'already tracking logical_id={self.logical_target_id} '
                    f'track_id={self.active_track_id}')
                return
            self.logical_target_id = self.next_logical_target_id
            self.next_logical_target_id += 1
            self.active_track_id = int(msg.track_id)
            self.locked = True
            self.last_seen_time = self.get_clock().now()
            self.last_update_time = self.last_seen_time
            self.get_logger().info(
                f'Locked target logical_id={self.logical_target_id} '
                f'track_id={self.active_track_id}')

    def detections_callback(self, msg: HumanDetection2DArray):
        self.latest_detections = list(msg.detections)
        self.update_prediction()
        detection = self.find_target_detection(msg.detections)
        state = TargetState()
        state.header = msg.header
        state.target_id = int(self.logical_target_id)
        state.source_state = TargetState.SOURCE_CAMERA_ONLY

        if not self.locked or self.logical_target_id < 0:
            state.lock_state = (
                TargetState.LOCK_CANDIDATE_VISIBLE
                if len(msg.detections) > 0 else TargetState.LOCK_NO_TARGET)
            state.source_state = TargetState.SOURCE_NONE
            state.confidence = 0.0
        elif detection is not None:
            self.update_target_from_detection(detection)
            self.last_confidence = float(detection.score)
            state.lock_state = TargetState.LOCK_TARGET_LOCKED
            state.bbox = self.last_bbox
            state.confidence = self.last_confidence
            state.camera_visible = True
        else:
            dropout = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
            state.bbox = self.predicted_bbox
            state.confidence = max(0.0, self.last_confidence * (1.0 - dropout / max(self.max_camera_dropout_sec, 1e-3)))
            state.camera_visible = False
            state.lock_state = (
                TargetState.LOCK_TARGET_LOCKED
                if dropout <= self.max_camera_dropout_sec else TargetState.LOCK_TARGET_LOST)
            if dropout > self.target_memory_sec:
                state.confidence = 0.0
                state.source_state = TargetState.SOURCE_NONE
                self.locked = False
                self.active_track_id = -1

        self.latest_state = state
        self.target_pub.publish(state)
        self.publish_debug(state, detection, msg.detections)

    def clear_target(self):
        self.logical_target_id = -1
        self.active_track_id = -1
        self.locked = False
        self.last_bbox = [0.0, 0.0, 0.0, 0.0]
        self.predicted_bbox = [0.0, 0.0, 0.0, 0.0]
        self.last_confidence = 0.0
        self.velocity_px = [0.0, 0.0]
        state = TargetState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.target_id = -1
        state.lock_state = TargetState.LOCK_NO_TARGET
        state.source_state = TargetState.SOURCE_NONE
        self.latest_state = state
        self.target_pub.publish(state)

    def find_target_detection(self, detections: List[HumanDetection2D]) -> Optional[HumanDetection2D]:
        for detection in detections:
            if int(detection.track_id) < 0:
                continue
            if int(detection.track_id) == self.active_track_id and detection.score >= self.min_lock_confidence:
                return detection
        if self.logical_target_id < 0 or self.last_bbox[2] <= self.last_bbox[0]:
            return None
        dropout = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
        if dropout > self.target_memory_sec:
            return None

        best = None
        best_score = 0.0
        for detection in detections:
            if int(detection.track_id) < 0:
                continue
            if detection.score < self.min_lock_confidence:
                continue
            overlap = self.iou(self.predicted_bbox, detection.bbox)
            center_distance = self.center_distance(self.predicted_bbox, detection.bbox)
            distance_score = max(
                0.0,
                1.0 - center_distance / max(self.reacquire_max_center_distance_px, 1.0))
            size_score = self.size_similarity(self.predicted_bbox, detection.bbox)
            candidate_score = 0.50 * overlap + 0.35 * distance_score + 0.15 * size_score
            if candidate_score > best_score:
                best = detection
                best_score = candidate_score
        if best is not None and (best_score >= self.reacquire_min_score or self.iou(self.predicted_bbox, best.bbox) >= self.reacquire_iou):
            old_track_id = self.active_track_id
            self.active_track_id = int(best.track_id)
            if old_track_id != self.active_track_id:
                self.get_logger().info(
                    f'Reacquired logical_id={self.logical_target_id}: '
                    f'track_id {old_track_id} -> {self.active_track_id} '
                    f'score={best_score:.2f}')
            return best
        return None

    def update_prediction(self):
        if self.last_bbox[2] <= self.last_bbox[0]:
            return
        now = self.get_clock().now()
        dt = (now - self.last_seen_time).nanoseconds * 1e-9
        dx = self.velocity_px[0] * dt
        dy = self.velocity_px[1] * dt
        self.predicted_bbox = [
            self.last_bbox[0] + dx,
            self.last_bbox[1] + dy,
            self.last_bbox[2] + dx,
            self.last_bbox[3] + dy,
        ]

    def update_target_from_detection(self, detection: HumanDetection2D):
        now = self.get_clock().now()
        bbox = [float(v) for v in detection.bbox]
        if self.last_bbox[2] > self.last_bbox[0]:
            dt = max(1e-3, (now - self.last_seen_time).nanoseconds * 1e-9)
            old_center = self.bbox_center(self.last_bbox)
            new_center = self.bbox_center(bbox)
            measured_velocity = [
                (new_center[0] - old_center[0]) / dt,
                (new_center[1] - old_center[1]) / dt,
            ]
            self.velocity_px[0] = 0.7 * self.velocity_px[0] + 0.3 * measured_velocity[0]
            self.velocity_px[1] = 0.7 * self.velocity_px[1] + 0.3 * measured_velocity[1]
        self.last_bbox = bbox
        self.predicted_bbox = bbox
        self.last_seen_time = now
        self.last_update_time = now
        self.active_track_id = int(detection.track_id)

    def publish_debug(
            self,
            state: TargetState,
            matched_detection: Optional[HumanDetection2D],
            detections: List[HumanDetection2D]):
        now = self.get_clock().now()
        dropout = (now - self.last_seen_time).nanoseconds * 1e-9
        payload = {
            'locked': bool(self.locked),
            'logical_target_id': int(self.logical_target_id),
            'active_track_id': int(self.active_track_id),
            'latest_detection_count': len(detections),
            'latest_detection_track_ids': [int(d.track_id) for d in detections],
            'matched_track_id': int(matched_detection.track_id) if matched_detection else -1,
            'lock_state': int(state.lock_state),
            'camera_visible': bool(state.camera_visible),
            'confidence': round(float(state.confidence), 3),
            'dropout_sec': round(float(dropout), 3),
            'last_bbox': [round(float(v), 1) for v in self.last_bbox],
            'predicted_bbox': [round(float(v), 1) for v in self.predicted_bbox],
        }
        self.debug_pub.publish(String(data=json.dumps(payload)))

    @staticmethod
    def iou(a, b) -> float:
        x1 = max(float(a[0]), float(b[0]))
        y1 = max(float(a[1]), float(b[1]))
        x2 = min(float(a[2]), float(b[2]))
        y2 = min(float(a[3]), float(b[3]))
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area_a = max(0.0, float(a[2]) - float(a[0])) * max(0.0, float(a[3]) - float(a[1]))
        area_b = max(0.0, float(b[2]) - float(b[0])) * max(0.0, float(b[3]) - float(b[1]))
        denom = area_a + area_b - inter
        return inter / denom if denom > 0.0 else 0.0

    @staticmethod
    def bbox_center(bbox) -> List[float]:
        return [
            0.5 * (float(bbox[0]) + float(bbox[2])),
            0.5 * (float(bbox[1]) + float(bbox[3])),
        ]

    @classmethod
    def center_distance(cls, a, b) -> float:
        ac = cls.bbox_center(a)
        bc = cls.bbox_center(b)
        return math.hypot(ac[0] - bc[0], ac[1] - bc[1])

    @staticmethod
    def size_similarity(a, b) -> float:
        aw = max(1.0, float(a[2]) - float(a[0]))
        ah = max(1.0, float(a[3]) - float(a[1]))
        bw = max(1.0, float(b[2]) - float(b[0]))
        bh = max(1.0, float(b[3]) - float(b[1]))
        width_ratio = min(aw, bw) / max(aw, bw)
        height_ratio = min(ah, bh) / max(ah, bh)
        return 0.5 * (width_ratio + height_ratio)

    def image_callback(self, msg: Image):
        if self.overlay_pub is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert target overlay image: {exc}')
            return
        for detection in self.latest_detections:
            x1, y1, x2, y2 = [int(round(v)) for v in detection.bbox]
            is_target = int(detection.track_id) == self.active_track_id
            color = (0, 0, 255) if is_target else (100, 100, 100)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                image,
                f'track={detection.track_id}' + (
                    f' TARGET {self.logical_target_id}' if is_target else ''),
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2)
        if self.locked and not self.latest_state.camera_visible and self.predicted_bbox[2] > self.predicted_bbox[0]:
            x1, y1, x2, y2 = [int(round(v)) for v in self.predicted_bbox]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 180, 255), 2)
            cv2.putText(
                image,
                f'PRED TARGET {self.logical_target_id}',
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 180, 255),
                2)
        cv2.putText(
            image,
            f'lock={self.latest_state.lock_state} target={self.logical_target_id} track={self.active_track_id} visible={self.latest_state.camera_visible}',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2)
        out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        out.header = msg.header
        self.overlay_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetLockNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
