#!/usr/bin/env python3

import math
import json
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from track_robot_interfaces.msg import CameraTarget, GestureState, HumanDetection2D, HumanDetection2DArray


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
        self.identity_debug_topic = self.declare_parameter(
            'identity_debug_topic', '/human_tracking/camera_identity_debug').value
        self.reset_service = self.declare_parameter(
            'reset_service', '/human_tracking/reset_target').value
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
        self.identity_min_score = float(
            self.declare_parameter('identity_min_score', 0.65).value)
        self.identity_min_margin = float(
            self.declare_parameter('identity_min_margin', 0.15).value)
        self.identity_switch_confirm_frames = max(
            1, int(self.declare_parameter('identity_switch_confirm_frames', 2).value))
        self.identity_profile_update_confidence = float(
            self.declare_parameter('identity_profile_update_confidence', 0.75).value)
        self.image_sync_tolerance_sec = float(
            self.declare_parameter('image_sync_tolerance_sec', 0.08).value)
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
        self.lock_start_time = self.get_clock().now()
        self.last_unlock_reason = 'none'
        self.latest_detections: List[HumanDetection2D] = []
        self.latest_state = CameraTarget()
        self.locked = False
        self.latest_image = None
        self.latest_image_stamp_ns = 0
        self.current_measurement_time = self.get_clock().now()
        self.appearance_profile = None
        self.pose_profile = None
        self.pending_visual_track_id = -1
        self.pending_visual_count = 0
        self.last_identity_score = 0.0
        self.last_identity_margin = 0.0
        self.last_identity_state = CameraTarget.IDENTITY_NONE
        self.last_selected_keypoints = []

        self.target_pub = self.create_publisher(CameraTarget, self.output_topic, 5)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 5)
        self.identity_debug_pub = self.create_publisher(String, self.identity_debug_topic, 5)
        self.overlay_pub = None
        if self.publish_overlay:
            self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 5)
        self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.create_subscription(
            HumanDetection2DArray, self.detections_topic, self.detections_callback, 5)
        self.create_subscription(GestureState, self.gesture_topic, self.gesture_callback, 5)
        self.create_service(Trigger, self.reset_service, self.reset_target_callback)

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
                self.clear_target('stop_gesture')
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
            self.last_seen_time = self.current_measurement_time
            self.last_update_time = self.last_seen_time
            self.lock_start_time = self.last_seen_time
            self.last_unlock_reason = 'none'
            self.pending_visual_track_id = -1
            self.pending_visual_count = 0
            detection = next(
                (item for item in self.latest_detections
                 if int(item.track_id) == self.active_track_id), None)
            if detection is not None:
                self.initialize_identity_profile(detection)
                self.update_target_from_detection(detection)
            self.get_logger().info(
                f'Locked target logical_id={self.logical_target_id} '
                f'track_id={self.active_track_id}')

    def detections_callback(self, msg: HumanDetection2DArray):
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            measurement_time = Time.from_msg(msg.header.stamp)
            if (measurement_time - self.current_measurement_time).nanoseconds < -100000000:
                self.clear_target('timestamp_reset')
                self.velocity_px = [0.0, 0.0]
            self.current_measurement_time = measurement_time
        self.latest_detections = list(msg.detections)
        self.update_prediction()
        detection = self.find_target_detection(msg.detections)
        state = CameraTarget()
        state.header = msg.header
        state.logical_target_id = int(self.logical_target_id)
        state.visual_track_id = int(self.active_track_id)
        state.identity_state = int(self.last_identity_state)
        state.identity_confidence = float(self.last_identity_score)

        if not self.locked or self.logical_target_id < 0:
            state.lock_state = (
                CameraTarget.LOCK_CANDIDATE_VISIBLE
                if len(msg.detections) > 0 else CameraTarget.LOCK_NO_TARGET)
            state.identity_state = CameraTarget.IDENTITY_NONE
        elif detection is not None:
            self.update_target_from_detection(detection)
            self.last_confidence = float(detection.score)
            state.lock_state = CameraTarget.LOCK_TARGET_LOCKED
            state.bbox = self.last_bbox
            state.keypoints = list(detection.keypoints)
            state.detector_confidence = self.last_confidence
            state.identity_confidence = float(self.last_identity_score)
            state.identity_state = CameraTarget.IDENTITY_CONFIRMED
            state.visual_track_id = int(self.active_track_id)
            state.camera_visible = True
            self.update_identity_profile(detection)
        else:
            dropout = (self.current_measurement_time - self.last_seen_time).nanoseconds * 1e-9
            state.bbox = self.predicted_bbox
            state.keypoints = list(self.last_selected_keypoints)
            state.detector_confidence = max(
                0.0, self.last_confidence *
                (1.0 - dropout / max(self.max_camera_dropout_sec, 1e-3)))
            state.identity_confidence = float(self.last_identity_score)
            state.camera_visible = False
            state.lock_state = (
                CameraTarget.LOCK_TARGET_LOCKED
                if dropout <= self.max_camera_dropout_sec else CameraTarget.LOCK_TARGET_LOST)
            state.identity_state = (
                CameraTarget.IDENTITY_AMBIGUOUS
                if self.last_identity_state == CameraTarget.IDENTITY_AMBIGUOUS
                else CameraTarget.IDENTITY_OCCLUDED)
            if dropout > self.target_memory_sec:
                state.detector_confidence = 0.0

        self.latest_state = state
        self.target_pub.publish(state)
        self.publish_debug(state, detection, msg.detections)

    def reset_target_callback(self, _request, response):
        previous_target_id = self.logical_target_id
        self.clear_target('operator_reset')
        response.success = True
        response.message = f'Cleared logical target {previous_target_id}'
        return response

    def clear_target(self, reason: str):
        self.last_unlock_reason = reason
        self.logical_target_id = -1
        self.active_track_id = -1
        self.locked = False
        self.last_bbox = [0.0, 0.0, 0.0, 0.0]
        self.predicted_bbox = [0.0, 0.0, 0.0, 0.0]
        self.last_confidence = 0.0
        self.velocity_px = [0.0, 0.0]
        self.pending_visual_track_id = -1
        self.pending_visual_count = 0
        self.appearance_profile = None
        self.pose_profile = None
        self.last_selected_keypoints = []
        self.last_identity_score = 0.0
        self.last_identity_margin = 0.0
        self.last_identity_state = CameraTarget.IDENTITY_NONE
        state = CameraTarget()
        state.header.stamp = self.get_clock().now().to_msg()
        state.logical_target_id = -1
        state.visual_track_id = -1
        state.lock_state = CameraTarget.LOCK_NO_TARGET
        state.identity_state = CameraTarget.IDENTITY_NONE
        self.latest_state = state
        self.target_pub.publish(state)

    def find_target_detection(self, detections: List[HumanDetection2D]) -> Optional[HumanDetection2D]:
        self.last_identity_state = CameraTarget.IDENTITY_OCCLUDED
        if self.logical_target_id < 0:
            return None
        if self.last_bbox[2] <= self.last_bbox[0]:
            initial = next(
                (item for item in detections
                 if int(item.track_id) == self.active_track_id and
                 item.score >= self.min_lock_confidence), None)
            if initial is not None:
                self.initialize_identity_profile(initial)
            return initial
        dropout = (self.current_measurement_time - self.last_seen_time).nanoseconds * 1e-9
        if dropout > self.target_memory_sec:
            return None

        scored = []
        for detection in detections:
            if int(detection.track_id) < 0:
                continue
            if detection.score < self.min_lock_confidence:
                continue
            scored.append((self.identity_score(detection), detection))
        scored.sort(key=lambda item: item[0], reverse=True)
        if not scored:
            return None
        best_score, best = scored[0]
        second_score = scored[1][0] if len(scored) > 1 else 0.0
        margin = best_score - second_score
        self.last_identity_score = float(best_score)
        self.last_identity_margin = float(margin)
        if best_score < self.identity_min_score or margin < self.identity_min_margin:
            self.last_identity_state = CameraTarget.IDENTITY_AMBIGUOUS
            self.pending_visual_track_id = -1
            self.pending_visual_count = 0
            return None
        best_id = int(best.track_id)
        if best_id != self.active_track_id:
            if self.pending_visual_track_id == best_id:
                self.pending_visual_count += 1
            else:
                self.pending_visual_track_id = best_id
                self.pending_visual_count = 1
            if self.pending_visual_count < self.identity_switch_confirm_frames:
                self.last_identity_state = CameraTarget.IDENTITY_AMBIGUOUS
                return None
            old_track_id = self.active_track_id
            self.active_track_id = best_id
            self.get_logger().info(
                f'Reacquired logical_id={self.logical_target_id}: '
                f'track_id {old_track_id} -> {best_id} score={best_score:.2f} '
                f'margin={margin:.2f}')
        self.pending_visual_track_id = -1
        self.pending_visual_count = 0
        self.last_identity_state = CameraTarget.IDENTITY_CONFIRMED
        return best
        return None

    def update_prediction(self):
        if self.last_bbox[2] <= self.last_bbox[0]:
            return
        now = self.current_measurement_time
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
        now = self.current_measurement_time
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
        self.last_selected_keypoints = list(detection.keypoints)

    def identity_score(self, detection: HumanDetection2D) -> float:
        overlap = self.iou(self.predicted_bbox, detection.bbox)
        center_distance = self.center_distance(self.predicted_bbox, detection.bbox)
        distance_score = max(
            0.0, 1.0 - center_distance /
            max(self.reacquire_max_center_distance_px, 1.0))
        motion_score = 0.65 * overlap + 0.35 * distance_score
        appearance = self.appearance_similarity(detection)
        pose = self.pose_similarity(detection)
        size = self.size_similarity(self.predicted_bbox, detection.bbox)
        return 0.35 * appearance + 0.30 * motion_score + 0.20 * pose + 0.15 * size

    def initialize_identity_profile(self, detection: HumanDetection2D):
        self.appearance_profile = self.appearance_histogram(detection)
        self.pose_profile = self.pose_signature(detection)
        self.last_selected_keypoints = list(detection.keypoints)
        self.last_identity_score = 1.0
        self.last_identity_margin = 1.0
        self.last_identity_state = CameraTarget.IDENTITY_CONFIRMED

    def update_identity_profile(self, detection: HumanDetection2D):
        if (detection.score < self.identity_profile_update_confidence or
                self.last_identity_state != CameraTarget.IDENTITY_CONFIRMED or
                self.last_identity_margin < self.identity_min_margin):
            return
        histogram = self.appearance_histogram(detection)
        signature = self.pose_signature(detection)
        if histogram is not None:
            self.appearance_profile = histogram if self.appearance_profile is None else (
                0.9 * self.appearance_profile + 0.1 * histogram)
            self.appearance_profile /= max(float(self.appearance_profile.sum()), 1e-6)
        if signature is not None:
            self.pose_profile = signature if self.pose_profile is None else (
                0.9 * self.pose_profile + 0.1 * signature)

    def appearance_similarity(self, detection: HumanDetection2D) -> float:
        histogram = self.appearance_histogram(detection)
        if histogram is None or self.appearance_profile is None:
            return 0.70
        distance = cv2.compareHist(
            self.appearance_profile.astype(np.float32), histogram.astype(np.float32),
            cv2.HISTCMP_BHATTACHARYYA)
        return max(0.0, min(1.0, 1.0 - float(distance)))

    def appearance_histogram(self, detection: HumanDetection2D):
        if self.latest_image is None:
            return None
        detection_stamp_ns = (
            int(detection.header.stamp.sec) * 1000000000 +
            int(detection.header.stamp.nanosec))
        if (detection_stamp_ns and self.latest_image_stamp_ns and
                abs(detection_stamp_ns - self.latest_image_stamp_ns) * 1e-9 >
                self.image_sync_tolerance_sec):
            return None
        x1, y1, x2, y2 = self.torso_roi(detection)
        height, width = self.latest_image.shape[:2]
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None
        hsv = cv2.cvtColor(self.latest_image[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        histogram = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
        histogram = histogram.astype(np.float32)
        histogram /= max(float(histogram.sum()), 1e-6)
        return histogram

    def torso_roi(self, detection: HumanDetection2D):
        bbox = [float(value) for value in detection.bbox]
        points = self.decode_keypoints(detection.keypoints)
        if points is not None and all(
                index < len(points) and points[index][2] >= 0.35
                for index in (5, 6, 11, 12)):
            xs = [points[index][0] for index in (5, 6, 11, 12)]
            ys = [points[index][1] for index in (5, 6, 11, 12)]
            margin_x = 0.12 * max(1.0, bbox[2] - bbox[0])
            margin_y = 0.08 * max(1.0, bbox[3] - bbox[1])
            return (int(min(xs) - margin_x), int(min(ys) - margin_y),
                    int(max(xs) + margin_x), int(max(ys) + margin_y))
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        return (int(bbox[0] + 0.25 * width), int(bbox[1] + 0.20 * height),
                int(bbox[0] + 0.75 * width), int(bbox[1] + 0.70 * height))

    @staticmethod
    def decode_keypoints(flat):
        if len(flat) < 39:
            return None
        return [(float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
                for i in range(0, len(flat) - 2, 3)]

    def pose_signature(self, detection: HumanDetection2D):
        points = self.decode_keypoints(detection.keypoints)
        if points is None or not all(points[index][2] >= 0.35 for index in (5, 6, 11, 12)):
            return None
        shoulder = max(1.0, math.hypot(
            points[5][0] - points[6][0], points[5][1] - points[6][1]))
        hip = math.hypot(points[11][0] - points[12][0], points[11][1] - points[12][1])
        shoulder_mid = ((points[5][0] + points[6][0]) * 0.5,
                        (points[5][1] + points[6][1]) * 0.5)
        hip_mid = ((points[11][0] + points[12][0]) * 0.5,
                   (points[11][1] + points[12][1]) * 0.5)
        torso = math.hypot(shoulder_mid[0] - hip_mid[0], shoulder_mid[1] - hip_mid[1])
        return np.asarray([hip / shoulder, torso / shoulder], dtype=np.float32)

    def pose_similarity(self, detection: HumanDetection2D) -> float:
        signature = self.pose_signature(detection)
        if signature is None or self.pose_profile is None:
            return 0.70
        return max(0.0, min(1.0, 1.0 - float(np.linalg.norm(signature - self.pose_profile)) / 1.5))

    def publish_debug(
            self,
            state: CameraTarget,
            matched_detection: Optional[HumanDetection2D],
            detections: List[HumanDetection2D]):
        now = self.current_measurement_time
        dropout = (now - self.last_seen_time).nanoseconds * 1e-9
        payload = {
            'locked': bool(self.locked),
            'identity_persistent': bool(self.locked and self.logical_target_id >= 0),
            'logical_target_id': int(self.logical_target_id),
            'active_track_id': int(self.active_track_id),
            'latest_detection_count': len(detections),
            'latest_detection_track_ids': [int(d.track_id) for d in detections],
            'matched_track_id': int(matched_detection.track_id) if matched_detection else -1,
            'lock_state': int(state.lock_state),
            'camera_visible': bool(state.camera_visible),
            'detector_confidence': round(float(state.detector_confidence), 3),
            'identity_confidence': round(float(state.identity_confidence), 3),
            'identity_state': int(state.identity_state),
            'identity_margin': round(float(self.last_identity_margin), 3),
            'pending_visual_track_id': int(self.pending_visual_track_id),
            'pending_visual_count': int(self.pending_visual_count),
            'dropout_sec': round(float(dropout), 3),
            'bbox_reacquire_expired': bool(dropout > self.target_memory_sec),
            'lock_age_sec': round(float(
                (now - self.lock_start_time).nanoseconds * 1e-9), 3),
            'unlock_reason': self.last_unlock_reason,
            'last_bbox': [round(float(v), 1) for v in self.last_bbox],
            'predicted_bbox': [round(float(v), 1) for v in self.predicted_bbox],
        }
        message = String(data=json.dumps(payload))
        self.debug_pub.publish(message)
        self.identity_debug_pub.publish(message)

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
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert target overlay image: {exc}')
            return
        self.latest_image = image.copy()
        self.latest_image_stamp_ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        if self.overlay_pub is None:
            return
        for detection in self.latest_detections:
            x1, y1, x2, y2 = [int(round(v)) for v in detection.bbox]
            is_target = int(detection.track_id) == self.active_track_id
            if is_target and self.latest_state.identity_state == CameraTarget.IDENTITY_AMBIGUOUS:
                color = (0, 165, 255)
            else:
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
        identity_names = {
            CameraTarget.IDENTITY_NONE: 'NONE',
            CameraTarget.IDENTITY_CONFIRMED: 'CONFIRMED',
            CameraTarget.IDENTITY_AMBIGUOUS: 'AMBIGUOUS',
            CameraTarget.IDENTITY_OCCLUDED: 'OCCLUDED',
        }
        cv2.putText(
            image,
            f'identity={identity_names.get(self.latest_state.identity_state, "UNKNOWN")} '
            f'score={self.latest_state.identity_confidence:.2f} '
            f'margin={self.last_identity_margin:.2f}',
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 165, 255) if self.latest_state.identity_state == CameraTarget.IDENTITY_AMBIGUOUS
            else (0, 255, 255),
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
