#!/usr/bin/env python3

from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from track_robot_interfaces.msg import GestureState, HumanDetection2D, HumanDetection2DArray


LSHOULDER = 5
RSHOULDER = 6
LELBOW = 7
RELBOW = 8
LWRIST = 9
RWRIST = 10
LHIP = 11
RHIP = 12


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class GestureTriggerNode(Node):
    def __init__(self):
        super().__init__('gesture_trigger_node')

        self.detections_topic = self.declare_parameter(
            'detections_topic', '/human_tracking/detections').value
        self.image_topic = self.declare_parameter(
            'image_topic', '/zed/zed_node/left/image_rect_color').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/human_tracking/gesture_state').value
        self.overlay_topic = self.declare_parameter(
            'overlay_topic', '/human_tracking/gesture_overlay').value
        self.min_keypoint_confidence = float(
            self.declare_parameter('min_keypoint_confidence', 0.35).value)
        self.trigger_hold_frames = max(
            1, int(self.declare_parameter('trigger_hold_frames', 5).value))
        self.trigger_cooldown_sec = float(
            self.declare_parameter('trigger_cooldown_sec', 2.0).value)
        self.stop_hold_frames = max(
            1, int(self.declare_parameter('stop_hold_frames', 15).value))
        self.start_requires_wrist_above_head = parse_bool(
            self.declare_parameter('start_requires_wrist_above_head', False).value)
        self.start_window_frames = max(
            4, int(self.declare_parameter('start_window_frames', 12).value))
        self.start_min_both_high_frames = max(
            2, int(self.declare_parameter('start_min_both_high_frames', 6).value))
        self.start_min_cross_frames = max(
            1, int(self.declare_parameter('start_min_cross_frames', 1).value))
        self.start_min_order_changes = max(
            0, int(self.declare_parameter('start_min_order_changes', 1).value))
        self.start_min_lateral_motion_shoulder_width = float(
            self.declare_parameter('start_min_lateral_motion_shoulder_width', 0.35).value)
        self.start_max_one_hand_high_frames = max(
            0, int(self.declare_parameter('start_max_one_hand_high_frames', 3).value))
        self.stop_visualization_hold_sec = float(
            self.declare_parameter('stop_visualization_hold_sec', 1.0).value)
        self.publish_overlay = parse_bool(
            self.declare_parameter('publish_overlay', True).value)

        self.bridge = CvBridge()
        self.latest_detections: List[HumanDetection2D] = []
        self.last_trigger_time = self.get_clock().now()
        self.track_history: Dict[int, deque] = defaultdict(
            lambda: deque(maxlen=self.start_window_frames))
        self.start_hold_counts: Dict[int, int] = defaultdict(int)
        self.stop_hold_counts: Dict[int, int] = defaultdict(int)
        self.latest_track_commands: Dict[int, Tuple[str, str, float]] = {}
        self.fsm_state = 'WAITING_FOR_START'
        self.active_target_id = -1
        self.last_stop_target_id = -1
        self.last_stop_until = self.get_clock().now()
        self.latest_state = GestureState()

        self.state_pub = self.create_publisher(GestureState, self.output_topic, 5)
        self.overlay_pub = None
        if self.publish_overlay:
            self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 5)
            self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.create_subscription(
            HumanDetection2DArray, self.detections_topic, self.detections_callback, 5)

        self.get_logger().info(
            f'gesture_trigger_node listening to {self.detections_topic}; '
            f'publishing {self.output_topic}')

    def detections_callback(self, msg: HumanDetection2DArray):
        self.latest_detections = list(msg.detections)
        self.latest_track_commands = {}
        best_state = GestureState()
        best_state.header = msg.header
        best_state.track_id = -1
        best_state.gesture_name = 'none'
        best_state.command = 'none'
        best_state.confidence = 0.0
        best_state.candidate_visible = len(msg.detections) > 0

        for detection in msg.detections:
            track_id = int(detection.track_id)
            if track_id < 0:
                continue
            allow_start = self.fsm_state == 'WAITING_FOR_START'
            allow_stop = self.fsm_state == 'TRACKING' and self.active_target_id == track_id
            active, name, command, confidence = self.evaluate_detection(
                detection,
                allow_start=allow_start,
                allow_stop=allow_stop)
            self.latest_track_commands[track_id] = (name, command, confidence)
            if confidence > best_state.confidence:
                best_state.track_id = track_id
                best_state.gesture_name = name
                best_state.command = command
                best_state.confidence = float(confidence)
            if active:
                best_state.track_id = track_id
                best_state.gesture_name = name
                best_state.command = command
                best_state.confidence = float(confidence)
                best_state.trigger_active = True
                if command == 'start_tracking':
                    self.fsm_state = 'TRACKING'
                    self.active_target_id = track_id
                    self.last_stop_target_id = -1
                elif command == 'stop_tracking' and track_id == self.active_target_id:
                    self.fsm_state = 'WAITING_FOR_START'
                    self.last_stop_target_id = track_id
                    self.last_stop_until = self.get_clock().now() + Duration(
                        seconds=self.stop_visualization_hold_sec)
                    self.active_target_id = -1
                break

        self.latest_state = best_state
        self.state_pub.publish(best_state)

    def evaluate_detection(
            self,
            detection: HumanDetection2D,
            allow_start: bool,
            allow_stop: bool) -> Tuple[bool, str, str, float]:
        keypoints = self.decode_keypoints(detection.keypoints)
        track_id = int(detection.track_id)
        if keypoints is None:
            self.start_hold_counts[track_id] = 0
            self.stop_hold_counts[track_id] = 0
            return False, 'no_pose', 'none', 0.0

        features = self.extract_pose_features(keypoints)
        if features is None:
            self.start_hold_counts[track_id] = 0
            self.stop_hold_counts[track_id] = 0
            return False, 'low_pose_confidence', 'none', 0.0

        self.track_history[track_id].append(features)
        start_gesture, start_name, raw_start_confidence = self.start_wave_gesture(track_id)
        start_gesture = allow_start and start_gesture
        stop_one_hand_high = allow_stop and bool(features['one_hand_high'])

        if start_gesture:
            self.start_hold_counts[track_id] += 1
        else:
            self.start_hold_counts[track_id] = max(0, self.start_hold_counts[track_id] - 1)
        if stop_one_hand_high:
            self.stop_hold_counts[track_id] += 1
        else:
            self.stop_hold_counts[track_id] = max(0, self.stop_hold_counts[track_id] - 1)

        cooldown_ok = (
            self.get_clock().now() - self.last_trigger_time).nanoseconds * 1e-9 >= (
                self.trigger_cooldown_sec)

        stop_confidence = min(
            1.0, float(self.stop_hold_counts[track_id]) / float(self.stop_hold_frames))
        if self.stop_hold_counts[track_id] >= self.stop_hold_frames and cooldown_ok:
            self.last_trigger_time = self.get_clock().now()
            return True, 'one_hand_high', 'stop_tracking', stop_confidence

        start_hold_confidence = min(
            1.0, float(self.start_hold_counts[track_id]) / float(self.trigger_hold_frames))
        start_confidence = min(start_hold_confidence, raw_start_confidence)
        if self.start_hold_counts[track_id] >= self.trigger_hold_frames and cooldown_ok:
            self.last_trigger_time = self.get_clock().now()
            return True, start_name, 'start_tracking', start_confidence

        if stop_one_hand_high:
            return False, 'one_hand_high', 'stop_tracking', stop_confidence
        if start_gesture:
            return False, start_name, 'start_tracking', start_confidence
        return False, 'none', 'none', 0.0

    def decode_keypoints(self, flat: List[float]) -> Optional[List[Tuple[float, float, float]]]:
        if len(flat) < 33:
            return None
        points = []
        for offset in range(0, len(flat) - 2, 3):
            points.append((float(flat[offset]), float(flat[offset + 1]), float(flat[offset + 2])))
        return points

    def valid_point(self, keypoints, index: int) -> bool:
        return index < len(keypoints) and keypoints[index][2] >= self.min_keypoint_confidence

    def extract_pose_features(self, keypoints) -> Optional[Dict[str, float]]:
        needed = [LSHOULDER, RSHOULDER, LWRIST, RWRIST]
        if not all(self.valid_point(keypoints, index) for index in needed):
            return None

        jaw_y = self.jaw_line_y(keypoints)
        if self.start_requires_wrist_above_head and self.head_line_y(keypoints) is not None:
            start_y = self.head_line_y(keypoints)
        else:
            start_y = jaw_y

        left_wrist_x = keypoints[LWRIST][0]
        right_wrist_x = keypoints[RWRIST][0]
        left_high = keypoints[LWRIST][1] < start_y
        right_high = keypoints[RWRIST][1] < start_y
        shoulder_width = max(
            30.0,
            abs(keypoints[RSHOULDER][0] - keypoints[LSHOULDER][0]))
        crossed = self.wrists_crossed(keypoints)

        return {
            'left_wrist_x': left_wrist_x,
            'right_wrist_x': right_wrist_x,
            'both_high': left_high and right_high,
            'one_hand_high': left_high != right_high,
            'crossed': crossed,
            'wrist_order': 1.0 if left_wrist_x > right_wrist_x else -1.0,
            'shoulder_width': shoulder_width,
        }

    def start_wave_gesture(self, track_id: int) -> Tuple[bool, str, float]:
        history = list(self.track_history[track_id])
        if len(history) < self.start_min_both_high_frames:
            return False, 'none', 0.0

        high_frames = [item for item in history if item['both_high']]
        high_count = len(high_frames)
        one_hand_count = sum(1 for item in history if item['one_hand_high'])
        if high_count < self.start_min_both_high_frames:
            confidence = high_count / float(self.start_min_both_high_frames)
            return False, 'both_hands_high_pending', min(0.5, confidence)
        if one_hand_count > self.start_max_one_hand_high_frames:
            return False, 'one_hand_high_not_start', 0.0

        shoulder_width = max(30.0, high_frames[-1]['shoulder_width'])
        left_range = (
            max(item['left_wrist_x'] for item in high_frames) -
            min(item['left_wrist_x'] for item in high_frames))
        right_range = (
            max(item['right_wrist_x'] for item in high_frames) -
            min(item['right_wrist_x'] for item in high_frames))
        lateral_motion = max(left_range, right_range) / shoulder_width
        cross_count = sum(1 for item in high_frames if item['crossed'])
        order_changes = 0
        previous_order = high_frames[0]['wrist_order']
        for item in high_frames[1:]:
            current_order = item['wrist_order']
            if current_order != previous_order:
                order_changes += 1
            previous_order = current_order

        crossed_motion = cross_count >= self.start_min_cross_frames
        swapped_motion = order_changes >= self.start_min_order_changes
        lateral_motion_ok = lateral_motion >= self.start_min_lateral_motion_shoulder_width
        active_wave = (
            (crossed_motion and (swapped_motion or lateral_motion_ok)) or
            (swapped_motion and lateral_motion_ok))
        confidence_parts = [
            min(1.0, high_count / float(self.start_min_both_high_frames)),
            min(1.0, lateral_motion / max(0.01, self.start_min_lateral_motion_shoulder_width)),
        ]
        if self.start_min_order_changes > 0:
            confidence_parts.append(min(1.0, order_changes / float(self.start_min_order_changes)))
        if self.start_min_cross_frames > 0:
            confidence_parts.append(min(1.0, cross_count / float(self.start_min_cross_frames)))
        confidence = sum(confidence_parts) / float(len(confidence_parts))

        if not active_wave:
            return False, 'both_hands_high_pending', min(0.8, confidence)
        if crossed_motion:
            return True, 'two_hand_cross_wave', confidence
        return True, 'two_hand_wave', confidence

    def arms_high_above_head(self, keypoints) -> bool:
        head_y = self.head_line_y(keypoints)
        if head_y is None:
            return False
        high_count = 0
        for wrist_index in (LWRIST, RWRIST):
            if self.valid_point(keypoints, wrist_index) and keypoints[wrist_index][1] < head_y:
                high_count += 1
        return high_count > 0

    def exactly_one_wrist_above_jaw_line(self, keypoints) -> bool:
        needed = [LSHOULDER, RSHOULDER, LWRIST, RWRIST]
        if not all(self.valid_point(keypoints, index) for index in needed):
            return False
        line_y = self.jaw_line_y(keypoints)
        left_high = keypoints[LWRIST][1] < line_y
        right_high = keypoints[RWRIST][1] < line_y
        return left_high != right_high

    def jaw_line_y(self, keypoints) -> float:
        face_indices = [0, 1, 2, 3, 4]
        visible = [
            keypoints[index][1]
            for index in face_indices
            if self.valid_point(keypoints, index)
        ]
        if visible:
            return max(visible) + 15.0
        shoulder_y = min(keypoints[LSHOULDER][1], keypoints[RSHOULDER][1])
        return shoulder_y - 30.0

    def head_line_y(self, keypoints) -> Optional[float]:
        face_indices = [0, 1, 2, 3, 4]
        visible = [
            keypoints[index][1]
            for index in face_indices
            if self.valid_point(keypoints, index)
        ]
        if visible:
            return min(visible)
        if self.valid_point(keypoints, LSHOULDER) and self.valid_point(keypoints, RSHOULDER):
            shoulder_y = min(keypoints[LSHOULDER][1], keypoints[RSHOULDER][1])
            if self.valid_point(keypoints, LHIP) and self.valid_point(keypoints, RHIP):
                hip_y = max(keypoints[LHIP][1], keypoints[RHIP][1])
                torso_height = max(40.0, hip_y - shoulder_y)
                return shoulder_y - 0.45 * torso_height
            return shoulder_y - 70.0
        return None

    def wrists_crossed(self, keypoints) -> bool:
        needed = [LSHOULDER, RSHOULDER, LWRIST, RWRIST]
        if not all(self.valid_point(keypoints, index) for index in needed):
            return False
        left_shoulder_x = keypoints[LSHOULDER][0]
        right_shoulder_x = keypoints[RSHOULDER][0]
        left_wrist_x = keypoints[LWRIST][0]
        right_wrist_x = keypoints[RWRIST][0]
        shoulder_min = min(left_shoulder_x, right_shoulder_x)
        shoulder_max = max(left_shoulder_x, right_shoulder_x)
        wrists_swapped = left_wrist_x > right_wrist_x
        wrists_near_torso = shoulder_min <= left_wrist_x <= shoulder_max and shoulder_min <= right_wrist_x <= shoulder_max
        return wrists_swapped and wrists_near_torso

    def image_callback(self, msg: Image):
        if self.overlay_pub is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert overlay image: {exc}')
            return
        for detection in self.latest_detections:
            x1, y1, x2, y2 = [int(round(v)) for v in detection.bbox]
            track_id = int(detection.track_id)
            gesture_name, command, confidence = self.latest_track_commands.get(
                track_id, ('none', 'none', 0.0))
            stop_hold_active = (
                track_id == self.last_stop_target_id and
                self.get_clock().now() < self.last_stop_until)
            if (command == 'stop_tracking' and confidence > 0.0) or stop_hold_active:
                color = (0, 0, 255)
                label = f'id={detection.track_id} STOP {confidence:.2f}'
            elif command == 'start_tracking' and confidence > 0.0:
                color = (0, 255, 0)
                label = f'id={detection.track_id} START {confidence:.2f}'
            else:
                color = (180, 180, 180)
                label = f'id={detection.track_id}'
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                image,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2)
        cv2.putText(
            image,
            f'state={self.fsm_state} active_target={self.active_target_id} gesture={self.latest_state.gesture_name} cmd={self.latest_state.command} trigger={self.latest_state.trigger_active}',
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
    node = GestureTriggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
