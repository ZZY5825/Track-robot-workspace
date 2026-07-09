#!/usr/bin/env python3

import argparse
import json
from dataclasses import dataclass, field
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from track_robot_interfaces.msg import (
    GestureState,
    HumanDetection2DArray,
    LidarClusterArray,
    LidarTrackletArray,
    TargetState,
)
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class MarkerStats:
    messages: int = 0
    add_markers: int = 0
    delete_markers: int = 0
    max_add_markers: int = 0
    last_add_markers: int = 0
    last_frame: str = ''
    namespaces: Dict[str, int] = field(default_factory=dict)


@dataclass
class CloudStats:
    messages: int = 0
    max_width: int = 0
    last_width: int = 0
    last_frame: str = ''


class HumanTrackingPipelineDiagnostic(Node):
    def __init__(self, duration_sec: float):
        super().__init__('human_tracking_pipeline_diagnostic')
        self.duration_sec = max(1.0, float(duration_sec))
        self.start_time = self.get_clock().now()

        self.marker_stats: Dict[str, MarkerStats] = {}
        self.cloud_stats: Dict[str, CloudStats] = {}
        self.last_camera_target: Optional[TargetState] = None
        self.last_fused_target: Optional[TargetState] = None
        self.last_target_tracker_debug = {}
        self.last_camera_target_debug = {}
        self.last_lidar_debug = {}
        self.detection_messages = 0
        self.detection_count = 0
        self.max_detection_count = 0
        self.last_detection_track_ids = []
        self.max_detection_track_ids = []
        self.last_gesture: Optional[GestureState] = None
        self.tracker_debug = {}
        self.tracklet_count = 0
        self.candidate_count = 0
        self.oversized_candidate_count = 0
        self.human_candidate_count = 0

        self.create_subscription(
            TargetState, '/human_tracking/camera_target',
            self.camera_target_callback, 10)
        self.create_subscription(
            TargetState, '/human_tracking/fused_target_state',
            self.fused_target_callback, 10)
        self.create_subscription(
            HumanDetection2DArray, '/human_tracking/detections',
            self.detections_callback, 10)
        self.create_subscription(
            GestureState, '/human_tracking/gesture_state',
            self.gesture_callback, 10)
        self.create_subscription(
            LidarTrackletArray, '/human_tracking/lidar_tracklets',
            self.tracklets_callback, 10)
        self.create_subscription(
            LidarClusterArray, '/human_tracking/lidar_candidate_clusters',
            self.candidates_callback, 10)
        self.create_subscription(
            String, '/human_tracking/target_tracker_debug',
            self.target_tracker_debug_callback, 10)
        self.create_subscription(
            String, '/human_tracking/camera_target_debug',
            self.camera_target_debug_callback, 10)
        self.create_subscription(
            String, '/human_tracking/lidar_tracklet_debug',
            self.lidar_debug_callback, 10)
        self.create_subscription(
            String, '/human_tracking/tracker_debug',
            self.tracker_debug_callback, 10)

        for topic in (
            '/human_tracking/selected_tracklet_marker',
            '/human_tracking/selected_target_marker',
            '/human_tracking/target_prediction_gate_marker',
            '/human_tracking/fused_target_marker',
            '/human_tracking/lidar_candidate_cluster_markers',
            '/human_tracking/lidar_tracklet_markers',
        ):
            self.marker_stats[topic] = MarkerStats()
            self.create_subscription(
                MarkerArray,
                topic,
                lambda msg, topic=topic: self.marker_callback(topic, msg),
                10)

        for topic in (
            '/human_tracking/camera_guided_target_points',
            '/rslidar_points',
        ):
            self.cloud_stats[topic] = CloudStats()
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, topic=topic: self.cloud_callback(topic, msg),
                10)

        self.timer = self.create_timer(0.25, self.timer_callback)
        self.get_logger().info(
            f'Collecting human tracking diagnostics for {self.duration_sec:.1f}s')

    def camera_target_callback(self, msg: TargetState):
        self.last_camera_target = msg

    def fused_target_callback(self, msg: TargetState):
        self.last_fused_target = msg

    def detections_callback(self, msg: HumanDetection2DArray):
        self.detection_messages += 1
        self.detection_count = len(msg.detections)
        self.last_detection_track_ids = [int(d.track_id) for d in msg.detections]
        if self.detection_count >= self.max_detection_count:
            self.max_detection_count = self.detection_count
            self.max_detection_track_ids = list(self.last_detection_track_ids)

    def gesture_callback(self, msg: GestureState):
        self.last_gesture = msg

    def tracklets_callback(self, msg: LidarTrackletArray):
        self.tracklet_count = len(msg.tracklets)

    def candidates_callback(self, msg: LidarClusterArray):
        self.candidate_count = len(msg.clusters)
        self.oversized_candidate_count = sum(1 for c in msg.clusters if c.oversized or c.too_large)
        self.human_candidate_count = sum(1 for c in msg.clusters if c.human_candidate)

    def target_tracker_debug_callback(self, msg: String):
        self.last_target_tracker_debug = self.parse_json(msg.data)

    def camera_target_debug_callback(self, msg: String):
        self.last_camera_target_debug = self.parse_json(msg.data)

    def lidar_debug_callback(self, msg: String):
        self.last_lidar_debug = self.parse_json(msg.data)

    def tracker_debug_callback(self, msg: String):
        self.tracker_debug = self.parse_json(msg.data)

    def marker_callback(self, topic: str, msg: MarkerArray):
        stats = self.marker_stats[topic]
        stats.messages += 1
        add_count = 0
        delete_count = 0
        namespaces = {}
        frame = ''
        for marker in msg.markers:
            if marker.action == Marker.ADD:
                add_count += 1
                namespaces[marker.ns] = namespaces.get(marker.ns, 0) + 1
                frame = marker.header.frame_id or frame
            elif marker.action in (Marker.DELETE, Marker.DELETEALL):
                delete_count += 1
                frame = marker.header.frame_id or frame
        stats.add_markers += add_count
        stats.delete_markers += delete_count
        stats.last_add_markers = add_count
        stats.max_add_markers = max(stats.max_add_markers, add_count)
        stats.last_frame = frame or stats.last_frame
        stats.namespaces = namespaces or stats.namespaces

    def cloud_callback(self, topic: str, msg: PointCloud2):
        stats = self.cloud_stats[topic]
        stats.messages += 1
        stats.last_width = int(msg.width)
        stats.max_width = max(stats.max_width, int(msg.width))
        stats.last_frame = msg.header.frame_id

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= self.duration_sec:
            self.print_report()
            rclpy.shutdown()

    @staticmethod
    def parse_json(data: str):
        try:
            return json.loads(data)
        except Exception:
            return {'raw': data}

    @staticmethod
    def lock_name(value: int) -> str:
        names = {
            TargetState.LOCK_NO_TARGET: 'NO_TARGET',
            TargetState.LOCK_CANDIDATE_VISIBLE: 'CANDIDATE_VISIBLE',
            TargetState.LOCK_TARGET_LOCKED: 'TARGET_LOCKED',
            TargetState.LOCK_TARGET_LOST: 'TARGET_LOST',
        }
        return names.get(int(value), str(int(value)))

    @staticmethod
    def source_name(value: int) -> str:
        names = {
            TargetState.SOURCE_NONE: 'NONE',
            TargetState.SOURCE_CAMERA_ONLY: 'CAMERA_ONLY',
            TargetState.SOURCE_CAMERA_LIDAR: 'CAMERA_LIDAR',
            TargetState.SOURCE_LIDAR_ONLY: 'LIDAR_ONLY',
            TargetState.SOURCE_PREDICTION_ONLY: 'PREDICTION_ONLY',
        }
        return names.get(int(value), str(int(value)))

    @staticmethod
    def track_name(value: int) -> str:
        names = {
            TargetState.TRACK_NO_TARGET: 'NO_TARGET',
            TargetState.TRACK_CAMERA_LOCKED: 'CAMERA_LOCKED',
            TargetState.TRACK_CAMERA_LIDAR_TRACKED: 'CAMERA_LIDAR_TRACKED',
            TargetState.TRACK_LIDAR_ONLY_TRACKING: 'LIDAR_ONLY_TRACKING',
            TargetState.TRACK_PREDICTION_ONLY: 'PREDICTION_ONLY',
            TargetState.TRACK_TARGET_LOST: 'TARGET_LOST',
        }
        return names.get(int(value), str(int(value)))

    def print_report(self):
        print('\n=== Human Tracking Pipeline Diagnostic ===')
        print('Inputs:')
        print(
            f'  detections: last={self.detection_count} max={self.max_detection_count} '
            f'messages={self.detection_messages} '
            f'last_track_ids={self.last_detection_track_ids} '
            f'max_track_ids={self.max_detection_track_ids}')
        if self.last_gesture is None:
            print('  gesture_state: MISSING')
        else:
            gesture = self.last_gesture
            print(
                '  gesture_state: '
                f'track_id={gesture.track_id} command={gesture.command} '
                f'active={gesture.trigger_active} conf={gesture.confidence:.2f}')
        print(f'  lidar_tracklets: {self.tracklet_count}')
        print(
            f'  lidar_candidate_clusters: {self.candidate_count} '
            f'(human_candidate={self.human_candidate_count}, '
            f'oversized_or_too_large={self.oversized_candidate_count})')

        if self.last_camera_target is None:
            print('  camera_target: MISSING')
        else:
            target = self.last_camera_target
            print(
                '  camera_target: '
                f'id={target.target_id} lock={self.lock_name(target.lock_state)} '
                f'visible={target.camera_visible} conf={target.confidence:.2f} '
                f'bbox={[round(float(v), 1) for v in target.bbox]}')

        if self.last_fused_target is None:
            print('  fused_target_state: MISSING')
        else:
            target = self.last_fused_target
            print(
                '  fused_target_state: '
                f'id={target.target_id} track={self.track_name(target.track_state)} '
                f'source={self.source_name(target.source_state)} '
                f'conf={target.confidence:.2f} '
                f'pos=({target.position_base.x:.2f}, {target.position_base.y:.2f}, '
                f'{target.position_base.z:.2f})')

        print('\nPointCloud topics:')
        for topic, stats in self.cloud_stats.items():
            print(
                f'  {topic}: messages={stats.messages} '
                f'last_width={stats.last_width} max_width={stats.max_width} '
                f'frame={stats.last_frame or "unknown"}')

        print('\nMarker topics:')
        for topic, stats in self.marker_stats.items():
            status = 'ACTIVE' if stats.max_add_markers > 0 else 'EMPTY_OR_DELETEONLY'
            print(
                f'  {topic}: {status} messages={stats.messages} '
                f'last_add={stats.last_add_markers} max_add={stats.max_add_markers} '
                f'frame={stats.last_frame or "unknown"} ns={stats.namespaces}')

        print('\nSelected target tracker debug:')
        print(json.dumps(self.last_target_tracker_debug, indent=2, sort_keys=True))
        print('\nCamera target-lock debug:')
        print(json.dumps(self.last_camera_target_debug, indent=2, sort_keys=True))
        print('\nTracker debug:')
        print(json.dumps(self.tracker_debug, indent=2, sort_keys=True))
        print('\nLiDAR tracklet debug:')
        print(json.dumps(self.last_lidar_debug, indent=2, sort_keys=True))

        print('\nPublishers:')
        for topic in (
            '/human_tracking/detections',
            '/human_tracking/gesture_state',
            '/human_tracking/camera_target',
            '/human_tracking/camera_target_debug',
            '/human_tracking/fused_target_state',
        ):
            publishers = self.get_publishers_info_by_topic(topic)
            names = [
                f'{info.node_namespace.rstrip("/")}/{info.node_name}'.replace('//', '/')
                for info in publishers
            ]
            print(f'  {topic}: {len(publishers)} publishers {names}')

        print('\nLikely blockers:')
        blockers = self.blockers()
        if blockers:
            for blocker in blockers:
                print(f'  - {blocker}')
        else:
            print('  - No obvious topic-level blocker found. If RViz is blank, check Fixed Frame and TF.')

    def blockers(self):
        blockers = []
        if self.detection_messages == 0:
            blockers.append('/human_tracking/detections has no messages in this ROS graph.')
        elif self.max_detection_count > 0 and self.detection_count == 0:
            blockers.append(
                'YOLO detections were seen earlier, but the last detection message had 0 people. '
                'Run the diagnostic during the visible/locked part of the bag.')
        elif self.max_detection_count == 0:
            blockers.append(
                '/human_tracking/detections is publishing, but all sampled messages had 0 people.')

        if self.last_camera_target is None:
            blockers.append('/human_tracking/camera_target is missing.')
        elif self.last_camera_target.lock_state != TargetState.LOCK_TARGET_LOCKED:
            blockers.append(
                'Camera target is not locked. YOLO may be working, but target_lock needs a '
                'start gesture and then the same/reacquired target track.')
        elif not self.last_camera_target.camera_visible:
            blockers.append('Camera target is locked but not visible; camera-guided extraction will not run.')

        status = self.last_target_tracker_debug.get('camera_guided_status')
        if status and status != 'ok':
            blockers.append(f'Camera-guided extraction status is {status}.')
        if self.last_target_tracker_debug.get('camera_guided_points', 0) == 0:
            blockers.append('Camera-guided extraction selected 0 points.')

        if self.cloud_stats['/human_tracking/camera_guided_target_points'].max_width == 0:
            blockers.append('/human_tracking/camera_guided_target_points is empty.')

        for topic in (
            '/human_tracking/selected_tracklet_marker',
            '/human_tracking/selected_target_marker',
            '/human_tracking/target_prediction_gate_marker',
            '/human_tracking/fused_target_marker',
        ):
            if self.marker_stats[topic].max_add_markers == 0:
                blockers.append(f'{topic} only published empty/delete markers.')

        if self.last_fused_target is not None and self.last_fused_target.source_state == TargetState.SOURCE_NONE:
            blockers.append('Fused target source is NONE.')
        return blockers


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=20.0)
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = HumanTrackingPipelineDiagnostic(parsed.duration)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
