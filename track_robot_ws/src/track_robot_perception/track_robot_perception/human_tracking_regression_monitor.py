#!/usr/bin/env python3

import argparse
import hashlib
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from track_robot_interfaces.msg import (
    CameraTarget, LidarTrackletArray, SelectedLidarTracklet, TargetState)
from visualization_msgs.msg import MarkerArray


class HumanTrackingRegressionMonitor(Node):
    def __init__(self, duration_sec, output_path=''):
        super().__init__('human_tracking_regression_monitor')
        self.duration_sec = max(1.0, duration_sec)
        self.started = time.monotonic()
        self.output_path = output_path
        self.logical_ids = set()
        self.selected_ids = []
        self.fused_messages = 0
        self.tracklet_messages = 0
        self.marker_messages = 0
        self.ambiguous_messages = 0
        self.prediction_messages = 0
        self.lost_messages = 0
        self.max_sync_offset = 0.0
        self.max_position_step = 0.0
        self.last_position = None
        self.sensor_samples = {}
        self.last_tracker_debug = {}
        self.unconfirmed_tracklet_switches = 0
        self.create_subscription(CameraTarget, '/human_tracking/camera_target', self.camera_cb, 10)
        self.create_subscription(
            SelectedLidarTracklet, '/human_tracking/selected_lidar_tracklet',
            self.selected_cb, 10)
        self.create_subscription(TargetState, '/human_tracking/fused_target_state', self.fused_cb, 10)
        self.create_subscription(
            LidarTrackletArray, '/human_tracking/lidar_tracklets', self.tracklets_cb, 10)
        self.create_subscription(
            MarkerArray, '/human_tracking/selected_target_marker', self.marker_cb, 10)
        self.create_subscription(
            String, '/human_tracking/target_tracker_debug', self.debug_cb, 10)
        self.timer = self.create_timer(0.1, self.check_done)

    def camera_cb(self, msg):
        if msg.logical_target_id >= 0:
            self.logical_ids.add(int(msg.logical_target_id))

    def selected_cb(self, msg):
        if msg.selected and msg.selected_tracklet_id >= 0:
            value = int(msg.selected_tracklet_id)
            if not self.selected_ids or self.selected_ids[-1] != value:
                if self.selected_ids:
                    confirmed_reason = (
                        self.last_tracker_debug.get('measurement_source') ==
                        'strict_local_relink' or
                        self.last_tracker_debug.get('switch_reject_reason') ==
                        'switched_after_depth_hysteresis')
                    if not confirmed_reason:
                        self.unconfirmed_tracklet_switches += 1
                self.selected_ids.append(value)

    def fused_cb(self, msg):
        self.fused_messages += 1
        if msg.association_state == TargetState.ASSOCIATION_AMBIGUOUS:
            self.ambiguous_messages += 1
        if msg.association_state == TargetState.ASSOCIATION_PREDICTED:
            self.prediction_messages += 1
        if msg.association_state == TargetState.ASSOCIATION_LOST:
            self.lost_messages += 1
        position = (float(msg.position_base.x), float(msg.position_base.y))
        stamp_ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        self.sensor_samples[stamp_ns] = (
            int(msg.target_id), int(msg.track_state), int(msg.association_state),
            int(msg.selected_tracklet_id), round(position[0], 3), round(position[1], 3))
        if self.last_position is not None:
            self.max_position_step = max(
                self.max_position_step,
                math.hypot(position[0] - self.last_position[0],
                           position[1] - self.last_position[1]))
        self.last_position = position

    def tracklets_cb(self, _msg):
        self.tracklet_messages += 1

    def marker_cb(self, _msg):
        self.marker_messages += 1

    def debug_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            self.last_tracker_debug = payload
            offset = float(payload.get('sync_offset_sec', 0.0))
            if math.isfinite(offset):
                self.max_sync_offset = max(self.max_sync_offset, offset)
        except (ValueError, TypeError, json.JSONDecodeError):
            pass

    def check_done(self):
        if time.monotonic() - self.started < self.duration_sec:
            return
        elapsed = max(1e-3, time.monotonic() - self.started)
        serialized_samples = json.dumps(
            sorted(self.sensor_samples.items()), separators=(',', ':')).encode('utf-8')
        report = {
            'duration_sec': round(elapsed, 3),
            'logical_target_ids': sorted(self.logical_ids),
            'logical_id_switch_count': max(0, len(self.logical_ids) - 1),
            'selected_tracklet_sequence': self.selected_ids,
            'selected_tracklet_switch_count': max(0, len(self.selected_ids) - 1),
            'unconfirmed_tracklet_switch_count': self.unconfirmed_tracklet_switches,
            'fused_rate_hz': round(self.fused_messages / elapsed, 3),
            'tracklet_measurement_rate_hz': round(self.tracklet_messages / elapsed, 3),
            'selected_marker_rate_hz': round(self.marker_messages / elapsed, 3),
            'ambiguous_messages': self.ambiguous_messages,
            'prediction_messages': self.prediction_messages,
            'lost_messages': self.lost_messages,
            'max_sync_offset_sec': round(self.max_sync_offset, 4),
            'max_output_position_step_m': round(self.max_position_step, 4),
            'rate_target_met': (
                self.fused_messages / elapsed >= 15.0 and
                self.tracklet_messages / elapsed >= 15.0),
            'sync_target_met': self.max_sync_offset <= 0.08,
            'sensor_sample_count': len(self.sensor_samples),
            'sensor_sequence_sha256': hashlib.sha256(serialized_samples).hexdigest(),
        }
        output = json.dumps(report, indent=2, sort_keys=True)
        print(output)
        if self.output_path:
            with open(self.output_path, 'w', encoding='utf-8') as output_file:
                output_file.write(output + '\n')
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--output', default='')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = HumanTrackingRegressionMonitor(parsed.duration, parsed.output)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
