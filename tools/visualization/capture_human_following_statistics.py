#!/usr/bin/env python3
"""Capture normalized statistical evidence from one human-following ROS replay."""

import argparse
from collections import deque
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from std_msgs.msg import String
from track_robot_interfaces.msg import CameraTarget, LidarTrackletArray, TargetState

from human_following_statistics import summarize_run


LOCK_NAMES = {
    TargetState.LOCK_NO_TARGET: 'NO_TARGET',
    TargetState.LOCK_CANDIDATE_VISIBLE: 'CANDIDATE_VISIBLE',
    TargetState.LOCK_TARGET_LOCKED: 'TARGET_LOCKED',
    TargetState.LOCK_TARGET_LOST: 'TARGET_LOST',
}
SOURCE_NAMES = {
    TargetState.SOURCE_NONE: 'NONE',
    TargetState.SOURCE_CAMERA_ONLY: 'CAMERA_ONLY',
    TargetState.SOURCE_CAMERA_LIDAR: 'CAMERA_LIDAR',
    TargetState.SOURCE_LIDAR_ONLY: 'LIDAR_ONLY',
    TargetState.SOURCE_PREDICTION_ONLY: 'PREDICTION_ONLY',
}
TRACK_NAMES = {
    TargetState.TRACK_NO_TARGET: 'NO_TARGET',
    TargetState.TRACK_CAMERA_LOCKED: 'CAMERA_LOCKED',
    TargetState.TRACK_CAMERA_LIDAR_TRACKED: 'CAMERA_LIDAR_TRACKED',
    TargetState.TRACK_LIDAR_ONLY_TRACKING: 'LIDAR_ONLY_TRACKING',
    TargetState.TRACK_PREDICTION_ONLY: 'PREDICTION_ONLY',
    TargetState.TRACK_TARGET_LOST: 'TARGET_LOST',
}
ASSOCIATION_NAMES = {
    TargetState.ASSOCIATION_UNBOUND: 'UNBOUND',
    TargetState.ASSOCIATION_CONFIRMED: 'CONFIRMED',
    TargetState.ASSOCIATION_AMBIGUOUS: 'AMBIGUOUS',
    TargetState.ASSOCIATION_PREDICTED: 'PREDICTED',
    TargetState.ASSOCIATION_LOST: 'LOST',
}

BASE_TO_LIDAR_Z_M = 0.70
OPTICAL_TRANSLATION = np.asarray([0.06, -0.065, -0.26], dtype=float)
OPTICAL_QUATERNION_XYZW = [0.5, -0.5, 0.5, 0.5]
ANCHOR_XY_GATE_M = 1.0
ANCHOR_RANGE_GATE_M = 0.75
PROJECTION_CENTER_GATE_PX = 220.0
MIN_ASSOCIATION_SCORE = 0.65
MAX_CAMERA_ANCHOR_NIS_XY = 25.0
MAX_TRACKLET_NIS_XY = 9.21


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--run-index', type=int, default=1)
    parser.add_argument('--playback-rate', type=float, default=0.5)
    parser.add_argument('--recorded-duration-sec', type=float, required=True)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--state-sync-tolerance', type=float, default=0.12)
    parser.add_argument('--camera-sync-tolerance', type=float, default=0.20)
    return parser.parse_args()


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def nearest(rows, stamp, tolerance):
    if not rows:
        return None
    candidate = min(rows, key=lambda row: abs(float(row['stamp']) - stamp))
    return candidate if abs(float(candidate['stamp']) - stamp) <= tolerance else None


def pointcloud_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype != PointField.FLOAT32:
            return np.empty((0, 3), dtype=np.float32)
        formats.append(('>' if msg.is_bigendian else '<') + 'f4')
        offsets.append(field.offset)
    dtype = np.dtype({
        'names': ('x', 'y', 'z'), 'formats': formats,
        'offsets': offsets, 'itemsize': msg.point_step,
    })
    records = np.frombuffer(msg.data, dtype=dtype, count=int(msg.width) * int(msg.height))
    points = np.column_stack((records['x'], records['y'], records['z']))
    return points[np.isfinite(points).all(axis=1)]


def quaternion_rotation(x, y, z, w):
    quaternion = np.asarray([w, x, y, z], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    return np.asarray([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


class StatisticsCapture(Node):
    def __init__(self, args):
        super().__init__('human_following_statistics_capture')
        self.args = args
        self.deadline = time.monotonic() + args.timeout
        self.states = []
        self.tracklets = []
        self.anchors = []
        self.camera_targets = []
        self.debug_rows = []
        self.camera_info = None
        self.latest_state_stamp = None

        self.create_subscription(TargetState, '/human_tracking/fused_target_state',
                                 self.on_state, 30)
        self.create_subscription(LidarTrackletArray, '/human_tracking/lidar_tracklets',
                                 self.on_tracklets, 15)
        self.create_subscription(PointCloud2, '/human_tracking/camera_guided_target_points',
                                 self.on_anchor, qos_profile_sensor_data)
        self.create_subscription(CameraTarget, '/human_tracking/camera_target',
                                 self.on_camera_target, 15)
        self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info',
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(String, '/human_tracking/target_tracker_debug',
                                 self.on_debug, 30)

    def on_state(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.latest_state_stamp = stamp
        covariance = np.asarray(msg.position_covariance, dtype=float).reshape(3, 3)
        self.states.append({
            'stamp': stamp,
            'target_id': int(msg.target_id),
            'lock_state': LOCK_NAMES.get(int(msg.lock_state), f'UNKNOWN_{msg.lock_state}'),
            'source_state': SOURCE_NAMES.get(int(msg.source_state), f'UNKNOWN_{msg.source_state}'),
            'track_state': TRACK_NAMES.get(int(msg.track_state), f'UNKNOWN_{msg.track_state}'),
            'association_state': ASSOCIATION_NAMES.get(
                int(msg.association_state), f'UNKNOWN_{msg.association_state}'),
            'selected_tracklet_id': int(msg.selected_tracklet_id),
            'camera_visible': bool(msg.camera_visible),
            'lidar_visible': bool(msg.lidar_visible),
            'position_valid': bool(msg.position_base_valid),
            'position': [float(msg.position_base.x), float(msg.position_base.y),
                         float(msg.position_base.z)],
            'velocity': [float(msg.velocity.x), float(msg.velocity.y), float(msg.velocity.z)],
            'distance': float(msg.distance),
            'identity_confidence': float(msg.identity_confidence),
            'geometry_confidence': float(msg.geometry_confidence),
            'overall_confidence': float(msg.overall_confidence),
            'covariance_trace_xy': float(covariance[0, 0] + covariance[1, 1]),
        })

    def on_tracklets(self, msg):
        self.tracklets.append({
            'stamp': stamp_seconds(msg.header.stamp),
            'tracklets': [{
                'id': int(row.tracklet_id),
                'active': bool(row.active),
                'confirmed': bool(row.confirmed),
                'position': [float(row.position.x), float(row.position.y), float(row.position.z)],
                'velocity': [float(row.velocity.x), float(row.velocity.y), float(row.velocity.z)],
                'confidence': float(row.confidence),
                'observation_quality': float(row.observation_quality),
            } for row in msg.tracklets],
        })

    def on_anchor(self, msg):
        points = pointcloud_xyz(msg)
        if len(points) >= 3:
            self.anchors.append({
                'stamp': stamp_seconds(msg.header.stamp),
                'position': np.median(points, axis=0).astype(float).tolist(),
                'point_count': int(len(points)),
            })

    def on_camera_target(self, msg):
        self.camera_targets.append({
            'stamp': stamp_seconds(msg.header.stamp),
            'logical_target_id': int(msg.logical_target_id),
            'lock_state': int(msg.lock_state),
            'identity_state': int(msg.identity_state),
            'bbox': [float(value) for value in msg.bbox],
            'camera_visible': bool(msg.camera_visible),
        })

    def on_camera_info(self, msg):
        self.camera_info = {
            'fx': float(msg.p[0]), 'fy': float(msg.p[5]),
            'cx': float(msg.p[2]), 'cy': float(msg.p[6]),
        }

    def on_debug(self, msg):
        if self.latest_state_stamp is None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        payload['stamp'] = self.latest_state_stamp
        self.debug_rows.append(payload)


def project_position(position, camera_info):
    rotation = quaternion_rotation(*OPTICAL_QUATERNION_XYZW)
    lidar_position = np.asarray(position, dtype=float) - np.asarray([0.0, 0.0, BASE_TO_LIDAR_Z_M])
    camera_position = rotation @ lidar_position + OPTICAL_TRANSLATION
    if camera_position[2] <= 0.05:
        return None
    return [
        float(camera_info['fx'] * camera_position[0] / camera_position[2] + camera_info['cx']),
        float(camera_info['fy'] * camera_position[1] / camera_position[2] + camera_info['cy']),
    ]


def synchronized_context(capture, stamp):
    tracklets = nearest(capture.tracklets, stamp, capture.args.state_sync_tolerance)
    anchor = nearest(capture.anchors, stamp, capture.args.state_sync_tolerance)
    camera = nearest(capture.camera_targets, stamp, capture.args.camera_sync_tolerance)
    debug = nearest(capture.debug_rows, stamp, capture.args.state_sync_tolerance)
    return tracklets, anchor, camera, debug


def gate_rows(tracklet_row, anchor_row, camera_row, camera_info):
    if None in (tracklet_row, anchor_row, camera_row) or camera_info is None:
        return []
    anchor = np.asarray(anchor_row['position'], dtype=float)
    anchor_range = float(np.linalg.norm(anchor[:2]))
    bbox = np.asarray(camera_row['bbox'], dtype=float)
    bbox_center = np.asarray([0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3])])
    rows = []
    for tracklet in tracklet_row['tracklets']:
        if not tracklet['active'] or not tracklet['confirmed']:
            continue
        position = np.asarray(tracklet['position'], dtype=float)
        anchor_distance = float(np.linalg.norm(position[:2] - anchor[:2]))
        range_difference = abs(float(np.linalg.norm(position[:2])) - anchor_range)
        projected = project_position(position, camera_info)
        projection_error = (
            float(np.linalg.norm(np.asarray(projected) - bbox_center))
            if projected is not None else None)
        rows.append({
            'id': int(tracklet['id']),
            'anchor_distance_m': anchor_distance,
            'range_difference_m': range_difference,
            'projection_center_error_px': projection_error,
            'anchor_pass': anchor_distance <= ANCHOR_XY_GATE_M,
            'range_pass': (
                anchor_distance <= ANCHOR_XY_GATE_M and
                range_difference <= ANCHOR_RANGE_GATE_M),
            'projection_valid': (
                anchor_distance <= ANCHOR_XY_GATE_M and
                range_difference <= ANCHOR_RANGE_GATE_M and
                projected is not None),
        })
    return rows


def build_record(capture):
    states_by_stamp = {row['stamp']: row for row in capture.states}
    states = [states_by_stamp[stamp] for stamp in sorted(states_by_stamp)]
    associations = []
    debug_samples = []
    funnel_updates = []

    for state in states:
        tracklet_row, anchor_row, camera_row, debug = synchronized_context(
            capture, float(state['stamp']))
        gates = gate_rows(tracklet_row, anchor_row, camera_row, capture.camera_info)
        gate_by_id = {row['id']: row for row in gates}
        selected_id = int(state['selected_tracklet_id'])
        selected_gate = gate_by_id.get(selected_id)
        if selected_gate is not None and debug is not None:
            hypotheses = sorted(
                [row for row in debug.get('hypotheses', []) if 'id' in row and 'score' in row],
                key=lambda row: float(row['score']), reverse=True)
            score = float(debug.get('selected_tracklet_score', 0.0))
            if score <= 0.0:
                match = next((row for row in hypotheses if int(row['id']) == selected_id), None)
                score = float(match['score']) if match else None
            margin = (
                float(hypotheses[0]['score']) - float(hypotheses[1]['score'])
                if len(hypotheses) >= 2 else None)
            associations.append({
                'stamp': float(state['stamp']),
                'target_id': int(state['target_id']),
                'selected_tracklet_id': selected_id,
                'anchor_distance_m': selected_gate['anchor_distance_m'],
                'range_difference_m': selected_gate['range_difference_m'],
                'projection_center_error_px': selected_gate['projection_center_error_px'],
                'association_score': score,
                'top_two_margin': margin,
                'hypothesis_count': len(hypotheses),
                'sync_offsets_sec': {
                    'tracklet': abs(float(tracklet_row['stamp']) - float(state['stamp'])),
                    'anchor': abs(float(anchor_row['stamp']) - float(state['stamp'])),
                    'camera': abs(float(camera_row['stamp']) - float(state['stamp'])),
                    'debug': abs(float(debug['stamp']) - float(state['stamp'])),
                },
            })

    unique_debug = {float(row['stamp']): row for row in capture.debug_rows}
    for stamp in sorted(unique_debug):
        debug = unique_debug[stamp]
        tracklet_row = nearest(capture.tracklets, stamp, capture.args.state_sync_tolerance)
        anchor_row = nearest(capture.anchors, stamp, capture.args.state_sync_tolerance)
        camera_row = nearest(capture.camera_targets, stamp, capture.args.camera_sync_tolerance)
        gates = gate_rows(tracklet_row, anchor_row, camera_row, capture.camera_info)
        valid_ids = {row['id'] for row in gates if row['projection_valid']}
        hypotheses = [
            row for row in debug.get('hypotheses', [])
            if int(row.get('id', -1)) in valid_ids and isinstance(row.get('score'), (int, float))]
        hypothesis_ids = {int(row['id']) for row in hypotheses}
        score_ids = {
            int(row['id']) for row in hypotheses if float(row['score']) >= MIN_ASSOCIATION_SCORE}
        selected_id = int(debug.get('selected_lidar_tracklet_id', -1))
        funnel_updates.append({
            'stamp': stamp,
            'raw_candidate_clusters': int(debug.get('candidate_count', 0)),
            'active_tracklets': sum(
                1 for row in (tracklet_row['tracklets'] if tracklet_row else []) if row['active']),
            'confirmed_evaluations': len(gates),
            'anchor_gate_pass': sum(row['anchor_pass'] for row in gates),
            'range_gate_pass': sum(row['range_pass'] for row in gates),
            'valid_projection': len(valid_ids),
            'published_hypotheses': len(hypothesis_ids),
            'score_threshold_pass': len(score_ids),
            'selected': int(selected_id in score_ids),
        })
        probabilities = debug.get('imm_probabilities', [])
        debug_samples.append({
            'stamp': stamp,
            'trigger': debug.get('trigger', 'unknown'),
            'measurement_source': debug.get('measurement_source', 'unknown'),
            'measurement_accepted': bool(debug.get('measurement_accepted', False)),
            'rejection_reason': debug.get('rejection_reason', 'none'),
            'kalman_nis_xy': float(debug.get('kalman_nis_xy', 0.0)),
            'imm_probabilities': (
                [float(value) for value in probabilities] if len(probabilities) == 3 else None),
            'selected_tracklet_score': float(debug.get('selected_tracklet_score', 0.0)),
            'association_ambiguous': bool(debug.get('association_ambiguous', False)),
            'fresh_cloud_processed': bool(debug.get('fresh_cloud_processed', False)),
        })

    record = {
        'schema_version': 'human_following_statistics/1.0.0',
        'source': 'recorded_rosbag_replay',
        'bag': capture.args.bag,
        'run_index': int(capture.args.run_index),
        'playback_rate': float(capture.args.playback_rate),
        'recorded_duration_sec': float(capture.args.recorded_duration_sec),
        'states': states,
        'associations': associations,
        'debug_samples': debug_samples,
        'funnel_updates': funnel_updates,
        'capture_counts': {
            'states': len(states), 'tracklet_arrays': len(capture.tracklets),
            'anchors': len(capture.anchors), 'camera_targets': len(capture.camera_targets),
            'debug_messages': len(capture.debug_rows),
        },
        'configuration': {
            'state_sync_tolerance_sec': capture.args.state_sync_tolerance,
            'camera_sync_tolerance_sec': capture.args.camera_sync_tolerance,
            'anchor_xy_gate_m': ANCHOR_XY_GATE_M,
            'anchor_range_gate_m': ANCHOR_RANGE_GATE_M,
            'projection_center_gate_px': PROJECTION_CENTER_GATE_PX,
            'min_association_score': MIN_ASSOCIATION_SCORE,
            'max_camera_anchor_nis_xy': MAX_CAMERA_ANCHOR_NIS_XY,
            'max_tracklet_nis_xy': MAX_TRACKLET_NIS_XY,
            'base_to_lidar_z_m': BASE_TO_LIDAR_Z_M,
            'optical_translation_xyz_m': OPTICAL_TRANSLATION.tolist(),
            'optical_quaternion_xyzw': OPTICAL_QUATERNION_XYZW,
        },
        'limitations': [
            'No external ground truth is present; geometric residuals measure internal consistency.',
            'Tracklet IDs are local to each independent replay.',
        ],
    }
    record['summary'] = summarize_run(record)
    return record


def main():
    args = parse_args()
    rclpy.init()
    capture = StatisticsCapture(args)
    try:
        while rclpy.ok() and time.monotonic() < capture.deadline:
            rclpy.spin_once(capture, timeout_sec=0.2)
    finally:
        capture.destroy_node()
        rclpy.shutdown()
    record = build_record(capture)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({
        'output': str(output),
        'bag': args.bag,
        'run_index': args.run_index,
        'locked': record['summary']['locked'],
        'associations': len(record['associations']),
        'debug_samples': len(record['debug_samples']),
    }))


if __name__ == '__main__':
    main()
