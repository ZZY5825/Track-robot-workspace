#!/usr/bin/env python3
"""Capture publication-style selected-target estimation dynamics from ROS replay."""

import argparse
from collections import deque
import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-target-estimation-dynamics')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from track_robot_interfaces.msg import LidarTrackletArray, TargetState


SOURCE_NAMES = {
    TargetState.SOURCE_NONE: 'none',
    TargetState.SOURCE_CAMERA_ONLY: 'camera only',
    TargetState.SOURCE_CAMERA_LIDAR: 'camera + LiDAR',
    TargetState.SOURCE_LIDAR_ONLY: 'LiDAR only',
    TargetState.SOURCE_PREDICTION_ONLY: 'prediction only',
}
SOURCE_COLORS = {
    TargetState.SOURCE_NONE: '#C8CDD1',
    TargetState.SOURCE_CAMERA_ONLY: '#6F4C9B',
    TargetState.SOURCE_CAMERA_LIDAR: '#2A7F76',
    TargetState.SOURCE_LIDAR_ONLY: '#4477AA',
    TargetState.SOURCE_PREDICTION_ONLY: '#D89B2B',
}
COLORS = {
    'fused': '#2A7F76',
    'tracklet': '#CC3311',
    'anchor': '#EE7733',
    'identity': '#6F4C9B',
    'geometry': '#4477AA',
    'overall': '#202428',
    'low': '#4477AA',
    'nominal': '#2A7F76',
    'maneuver': '#CC6677',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    parser.add_argument('--bag-label', default='human_tracking_lidar_20260706_145900')
    parser.add_argument('--timeout', type=float, default=48.0)
    parser.add_argument('--sync-tolerance', type=float, default=0.12)
    return parser.parse_args()


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pointcloud_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    formats = []
    offsets = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype != PointField.FLOAT32:
            return np.empty((0, 3), dtype=np.float32)
        formats.append(('>' if msg.is_bigendian else '<') + 'f4')
        offsets.append(field.offset)
    dtype = np.dtype({
        'names': ('x', 'y', 'z'),
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })
    records = np.frombuffer(
        msg.data, dtype=dtype, count=int(msg.width) * int(msg.height))
    points = np.column_stack((records['x'], records['y'], records['z']))
    return points[np.isfinite(points).all(axis=1)]


def nearest(rows, stamp, tolerance):
    if not rows:
        return None
    candidate = min(rows, key=lambda row: abs(row['stamp'] - stamp))
    return candidate if abs(candidate['stamp'] - stamp) <= tolerance else None


def deduplicate_by_stamp(rows):
    unique = {}
    for row in rows:
        unique[row['stamp']] = row
    return [unique[stamp] for stamp in sorted(unique)]


class DynamicsCapture(Node):
    def __init__(self, args):
        super().__init__('target_estimation_dynamics_capture')
        self.args = args
        self.deadline = time.monotonic() + args.timeout
        self.states = []
        self.tracklets = []
        self.anchors = []
        self.imm_samples = []
        self.latest_state_stamp = None
        self.first_target_id = None
        self.complete = False

        self.create_subscription(
            TargetState, '/human_tracking/fused_target_state', self.on_state, 20)
        self.create_subscription(
            LidarTrackletArray, '/human_tracking/lidar_tracklets',
            self.on_tracklets, 10)
        self.create_subscription(
            PointCloud2, '/human_tracking/camera_guided_target_points',
            self.on_guided_cloud, qos_profile_sensor_data)
        self.create_subscription(
            String, '/human_tracking/target_tracker_debug', self.on_debug, 20)

    def on_state(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.latest_state_stamp = stamp
        if (
            self.first_target_id is None and msg.target_id > 0 and
            msg.lock_state == TargetState.LOCK_TARGET_LOCKED
        ):
            self.first_target_id = int(msg.target_id)
        if self.first_target_id is not None and int(msg.target_id) == self.first_target_id:
            covariance = np.asarray(msg.position_covariance, dtype=float).reshape(3, 3)
            self.states.append({
                'stamp': stamp,
                'target_id': int(msg.target_id),
                'lock_state': int(msg.lock_state),
                'source_state': int(msg.source_state),
                'track_state': int(msg.track_state),
                'association_state': int(msg.association_state),
                'selected_tracklet_id': int(msg.selected_tracklet_id),
                'camera_visible': bool(msg.camera_visible),
                'lidar_visible': bool(msg.lidar_visible),
                'position_valid': bool(msg.position_base_valid),
                'position': np.asarray([
                    msg.position_base.x, msg.position_base.y, msg.position_base.z], dtype=float),
                'velocity': np.asarray([
                    msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float),
                'covariance': covariance,
                'distance': float(msg.distance),
                'identity_confidence': float(msg.identity_confidence),
                'geometry_confidence': float(msg.geometry_confidence),
                'overall_confidence': float(msg.overall_confidence),
            })
        elif (
            self.first_target_id is not None and
            msg.lock_state == TargetState.LOCK_NO_TARGET and self.states
        ):
            self.complete = True

    def on_tracklets(self, msg):
        self.tracklets.append({
            'stamp': stamp_seconds(msg.header.stamp),
            'tracklets': [{
                'id': int(tracklet.tracklet_id),
                'active': bool(tracklet.active),
                'confirmed': bool(tracklet.confirmed),
                'position': np.asarray([
                    tracklet.position.x, tracklet.position.y, tracklet.position.z], dtype=float),
            } for tracklet in msg.tracklets],
        })

    def on_guided_cloud(self, msg):
        points = pointcloud_xyz(msg)
        if len(points) >= 3:
            self.anchors.append({
                'stamp': stamp_seconds(msg.header.stamp),
                'position': np.median(points, axis=0),
                'point_count': int(len(points)),
            })

    def on_debug(self, msg):
        if self.latest_state_stamp is None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        probabilities = payload.get('imm_probabilities')
        if not isinstance(probabilities, list) or len(probabilities) != 3:
            return
        self.imm_samples.append({
            'stamp': self.latest_state_stamp,
            'probabilities': np.asarray(probabilities, dtype=float),
            'trigger': payload.get('trigger', 'unknown'),
            'processing_ms': float(payload.get('processing_ms', 0.0)),
        })


def prepare_data(capture):
    states = deduplicate_by_stamp(capture.states)
    states = [row for row in states if row['position_valid']]
    if len(states) < 20:
        raise RuntimeError(f'insufficient target-state samples: {len(states)}')
    start, end = states[0]['stamp'], states[-1]['stamp']

    tracklet_measurements = []
    for state in states:
        if state['selected_tracklet_id'] < 0:
            continue
        tracklet_row = nearest(capture.tracklets, state['stamp'], capture.args.sync_tolerance)
        if tracklet_row is None:
            continue
        selected = next((
            tracklet for tracklet in tracklet_row['tracklets']
            if tracklet['id'] == state['selected_tracklet_id'] and
            tracklet['active'] and tracklet['confirmed']
        ), None)
        if selected is not None:
            tracklet_measurements.append({
                'stamp': state['stamp'],
                'tracklet_id': selected['id'],
                'position': selected['position'],
                'sync_offset': abs(tracklet_row['stamp'] - state['stamp']),
            })
    tracklet_measurements = deduplicate_by_stamp(tracklet_measurements)
    anchors = deduplicate_by_stamp([
        row for row in capture.anchors if start <= row['stamp'] <= end])
    imm_samples = deduplicate_by_stamp([
        row for row in capture.imm_samples if start <= row['stamp'] <= end])
    if len(tracklet_measurements) < 3:
        raise RuntimeError(
            f'insufficient synchronized selected-tracklet measurements: {len(tracklet_measurements)}')
    if len(anchors) < 3:
        raise RuntimeError(f'insufficient camera-guided anchors: {len(anchors)}')
    return states, tracklet_measurements, anchors, imm_samples


def add_covariance_ellipse(axis, state, color):
    covariance_xy = state['covariance'][:2, :2]
    covariance_plot = covariance_xy[[1, 0]][:, [1, 0]]
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_plot)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = np.maximum(0.0, eigenvalues[order])
    eigenvectors = eigenvectors[:, order]
    angle = math.degrees(math.atan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    ellipse = Ellipse(
        (state['position'][1], state['position'][0]),
        width=4.0 * math.sqrt(eigenvalues[0]),
        height=4.0 * math.sqrt(eigenvalues[1]),
        angle=angle, fill=False, edgecolor=color,
        linestyle='--', linewidth=0.9, alpha=0.58)
    axis.add_patch(ellipse)


def contiguous_spans(times, values):
    if not len(times):
        return []
    spans = []
    start = float(times[0])
    current = int(values[0])
    for index in range(1, len(times)):
        if int(values[index]) != current:
            spans.append((start, float(times[index]), current))
            start = float(times[index])
            current = int(values[index])
    spans.append((start, float(times[-1]), current))
    return spans


def render(capture, states, tracklets, anchors, imm_samples):
    t0 = states[0]['stamp']
    times = np.asarray([row['stamp'] - t0 for row in states])
    positions = np.asarray([row['position'] for row in states])
    velocities = np.asarray([row['velocity'] for row in states])
    speed = np.linalg.norm(velocities[:, :2], axis=1)
    distance = np.asarray([row['distance'] for row in states])
    sources = np.asarray([row['source_state'] for row in states])
    identity = np.asarray([row['identity_confidence'] for row in states])
    geometry = np.asarray([row['geometry_confidence'] for row in states])
    overall = np.asarray([row['overall_confidence'] for row in states])
    confirmed = np.asarray([
        row['association_state'] == TargetState.ASSOCIATION_CONFIRMED and
        row['selected_tracklet_id'] >= 0 for row in states])

    tracklet_positions = np.asarray([row['position'] for row in tracklets])
    anchor_positions = np.asarray([row['position'] for row in anchors])

    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 9.5,
        'axes.titlesize': 11.2,
        'axes.labelsize': 9.5,
    })
    fig, axes = plt.subplots(2, 2, figsize=(14.8, 9.2), facecolor='white')
    trajectory_axis, kinematics_axis = axes[0]
    confidence_axis, imm_axis = axes[1]
    fig.subplots_adjust(hspace=0.30, wspace=0.24, bottom=0.15)

    trajectory_axis.plot(
        positions[:, 1], positions[:, 0], color=COLORS['fused'],
        linewidth=2.2, label='IMM fused state', zorder=7)
    trajectory_axis.scatter(
        anchor_positions[:, 1], anchor_positions[:, 0], marker='x', s=27,
        color=COLORS['anchor'], alpha=0.62, linewidths=1.1,
        label='camera-guided anchors', zorder=6)
    trajectory_axis.scatter(
        tracklet_positions[:, 1], tracklet_positions[:, 0], s=19,
        facecolors='none', edgecolors=COLORS['tracklet'], linewidths=0.9,
        alpha=0.72, label='selected LiDAR tracklets', zorder=8)
    covariance_indices = np.linspace(0, len(states) - 1, 7).astype(int)
    for index in covariance_indices:
        add_covariance_ellipse(trajectory_axis, states[index], COLORS['fused'])
    trajectory_axis.scatter(
        [positions[0, 1]], [positions[0, 0]], marker='o', s=75,
        color='#202428', edgecolor='white', linewidth=0.8, zorder=10, label='start')
    trajectory_axis.scatter(
        [positions[-1, 1]], [positions[-1, 0]], marker='s', s=70,
        color='#202428', edgecolor='white', linewidth=0.8, zorder=10, label='end')
    trajectory_axis.set_xlabel('lateral position y [m]')
    trajectory_axis.set_ylabel('forward position x [m]')
    trajectory_axis.set_aspect('equal', adjustable='datalim')
    trajectory_axis.grid(True, color='#D9DDE0', linewidth=0.55)
    trajectory_axis.set_title('(a) Planar target-state estimation', loc='left')
    trajectory_axis.legend(loc='best', fontsize=8.0, frameon=True)

    for start, end, source in contiguous_spans(times, sources):
        kinematics_axis.axvspan(
            start, end, color=SOURCE_COLORS.get(source, '#C8CDD1'), alpha=0.095)
    distance_line = kinematics_axis.plot(
        times, distance, color=COLORS['fused'], linewidth=2.0,
        label='target range')[0]
    kinematics_axis.set_xlabel('time from target lock [s]')
    kinematics_axis.set_ylabel('target range [m]', color=COLORS['fused'])
    kinematics_axis.tick_params(axis='y', labelcolor=COLORS['fused'])
    speed_axis = kinematics_axis.twinx()
    speed_line = speed_axis.plot(
        times, speed, color=COLORS['tracklet'], linewidth=1.55,
        alpha=0.86, label='estimated planar speed')[0]
    speed_axis.set_ylabel('estimated speed [m s$^{-1}$]', color=COLORS['tracklet'])
    speed_axis.tick_params(axis='y', labelcolor=COLORS['tracklet'])
    kinematics_axis.grid(True, color='#D9DDE0', linewidth=0.55)
    kinematics_axis.set_title('(b) Range and fused planar velocity', loc='left')
    kinematics_axis.legend(
        [distance_line, speed_line], ['target range', 'estimated planar speed'],
        loc='upper right', fontsize=8.2, frameon=True)

    for start, end, source in contiguous_spans(times, sources):
        confidence_axis.axvspan(
            start, end, color=SOURCE_COLORS.get(source, '#C8CDD1'), alpha=0.11)
    confidence_axis.plot(
        times, identity, color=COLORS['identity'], linewidth=1.5,
        label='identity confidence')
    confidence_axis.plot(
        times, geometry, color=COLORS['geometry'], linewidth=1.5,
        label='geometry confidence')
    confidence_axis.plot(
        times, overall, color=COLORS['overall'], linewidth=2.0,
        label='overall confidence')
    confidence_axis.fill_between(
        times, 0.0, 0.045, where=confirmed, step='post',
        color=COLORS['tracklet'], alpha=0.92, label='confirmed binding')
    confidence_axis.set_xlim(times.min(), times.max())
    confidence_axis.set_ylim(-0.02, 1.06)
    confidence_axis.set_xlabel('time from target lock [s]')
    confidence_axis.set_ylabel('confidence')
    confidence_axis.grid(True, color='#D9DDE0', linewidth=0.55)
    confidence_axis.set_title('(c) Confidence and measurement-source continuity', loc='left')
    confidence_axis.legend(loc='lower left', fontsize=8.0, frameon=True, ncol=2)
    source_handles = []
    for source in sorted(set(sources.tolist())):
        source_handles.append(Patch(
            facecolor=SOURCE_COLORS.get(source, '#C8CDD1'), alpha=0.22,
            label=SOURCE_NAMES.get(source, str(source))))
    if source_handles:
        source_legend = confidence_axis.legend(
            handles=source_handles, loc='upper right', fontsize=7.7,
            frameon=True, title='background state', title_fontsize=7.7)
        confidence_axis.add_artist(source_legend)

    if imm_samples:
        imm_times = np.asarray([row['stamp'] - t0 for row in imm_samples])
        probabilities = np.asarray([row['probabilities'] for row in imm_samples])
        imm_axis.plot(
            imm_times, probabilities[:, 0], color=COLORS['low'], linewidth=1.8,
            marker='o', markersize=3.0, label='low dynamics  $\sigma_a=0.15$')
        imm_axis.plot(
            imm_times, probabilities[:, 1], color=COLORS['nominal'], linewidth=2.0,
            marker='o', markersize=3.0, label='nominal  $\sigma_a=0.80$')
        imm_axis.plot(
            imm_times, probabilities[:, 2], color=COLORS['maneuver'], linewidth=1.8,
            marker='o', markersize=3.0, label='maneuvering  $\sigma_a=2.50$')
    else:
        imm_axis.text(
            0.5, 0.5, 'No timestamp-aligned IMM debug samples',
            ha='center', va='center', transform=imm_axis.transAxes, color='#5B6268')
    imm_axis.set_xlim(times.min(), times.max())
    imm_axis.set_ylim(-0.02, 1.02)
    imm_axis.set_xlabel('time from target lock [s]')
    imm_axis.set_ylabel('model probability')
    imm_axis.grid(True, color='#D9DDE0', linewidth=0.55)
    imm_axis.set_title('(d) Three-model IMM probability evolution', loc='left')
    imm_axis.legend(loc='upper right', fontsize=8.0, frameon=True)

    differences = np.diff(positions[:, :2], axis=0)
    path_length = float(np.linalg.norm(differences, axis=1).sum())
    dt = np.diff(times, append=times[-1])
    confirmed_duration = float(np.sum(dt * confirmed.astype(float)))
    active_ids = [row['tracklet_id'] for row in tracklets]
    id_switches = sum(a != b for a, b in zip(active_ids, active_ids[1:]))
    fig.text(0.03, 0.075, 'RECORDED REPLAY', fontsize=8, weight='bold', color='#5B6268')
    fig.text(0.03, 0.042, capture.args.bag_label, fontsize=9.0, weight='bold', color='#202428')
    fig.text(0.30, 0.075, 'TRACKING INTERVAL', fontsize=8, weight='bold', color='#5B6268')
    fig.text(
        0.30, 0.042, f'{times[-1]:.1f} s  ·  confirmed binding {confirmed_duration:.1f} s',
        fontsize=9.0, weight='bold', color='#202428')
    fig.text(0.58, 0.075, 'TARGET MOTION', fontsize=8, weight='bold', color='#5B6268')
    fig.text(
        0.58, 0.042,
        f'path {path_length:.2f} m  ·  range {distance.min():.2f}–{distance.max():.2f} m  ·  max speed {speed.max():.2f} m s$^{{-1}}$',
        fontsize=9.0, weight='bold', color='#202428')
    fig.text(0.88, 0.075, 'IDENTITY', fontsize=8, weight='bold', color='#5B6268')
    fig.text(
        0.88, 0.042, f'{id_switches} selected-ID switch(es)',
        fontsize=9.0, weight='bold', color='#202428')

    output = Path(capture.args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'source': 'recorded_rosbag_replay',
        'bag': capture.args.bag_label,
        'target_id': capture.first_target_id,
        'target_state_sample_count': len(states),
        'selected_tracklet_measurement_count': len(tracklets),
        'camera_guided_anchor_count': len(anchors),
        'imm_probability_sample_count': len(imm_samples),
        'tracking_interval_sec': float(times[-1]),
        'confirmed_binding_duration_sec': confirmed_duration,
        'fused_path_length_m': path_length,
        'target_range_min_m': float(distance.min()),
        'target_range_max_m': float(distance.max()),
        'estimated_speed_max_mps': float(speed.max()),
        'selected_tracklet_ids': sorted(set(active_ids)),
        'selected_tracklet_id_switch_count': id_switches,
        'max_tracklet_state_sync_offset_sec': float(max(
            row['sync_offset'] for row in tracklets)),
        'imm_models': [
            {'name': 'low_dynamics', 'acceleration_std_mps2': 0.15},
            {'name': 'nominal', 'acceleration_std_mps2': 0.80},
            {'name': 'maneuvering', 'acceleration_std_mps2': 2.50},
        ],
        'imm_alignment_note': (
            'Debug probabilities are aligned to the latest received fused-state stamp; '
            'state, tracklet, and anchor trajectories retain sensor timestamps.'),
    }
    metadata_output = Path(capture.args.metadata_output)
    metadata_output.parent.mkdir(parents=True, exist_ok=True)
    metadata_output.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    rclpy.init()
    capture = DynamicsCapture(args)
    try:
        while (
            rclpy.ok() and time.monotonic() < capture.deadline and
            not capture.complete
        ):
            rclpy.spin_once(capture, timeout_sec=0.2)
    finally:
        capture.destroy_node()
        rclpy.shutdown()
    states, tracklets, anchors, imm_samples = prepare_data(capture)
    render(capture, states, tracklets, anchors, imm_samples)
    print(json.dumps({
        'output': args.output,
        'states': len(states),
        'tracklets': len(tracklets),
        'anchors': len(anchors),
        'imm_samples': len(imm_samples),
    }))


if __name__ == '__main__':
    main()
