#!/usr/bin/env python3
"""Build a publication-style human-following sequence from recorded ROS data."""

import argparse
import bisect
import json
import os
import time
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-human-following-sequence')

import cv2
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from track_robot_interfaces.msg import LidarTrackletArray, TargetState


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


def nearest_by_stamp(rows, stamp):
    if not rows:
        return None, float('inf')
    stamps = [row['stamp'] for row in rows]
    index = bisect.bisect_left(stamps, stamp)
    candidates = rows[max(0, index - 1):min(len(rows), index + 2)]
    best = min(candidates, key=lambda row: abs(row['stamp'] - stamp))
    return best, abs(best['stamp'] - stamp)


class SequenceCapture(Node):
    def __init__(self, args):
        super().__init__('human_following_sequence_capture')
        self.args = args
        self.bridge = CvBridge()
        self.deadline = time.monotonic() + args.timeout
        self.overlays = []
        self.states = []
        self.tracklets = []
        self.create_subscription(
            Image, '/human_tracking/target_overlay', self.on_overlay,
            qos_profile_sensor_data)
        self.create_subscription(
            TargetState, '/human_tracking/fused_target_state', self.on_state, 10)
        self.create_subscription(
            LidarTrackletArray, '/human_tracking/lidar_tracklets', self.on_tracklets, 10)

    def on_overlay(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 91])
        if ok:
            self.overlays.append({
                'stamp': stamp_seconds(msg.header.stamp),
                'jpeg': encoded,
            })

    def on_state(self, msg):
        self.states.append({
            'stamp': stamp_seconds(msg.header.stamp),
            'target_id': int(msg.target_id),
            'lock_state': int(msg.lock_state),
            'source_state': int(msg.source_state),
            'track_state': int(msg.track_state),
            'association_state': int(msg.association_state),
            'selected_tracklet_id': int(msg.selected_tracklet_id),
            'camera_visible': bool(msg.camera_visible),
            'lidar_visible': bool(msg.lidar_visible),
            'distance': float(msg.distance),
            'identity_confidence': float(msg.identity_confidence),
            'geometry_confidence': float(msg.geometry_confidence),
            'overall_confidence': float(msg.overall_confidence),
            'bbox': [float(value) for value in msg.bbox],
        })

    def on_tracklets(self, msg):
        self.tracklets.append({
            'stamp': stamp_seconds(msg.header.stamp),
            'active_ids': [
                int(tracklet.tracklet_id)
                for tracklet in msg.tracklets
                if tracklet.active and tracklet.confirmed
            ],
        })


def paired_frames(capture):
    capture.states.sort(key=lambda row: row['stamp'])
    capture.tracklets.sort(key=lambda row: row['stamp'])
    frames = []
    for overlay in sorted(capture.overlays, key=lambda row: row['stamp']):
        state, state_offset = nearest_by_stamp(capture.states, overlay['stamp'])
        tracklets, tracklet_offset = nearest_by_stamp(capture.tracklets, overlay['stamp'])
        if state is None or state_offset > capture.args.sync_tolerance:
            continue
        active_ids = tracklets['active_ids'] if (
            tracklets is not None and tracklet_offset <= capture.args.sync_tolerance) else []
        state = dict(state)
        state['selected_active'] = state['selected_tracklet_id'] in active_ids
        state['overlay_offset_sec'] = state_offset
        state['tracklet_offset_sec'] = tracklet_offset
        frames.append({'overlay': overlay, 'state': state})
    return frames


def distinct_append(selected, candidate, minimum_gap=0.35):
    if candidate is None:
        return
    if all(abs(candidate['overlay']['stamp'] - row['overlay']['stamp']) >= minimum_gap
           for row in selected):
        selected.append(candidate)


def select_sequence(frames):
    locked = [
        frame for frame in frames
        if frame['state']['camera_visible'] and
        frame['state']['lock_state'] == TargetState.LOCK_TARGET_LOCKED
    ]
    first_lock = locked[0]
    released = next((
        frame for frame in frames
        if frame['overlay']['stamp'] > first_lock['overlay']['stamp'] + 0.5 and
        frame['state']['lock_state'] != TargetState.LOCK_TARGET_LOCKED and
        not frame['state']['camera_visible']
    ), None)
    release_stamp = (
        released['overlay']['stamp'] if released is not None else float('inf'))
    first_episode_locked = [
        frame for frame in locked if frame['overlay']['stamp'] < release_stamp]
    confirmed = [
        frame for frame in first_episode_locked
        if frame['state']['lidar_visible'] and
        frame['state']['selected_tracklet_id'] >= 0 and
        frame['state']['selected_active'] and
        frame['state']['association_state'] == TargetState.ASSOCIATION_CONFIRMED
    ]
    if not locked or len(confirmed) < 3:
        raise RuntimeError(
            f'insufficient replay evidence: locked={len(locked)}, confirmed={len(confirmed)}')

    first_bound = confirmed[0]
    after_first = [
        frame for frame in confirmed
        if frame['overlay']['stamp'] >= first_bound['overlay']['stamp'] + 0.45
    ]
    rightmost = max(
        after_first or confirmed,
        key=lambda frame: 0.5 * (
            frame['state']['bbox'][0] + frame['state']['bbox'][2]))
    after_right = [
        frame for frame in confirmed
        if frame['overlay']['stamp'] >= rightmost['overlay']['stamp'] + 0.55
    ]
    continued = after_right[len(after_right) // 2] if after_right else confirmed[-1]
    selected = []
    for frame in (first_lock, first_bound, rightmost, continued, released):
        distinct_append(selected, frame)
    if len(selected) < 5:
        for index in np.linspace(0, len(confirmed) - 1, 7).astype(int):
            distinct_append(selected, confirmed[index], minimum_gap=0.20)
            if len(selected) == 5:
                break
    if len(selected) < 5:
        raise RuntimeError(f'only {len(selected)} distinct synchronized frames available')
    return sorted(selected[:5], key=lambda row: row['overlay']['stamp'])


def render(capture, frames, selected):
    t0 = selected[0]['overlay']['stamp']
    panel_titles = [
        'Target lock',
        '3D association',
        'Lateral motion',
        'Continued fusion',
        'Safe target release',
    ]
    panel_letters = 'abcde'

    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 9,
        'axes.titlesize': 10.5,
        'axes.labelsize': 9,
    })
    fig = plt.figure(figsize=(18, 5.2), facecolor='white')
    grid = fig.add_gridspec(2, 5, height_ratios=(2.25, 1.55), hspace=0.24, wspace=0.045)

    for index, (frame, title) in enumerate(zip(selected, panel_titles)):
        axis = fig.add_subplot(grid[0, index])
        image = cv2.imdecode(frame['overlay']['jpeg'], cv2.IMREAD_COLOR)
        axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        state = frame['state']
        dt = frame['overlay']['stamp'] - t0
        axis.set_title(f'({panel_letters[index]}) {title}   $t={dt:.1f}$ s', loc='left')
        axis.axis('off')
        tracklet = state['selected_tracklet_id']
        source = SOURCE_NAMES.get(state['source_state'], str(state['source_state']))
        if tracklet >= 0 and state['selected_active']:
            status = f'{source}  ·  T{tracklet}  ·  {state["distance"]:.2f} m'
            color = '#1F6F67'
        else:
            status = f'{source}  ·  association pending'
            color = '#8A6518'
        axis.text(
            0.02, 0.025, status, transform=axis.transAxes,
            color='white', fontsize=8.3, weight='bold',
            bbox={'boxstyle': 'round,pad=0.3', 'facecolor': color,
                  'edgecolor': 'none', 'alpha': 0.92})

    timeline = fig.add_subplot(grid[1, :])
    state_rows = sorted(capture.states, key=lambda row: row['stamp'])
    start = t0 - 0.20
    end = selected[-1]['overlay']['stamp'] + 0.45
    timeline_rows = [row for row in state_rows if start <= row['stamp'] <= end]
    times = np.asarray([row['stamp'] - t0 for row in timeline_rows])
    signals = [
        ('target lock', [row['lock_state'] == TargetState.LOCK_TARGET_LOCKED for row in timeline_rows]),
        ('camera visible', [row['camera_visible'] for row in timeline_rows]),
        ('LiDAR geometry', [row['lidar_visible'] for row in timeline_rows]),
        ('confirmed binding', [
            row['selected_tracklet_id'] >= 0 and
            row['association_state'] == TargetState.ASSOCIATION_CONFIRMED
            for row in timeline_rows]),
    ]
    colors = ['#6F4C9B', '#2F6B9A', '#2A7F76', '#B83A3A']
    y_positions = [3, 2, 1, 0]
    for (label, values), color, y in zip(signals, colors, y_positions):
        values = np.asarray(values, dtype=float)
        timeline.fill_between(times, y - 0.28, y + 0.28, where=values > 0,
                              step='post', color=color, alpha=0.88)
        timeline.plot(times, np.full_like(times, y), color='#D6DADF', linewidth=0.6)
    for index, frame in enumerate(selected):
        dt = frame['overlay']['stamp'] - t0
        timeline.axvline(dt, color='#343A40', linewidth=0.8, linestyle='--', alpha=0.75)
        timeline.text(dt, 3.52, panel_letters[index].upper(), ha='center', va='bottom',
                      fontsize=8.5, weight='bold', color='#202428')
    timeline.set_yticks(y_positions, [row[0] for row in signals])
    timeline.set_xlabel('time relative to first selected frame [s]')
    timeline.set_ylim(-0.65, 3.75)
    timeline.set_xlim(times.min(), times.max())
    timeline.grid(axis='x', color='#E0E3E5', linewidth=0.55)
    timeline.spines[['top', 'right', 'left']].set_visible(False)
    timeline.tick_params(axis='y', length=0)
    timeline.set_title('Recorded state timeline', loc='left', pad=8)

    output = Path(capture.args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'source': 'recorded_rosbag_replay',
        'bag': capture.args.bag_label,
        'sync_tolerance_sec': capture.args.sync_tolerance,
        'captured_overlay_count': len(capture.overlays),
        'captured_state_count': len(capture.states),
        'eligible_paired_frame_count': len(frames),
        'selected_frames': [],
    }
    for letter, title, frame in zip(panel_letters, panel_titles, selected):
        state = frame['state']
        metadata['selected_frames'].append({
            'panel': letter,
            'title': title,
            'stamp': frame['overlay']['stamp'],
            'relative_time_sec': frame['overlay']['stamp'] - t0,
            'source_state': SOURCE_NAMES.get(state['source_state'], str(state['source_state'])),
            'track_state': TRACK_NAMES.get(state['track_state'], str(state['track_state'])),
            'selected_tracklet_id': state['selected_tracklet_id'],
            'selected_tracklet_active': state['selected_active'],
            'distance_m': state['distance'],
            'overall_confidence': state['overall_confidence'],
            'overlay_state_offset_sec': state['overlay_offset_sec'],
            'overlay_tracklet_offset_sec': state['tracklet_offset_sec'],
        })
    metadata_output = Path(capture.args.metadata_output)
    metadata_output.parent.mkdir(parents=True, exist_ok=True)
    metadata_output.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    rclpy.init()
    capture = SequenceCapture(args)
    try:
        while rclpy.ok() and time.monotonic() < capture.deadline:
            rclpy.spin_once(capture, timeout_sec=0.2)
    finally:
        capture.destroy_node()
        rclpy.shutdown()
    frames = paired_frames(capture)
    selected = select_sequence(frames)
    render(capture, frames, selected)
    print(json.dumps({
        'overlays': len(capture.overlays),
        'states': len(capture.states),
        'paired': len(frames),
        'output': args.output,
    }))


if __name__ == '__main__':
    main()
