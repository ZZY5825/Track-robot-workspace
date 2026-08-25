#!/usr/bin/env python3
"""Capture an evidence-backed association candidate ranking from ROS replay."""

import argparse
from collections import deque
import json
import os
import time
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-association-ranking')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import rclpy
from std_msgs.msg import String
from track_robot_interfaces.msg import TargetState

from capture_camera_lidar_association import (
    AssociationCapture, pointcloud_xyz, quaternion_rotation)


COLORS = {
    'selected': '#CC3311',
    'valid': '#2A7F76',
    'rejected': '#9AA1A6',
    'anchor': '#EE7733',
    'gate': '#6F4C9B',
    'cloud': '#737B80',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    parser.add_argument('--bag-label', default='human_tracking_lidar_20260706_145900')
    parser.add_argument('--timeout', type=float, default=48.0)
    parser.add_argument('--state-sync-tolerance', type=float, default=0.12)
    parser.add_argument('--camera-cloud-tolerance', type=float, default=0.20)
    return parser.parse_args()


class RankingCapture(AssociationCapture):
    def __init__(self, args):
        self.debug_rows = deque(maxlen=300)
        self.latest_state_stamp = None
        self.best_snapshot = None
        self.best_quality = (-1, -1, -1)
        super().__init__(args)
        self.create_subscription(
            String, '/human_tracking/target_tracker_debug', self.on_debug, 20)

    def on_state(self, msg):
        self.latest_state_stamp = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9)
        super().on_state(msg)

    def on_debug(self, msg):
        if self.latest_state_stamp is None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.debug_rows.append((self.latest_state_stamp, payload))
        self.try_capture()

    def try_capture(self):
        previous = self.snapshot
        self.snapshot = None
        AssociationCapture.try_capture(self)
        candidate = self.snapshot
        self.snapshot = previous
        if candidate is None or not self.debug_rows:
            return
        state_stamp = candidate['state'][0]
        debug_row = min(self.debug_rows, key=lambda row: abs(row[0] - state_stamp))
        if abs(debug_row[0] - state_stamp) > self.args.state_sync_tolerance:
            return
        payload = debug_row[1]
        confirmed = sum(
            1 for tracklet in candidate['tracklets'][1].tracklets
            if tracklet.active and tracklet.confirmed)
        hypotheses = payload.get('hypotheses', [])
        quality = (len(hypotheses), confirmed, int(payload.get('candidate_count', 0)))
        if quality > self.best_quality:
            candidate['debug'] = debug_row
            self.best_snapshot = candidate
            self.best_quality = quality


def candidate_metrics(snapshot):
    state_stamp, state = snapshot['state']
    tracklet_stamp, tracklet_msg = snapshot['tracklets']
    _, guided_msg = snapshot['guided']
    _, debug = snapshot['debug']
    camera_target = snapshot['camera_target'][1]
    camera_info = snapshot['camera_info']

    anchor = np.median(pointcloud_xyz(guided_msg).astype(float), axis=0)
    anchor_range = float(np.linalg.norm(anchor[:2]))
    bbox = np.asarray(camera_target.bbox, dtype=float)
    bbox_center = np.asarray([
        0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3])])
    rotation = quaternion_rotation(0.5, -0.5, 0.5, 0.5)
    translation = np.asarray([0.06, -0.065, -0.26])
    fx, fy, cx, cy = (
        float(camera_info.p[0]), float(camera_info.p[5]),
        float(camera_info.p[2]), float(camera_info.p[6]))
    score_by_id = {
        int(row['id']): float(row['score'])
        for row in debug.get('hypotheses', [])
        if 'id' in row and 'score' in row
    }
    selected_id = int(state.selected_tracklet_id)
    if selected_id >= 0 and selected_id not in score_by_id:
        score_by_id[selected_id] = float(debug.get('selected_tracklet_score', 0.0))

    rows = []
    for tracklet in tracklet_msg.tracklets:
        if not tracklet.active or not tracklet.confirmed:
            continue
        position = np.asarray([
            tracklet.position.x, tracklet.position.y, tracklet.position.z], dtype=float)
        anchor_distance = float(np.linalg.norm(position[:2] - anchor[:2]))
        range_difference = abs(float(np.linalg.norm(position[:2])) - anchor_range)
        lidar_position = position - np.asarray([0.0, 0.0, 0.70])
        camera_position = rotation @ lidar_position + translation
        if camera_position[2] > 0.05:
            u = float(fx * camera_position[0] / camera_position[2] + cx)
            v = float(fy * camera_position[1] / camera_position[2] + cy)
            center_error = float(np.linalg.norm(np.asarray([u, v]) - bbox_center))
            projection_valid = True
        else:
            u, v, center_error, projection_valid = np.nan, np.nan, np.inf, False
        if anchor_distance > 1.0:
            reason = 'anchor XY gate'
        elif range_difference > 0.75:
            reason = 'range gate'
        elif not projection_valid:
            reason = 'invalid projection'
        elif int(tracklet.tracklet_id) not in score_by_id:
            reason = 'not ranked'
        else:
            reason = 'valid hypothesis'
        rows.append({
            'id': int(tracklet.tracklet_id),
            'selected': int(tracklet.tracklet_id) == selected_id,
            'position': position,
            'anchor_distance': anchor_distance,
            'range_difference': range_difference,
            'projection_center_error': center_error,
            'projected_uv': [u, v],
            'score': score_by_id.get(int(tracklet.tracklet_id)),
            'reason': reason,
        })
    rows.sort(key=lambda row: (
        row['score'] is not None, row['score'] or -1.0), reverse=True)
    return rows, anchor, debug, state_stamp, tracklet_stamp


def render(capture):
    snapshot = capture.best_snapshot
    rows, anchor, debug, state_stamp, tracklet_stamp = candidate_metrics(snapshot)
    selected_id = int(snapshot['state'][1].selected_tracklet_id)
    raw_points = pointcloud_xyz(snapshot['cloud'][1]).astype(float)
    base_points = raw_points + np.asarray([0.0, 0.0, 0.70])
    ranges = np.linalg.norm(base_points[:, :2], axis=1)
    mask = (
        (ranges >= 0.4) & (ranges <= 8.0) &
        (base_points[:, 2] >= -0.4) & (base_points[:, 2] <= 2.5))
    cloud = base_points[mask]
    if len(cloud) > 22000:
        cloud = cloud[::max(1, len(cloud) // 22000)]

    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.4,
        'axes.titlesize': 11.3, 'axes.labelsize': 9.4,
    })
    fig = plt.figure(figsize=(17.2, 6.2), facecolor='white')
    grid = fig.add_gridspec(2, 3, height_ratios=(8.1, 1.15), hspace=0.16, wspace=0.24)
    spatial = fig.add_subplot(grid[0, 0])
    gates = fig.add_subplot(grid[0, 1])
    ranking = fig.add_subplot(grid[0, 2])
    footer = fig.add_subplot(grid[1, :])

    spatial.scatter(cloud[:, 1], cloud[:, 0], s=1.0, c=COLORS['cloud'], alpha=0.20,
                    linewidths=0, rasterized=True)
    spatial.scatter([0], [0], marker='^', s=115, c='#30363B', edgecolor='white',
                    linewidth=0.8, zorder=8, label='robot')
    spatial.scatter([anchor[1]], [anchor[0]], marker='X', s=145,
                    c=COLORS['anchor'], edgecolor='white', linewidth=0.8,
                    zorder=10, label='camera-guided anchor')
    spatial.add_patch(Circle((anchor[1], anchor[0]), 1.0, fill=False,
                             edgecolor=COLORS['gate'], linestyle='--', linewidth=1.6,
                             label='1.0 m anchor gate'))
    for row in rows:
        color = COLORS['selected'] if row['selected'] else (
            COLORS['valid'] if row['score'] is not None else COLORS['rejected'])
        spatial.scatter([row['position'][1]], [row['position'][0]], s=130 if row['selected'] else 70,
                        facecolors='none', edgecolors=color,
                        linewidths=2.1 if row['selected'] else 1.3, zorder=9)
        spatial.text(row['position'][1] + 0.08, row['position'][0] + 0.08,
                     f"T{row['id']}", color=color, fontsize=8, weight='bold',
                     clip_on=True)
    spatial.set_xlim(-3.6, 3.6)
    spatial.set_ylim(-1.0, 6.4)
    spatial.set_aspect('equal', adjustable='box')
    spatial.set_xlabel('lateral position y [m]')
    spatial.set_ylabel('forward position x [m]')
    spatial.grid(True, color='#D9DDE0', linewidth=0.55)
    spatial.set_facecolor('#FAFAF9')
    spatial.set_title('(a) Spatial candidate set', loc='left')
    spatial.legend(loc='upper right', frameon=True, fontsize=7.5)

    for row in rows:
        color = COLORS['selected'] if row['selected'] else (
            COLORS['valid'] if row['score'] is not None else COLORS['rejected'])
        gates.scatter(row['anchor_distance'], row['range_difference'], s=125 if row['selected'] else 70,
                      facecolors=color if row['selected'] else 'white', edgecolors=color,
                      linewidths=1.8, zorder=5)
        gates.annotate(f"T{row['id']}", (row['anchor_distance'], row['range_difference']),
                       xytext=(5, 5), textcoords='offset points', color=color,
                       fontsize=8, weight='bold', annotation_clip=True)
    gates.axvline(1.0, color=COLORS['gate'], linestyle='--', linewidth=1.5)
    gates.axhline(0.75, color=COLORS['gate'], linestyle=':', linewidth=1.5)
    max_x = max([row['anchor_distance'] for row in rows] + [1.1])
    max_y = max([row['range_difference'] for row in rows] + [0.85])
    gates.set_xlim(-0.04, min(max(1.2, max_x * 1.12), 5.0))
    gates.set_ylim(-0.03, min(max(0.9, max_y * 1.15), 4.0))
    gates.set_xlabel('anchor distance [m]')
    gates.set_ylabel('range difference [m]')
    gates.grid(True, color='#E0E3E5', linewidth=0.55)
    gates.set_facecolor('#FAFAF9')
    gates.set_title('(b) Hard-gate rejection', loc='left')
    gates.text(0.03, 0.04, 'valid region', transform=gates.transAxes,
               ha='left', va='bottom', color=COLORS['gate'], fontsize=8.3, weight='bold')

    display_rows = rows[:10]
    y = np.arange(len(display_rows))
    values = [row['score'] if row['score'] is not None else 0.0 for row in display_rows]
    colors = [COLORS['selected'] if row['selected'] else (
        COLORS['valid'] if row['score'] is not None else COLORS['rejected']) for row in display_rows]
    ranking.barh(y, values, color=colors, height=0.62)
    ranking.set_yticks(y, [f"T{row['id']}" for row in display_rows])
    ranking.invert_yaxis()
    ranking.set_xlim(0, 1.02)
    ranking.axvline(0.65, color=COLORS['gate'], linestyle='--', linewidth=1.5,
                   label='minimum association score')
    for index, row in enumerate(display_rows):
        label = f"{row['score']:.3f}" if row['score'] is not None else row['reason']
        ranking.text(min(values[index] + 0.025, 0.86), index, label,
                     va='center', fontsize=8.0,
                     color='#202428' if row['score'] is not None else '#6B7277')
    ranking.set_xlabel('published association score')
    ranking.grid(True, axis='x', color='#E0E3E5', linewidth=0.55)
    ranking.set_facecolor('#FAFAF9')
    ranking.set_title('(c) Ranked valid hypotheses', loc='left')
    ranking.legend(loc='lower right', frameon=True, fontsize=7.6)

    selected = next(row for row in rows if row['id'] == selected_id)
    valid_count = sum(row['score'] is not None for row in rows)
    footer.axis('off')
    footer.axhline(0.98, color='#C8CDD1', linewidth=0.8)
    blocks = [
        ('RECORDED REPLAY', capture.args.bag_label),
        ('CANDIDATE SET', f'{len(rows)} confirmed · {valid_count} passed hard gates'),
        ('SELECTED HYPOTHESIS', f'T{selected_id} · score {selected["score"]:.3f}'),
        ('SELECTED CONSISTENCY',
         f'Δxy {selected["anchor_distance"]:.2f} m · Δrange {selected["range_difference"]:.2f} m'),
    ]
    for index, (heading, value) in enumerate(blocks):
        x = 0.01 + index * 0.25
        footer.text(x, 0.66, heading, transform=footer.transAxes, fontsize=8,
                    weight='bold', color='#5B6268')
        footer.text(x, 0.22, value, transform=footer.transAxes, fontsize=8.9,
                    weight='bold', color=COLORS['selected'] if index == 2 else '#202428')

    output = Path(capture.args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'source': 'recorded_rosbag_replay',
        'bag': capture.args.bag_label,
        'target_state_stamp': state_stamp,
        'tracklet_stamp': tracklet_stamp,
        'selected_tracklet_id': selected_id,
        'published_selected_score': selected['score'],
        'published_hypotheses': debug.get('hypotheses', []),
        'association_ambiguous': bool(debug.get('association_ambiguous', False)),
        'gates': {
            'initial_anchor_xy_gate_m': 1.0,
            'initial_anchor_range_gate_m': 0.75,
            'minimum_association_score': 0.65,
        },
        'candidates': [{
            key: (value.tolist() if isinstance(value, np.ndarray) else value)
            for key, value in row.items()
        } for row in rows],
        'note': 'Scores are copied from /human_tracking/target_tracker_debug; gate metrics are recomputed from synchronized messages.',
    }
    Path(capture.args.metadata_output).write_text(
        json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    rclpy.init()
    capture = RankingCapture(args)
    try:
        while rclpy.ok() and time.monotonic() < capture.deadline:
            rclpy.spin_once(capture, timeout_sec=0.2)
    finally:
        capture.destroy_node()
        rclpy.shutdown()
    if capture.best_snapshot is None:
        raise SystemExit('No synchronized confirmed association candidate set captured')
    render(capture)
    print(json.dumps({'output': args.output, 'quality': capture.best_quality}))


if __name__ == '__main__':
    main()
