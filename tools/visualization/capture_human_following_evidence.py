#!/usr/bin/env python3
"""Capture a publication-ready human-tracking replay evidence figure."""

import argparse
import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-human-following-evidence')

import cv2
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from cv_bridge import CvBridge
from matplotlib.patches import Ellipse
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from track_robot_interfaces.msg import CameraTarget, LidarTrackletArray, TargetState


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


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    parser.add_argument('--bag-label', default='human_tracking_lidar_20260706_145900')
    parser.add_argument('--timeout', type=float, default=75.0)
    parser.add_argument('--settle-time', type=float, default=1.0)
    parser.add_argument('--allow-unbound', action='store_true')
    return parser.parse_args()


def pointcloud_xyz(msg):
    field_by_name = {field.name: field for field in msg.fields}
    if not all(name in field_by_name for name in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    formats = []
    offsets = []
    for name in ('x', 'y', 'z'):
        field = field_by_name[name]
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
    count = int(msg.width) * int(msg.height)
    records = np.frombuffer(msg.data, dtype=dtype, count=count)
    points = np.column_stack((records['x'], records['y'], records['z']))
    return points[np.isfinite(points).all(axis=1)]


class EvidenceCapture(Node):
    def __init__(self, args):
        super().__init__('human_following_evidence_capture')
        self.args = args
        self.bridge = CvBridge()
        self.overlay = None
        self.cloud = None
        self.tracklets = None
        self.target = None
        self.camera_target = None
        self.first_ready_time = None
        self.captured = False
        self.deadline = time.monotonic() + args.timeout

        self.create_subscription(
            Image, '/human_tracking/target_overlay', self.on_overlay,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/rslidar_points', self.on_cloud,
            qos_profile_sensor_data)
        self.create_subscription(
            LidarTrackletArray, '/human_tracking/lidar_tracklets',
            self.on_tracklets, 10)
        self.create_subscription(
            TargetState, '/human_tracking/fused_target_state', self.on_target, 10)
        self.create_subscription(
            CameraTarget, '/human_tracking/camera_target', self.on_camera_target, 10)

    def on_overlay(self, msg):
        self.overlay = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.maybe_capture()

    def on_cloud(self, msg):
        self.cloud = pointcloud_xyz(msg)
        self.maybe_capture()

    def on_tracklets(self, msg):
        self.tracklets = msg
        self.maybe_capture()

    def on_target(self, msg):
        self.target = msg
        self.maybe_capture()

    def on_camera_target(self, msg):
        self.camera_target = msg

    def ready(self):
        if self.overlay is None or self.cloud is None or self.tracklets is None:
            return False
        if self.target is None or not self.target.camera_visible:
            return False
        if self.target.lock_state != TargetState.LOCK_TARGET_LOCKED:
            return False
        if not self.args.allow_unbound:
            return self.target.lidar_visible and self.target.selected_tracklet_id >= 0
        return any(tracklet.active for tracklet in self.tracklets.tracklets)

    def maybe_capture(self):
        if self.captured or not self.ready():
            self.first_ready_time = None
            return
        if self.first_ready_time is None:
            self.first_ready_time = time.monotonic()
            return
        if time.monotonic() - self.first_ready_time < self.args.settle_time:
            return
        self.render()
        self.captured = True

    def render(self):
        target = self.target
        points = self.cloud
        horizontal_range = np.hypot(points[:, 0], points[:, 1])
        visible = (
            (horizontal_range >= 0.4) & (horizontal_range <= 10.0) &
            (points[:, 2] >= -0.4) & (points[:, 2] <= 2.5)
        )
        points = points[visible]
        if len(points) > 28000:
            stride = max(1, len(points) // 28000)
            points = points[::stride]

        plt.rcParams.update({
            'font.family': 'DejaVu Sans',
            'font.size': 10,
            'axes.titlesize': 12,
            'axes.labelsize': 9,
        })
        fig = plt.figure(figsize=(16, 8.5), facecolor='white')
        grid = fig.add_gridspec(
            2, 2, height_ratios=(8.4, 1.6), width_ratios=(1.18, 1.0),
            hspace=0.08, wspace=0.10)
        camera_ax = fig.add_subplot(grid[0, 0])
        lidar_ax = fig.add_subplot(grid[0, 1])
        status_ax = fig.add_subplot(grid[1, :])

        camera_ax.imshow(cv2.cvtColor(self.overlay, cv2.COLOR_BGR2RGB))
        camera_ax.set_title('(a) Camera identity and gesture-authorized target lock', loc='left')
        camera_ax.axis('off')

        if len(points):
            lidar_ax.scatter(
                points[:, 1], points[:, 0], c=points[:, 2], cmap='cividis',
                s=1.0, alpha=0.42, linewidths=0, rasterized=True)
        lidar_ax.scatter(
            [0.0], [0.0], marker='^', s=115, color='#30363B',
            edgecolor='white', linewidth=0.8, zorder=8, label='robot')
        lidar_ax.annotate(
            'forward', xy=(0, 1.0), xytext=(0, 0.25), ha='center',
            arrowprops={'arrowstyle': '->', 'color': '#30363B', 'lw': 1.0},
            fontsize=8, color='#30363B')

        active_tracklets = [t for t in self.tracklets.tracklets if t.active]
        selected = None
        for tracklet in active_tracklets:
            color = '#2A7F76'
            linewidth = 1.2
            size = 105
            if tracklet.tracklet_id == target.selected_tracklet_id:
                selected = tracklet
                color = '#B83A3A'
                linewidth = 2.4
                size = 170
            lidar_ax.scatter(
                [tracklet.position.y], [tracklet.position.x], s=size,
                facecolors='none', edgecolors=color, linewidths=linewidth,
                zorder=9)
            lidar_ax.text(
                tracklet.position.y + 0.10, tracklet.position.x + 0.10,
                f'T{tracklet.tracklet_id}', color=color, fontsize=7.5,
                weight='bold', zorder=10)

        if target.position_base_valid:
            marker_color = '#B83A3A' if selected is not None else '#A36D1D'
            marker_label = 'selected fused target' if selected is not None else 'predicted target state'
            lidar_ax.scatter(
                [target.position_base.y], [target.position_base.x], marker='D',
                s=90, color=marker_color, edgecolor='white', linewidth=0.8,
                zorder=11, label=marker_label)
            lidar_ax.plot(
                [0.0, target.position_base.y], [0.0, target.position_base.x],
                color=marker_color, linestyle='--', linewidth=1.0, alpha=0.85)
            if selected is not None:
                covariance = np.asarray(target.position_covariance, dtype=float).reshape(3, 3)
                sigma_y = math.sqrt(max(0.0, covariance[1, 1]))
                sigma_x = math.sqrt(max(0.0, covariance[0, 0]))
                lidar_ax.add_patch(Ellipse(
                    (target.position_base.y, target.position_base.x),
                    width=4.0 * sigma_y, height=4.0 * sigma_x,
                    fill=False, color=marker_color, linestyle='--', linewidth=1.0))

        lidar_ax.set_title('(b) LiDAR geometry and target-association state', loc='left')
        lidar_ax.set_xlabel('lateral position y [m]')
        lidar_ax.set_ylabel('forward position x [m]')
        lidar_ax.set_xlim(-7.0, 7.0)
        lidar_ax.set_ylim(-7.0, 9.0)
        lidar_ax.set_aspect('equal', adjustable='box')
        lidar_ax.grid(True, color='#D9DDE0', linewidth=0.55)
        lidar_ax.set_facecolor('#FAFAF9')
        lidar_ax.legend(loc='upper right', frameon=True, fontsize=8)

        source_name = SOURCE_NAMES.get(target.source_state, str(target.source_state))
        track_name = TRACK_NAMES.get(target.track_state, str(target.track_state))
        association_name = ASSOCIATION_NAMES.get(
            target.association_state, str(target.association_state))
        bound = target.selected_tracklet_id >= 0 and target.lidar_visible
        association_color = '#2A7F76' if bound else '#A36D1D'

        status_ax.axis('off')
        status_ax.axhline(0.98, color='#C8CDD1', linewidth=0.8)
        status_ax.text(
            0.01, 0.72, 'RECORDED REPLAY', transform=status_ax.transAxes,
            fontsize=8, weight='bold', color='#5B6268')
        status_ax.text(
            0.01, 0.41, self.args.bag_label, transform=status_ax.transAxes,
            fontsize=8.6, weight='bold', color='#202428')
        status_ax.text(
            0.23, 0.66, 'CAMERA LOCK', transform=status_ax.transAxes,
            fontsize=8, weight='bold', color='#5B6268')
        status_ax.text(
            0.23, 0.34, LOCK_NAMES.get(target.lock_state, str(target.lock_state)),
            transform=status_ax.transAxes, fontsize=9.2, weight='bold', color='#2A7F76')
        status_ax.text(
            0.38, 0.66, 'TRACKING STATE', transform=status_ax.transAxes,
            fontsize=8, weight='bold', color='#5B6268')
        status_ax.text(
            0.38, 0.34, f'{source_name}\n{track_name}',
            transform=status_ax.transAxes, fontsize=8.8, weight='bold',
            color=association_color)
        status_ax.text(
            0.59, 0.66, 'ASSOCIATION', transform=status_ax.transAxes,
            fontsize=8, weight='bold', color='#5B6268')
        tracklet_label = str(target.selected_tracklet_id) if bound else 'none'
        status_ax.text(
            0.59, 0.34, f'{association_name}\nselected tracklet: {tracklet_label}',
            transform=status_ax.transAxes, fontsize=8.8, weight='bold',
            color=association_color)
        status_ax.text(
            0.78, 0.66, 'CONFIDENCE / RANGE', transform=status_ax.transAxes,
            fontsize=8, weight='bold', color='#5B6268')
        range_label = f'{target.distance:.2f} m' if target.position_base_valid else 'n/a'
        status_ax.text(
            0.78, 0.34,
            f'id {target.identity_confidence:.2f} · geom {target.geometry_confidence:.2f}\nrange {range_label}',
            transform=status_ax.transAxes, fontsize=8.8, weight='bold',
            color='#202428')
        if not bound:
            status_ax.text(
                0.01, 0.04,
                'Evidence note: the current replay produced a valid camera lock and LiDAR tracklets, '
                f'but no confirmed camera–LiDAR binding; the target state shown is {track_name.lower().replace("_", "-")}.',
                transform=status_ax.transAxes, fontsize=8.5, color='#87514F')

        output = Path(self.args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=180, bbox_inches='tight', facecolor='white')
        fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
        plt.close(fig)

        metadata = {
            'source': 'recorded_rosbag_replay',
            'bag': self.args.bag_label,
            'topics': {
                'camera_overlay': '/human_tracking/target_overlay',
                'point_cloud': '/rslidar_points',
                'tracklets': '/human_tracking/lidar_tracklets',
                'target_state': '/human_tracking/fused_target_state',
            },
            'frame_id': target.header.frame_id,
            'stamp': {
                'sec': int(target.header.stamp.sec),
                'nanosec': int(target.header.stamp.nanosec),
            },
            'target_id': int(target.target_id),
            'lock_state': LOCK_NAMES.get(target.lock_state, str(target.lock_state)),
            'source_state': source_name,
            'track_state': track_name,
            'association_state': association_name,
            'selected_tracklet_id': int(target.selected_tracklet_id),
            'camera_visible': bool(target.camera_visible),
            'lidar_visible': bool(target.lidar_visible),
            'position_base_valid': bool(target.position_base_valid),
            'distance_m': float(target.distance),
            'bearing_rad': float(target.bearing),
            'identity_confidence': float(target.identity_confidence),
            'geometry_confidence': float(target.geometry_confidence),
            'overall_confidence': float(target.overall_confidence),
            'active_tracklet_count': len(active_tracklets),
            'confirmed_camera_lidar_binding': bool(bound),
            'figure_note': (
                'Confirmed camera-LiDAR binding.' if bound else
                'Camera lock and LiDAR tracklets are real replay outputs; '
                'no confirmed camera-LiDAR binding was produced by this replay.'
            ),
        }
        metadata_output = Path(self.args.metadata_output)
        metadata_output.parent.mkdir(parents=True, exist_ok=True)
        metadata_output.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    rclpy.init()
    node = EvidenceCapture(args)
    try:
        while rclpy.ok() and time.monotonic() < node.deadline and not node.captured:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not node.captured:
        raise SystemExit('No eligible synchronized target frame captured before timeout')


if __name__ == '__main__':
    main()
