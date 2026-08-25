#!/usr/bin/env python3
"""Capture a publication-style camera--LiDAR association figure from ROS replay."""

import argparse
from collections import deque
import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-camera-lidar-association')

import cv2
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from track_robot_interfaces.msg import CameraTarget, LidarTrackletArray, TargetState


COLORS = {
    'all': '#737B80',
    'roi': '#4477AA',
    'guided': '#EE7733',
    'selected': '#CC3311',
    'other': '#228833',
    'gate': '#AA3377',
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


def quaternion_rotation(x, y, z, w):
    quaternion = np.asarray([w, x, y, z], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    return np.asarray([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def nearest(rows, stamp, tolerance):
    if not rows:
        return None
    candidate = min(rows, key=lambda row: abs(row[0] - stamp))
    return candidate if abs(candidate[0] - stamp) <= tolerance else None


def expand_roi(xs, ys, bbox, x_margin_fraction, y_margin_fraction):
    x1, y1, x2, y2 = bbox
    width = max(1.0, x2 - x1)
    height = max(1.0, y2 - y1)
    return np.asarray([
        max(x1, min(xs) - x_margin_fraction * width),
        max(y1, min(ys) - y_margin_fraction * height),
        min(x2, max(xs) + x_margin_fraction * width),
        min(y2, max(ys) + y_margin_fraction * height),
    ], dtype=float)


def target_body_roi(camera_target, min_confidence=0.35):
    bbox = np.asarray(camera_target.bbox, dtype=float)
    keypoints = list(camera_target.keypoints)

    def valid(index):
        offset = index * 3
        return len(keypoints) > offset + 2 and keypoints[offset + 2] >= min_confidence

    if valid(5) and valid(6) and valid(11) and valid(12):
        return expand_roi(
            [keypoints[15], keypoints[18], keypoints[33], keypoints[36]],
            [keypoints[16], keypoints[19], keypoints[34], keypoints[37]],
            bbox, 0.30, 0.20), 'skeleton torso ROI'
    if valid(5) and valid(6):
        y_mid = 0.5 * (keypoints[16] + keypoints[19])
        return expand_roi(
            [keypoints[15], keypoints[18]],
            [y_mid, bbox[1] + 0.62 * (bbox[3] - bbox[1])],
            bbox, 0.45, 0.15), 'skeleton upper-body ROI'
    width = max(1.0, bbox[2] - bbox[0])
    height = max(1.0, bbox[3] - bbox[1])
    center_x = 0.5 * (bbox[0] + bbox[2])
    half_width = 0.25 * width
    return np.asarray([
        center_x - half_width,
        bbox[1] + 0.20 * height,
        center_x + half_width,
        bbox[1] + 0.70 * height,
    ]), 'central bounding-box ROI'


class AssociationCapture(Node):
    def __init__(self, args):
        super().__init__('camera_lidar_association_capture')
        self.args = args
        self.bridge = CvBridge()
        self.deadline = time.monotonic() + args.timeout
        self.images = deque(maxlen=40)
        self.clouds = deque(maxlen=16)
        self.guided_clouds = deque(maxlen=16)
        self.tracklets = deque(maxlen=40)
        self.states = deque(maxlen=120)
        self.camera_targets = deque(maxlen=40)
        self.camera_info = None
        self.snapshot = None

        self.create_subscription(
            Image, '/zed/zed_node/left/image_rect_color', self.on_image,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/rslidar_points', self.on_cloud,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/human_tracking/camera_guided_target_points',
            self.on_guided_cloud, qos_profile_sensor_data)
        self.create_subscription(
            LidarTrackletArray, '/human_tracking/lidar_tracklets',
            self.on_tracklets, 10)
        self.create_subscription(
            TargetState, '/human_tracking/fused_target_state', self.on_state, 10)
        self.create_subscription(
            CameraTarget, '/human_tracking/camera_target', self.on_camera_target, 10)
        self.create_subscription(
            CameraInfo, '/zed/zed_node/left/camera_info', self.on_camera_info,
            qos_profile_sensor_data)

    def on_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 94])
        if ok:
            self.images.append((stamp_seconds(msg.header.stamp), encoded))
        self.try_capture()

    def on_cloud(self, msg):
        self.clouds.append((stamp_seconds(msg.header.stamp), msg))
        self.try_capture()

    def on_guided_cloud(self, msg):
        self.guided_clouds.append((stamp_seconds(msg.header.stamp), msg))
        self.try_capture()

    def on_tracklets(self, msg):
        self.tracklets.append((stamp_seconds(msg.header.stamp), msg))
        self.try_capture()

    def on_state(self, msg):
        self.states.append((stamp_seconds(msg.header.stamp), msg))
        self.try_capture()

    def on_camera_target(self, msg):
        self.camera_targets.append((stamp_seconds(msg.header.stamp), msg))
        self.try_capture()

    def on_camera_info(self, msg):
        self.camera_info = msg
        self.try_capture()

    def try_capture(self):
        if self.snapshot is not None or self.camera_info is None:
            return
        for state_row in reversed(self.states):
            state_stamp, state = state_row
            if not (
                state.camera_visible and state.lidar_visible and
                state.lock_state == TargetState.LOCK_TARGET_LOCKED and
                state.association_state == TargetState.ASSOCIATION_CONFIRMED and
                state.selected_tracklet_id >= 0
            ):
                continue
            tracklet_row = nearest(
                self.tracklets, state_stamp, self.args.state_sync_tolerance)
            cloud_row = nearest(
                self.clouds, state_stamp, self.args.state_sync_tolerance)
            guided_row = nearest(
                self.guided_clouds, state_stamp, self.args.state_sync_tolerance)
            camera_target_row = nearest(
                self.camera_targets, state_stamp, self.args.camera_cloud_tolerance)
            if None in (tracklet_row, cloud_row, guided_row, camera_target_row):
                continue
            camera_target_stamp, camera_target = camera_target_row
            image_row = nearest(self.images, camera_target_stamp, 0.06)
            if image_row is None:
                continue
            selected = next((
                tracklet for tracklet in tracklet_row[1].tracklets
                if tracklet.active and tracklet.confirmed and
                tracklet.tracklet_id == state.selected_tracklet_id
            ), None)
            if selected is None:
                continue
            guided_points = pointcloud_xyz(guided_row[1])
            if len(guided_points) < 3:
                continue
            self.snapshot = {
                'state': state_row,
                'tracklets': tracklet_row,
                'cloud': cloud_row,
                'guided': guided_row,
                'camera_target': camera_target_row,
                'image': image_row,
                'selected': selected,
                'camera_info': self.camera_info,
            }
            return


def rectangle(axis, box, color, label, linestyle='-', linewidth=1.7):
    x1, y1, x2, y2 = box
    patch = Rectangle(
        (x1, y1), x2 - x1, y2 - y1,
        fill=False, edgecolor=color, linewidth=linewidth,
        linestyle=linestyle, label=label)
    axis.add_patch(patch)
    return patch


def render(capture):
    snapshot = capture.snapshot
    state_stamp, state = snapshot['state']
    image_stamp, encoded_image = snapshot['image']
    cloud_stamp, cloud_msg = snapshot['cloud']
    guided_stamp, guided_msg = snapshot['guided']
    tracklet_stamp, tracklet_msg = snapshot['tracklets']
    camera_target_stamp, camera_target = snapshot['camera_target']
    selected = snapshot['selected']
    camera_info = snapshot['camera_info']

    image = cv2.imdecode(encoded_image, cv2.IMREAD_COLOR)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    height, width = image.shape[:2]
    raw_lidar = pointcloud_xyz(cloud_msg).astype(float)
    guided_base = pointcloud_xyz(guided_msg).astype(float)

    base_points = raw_lidar + np.asarray([0.0, 0.0, 0.70])
    rotation = quaternion_rotation(0.5, -0.5, 0.5, 0.5)
    translation = np.asarray([0.06, -0.065, -0.26])
    camera_points = raw_lidar @ rotation.T + translation
    guided_lidar = guided_base - np.asarray([0.0, 0.0, 0.70])
    guided_camera = guided_lidar @ rotation.T + translation

    fx, fy, cx, cy = (
        float(camera_info.p[0]), float(camera_info.p[5]),
        float(camera_info.p[2]), float(camera_info.p[6]))

    def project(points):
        depth = points[:, 2]
        u = fx * points[:, 0] / np.maximum(depth, 0.05) + cx
        v = fy * points[:, 1] / np.maximum(depth, 0.05) + cy
        return u, v, depth

    u, v, depth = project(camera_points)
    in_image = (
        (depth > 0.05) & (u >= 0) & (u < width) & (v >= 0) & (v < height))
    bbox = np.asarray(camera_target.bbox, dtype=float)
    roi, roi_name = target_body_roi(camera_target)
    in_roi = in_image & (
        (u >= roi[0]) & (u <= roi[2]) & (v >= roi[1]) & (v <= roi[3]))

    minimum = np.asarray([
        selected.minimum.x, selected.minimum.y, selected.minimum.z], dtype=float)
    maximum = np.asarray([
        selected.maximum.x, selected.maximum.y, selected.maximum.z], dtype=float)
    in_selected_bounds = np.all(
        (base_points >= minimum - 1e-4) & (base_points <= maximum + 1e-4), axis=1)
    selected_projected = in_image & in_selected_bounds

    guided_u, guided_v, guided_depth = project(guided_camera)
    guided_visible = (
        (guided_depth > 0.05) &
        (guided_u >= 0) & (guided_u < width) &
        (guided_v >= 0) & (guided_v < height))
    anchor = np.median(guided_base, axis=0)
    selected_position = np.asarray([
        selected.position.x, selected.position.y, selected.position.z], dtype=float)
    selected_lidar = selected_position - np.asarray([0.0, 0.0, 0.70])
    selected_camera = rotation @ selected_lidar + translation
    centroid_u = fx * selected_camera[0] / selected_camera[2] + cx
    centroid_v = fy * selected_camera[1] / selected_camera[2] + cy
    bbox_center = np.asarray([0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3])])
    center_error = float(np.linalg.norm(np.asarray([centroid_u, centroid_v]) - bbox_center))
    delta_xy = float(np.linalg.norm(selected_position[:2] - anchor[:2]))
    anchor_range = float(np.linalg.norm(anchor[:2]))
    selected_range = float(np.linalg.norm(selected_position[:2]))
    delta_range = abs(selected_range - anchor_range)

    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 9.5,
        'axes.titlesize': 11.5,
        'axes.labelsize': 9.5,
    })
    fig = plt.figure(figsize=(17.4, 6.2), facecolor='white')
    grid = fig.add_gridspec(2, 3, height_ratios=(8.0, 1.25), hspace=0.13, wspace=0.10)
    detector_axis = fig.add_subplot(grid[0, 0])
    projection_axis = fig.add_subplot(grid[0, 1])
    topdown_axis = fig.add_subplot(grid[0, 2])
    footer_axis = fig.add_subplot(grid[1, :])

    detector_axis.imshow(image_rgb)
    rectangle(detector_axis, bbox, COLORS['selected'], 'camera target', linewidth=2.0)
    rectangle(detector_axis, roi, COLORS['roi'], roi_name, linestyle='--', linewidth=2.0)
    detector_axis.set_title('(a) Camera target and body-constrained ROI', loc='left')
    detector_axis.axis('off')
    detector_axis.legend(loc='lower left', frameon=True, fontsize=8.2)

    projection_axis.imshow(image_rgb)
    all_indices = np.flatnonzero(in_image)
    if len(all_indices) > 18000:
        all_indices = all_indices[::max(1, len(all_indices) // 18000)]
    projection_axis.scatter(
        u[all_indices], v[all_indices], s=2.2, color=COLORS['all'],
        alpha=0.24, linewidths=0, label='all projected LiDAR points')
    projection_axis.scatter(
        u[in_roi], v[in_roi], s=4.0, color=COLORS['roi'],
        alpha=0.58, linewidths=0, label='points inside body ROI')
    projection_axis.scatter(
        guided_u[guided_visible], guided_v[guided_visible], s=12,
        facecolors='none', edgecolors=COLORS['guided'], linewidths=0.9,
        label='camera-guided depth mode')
    projection_axis.scatter(
        u[selected_projected], v[selected_projected], s=5.0,
        color=COLORS['selected'], alpha=0.58, linewidths=0,
        label=f'points inside selected T{selected.tracklet_id} bounds')
    projection_axis.scatter(
        [centroid_u], [centroid_v], marker='x', s=100,
        color=COLORS['selected'], linewidths=2.2,
        label=f'projected T{selected.tracklet_id} centroid')
    rectangle(projection_axis, bbox, COLORS['selected'], '_camera target', linewidth=1.7)
    rectangle(projection_axis, roi, COLORS['roi'], '_body ROI', linestyle='--', linewidth=1.7)
    projection_axis.add_patch(Circle(
        bbox_center, 220.0, fill=False, edgecolor=COLORS['gate'],
        linestyle=':', linewidth=1.4, label='projection center gate'))
    projection_axis.set_xlim(0, width)
    projection_axis.set_ylim(height, 0)
    projection_axis.set_title('(b) Image-plane LiDAR association evidence', loc='left')
    projection_axis.axis('off')
    projection_axis.legend(loc='lower left', frameon=True, fontsize=7.4)

    horizontal_range = np.linalg.norm(base_points[:, :2], axis=1)
    map_mask = (
        (horizontal_range >= 0.4) & (horizontal_range <= 10.0) &
        (base_points[:, 2] >= -0.4) & (base_points[:, 2] <= 2.5))
    map_points = base_points[map_mask]
    if len(map_points) > 26000:
        map_points = map_points[::max(1, len(map_points) // 26000)]
    topdown_axis.scatter(
        map_points[:, 1], map_points[:, 0], s=1.0,
        color=COLORS['all'], alpha=0.23, linewidths=0, rasterized=True)
    topdown_axis.scatter(
        [0], [0], marker='^', s=115, color='#30363B', edgecolor='white',
        linewidth=0.8, zorder=8, label='robot')
    topdown_axis.add_patch(Circle(
        (anchor[1], anchor[0]), 1.0, fill=False,
        edgecolor=COLORS['gate'], linestyle='--', linewidth=1.7,
        label='anchor XY gate (1.0 m)'))
    for radius in (max(0.05, anchor_range - 0.75), anchor_range + 0.75):
        topdown_axis.add_patch(Circle(
            (0, 0), radius, fill=False, edgecolor=COLORS['gate'],
            linestyle=':', linewidth=1.1, alpha=0.9))
    topdown_axis.scatter(
        [anchor[1]], [anchor[0]], marker='X', s=135,
        color=COLORS['guided'], edgecolor='white', linewidth=0.8,
        zorder=10, label='camera-guided anchor')
    for tracklet in tracklet_msg.tracklets:
        if not tracklet.active or not tracklet.confirmed:
            continue
        if not (-3.6 <= tracklet.position.y <= 3.6 and
                -1.0 <= tracklet.position.x <= 6.4):
            continue
        is_selected = tracklet.tracklet_id == selected.tracklet_id
        color = COLORS['selected'] if is_selected else COLORS['other']
        size = 145 if is_selected else 75
        linewidth = 2.2 if is_selected else 1.2
        topdown_axis.scatter(
            [tracklet.position.y], [tracklet.position.x], s=size,
            facecolors='none', edgecolors=color, linewidths=linewidth, zorder=9)
        topdown_axis.text(
            tracklet.position.y + 0.09, tracklet.position.x + 0.09,
            f'T{tracklet.tracklet_id}', color=color, fontsize=7.6,
            weight='bold', zorder=10, clip_on=True)
    topdown_axis.add_patch(Rectangle(
        (minimum[1], minimum[0]), maximum[1] - minimum[1], maximum[0] - minimum[0],
        fill=False, edgecolor=COLORS['selected'], linewidth=1.7,
        label=f'selected T{selected.tracklet_id} bounds'))
    topdown_axis.plot(
        [anchor[1], selected_position[1]], [anchor[0], selected_position[0]],
        color=COLORS['selected'], linewidth=1.5, linestyle='-')
    topdown_axis.set_xlim(-3.6, 3.6)
    topdown_axis.set_ylim(-1.0, 6.4)
    topdown_axis.set_aspect('equal', adjustable='box')
    topdown_axis.set_xlabel('lateral position y [m]')
    topdown_axis.set_ylabel('forward position x [m]')
    topdown_axis.grid(True, color='#D9DDE0', linewidth=0.55)
    topdown_axis.set_facecolor('#FAFAF9')
    topdown_axis.set_title('(c) Metric association gates in base frame', loc='left')
    topdown_axis.legend(loc='upper right', frameon=True, fontsize=7.3)

    footer_axis.axis('off')
    footer_axis.axhline(0.98, color='#C8CDD1', linewidth=0.8)
    footer_axis.text(
        0.01, 0.67, 'RECORDED REPLAY', transform=footer_axis.transAxes,
        fontsize=8, weight='bold', color='#5B6268')
    footer_axis.text(
        0.01, 0.27, capture.args.bag_label, transform=footer_axis.transAxes,
        fontsize=8.8, weight='bold', color='#202428')
    footer_axis.text(
        0.28, 0.67, 'CONFIRMED ASSOCIATION', transform=footer_axis.transAxes,
        fontsize=8, weight='bold', color='#5B6268')
    footer_axis.text(
        0.28, 0.27, f'T{selected.tracklet_id}  ·  range {state.distance:.2f} m',
        transform=footer_axis.transAxes, fontsize=9.2, weight='bold',
        color=COLORS['selected'])
    footer_axis.text(
        0.52, 0.67, 'METRIC CONSISTENCY', transform=footer_axis.transAxes,
        fontsize=8, weight='bold', color='#5B6268')
    footer_axis.text(
        0.52, 0.27, f'Δxy {delta_xy:.2f} m  ·  Δrange {delta_range:.2f} m',
        transform=footer_axis.transAxes, fontsize=9.2, weight='bold', color='#202428')
    footer_axis.text(
        0.76, 0.67, 'IMAGE CONSISTENCY', transform=footer_axis.transAxes,
        fontsize=8, weight='bold', color='#5B6268')
    footer_axis.text(
        0.76, 0.27, f'centroid error {center_error:.0f} px  ·  ROI {len(guided_base)} points',
        transform=footer_axis.transAxes, fontsize=9.2, weight='bold', color='#202428')

    output = Path(capture.args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    stamps = {
        'raw_image': image_stamp,
        'camera_target': camera_target_stamp,
        'raw_cloud': cloud_stamp,
        'camera_guided_cloud': guided_stamp,
        'tracklets': tracklet_stamp,
        'target_state': state_stamp,
    }
    metadata = {
        'source': 'recorded_rosbag_replay',
        'bag': capture.args.bag_label,
        'stamps': stamps,
        'state_group_spread_sec': max(
            cloud_stamp, guided_stamp, tracklet_stamp, state_stamp) - min(
            cloud_stamp, guided_stamp, tracklet_stamp, state_stamp),
        'image_camera_target_offset_sec': abs(image_stamp - camera_target_stamp),
        'camera_target_cloud_offset_sec': abs(camera_target_stamp - cloud_stamp),
        'selected_tracklet_id': int(selected.tracklet_id),
        'selected_tracklet_active': bool(selected.active),
        'association_state': 'CONFIRMED',
        'target_range_m': float(state.distance),
        'anchor_base_xyz_m': anchor.tolist(),
        'selected_centroid_base_xyz_m': selected_position.tolist(),
        'anchor_tracklet_delta_xy_m': delta_xy,
        'anchor_tracklet_delta_range_m': delta_range,
        'projected_centroid_error_px': center_error,
        'raw_projected_point_count': int(np.count_nonzero(in_image)),
        'body_roi_projected_point_count': int(np.count_nonzero(in_roi)),
        'camera_guided_point_count': int(len(guided_base)),
        'selected_bounds_projected_point_count': int(np.count_nonzero(selected_projected)),
        'body_roi_type': roi_name,
        'gates': {
            'initial_anchor_xy_gate_m': 1.0,
            'initial_anchor_range_gate_m': 0.75,
            'max_projection_center_error_px': 220.0,
        },
        'direct_lidar_to_camera_optical_transform': {
            'translation_xyz_m': translation.tolist(),
            'quaternion_xyzw': [0.5, -0.5, 0.5, 0.5],
        },
    }
    metadata_output = Path(capture.args.metadata_output)
    metadata_output.parent.mkdir(parents=True, exist_ok=True)
    metadata_output.write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    rclpy.init()
    capture = AssociationCapture(args)
    try:
        while rclpy.ok() and time.monotonic() < capture.deadline and capture.snapshot is None:
            rclpy.spin_once(capture, timeout_sec=0.2)
    finally:
        capture.destroy_node()
        rclpy.shutdown()
    if capture.snapshot is None:
        raise SystemExit('No confirmed synchronized camera--LiDAR association captured')
    render(capture)
    print(json.dumps({
        'output': args.output,
        'selected_tracklet_id': int(capture.snapshot['selected'].tracklet_id),
    }))


if __name__ == '__main__':
    main()
