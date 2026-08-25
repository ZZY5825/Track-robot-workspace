#!/usr/bin/env python3
"""Render five publication figures from extended human-following evidence."""

import argparse
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-human-following-extended')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from human_following_3d_geometry import build_visual_scene, convex_outer_surface
from human_following_extended_analysis import (
    build_continuity_lanes,
    run_estimator_ablation,
    sample_urdf_workspace,
)


COLORS = {
    'ink': '#202428', 'muted': '#626A70', 'line': '#C8CDD1',
    'paper': '#FAFAF9', 'blue': '#4477AA', 'green': '#2A7F76',
    'orange': '#EE7733', 'purple': '#6F4C9B', 'red': '#CC3311',
    'yellow': '#D89B2B', 'gray': '#A8AFB4', 'light': '#E8EBED',
}
CONDITION_COLORS = {
    'camera_guided_anchor_only': COLORS['orange'],
    'selected_lidar_only': COLORS['blue'],
    'cv_kf_fusion': COLORS['green'],
    'production_imm': COLORS['purple'],
}
CONDITION_LABELS = {
    'camera_guided_anchor_only': 'camera-guided\nanchor only',
    'selected_lidar_only': 'selected LiDAR\nonly',
    'cv_kf_fusion': 'single CV-KF\nfusion',
    'production_imm': 'production\n3-model IMM',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--replay', required=True)
    parser.add_argument('--extended', required=True)
    parser.add_argument('--safety', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--base-mesh', required=True)
    parser.add_argument('--output-dir', required=True)
    return parser.parse_args()


def load_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def configure_style():
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.2,
        'axes.titlesize': 11.0, 'axes.labelsize': 9.2,
        'legend.fontsize': 7.8, 'pdf.fonttype': 42,
        'ps.fonttype': 42,
    })


def style_axis(axis, grid=True):
    axis.set_facecolor(COLORS['paper'])
    if grid:
        axis.grid(True, color='#E0E3E5', linewidth=0.55, zorder=0)
    axis.spines[['top', 'right']].set_visible(False)


def save_figure(fig, output_dir, stem, metadata):
    output = Path(output_dir) / f'{stem}.png'
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=210, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)
    output.with_suffix('.json').write_text(
        json.dumps(json_safe(metadata), indent=2, allow_nan=False) + '\n', encoding='utf-8')


def json_safe(value):
    """Replace non-finite numeric sentinels with explicit JSON null values."""
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(float(value)):
        return None
    if isinstance(value, np.generic):
        return value.item()
    return value


def draw_footer(fig, blocks):
    footer = fig.add_axes([0.045, 0.012, 0.91, 0.082])
    footer.axis('off')
    footer.axhline(0.98, color=COLORS['line'], linewidth=0.8)
    width = 1.0 / len(blocks)
    for index, (heading, value) in enumerate(blocks):
        x = 0.01 + index * width
        footer.text(x, 0.61, heading, fontsize=7.5, weight='bold',
                    color=COLORS['muted'], transform=footer.transAxes)
        footer.text(x, 0.10, value, fontsize=8.3, weight='bold',
                    color=COLORS['ink'], transform=footer.transAxes)


def _segments(times, values):
    if not times:
        return []
    segments = []
    start = 0
    for index in range(1, len(values) + 1):
        if index == len(values) or values[index] != values[start]:
            left = float(times[start])
            right = float(times[index]) if index < len(times) else float(times[-1])
            segments.append((left, max(right, left + 1e-3), values[start]))
            start = index
    return segments


def render_continuity(replay, output_dir):
    lanes = build_continuity_lanes(replay)
    times = lanes['time_sec']
    if not times:
        raise ValueError('continuity figure requires one locked target episode')
    end = max(times)
    fig = plt.figure(figsize=(17.2, 8.3), facecolor='white')
    grid = fig.add_gridspec(4, 1, left=0.10, right=0.975, top=0.94, bottom=0.16,
                           hspace=0.34, height_ratios=[1.0, 1.0, 1.0, 1.35])

    visibility = fig.add_subplot(grid[0])
    visibility.fill_between(times, 0.55, 0.95, where=np.asarray(lanes['camera_visible'], bool),
                            step='post', color=COLORS['orange'], alpha=0.82,
                            label='camera observation')
    visibility.fill_between(times, 0.05, 0.45, where=np.asarray(lanes['lidar_visible'], bool),
                            step='post', color=COLORS['blue'], alpha=0.82,
                            label='selected LiDAR observation')
    visibility.set_yticks([0.25, 0.75], ['LiDAR', 'camera'])
    visibility.set_ylim(0, 1)
    visibility.set_xlim(0, end)
    visibility.set_title('(a) Sensor evidence availability', loc='left')
    visibility.legend(loc='upper right', ncol=2, frameon=True)
    style_axis(visibility)

    source = fig.add_subplot(grid[1], sharex=visibility)
    source_colors = {
        'CAMERA_LIDAR': COLORS['green'], 'CAMERA_ONLY': COLORS['orange'],
        'LIDAR_ONLY': COLORS['blue'], 'PREDICTION_ONLY': COLORS['yellow'],
        'NONE': COLORS['gray'],
    }
    for left, right, value in _segments(times, lanes['source_state']):
        source.axvspan(left, right, color=source_colors.get(value, COLORS['gray']), alpha=0.86)
        if right - left > 0.8:
            source.text((left + right) / 2, 0.5, value.replace('_', ' '),
                        ha='center', va='center', fontsize=8, weight='bold', color='white')
    source.set_ylim(0, 1)
    source.set_yticks([])
    source.set_title('(b) Published fusion source state', loc='left')
    style_axis(source)

    association = fig.add_subplot(grid[2], sharex=visibility)
    association_colors = {
        'CONFIRMED': COLORS['green'], 'AMBIGUOUS': COLORS['orange'],
        'PREDICTED': COLORS['yellow'], 'LOST': COLORS['red'],
        'UNBOUND': COLORS['gray'],
    }
    for left, right, value in _segments(times, lanes['association_state']):
        association.axvspan(
            left, right, color=association_colors.get(value, COLORS['gray']), alpha=0.86)
        if right - left > 0.8:
            association.text((left + right) / 2, 0.5, value, ha='center', va='center',
                             fontsize=8, weight='bold', color='white')
    association.set_ylim(0, 1)
    association.set_yticks([])
    association.set_title('(c) Association state of the authorized logical target', loc='left')
    style_axis(association)

    identity = fig.add_subplot(grid[3], sharex=visibility)
    ids = np.asarray(lanes['selected_tracklet_id'], dtype=float)
    ids[ids < 0] = np.nan
    identity.step(times, ids, where='post', color=COLORS['blue'], linewidth=1.9,
                  label='selected LiDAR tracklet ID')
    accepted = [event for event in lanes['measurement_events'] if event['accepted']]
    rejected = [event for event in lanes['measurement_events'] if not event['accepted']]
    baseline = np.nanmin(ids) - 0.45 if np.any(np.isfinite(ids)) else 0
    if accepted:
        identity.scatter([row['time_sec'] for row in accepted], [baseline] * len(accepted),
                         marker='|', s=90, linewidths=1.5, color=COLORS['green'],
                         label='accepted filter update')
    if rejected:
        identity.scatter([row['time_sec'] for row in rejected], [baseline] * len(rejected),
                         marker='x', s=26, linewidths=1.1, color=COLORS['red'],
                         label='rejected update')
    if lanes['release_time_sec'] is not None:
        identity.axvline(lanes['release_time_sec'], color=COLORS['red'], linestyle='--',
                         linewidth=1.5, label='safe release to NO_TARGET')
    identity.set_xlabel('time since primary target lock [s]')
    identity.set_ylabel('run-local\ntracklet ID')
    identity.set_title('(d) LiDAR identity continuity and estimator updates', loc='left')
    identity.legend(loc='upper right', ncol=2, frameon=True)
    style_axis(identity)
    plt.setp([visibility, source, association], xticklabels=[])

    draw_footer(fig, [
        ('LOGICAL TARGET', f"run-local ID {lanes['primary_target_id']}"),
        ('EPISODE', f'{end:.2f} s from lock to release'),
        ('SOURCE', 'recorded rosbag replay · 0.5×'),
        ('INTERPRETATION', 'continuity evidence; no external ground truth'),
    ])
    save_figure(fig, output_dir, 'human-following-perception-continuity-v1', {
        'source': replay.get('bag'), 'run_index': replay.get('run_index'),
        'playback_rate': replay.get('playback_rate'), 'lanes': lanes,
        'ground_truth_available': False,
        'interpretation': 'perception_and_identity_continuity',
    })


def _finite_values(rows, key):
    return [float(row[key]) for row in rows
            if isinstance(row.get(key), (int, float)) and np.isfinite(float(row[key]))]


def render_resources(extended, output_dir):
    device = extended['resources']['device_samples']
    processes = extended['resources']['process_samples']
    summary = extended['resources']['summary']
    idle_duration = float(extended['idle_duration_sec'])
    time_values = np.asarray([float(row['stamp']) for row in device])
    cpu = np.asarray([float(row['cpu_mean_pct']) for row in device])
    gpu = np.asarray([float(row['gpu_pct']) for row in device])
    ram = np.asarray([float(row['ram_used_mb']) / 1024.0 for row in device])
    temp = np.asarray([
        float(row['cpu_temp_c']) if row.get('cpu_temp_c') is not None else np.nan
        for row in device])

    fig = plt.figure(figsize=(17.2, 8.5), facecolor='white')
    grid = fig.add_gridspec(2, 3, left=0.065, right=0.975, top=0.94, bottom=0.16,
                           hspace=0.34, wspace=0.29)
    utilization = fig.add_subplot(grid[0, :2])
    utilization.plot(time_values, cpu, color=COLORS['blue'], linewidth=1.5,
                     label='CPU mean across active cores')
    utilization.plot(time_values, gpu, color=COLORS['green'], linewidth=1.5,
                     label='GPU utilization')
    utilization.axvspan(time_values.min(), idle_duration, color=COLORS['gray'],
                        alpha=0.13, label='idle baseline')
    utilization.axvline(idle_duration, color=COLORS['ink'], linestyle='--', linewidth=1.0)
    utilization.set_ylabel('device utilization [%]')
    utilization.set_xlabel('collector wall time [s]')
    utilization.set_ylim(bottom=0)
    utilization.set_title('(a) Device-wide utilization trace', loc='left')
    utilization.legend(loc='upper left', ncol=3, frameon=True)
    style_axis(utilization)

    memory = fig.add_subplot(grid[0, 2])
    memory.plot(time_values, ram, color=COLORS['purple'], linewidth=1.6, label='RAM used')
    memory.axvspan(time_values.min(), idle_duration, color=COLORS['gray'], alpha=0.13)
    memory.set_xlabel('collector wall time [s]')
    memory.set_ylabel('device RAM used [GiB]', color=COLORS['purple'])
    memory.tick_params(axis='y', labelcolor=COLORS['purple'])
    temp_axis = memory.twinx()
    temp_axis.plot(time_values, temp, color=COLORS['red'], linewidth=1.2, label='CPU temp')
    temp_axis.set_ylabel('CPU temperature [°C]', color=COLORS['red'])
    temp_axis.tick_params(axis='y', labelcolor=COLORS['red'])
    memory.set_title('(b) Memory and thermal trace', loc='left')
    style_axis(memory)

    nodes = ['camera_tracking', 'lidar_tracklets', 'target_fusion']
    node_labels = ['camera\ntracking', 'LiDAR\ntracklets', 'target\nfusion']
    node_colors = [COLORS['orange'], COLORS['blue'], COLORS['green']]
    cpu_axis = fig.add_subplot(grid[1, 0])
    cpu_values = [
        _finite_values([row for row in processes if row.get('node') == node], 'cpu_pct')
        for node in nodes]
    boxes = cpu_axis.boxplot(cpu_values, patch_artist=True, widths=0.58, showfliers=False,
                             medianprops={'color': COLORS['ink'], 'linewidth': 1.4})
    for patch, color in zip(boxes['boxes'], node_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.78)
    cpu_axis.set_xticks(range(1, 4), node_labels)
    cpu_axis.set_ylabel('process CPU [% of one core]')
    cpu_axis.set_title('(c) Per-node CPU distribution', loc='left')
    style_axis(cpu_axis)

    rss_axis = fig.add_subplot(grid[1, 1])
    rss = [float(summary['nodes'][node]['rss_peak_mb']) / 1024.0 for node in nodes]
    rss_axis.bar(node_labels, rss, color=node_colors, width=0.62)
    for index, value in enumerate(rss):
        rss_axis.text(index, value, f'{value:.2f}', ha='center', va='bottom', fontsize=8)
    rss_axis.set_ylabel('peak resident memory [GiB]')
    rss_axis.set_title('(d) Per-node peak resident memory', loc='left')
    style_axis(rss_axis)

    rates_axis = fig.add_subplot(grid[1, 2])
    rate_keys = ['fused_target_state', 'lidar_tracklet_arrays',
                 'camera_guided_anchors', 'tracker_debug']
    rate_labels = ['fused state', 'LiDAR tracklets', 'camera anchors', 'debug']
    rates = [extended['topic_rates'][key]['observed_rate_hz'] for key in rate_keys]
    rates_axis.barh(rate_labels[::-1], rates[::-1],
                    color=[COLORS['purple'], COLORS['orange'], COLORS['blue'],
                           COLORS['green']][::-1])
    for index, value in enumerate(rates[::-1]):
        rates_axis.text(value, index, f' {value:.1f}', va='center', fontsize=8)
    rates_axis.set_xlabel('observed message rate [Hz]')
    rates_axis.set_title('(e) Replay output rates', loc='left')
    style_axis(rates_axis)

    draw_footer(fig, [
        ('REPLAY', f"{extended['bag']} · {extended['playback_rate']}×"),
        ('IDLE BASELINE', f'{idle_duration:.0f} s before algorithm startup'),
        ('SCOPE', 'device telemetry includes other host workloads'),
        ('PROCESS CPU', '100% = one logical CPU core'),
    ])
    save_figure(fig, output_dir, 'human-following-deployment-resources-v1', {
        'source': extended['source'], 'bag': extended['bag'],
        'playback_rate': extended['playback_rate'],
        'idle_duration_sec': idle_duration,
        'device_sample_count': len(device),
        'process_sample_count': len(processes),
        'summary': summary, 'topic_rates': extended['topic_rates'],
        'scope_note': extended['resources']['scope_note'],
    })


def render_safety(safety, output_dir):
    columns = safety['matrix_columns']
    scenarios = safety['scenarios']
    values = np.asarray([
        [1.0 if row['verified'][column] is True else np.nan for column in columns]
        for row in scenarios])
    fig = plt.figure(figsize=(17.2, 8.2), facecolor='white')
    grid = fig.add_gridspec(1, 2, left=0.19, right=0.975, top=0.90, bottom=0.18,
                           width_ratios=[3.25, 1.35], wspace=0.20)
    axis = fig.add_subplot(grid[0])
    masked = np.ma.masked_invalid(values)
    axis.imshow(masked, cmap=ListedColormap([COLORS['green']]), vmin=0, vmax=1,
                aspect='auto', interpolation='nearest')
    axis.set_facecolor('#EEF0F1')
    axis.set_xticks(range(len(columns)), [
        'hazard / fault\ndetected', 'authorization\nrevoked', 'disarm\nrequested',
        'logical target\nreset', 'zero velocity\ncommand', 'no automatic\nresume'],
        rotation=0)
    axis.set_yticks(range(len(scenarios)), [row['name'] for row in scenarios])
    axis.tick_params(length=0)
    axis.set_xticks(np.arange(-.5, len(columns), 1), minor=True)
    axis.set_yticks(np.arange(-.5, len(scenarios), 1), minor=True)
    axis.grid(which='minor', color='white', linewidth=2.0)
    for y, row in enumerate(scenarios):
        for x, column in enumerate(columns):
            verified = row['verified'][column] is True
            axis.text(x, y, '✓' if verified else '—', ha='center', va='center',
                      fontsize=14 if verified else 11, weight='bold' if verified else 'normal',
                      color='white' if verified else COLORS['muted'])
    axis.set_title('(a) Explicitly asserted fail-safe behaviors', loc='left', pad=15)

    evidence = fig.add_subplot(grid[1])
    evidence.axis('off')
    evidence.set_title('(b) Evidence layer and test coverage', loc='left', pad=15)
    y = 0.94
    for row in scenarios:
        count = len(row['test_ids'])
        evidence.text(0.0, y, row['name'], fontsize=8.5, weight='bold',
                      color=COLORS['ink'], transform=evidence.transAxes)
        evidence.text(0.98, y, f"{row['layer']} · {count} test{'s' if count != 1 else ''}",
                      fontsize=7.8, ha='right', color=COLORS['muted'],
                      transform=evidence.transAxes)
        evidence.plot([0, 1], [y - 0.035, y - 0.035], color=COLORS['line'], linewidth=0.5,
                      transform=evidence.transAxes)
        y -= 0.12
    totals = safety['package_results']
    evidence.text(0.0, 0.035,
                  f"Safety: {totals['track_robot_safety']['individual_test_cases']}/"
                  f"{totals['track_robot_safety']['individual_test_cases']} passed\n"
                  f"Decision: {totals['track_robot_decision']['individual_test_cases']}/"
                  f"{totals['track_robot_decision']['individual_test_cases']} passed",
                  fontsize=10.0, weight='bold', color=COLORS['green'],
                  transform=evidence.transAxes)
    fig.text(0.5, 0.965, 'Automated Software Safety Scenario Matrix', ha='center',
             va='top', fontsize=13, weight='bold', color=COLORS['ink'])
    draw_footer(fig, [
        ('SYMBOL', '✓ explicitly asserted · — outside cited test scope'),
        ('CAMPAIGN', '54 individual tests · 0 failures'),
        ('EXECUTION', 'ROS 2 launch + C++ unit + static contract tests'),
        ('LIMIT', 'software evidence; not functional-safety certification'),
    ])
    save_figure(fig, output_dir, 'human-following-safety-fault-matrix-v1', safety)


ARM_LINKS = {
    'arm_base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6',
    'gripper_base', 'camera_holder', 'l515_visual', 'link7', 'link8',
}


def _link_color(link):
    if link == 'base_link':
        return '#343A40'
    if link == 'sensor_station_link':
        return '#6C757D'
    if link in ('camera_holder',):
        return '#202428'
    if link == 'l515_visual':
        return '#8D78A8'
    if link in ('gripper_base', 'link7', 'link8'):
        return '#4D555B'
    if link in ARM_LINKS:
        return '#D27A32'
    return '#8C949A'


def _add_scene_meshes(axis, scene, include_links=None, alpha=1.0, ghost_color=None):
    rendered_faces = 0
    for mesh in scene['meshes']:
        if include_links is not None and mesh['link'] not in include_links:
            continue
        triangles = mesh['vertices'][mesh['faces']]
        collection = Poly3DCollection(
            triangles, facecolors=ghost_color or _link_color(mesh['link']),
            linewidth=0, alpha=alpha, shade=True, zsort='average')
        collection.set_rasterized(True)
        axis.add_collection3d(collection)
        rendered_faces += len(triangles)
    return rendered_faces


def _cuboid_faces(center, size):
    cx, cy, cz = center
    sx, sy, sz = np.asarray(size, dtype=float) / 2.0
    vertices = np.asarray([
        [cx-sx, cy-sy, cz-sz], [cx+sx, cy-sy, cz-sz],
        [cx+sx, cy+sy, cz-sz], [cx-sx, cy+sy, cz-sz],
        [cx-sx, cy-sy, cz+sz], [cx+sx, cy-sy, cz+sz],
        [cx+sx, cy+sy, cz+sz], [cx-sx, cy+sy, cz+sz],
    ])
    indices = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
               [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return [vertices[index] for index in indices]


def _add_sensor_geometry(axis, zed, lidar, hfov_deg, vfov_deg, clip_range):
    camera_box = Poly3DCollection(
        _cuboid_faces(zed, (0.08, 0.19, 0.07)), facecolors=COLORS['orange'],
        edgecolors='white', linewidth=0.35, alpha=0.95, shade=True)
    axis.add_collection3d(camera_box)
    theta = np.linspace(0, 2 * np.pi, 72)
    cylinder_z = np.linspace(lidar[2] - 0.045, lidar[2] + 0.055, 2)
    theta_grid, z_grid = np.meshgrid(theta, cylinder_z)
    axis.plot_surface(
        lidar[0] + 0.065 * np.cos(theta_grid),
        lidar[1] + 0.065 * np.sin(theta_grid), z_grid,
        color=COLORS['blue'], alpha=0.95, linewidth=0, shade=True)

    solid_range = min(1.25, clip_range)
    half_width = solid_range * math.tan(math.radians(hfov_deg / 2))
    half_height = solid_range * math.tan(math.radians(vfov_deg / 2))
    far_x = zed[0] + solid_range
    bottom = max(0.0, zed[2] - half_height)
    top = zed[2] + half_height
    corners = np.asarray([
        [far_x, zed[1] - half_width, bottom],
        [far_x, zed[1] + half_width, bottom],
        [far_x, zed[1] + half_width, top],
        [far_x, zed[1] - half_width, top],
    ])
    for corner in corners:
        axis.plot([zed[0], corner[0]], [zed[1], corner[1]], [zed[2], corner[2]],
                  color=COLORS['orange'], linewidth=0.9, alpha=0.82)
    for first, second in zip(corners, np.roll(corners, -1, axis=0)):
        axis.plot([first[0], second[0]], [first[1], second[1]],
                  [first[2], second[2]], color=COLORS['orange'],
                  linewidth=0.9, alpha=0.82)

    far_width = clip_range * math.tan(math.radians(hfov_deg / 2))
    far_height = clip_range * math.tan(math.radians(vfov_deg / 2))
    far_corners = np.asarray([
        [zed[0] + clip_range, zed[1] - far_width, max(0.0, zed[2] - far_height)],
        [zed[0] + clip_range, zed[1] + far_width, max(0.0, zed[2] - far_height)],
        [zed[0] + clip_range, zed[1] + far_width, zed[2] + far_height],
        [zed[0] + clip_range, zed[1] - far_width, zed[2] + far_height],
    ])
    for near, far in zip(corners, far_corners):
        axis.plot([near[0], far[0]], [near[1], far[1]], [near[2], far[2]],
                  color=COLORS['orange'], linestyle='--', linewidth=0.55, alpha=0.20)

    for radius, linestyle, alpha in (
            (0.5, ':', 0.75), (solid_range, '-', 0.55), (clip_range, '--', 0.42)):
        axis.plot(lidar[0] + radius * np.cos(theta),
                  lidar[1] + radius * np.sin(theta),
                  np.full_like(theta, lidar[2]), color=COLORS['blue'],
                  linewidth=1.0, linestyle=linestyle, alpha=alpha)
    return {'solid_corners': corners, 'far_corners': far_corners,
            'solid_range_m': solid_range}


def _clean_3d_axis(axis):
    axis.set_facecolor('white')
    for pane in (axis.xaxis.pane, axis.yaxis.pane, axis.zaxis.pane):
        pane.set_facecolor((0.98, 0.98, 0.97, 0.0))
        pane.set_edgecolor((0.78, 0.80, 0.82, 0.55))
    axis.grid(True, color='#DDE1E3', linewidth=0.45)
    axis.tick_params(labelsize=8.0, pad=1)


def render_envelope(extended, urdf_path, base_mesh, output_dir):
    urdf_xml = Path(urdf_path).read_text(encoding='utf-8')
    workspace = sample_urdf_workspace(
        urdf_xml, 'base_link', ['gripper_base', 'l515_visual'], sample_count=30000, seed=7)
    gripper = np.asarray(workspace['points']['gripper_base'], dtype=float)
    gripper_ground = gripper + np.asarray([0.0, 0.0, 0.45])
    hull = convex_outer_surface(gripper_ground, max_points=5000, seed=7)
    source_root = Path(urdf_path).resolve().parents[2]
    package_roots = {
        'bunker_pro2': source_root / 'bunker_pro2',
        'piper_description': source_root / 'piper_description',
    }
    presentation_pose = {
        'joint1': 0.15, 'joint2': 1.10, 'joint3': -1.40,
        'joint4': 0.0, 'joint5': 0.50, 'joint6': 0.0,
        'joint7': 0.020, 'joint8': -0.020,
    }
    ghost_poses = [
        {'joint1': 1.30, 'joint2': 0.80, 'joint3': -1.20,
         'joint4': 0.25, 'joint5': 0.20, 'joint6': 0.40,
         'joint7': 0.018, 'joint8': -0.018},
        {'joint1': -1.10, 'joint2': 0.72, 'joint3': -1.05,
         'joint4': -0.30, 'joint5': 0.40, 'joint6': -0.35,
         'joint7': 0.018, 'joint8': -0.018},
        {'joint1': 0.05, 'joint2': 1.80, 'joint3': -2.20,
         'joint4': 0.0, 'joint5': -0.40, 'joint6': 0.0,
         'joint7': 0.018, 'joint8': -0.018},
    ]
    main_scene = build_visual_scene(
        urdf_xml, package_roots, 'robot_bottom', presentation_pose,
        max_faces_per_mesh=1500, seed=7)
    ghost_scenes = [
        build_visual_scene(
            urdf_xml, package_roots, 'robot_bottom', pose,
            max_faces_per_mesh=420, seed=30 + index)
        for index, pose in enumerate(ghost_poses)
    ]
    zed = main_scene['link_transforms']['zed_camera_link'][:3, 3]
    lidar = main_scene['link_transforms']['lidar_link'][:3, 3]
    camera = extended['camera_info']
    hfov = math.degrees(2 * math.atan(float(camera['width']) / (2 * float(camera['fx']))))
    vfov = math.degrees(2 * math.atan(float(camera['height']) / (2 * float(camera['fy']))))
    min_range, max_range = 0.5, 10.0
    clip_range = 2.5

    fig = plt.figure(figsize=(13.5, 9.8), facecolor='white')
    axis = fig.add_axes([0.015, 0.125, 0.97, 0.82], projection='3d')
    ground_x, ground_y = np.meshgrid(np.linspace(-0.9, 1.65, 2), np.linspace(-1.4, 1.4, 2))
    axis.plot_surface(ground_x, ground_y, np.zeros_like(ground_x), color='#ECEEED',
                      alpha=0.46, linewidth=0, shade=False)
    hull_collection = Poly3DCollection(
        hull['vertices'][hull['faces']], facecolors=COLORS['blue'],
        linewidth=0, alpha=0.10, shade=True, zsort='average')
    axis.add_collection3d(hull_collection)
    ghost_face_counts = []
    for index, scene in enumerate(ghost_scenes):
        ghost_face_counts.append(_add_scene_meshes(
            axis, scene, include_links=ARM_LINKS, alpha=0.15,
            ghost_color=['#73A6C9', '#9A87B8', '#6FA99D'][index]))
    main_face_count = _add_scene_meshes(axis, main_scene, alpha=0.98)
    sensing_geometry = _add_sensor_geometry(axis, zed, lidar, hfov, vfov, clip_range)

    gripper_tip = main_scene['link_transforms']['gripper_base'][:3, 3]
    axis.scatter(*gripper_tip, color=COLORS['red'], s=28, depthshade=False, zorder=20)
    axis.text(gripper_tip[0] + 0.05, gripper_tip[1], gripper_tip[2] + 0.04,
              'presentation pose', color=COLORS['red'], fontsize=8, weight='bold')
    axis.set_xlim(-0.90, 1.60)
    axis.set_ylim(-1.35, 1.35)
    axis.set_zlim(0.0, 1.68)
    axis.set_box_aspect((2.5, 2.7, 1.68))
    axis.view_init(elev=23, azim=-60)
    axis.set_xlabel('robot-forward x [m]', labelpad=7)
    axis.set_ylabel('robot-left y [m]', labelpad=8)
    axis.set_zlabel('height above ground [m]', labelpad=7)
    axis.set_title('Integrated 3D Sensing and Manipulation Envelope', loc='left',
                   pad=12, fontsize=13.0, weight='bold')
    _clean_3d_axis(axis)
    legend = [
        Patch(facecolor='#343A40', label='Bunker CAD'),
        Patch(facecolor='#D27A32', label='PiPER presentation pose'),
        Patch(facecolor=COLORS['blue'], alpha=0.18, label='gripper convex outer envelope'),
        Line2D([0], [0], color='#73A6C9', linewidth=4, alpha=0.45,
               label='representative ghost poses'),
        Patch(facecolor=COLORS['orange'], alpha=0.28, label='ZED recorded FoV'),
        Line2D([0], [0], color=COLORS['blue'], linestyle='--',
               label='LiDAR horizontal range'),
    ]
    axis.legend(handles=legend, loc='upper right', bbox_to_anchor=(0.98, 0.97),
                frameon=True, ncol=1, fontsize=8.0)
    fig.text(
        0.065, 0.825,
        'SENSING SCALE\n'
        f"solid near-field: {sensing_geometry['solid_range_m']:.2f} m\n"
        'dashed continuation: 2.50 m\n'
        'configured tracking: 0.5–10.0 m',
        fontsize=8.1, color=COLORS['muted'], linespacing=1.35,
        bbox={'boxstyle': 'round,pad=0.45', 'facecolor': 'white',
              'edgecolor': COLORS['line'], 'alpha': 0.92})

    draw_footer(fig, [
        ('MODEL', 'Bunker + PiPER CAD'),
        ('REACH SURFACE', f"30k samples · {hull['volume_m3']:.2f} m³ hull"),
        ('ZED INTRINSICS', f"{camera['width']}×{camera['height']} · H {hfov:.1f}° · V {vfov:.1f}°"),
        ('LIMIT', 'convex kinematic outer bound · not collision-free'),
    ])
    save_figure(fig, output_dir, 'robot-sensing-manipulation-envelope-v1', {
        'representation': 'single_isometric_3d_urdf_stl_scene',
        'sources': {
            'urdf': str(urdf_path), 'base_mesh': str(base_mesh),
            'camera_info': camera,
            'lidar_tracking_range_m': [min_range, max_range],
        },
        'sensor_origins_robot_bottom_m': {'zed': zed.tolist(), 'lidar': lidar.tolist()},
        'zed_fov_deg': {'horizontal': hfov, 'vertical': vfov},
        'visualization_clip_range_m': clip_range,
        'solid_near_field_range_m': sensing_geometry['solid_range_m'],
        'robot_meshes': {
            'main_scene_mesh_count': len(main_scene['meshes']),
            'main_scene_rendered_face_count': main_face_count,
            'ghost_scene_rendered_face_counts': ghost_face_counts,
            'package_roots': {key: str(value) for key, value in package_roots.items()},
        },
        'presentation_pose_rad_or_m': presentation_pose,
        'representative_ghost_poses_rad_or_m': ghost_poses,
        'workspace': {
            'sample_count': workspace['sample_count'], 'seed': workspace['seed'],
            'joint_names': workspace['joint_names'],
            'joint_limits_rad_or_m': workspace['joint_limits'],
            'gripper_bounds_base_link_m': {
                'min': gripper.min(axis=0).tolist(), 'max': gripper.max(axis=0).tolist()},
            'surface': {
                'type': 'convex_outer_hull',
                'sampled_point_count': hull['sampled_point_count'],
                'surface_face_count': hull['surface_face_count'],
                'volume_m3': hull['volume_m3'], 'area_m2': hull['area_m2'],
            },
        },
        'limitations': [
            'The translucent surface is a convex outer bound over sampled joint-limit positions; it can include collision-invalid or unsampled interior locations.',
            'The L515 is present as a visual URDF model; no calibrated operational FoV is claimed.',
            'The ZED FoV is computed from recorded CameraInfo and projected schematically robot-forward because the integrated model omits the full optical-frame visual chain.',
            'The LiDAR surface depicts horizontal coverage only; vertical FoV is not shown.',
            'Sensor coverage is clipped to 2.5 m in the scene while the configured tracking range is 0.5–10.0 m.',
        ],
    })


def render_ablation(extended, output_dir):
    result = run_estimator_ablation(extended['measurements'], extended['states'])
    names = ['camera_guided_anchor_only', 'selected_lidar_only',
             'cv_kf_fusion', 'production_imm']
    times = np.asarray(result['time_sec'], dtype=float)
    fig = plt.figure(figsize=(17.2, 8.5), facecolor='white')
    grid = fig.add_gridspec(2, 2, left=0.065, right=0.975, top=0.94, bottom=0.17,
                           hspace=0.33, wspace=0.25)
    trajectory = fig.add_subplot(grid[0, 0])
    for name in names:
        condition = result['conditions'][name]
        xy = np.asarray(condition['xy'], dtype=float)
        valid = np.asarray(condition['valid'], dtype=bool)
        trajectory.plot(xy[valid, 0], xy[valid, 1], color=CONDITION_COLORS[name],
                        linewidth=2.0 if name == 'production_imm' else 1.35,
                        marker='o', markersize=2.4, alpha=0.90,
                        label=CONDITION_LABELS[name].replace('\n', ' '))
    trajectory.set_xlabel('x in base frame [m]')
    trajectory.set_ylabel('y in base frame [m]')
    trajectory.set_title('(a) Estimator trajectories under masked inputs', loc='left')
    trajectory.legend(loc='best', frameon=True)
    trajectory.set_aspect('equal', adjustable='datalim')
    style_axis(trajectory)

    time_axis = fig.add_subplot(grid[0, 1])
    for name in names:
        condition = result['conditions'][name]
        xy = np.asarray(condition['xy'], dtype=float)
        valid = np.asarray(condition['valid'], dtype=bool)
        time_axis.plot(times[valid], xy[valid, 0], color=CONDITION_COLORS[name],
                       linewidth=2.0 if name == 'production_imm' else 1.3,
                       label=CONDITION_LABELS[name].replace('\n', ' '))
        if np.any(~valid):
            time_axis.scatter(times[~valid], np.full(np.sum(~valid), np.nanmin(xy[:, 0])),
                              marker='x', color=CONDITION_COLORS[name], s=18)
    time_axis.set_xlabel('time since first synchronized update [s]')
    time_axis.set_ylabel('estimated forward position x [m]')
    time_axis.set_title('(b) Temporal continuity of the estimated target', loc='left')
    style_axis(time_axis)

    continuity = fig.add_subplot(grid[1, 0])
    continuity_values = [result['conditions'][name]['continuity_fraction'] * 100 for name in names]
    covariance_values = [
        result['conditions'][name]['metrics']['covariance_trace_median_m2'] for name in names]
    x = np.arange(len(names))
    continuity.bar(x - 0.18, continuity_values, width=0.36,
                   color=[CONDITION_COLORS[name] for name in names], alpha=0.82)
    continuity.set_ylabel('valid estimate coverage [%]')
    continuity.set_ylim(0, 108)
    continuity.set_xticks(x, [CONDITION_LABELS[name] for name in names])
    covariance_axis = continuity.twinx()
    covariance_axis.plot(x + 0.18, covariance_values, color=COLORS['ink'], marker='D',
                         linewidth=1.3, label='median covariance trace')
    covariance_axis.set_ylabel('median covariance trace [m²]')
    continuity.set_title('(c) Continuity and estimator-reported uncertainty', loc='left')
    style_axis(continuity)

    metrics_axis = fig.add_subplot(grid[1, 1])
    metrics_axis.axis('off')
    metrics_axis.set_title('(d) Consistency metrics and interpretation', loc='left')
    headers = ['condition', 'mutual dev.\nRMS [m]', 'accel. RMS\n[m/s²]']
    cell_rows = []
    for name in names:
        metrics = result['conditions'][name]['metrics']
        deviation = metrics['trajectory_rms_mutual_deviation_m']
        acceleration = metrics['acceleration_rms_mps2']
        cell_rows.append([
            CONDITION_LABELS[name].replace('\n', ' '),
            'reference' if name == 'production_imm' else (f'{deviation:.3f}' if deviation is not None else 'n/a'),
            f'{acceleration:.3f}' if acceleration is not None else 'n/a',
        ])
    table = metrics_axis.table(cellText=cell_rows, colLabels=headers, loc='upper center',
                               cellLoc='center', colLoc='center', bbox=[0.0, 0.34, 1.0, 0.57])
    table.auto_set_font_size(False)
    table.set_fontsize(8.0)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('white')
        if row == 0:
            cell.set_facecolor(COLORS['ink'])
            cell.get_text().set_color('white')
            cell.get_text().set_weight('bold')
        else:
            cell.set_facecolor('#F0F2F3' if row % 2 else '#FAFAF9')
    metrics_axis.text(
        0.0, 0.22,
        'All conditions reuse production target authorization and association.\n'
        'Camera-only uses a camera-guided LiDAR depth anchor; it is not monocular 3D.\n'
        'Mutual deviation is relative to the production IMM, not trajectory error.',
        fontsize=8.5, color=COLORS['purple'], weight='bold', va='top',
        transform=metrics_axis.transAxes)

    draw_footer(fig, [
        ('DATA', f"{len(times)} synchronized estimator updates from one replay"),
        ('TIMEOUT', f"{result['timeout_sec']:.1f} s measurement-age validity"),
        ('REFERENCE', 'production three-model IMM'),
        ('CLAIM', 'estimator-input ablation; no external ground truth'),
    ])
    save_figure(fig, output_dir, 'human-following-estimator-ablation-v1', {
        'source': extended['bag'], 'playback_rate': extended['playback_rate'],
        'ablation': result,
        'conditions': {
            'camera_guided_anchor_only': 'camera-guided LiDAR depth anchor only',
            'selected_lidar_only': 'selected authorized LiDAR tracklet only',
            'cv_kf_fusion': 'both sources in one constant-velocity Kalman filter',
            'production_imm': 'production three-model IMM output',
        },
        'ground_truth_available': False,
        'interpretation': 'mutual_consistency_and_continuity_not_accuracy',
    })


def main():
    args = parse_args()
    configure_style()
    replay = load_json(args.replay)
    extended = load_json(args.extended)
    safety = load_json(args.safety)
    render_continuity(replay, args.output_dir)
    render_resources(extended, args.output_dir)
    render_safety(safety, args.output_dir)
    render_envelope(extended, args.urdf, args.base_mesh, args.output_dir)
    render_ablation(extended, args.output_dir)
    print(json.dumps({'output_dir': args.output_dir, 'figures': 5}))


if __name__ == '__main__':
    main()
