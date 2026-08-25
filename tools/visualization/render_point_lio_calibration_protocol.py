#!/usr/bin/env python3
"""Render the Point-LIO timing/extrinsic calibration model and evidence protocol."""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-point-lio-calibration')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


COLORS = {
    'lidar': '#4477AA', 'imu': '#EE7733', 'lio': '#2A7F76',
    'pending': '#B36B00', 'ink': '#202428', 'muted': '#5B6268',
    'line': '#C8CDD1',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    return parser.parse_args()


def box(axis, xy, wh, title, detail, color):
    patch = FancyBboxPatch(
        xy, wh[0], wh[1], boxstyle='round,pad=0.015,rounding_size=0.02',
        facecolor=color, edgecolor='white', linewidth=1.3)
    axis.add_patch(patch)
    axis.text(xy[0] + wh[0] / 2, xy[1] + wh[1] * 0.62, title,
              ha='center', va='center', fontsize=9.2, weight='bold', color=COLORS['ink'])
    axis.text(xy[0] + wh[0] / 2, xy[1] + wh[1] * 0.27, detail,
              ha='center', va='center', fontsize=7.8, color=COLORS['ink'])


def arrow(axis, start, end, color='#697176'):
    axis.add_patch(FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=12,
                                  linewidth=1.5, color=color))


def setup(axis, title):
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    axis.axis('off')
    axis.set_title(title, loc='left')


def render(args):
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.4,
        'axes.titlesize': 11.2,
    })
    fig = plt.figure(figsize=(17.2, 6.1), facecolor='white')
    grid = fig.add_gridspec(2, 3, height_ratios=(7.8, 1.25), hspace=0.18, wspace=0.14)
    timing = fig.add_subplot(grid[0, 0])
    frames = fig.add_subplot(grid[0, 1])
    validation = fig.add_subplot(grid[0, 2])
    footer = fig.add_subplot(grid[1, :])

    setup(timing, '(a) LiDAR–IMU temporal alignment')
    box(timing, (0.06, 0.67), (0.34, 0.18), 'RS-Helios cloud', 'timestamp  t_L', '#DCE6F2')
    box(timing, (0.06, 0.27), (0.34, 0.18), 'Phidget IMU', 'raw timestamp  t_I', '#FCE6D5')
    box(timing, (0.56, 0.42), (0.35, 0.24), 'IMU adapter', "t'_I = t_I − Δt\nframe rotation + deadband", '#E2F0EB')
    arrow(timing, (0.40, 0.76), (0.56, 0.58), COLORS['lidar'])
    arrow(timing, (0.40, 0.36), (0.56, 0.49), COLORS['imu'])
    timing.text(0.50, 0.13, 'Estimate Δt by coarse/fine sweep and trajectory closure',
                ha='center', fontsize=8.3, color=COLORS['muted'], weight='bold')

    setup(frames, '(b) Extrinsic model used by Point-LIO')
    box(frames, (0.08, 0.62), (0.32, 0.19), 'IMU body frame', 'B', '#FCE6D5')
    box(frames, (0.60, 0.62), (0.32, 0.19), 'LiDAR frame', 'L', '#DCE6F2')
    arrow(frames, (0.40, 0.715), (0.60, 0.715), COLORS['lio'])
    frames.text(0.50, 0.79, r'$^{B}\mathbf{T}_{L}=[\,^{B}\mathbf{R}_{L},\,^{B}\mathbf{t}_{L}\,]$',
                ha='center', fontsize=10.2, color=COLORS['lio'], weight='bold')
    box(frames, (0.24, 0.26), (0.52, 0.20), 'Current repository default',
        'identity placeholder · extrinsic estimation enabled', '#FFF1CE')
    frames.text(0.50, 0.13, 'Measured rigid transform is required before odometry is trusted.',
                ha='center', fontsize=8.3, color=COLORS['pending'], weight='bold')

    setup(validation, '(c) Required calibration evidence')
    items = [
        ('1', 'stationary IMU noise / gravity check'),
        ('2', 'straight out-and-back trajectory'),
        ('3', 'coarse then fine Δt sweep'),
        ('4', 'LiDAR-only control run'),
        ('5', 'endpoint drift and yaw closure'),
        ('6', 'registered-map sharpness review'),
    ]
    for index, (number, label) in enumerate(items):
        y = 0.83 - index * 0.125
        validation.text(0.08, y, number, ha='center', va='center', color='white',
                        fontsize=8.2, weight='bold',
                        bbox={'boxstyle': 'circle,pad=0.30', 'facecolor': COLORS['pending'],
                              'edgecolor': 'white'})
        validation.text(0.15, y, label, ha='left', va='center', fontsize=8.8,
                        color=COLORS['ink'])
    validation.text(0.08, 0.06, 'Recorded calibration bag: not found in repository',
                    fontsize=8.4, color=COLORS['pending'], weight='bold')

    footer.axis('off')
    footer.axhline(0.98, color=COLORS['line'], linewidth=0.8)
    footer.text(0.01, 0.62, 'INTEGRATION STATUS', transform=footer.transAxes,
                fontsize=8, weight='bold', color=COLORS['muted'])
    footer.text(0.01, 0.18, 'ROS 2 port and topic adapters implemented; calibration evidence pending.',
                transform=footer.transAxes, fontsize=9.1, weight='bold', color=COLORS['ink'])
    footer.text(0.68, 0.62, 'EXPECTED OUTPUTS', transform=footer.transAxes,
                fontsize=8, weight='bold', color=COLORS['muted'])
    footer.text(0.68, 0.18, '/Laser_map · /aft_mapped_to_init · /path · registered clouds',
                transform=footer.transAxes, fontsize=8.8, weight='bold', color=COLORS['ink'])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'source': 'repository_integration_configuration_and_documented_protocol',
        'measured_mapping_result': False,
        'status': 'CALIBRATION_DATA_PENDING',
        'time_model': "corrected_imu_stamp = raw_imu_stamp - imu_time_offset_sec",
        'extrinsic_definition': 'LiDAR pose expressed in the IMU body frame',
        'current_default': 'identity placeholder with extrinsic estimation enabled',
        'required_evidence': [label for _, label in items],
        'expected_outputs': ['/cloud_registered', '/cloud_registered_body', '/Laser_map',
                             '/aft_mapped_to_init', '/path'],
    }
    Path(args.metadata_output).write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    render(args)
    print(json.dumps({'output': args.output, 'status': 'CALIBRATION_DATA_PENDING'}))


if __name__ == '__main__':
    main()
