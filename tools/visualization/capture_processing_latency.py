#!/usr/bin/env python3
"""Capture replay processing latency distributions published by the ROS pipeline."""

import argparse
import json
import os
import time
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-processing-latency')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


COLORS = {
    'camera': '#6F4C9B', 'tracklet': '#4477AA', 'fusion': '#2A7F76',
    'parse': '#4477AA', 'projection': '#66A61E', 'association': '#E6AB02',
    'kalman': '#E7298A', 'publish': '#7570B3',
}
COMPONENTS = ['cloud_parse_ms', 'projection_ms', 'association_ms', 'kalman_ms', 'publish_ms']
COMPONENT_LABELS = ['cloud parse', 'projection', 'association', 'Kalman update', 'publish']


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    parser.add_argument('--bag-label', default='human_tracking_lidar_20260706_145900')
    parser.add_argument('--timeout', type=float, default=48.0)
    return parser.parse_args()


class LatencyCapture(Node):
    def __init__(self, args):
        super().__init__('human_tracking_latency_capture')
        self.args = args
        self.start = time.monotonic()
        self.deadline = self.start + args.timeout
        self.samples = {'camera': [], 'tracklet': [], 'fusion': []}
        self.create_subscription(String, '/human_tracking/tracker_debug',
                                 lambda msg: self.on_debug('camera', msg), 20)
        self.create_subscription(String, '/human_tracking/lidar_tracklet_debug',
                                 lambda msg: self.on_debug('tracklet', msg), 20)
        self.create_subscription(String, '/human_tracking/fusion_timing_debug',
                                 lambda msg: self.on_debug('fusion', msg), 30)

    def on_debug(self, stage, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if 'processing_ms' not in payload:
            return
        payload['_receive_elapsed_sec'] = time.monotonic() - self.start
        self.samples[stage].append(payload)


def percentile_summary(values):
    array = np.asarray(values, dtype=float)
    return {
        'count': int(len(array)), 'median_ms': float(np.median(array)),
        'p90_ms': float(np.percentile(array, 90)),
        'p95_ms': float(np.percentile(array, 95)),
        'p99_ms': float(np.percentile(array, 99)),
        'max_ms': float(np.max(array)),
    }


def render(capture):
    for stage, rows in capture.samples.items():
        if len(rows) < 5:
            raise RuntimeError(f'insufficient {stage} timing samples: {len(rows)}')
    fresh_fusion = [
        row for row in capture.samples['fusion']
        if bool(row.get('fresh_cloud_processed', False))]
    if len(fresh_fusion) < 5:
        raise RuntimeError(f'insufficient fresh fusion timing samples: {len(fresh_fusion)}')

    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.4,
        'axes.titlesize': 11.3, 'axes.labelsize': 9.4,
    })
    fig = plt.figure(figsize=(17.2, 6.4), facecolor='white')
    grid = fig.add_gridspec(2, 3, height_ratios=(8.2, 1.2), hspace=0.28, wspace=0.25)
    timeline = fig.add_subplot(grid[0, 0])
    components = fig.add_subplot(grid[0, 1])
    percentiles = fig.add_subplot(grid[0, 2])
    footer = fig.add_subplot(grid[1, :])

    for stage, label in [('camera', '2D pose + tracking'),
                         ('tracklet', 'LiDAR tracklet manager'),
                         ('fusion', 'selected-target fusion')]:
        rows = capture.samples[stage]
        timeline.scatter(
            [row['_receive_elapsed_sec'] for row in rows],
            [float(row['processing_ms']) for row in rows],
            s=13, alpha=0.62, linewidths=0, color=COLORS[stage], label=label)
    timeline.set_xlabel('replay wall time [s]')
    timeline.set_ylabel('per-callback processing time [ms]')
    timeline.set_yscale('log')
    timeline.grid(True, which='both', color='#E0E3E5', linewidth=0.55)
    timeline.set_facecolor('#FAFAF9')
    timeline.set_title('(a) Per-callback processing time', loc='left')
    timeline.legend(loc='upper right', frameon=True, fontsize=7.7)

    component_values = [
        [float(row.get(component, 0.0)) for row in fresh_fusion]
        for component in COMPONENTS]
    box = components.boxplot(
        component_values, labels=COMPONENT_LABELS, patch_artist=True,
        showfliers=False, widths=0.62,
        medianprops={'color': '#202428', 'linewidth': 1.5})
    for patch, key in zip(box['boxes'], ['parse', 'projection', 'association', 'kalman', 'publish']):
        patch.set_facecolor(COLORS[key])
        patch.set_alpha(0.82)
    components.tick_params(axis='x', rotation=18)
    components.set_ylabel('processing time [ms]')
    components.set_yscale('symlog', linthresh=0.05)
    components.grid(True, axis='y', color='#E0E3E5', linewidth=0.55)
    components.set_facecolor('#FAFAF9')
    components.set_title('(b) Fusion-stage latency distribution', loc='left')

    stages = [('camera', '2D pose'), ('tracklet', 'LiDAR tracklets'), ('fusion', 'fusion total')]
    x = np.arange(len(stages))
    width = 0.23
    quantiles = [('median', 50, '#8DA0CB'), ('p95', 95, '#FC8D62'), ('p99', 99, '#E78AC3')]
    for offset, (label, q, color) in enumerate(quantiles):
        values = [
            float(np.percentile(
                [float(row['processing_ms']) for row in capture.samples[key]], q))
            for key, _ in stages]
        percentiles.bar(x + (offset - 1) * width, values, width,
                        color=color, label=label)
        for xpos, value in zip(x + (offset - 1) * width, values):
            percentiles.text(xpos, value, f'{value:.1f}', ha='center', va='bottom',
                             fontsize=7.2, rotation=90)
    percentiles.set_xticks(x, [label for _, label in stages])
    percentiles.set_ylabel('processing time [ms]')
    percentiles.grid(True, axis='y', color='#E0E3E5', linewidth=0.55)
    percentiles.set_facecolor('#FAFAF9')
    percentiles.set_title('(c) Robust timing percentiles', loc='left')
    percentiles.legend(loc='upper left', frameon=True, fontsize=7.7)

    summaries = {
        stage: percentile_summary([float(row['processing_ms']) for row in rows])
        for stage, rows in capture.samples.items()
    }
    fusion_component_summary = {
        component: percentile_summary(values)
        for component, values in zip(COMPONENTS, component_values)
    }
    footer.axis('off')
    footer.axhline(0.98, color='#C8CDD1', linewidth=0.8)
    blocks = [
        ('MEASUREMENT MODE', 'recorded replay · wall-clock node timing'),
        ('CAMERA TRACKER', f"n={summaries['camera']['count']} · p95 {summaries['camera']['p95_ms']:.1f} ms"),
        ('LIDAR TRACKLETS', f"n={summaries['tracklet']['count']} · p95 {summaries['tracklet']['p95_ms']:.1f} ms"),
        ('FUSION', f"n={summaries['fusion']['count']} · p95 {summaries['fusion']['p95_ms']:.1f} ms"),
    ]
    for index, (heading, value) in enumerate(blocks):
        x0 = 0.01 + index * 0.25
        footer.text(x0, 0.66, heading, transform=footer.transAxes, fontsize=8,
                    weight='bold', color='#5B6268')
        footer.text(x0, 0.22, value, transform=footer.transAxes, fontsize=8.9,
                    weight='bold', color='#202428')

    output = Path(capture.args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'source': 'recorded_rosbag_replay_runtime_debug',
        'bag': capture.args.bag_label,
        'interpretation': 'Per-callback wall-clock processing duration published by each node; not sensor-to-actuator latency.',
        'pipeline_stage_summary': summaries,
        'fresh_fusion_sample_count': len(fresh_fusion),
        'fusion_component_summary': fusion_component_summary,
        'topics': {
            'camera': '/human_tracking/tracker_debug',
            'tracklet': '/human_tracking/lidar_tracklet_debug',
            'fusion': '/human_tracking/fusion_timing_debug',
        },
    }
    Path(capture.args.metadata_output).write_text(
        json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    rclpy.init()
    capture = LatencyCapture(args)
    try:
        while rclpy.ok() and time.monotonic() < capture.deadline:
            rclpy.spin_once(capture, timeout_sec=0.2)
    finally:
        capture.destroy_node()
        rclpy.shutdown()
    render(capture)
    print(json.dumps({key: len(value) for key, value in capture.samples.items()}))


if __name__ == '__main__':
    main()
