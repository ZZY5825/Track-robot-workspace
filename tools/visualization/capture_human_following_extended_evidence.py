#!/usr/bin/env python3
"""Capture resource and estimator-ablation evidence from one offline replay."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from track_robot_interfaces.msg import TargetState

from capture_human_following_statistics import (
    StatisticsCapture,
    build_record,
    nearest,
)
from human_following_extended_analysis import (
    parse_tegrastats_line,
    summarize_resources,
)


NODE_PATTERNS = {
    'camera_tracking': 'human_image_tracker_node',
    'lidar_tracklets': 'lidar_tracklet_manager_node',
    'target_fusion': 'selected_human_target_tracker_node',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--bag', default='human_tracking_lidar_20260706_145900')
    parser.add_argument('--run-index', type=int, default=1)
    parser.add_argument('--playback-rate', type=float, default=0.5)
    parser.add_argument('--recorded-duration-sec', type=float, default=52.765909521)
    parser.add_argument('--timeout', type=float, default=125.0)
    parser.add_argument('--idle-duration', type=float, default=8.0)
    parser.add_argument('--release-tail-sec', type=float, default=3.0)
    parser.add_argument('--state-sync-tolerance', type=float, default=0.12)
    parser.add_argument('--camera-sync-tolerance', type=float, default=0.20)
    parser.add_argument('--resource-interval', type=float, default=1.0)
    return parser.parse_args()


def read_process_snapshot(previous, now):
    """Read per-node CPU ticks/RSS from proc and return rows plus new state."""
    clock_ticks = float(os.sysconf(os.sysconf_names['SC_CLK_TCK']))
    rows = []
    current = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(
                'utf-8', errors='replace')
            node = next(
                (name for name, pattern in NODE_PATTERNS.items() if pattern in command), None)
            if node is None:
                continue
            fields = (entry / 'stat').read_text().split()
            ticks = float(fields[13]) + float(fields[14])
            status = (entry / 'status').read_text().splitlines()
            rss_kb = next(
                float(line.split()[1]) for line in status if line.startswith('VmRSS:'))
            pid = int(entry.name)
        except (OSError, StopIteration, ValueError, IndexError):
            continue
        current[pid] = (ticks, now, node)
        cpu_pct = None
        if pid in previous:
            previous_ticks, previous_time, _ = previous[pid]
            elapsed = now - previous_time
            if elapsed > 0:
                cpu_pct = 100.0 * (ticks - previous_ticks) / clock_ticks / elapsed
        if cpu_pct is not None and cpu_pct >= 0:
            rows.append({
                'stamp_monotonic': now,
                'pid': pid,
                'node': node,
                'cpu_pct': float(cpu_pct),
                'rss_mb': float(rss_kb / 1024.0),
            })
    return rows, current


class ResourceSampler:
    def __init__(self, idle_duration, interval):
        self.start = time.monotonic()
        self.idle_duration = float(idle_duration)
        self.interval = float(interval)
        self.device_rows = []
        self.process_rows = []
        self._stop = threading.Event()
        self._threads = []
        self._tegrastats = None

    def start_sampling(self):
        self._tegrastats = subprocess.Popen(
            ['tegrastats', '--interval', str(max(100, int(self.interval * 1000)))],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1)
        self._threads = [
            threading.Thread(target=self._read_tegrastats, daemon=True),
            threading.Thread(target=self._sample_processes, daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def phase(self, elapsed):
        return 'idle' if elapsed < self.idle_duration else 'replay'

    def _read_tegrastats(self):
        assert self._tegrastats is not None and self._tegrastats.stdout is not None
        for line in self._tegrastats.stdout:
            if self._stop.is_set():
                break
            now = time.monotonic()
            try:
                row = parse_tegrastats_line(line, stamp=now - self.start)
            except ValueError:
                continue
            row['phase'] = self.phase(now - self.start)
            self.device_rows.append(row)

    def _sample_processes(self):
        previous = {}
        while not self._stop.wait(self.interval):
            now = time.monotonic()
            rows, previous = read_process_snapshot(previous, now)
            for row in rows:
                row['stamp'] = float(now - self.start)
                row['phase'] = self.phase(now - self.start)
            self.process_rows.extend(rows)

    def stop_sampling(self):
        self._stop.set()
        if self._tegrastats is not None:
            self._tegrastats.terminate()
            try:
                self._tegrastats.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._tegrastats.kill()
                self._tegrastats.wait(timeout=3.0)
        for thread in self._threads:
            thread.join(timeout=2.0)


class ExtendedCapture(StatisticsCapture):
    def __init__(self, args):
        super().__init__(args)
        self.saw_lock = False
        self.release_deadline = None

    def on_state(self, msg):
        super().on_state(msg)
        if int(msg.lock_state) == int(TargetState.LOCK_TARGET_LOCKED):
            self.saw_lock = True
        elif self.saw_lock and int(msg.lock_state) == int(TargetState.LOCK_NO_TARGET):
            if self.release_deadline is None:
                self.release_deadline = time.monotonic() + self.args.release_tail_sec

    def on_camera_info(self, msg):
        super().on_camera_info(msg)
        self.camera_info.update({
            'width': int(msg.width),
            'height': int(msg.height),
            'frame_id': str(msg.header.frame_id),
        })


def measurement_sequence(capture):
    rows = []
    unique_debug = {float(row['stamp']): row for row in capture.debug_rows}
    for stamp in sorted(unique_debug):
        debug = unique_debug[stamp]
        anchor = nearest(capture.anchors, stamp, capture.args.state_sync_tolerance)
        tracklet_array = nearest(
            capture.tracklets, stamp, capture.args.state_sync_tolerance)
        selected_id = int(debug.get('selected_lidar_tracklet_id', -1))
        selected = None
        if tracklet_array is not None and selected_id >= 0:
            selected = next((
                row for row in tracklet_array['tracklets']
                if int(row['id']) == selected_id and row['active']), None)
        rows.append({
            'stamp': stamp,
            'camera_anchor': anchor['position'][:2] if anchor is not None else None,
            'selected_lidar': selected['position'][:2] if selected is not None else None,
            'selected_tracklet_id': selected_id,
            'measurement_source': debug.get('measurement_source', 'unknown'),
            'measurement_accepted': bool(debug.get('measurement_accepted', False)),
            'imm_probabilities': debug.get('imm_probabilities'),
        })
    return rows


def topic_rates(capture):
    sources = {
        'fused_target_state': capture.states,
        'lidar_tracklet_arrays': capture.tracklets,
        'camera_guided_anchors': capture.anchors,
        'camera_targets': capture.camera_targets,
        'tracker_debug': capture.debug_rows,
    }
    rates = {}
    for name, rows in sources.items():
        stamps = sorted(float(row['stamp']) for row in rows)
        span = stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0
        rates[name] = {
            'count': len(stamps),
            'stamp_span_sec': span,
            'observed_rate_hz': float((len(stamps) - 1) / span) if span > 0 else None,
        }
    return rates


def main():
    args = parse_args()
    resources = ResourceSampler(args.idle_duration, args.resource_interval)
    resources.start_sampling()
    rclpy.init()
    capture = ExtendedCapture(args)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(capture, timeout_sec=0.2)
            if capture.release_deadline is not None and time.monotonic() >= capture.release_deadline:
                break
    finally:
        capture.destroy_node()
        rclpy.shutdown()
        resources.stop_sampling()

    record = build_record(capture)
    idle_rows = [row for row in resources.device_rows if row['phase'] == 'idle']
    replay_rows = [row for row in resources.device_rows if row['phase'] == 'replay']
    process_rows = [row for row in resources.process_rows if row['phase'] == 'replay']
    extended = {
        'schema_version': 'human_following_extended_evidence/1.0.0',
        'source': 'recorded_rosbag_replay_plus_jetson_telemetry',
        'bag': args.bag,
        'run_index': int(args.run_index),
        'playback_rate': float(args.playback_rate),
        'idle_duration_sec': float(args.idle_duration),
        'states': record['states'],
        'debug_samples': record['debug_samples'],
        'measurements': measurement_sequence(capture),
        'camera_info': capture.camera_info,
        'topic_rates': topic_rates(capture),
        'resources': {
            'device_samples': resources.device_rows,
            'process_samples': process_rows,
            'summary': summarize_resources(idle_rows, replay_rows, process_rows),
            'scope_note': (
                'Device telemetry includes all host workloads; per-node samples identify only '
                'the three human-tracking processes.'),
        },
        'configuration': record['configuration'],
        'limitations': [
            'No physical sensors or robot motion were started.',
            'No external trajectory ground truth is available.',
            'Estimator ablations share production target authorization and association.',
        ],
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(extended, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({
        'output': str(output),
        'states': len(extended['states']),
        'measurements': len(extended['measurements']),
        'device_samples': len(resources.device_rows),
        'process_samples': len(process_rows),
        'locked': record['summary']['locked'],
    }))


if __name__ == '__main__':
    main()
