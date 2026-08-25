#!/usr/bin/env python3
"""Pure analysis helpers for extended human-following paper evidence."""

from collections import Counter, defaultdict
import math
import re
import xml.etree.ElementTree as ET

import numpy as np


def _finite(value):
    return isinstance(value, (int, float)) and np.isfinite(float(value))


def build_continuity_lanes(record):
    """Clip state/debug rows to the most persistent locked logical target."""
    states = sorted(record.get('states', []), key=lambda row: float(row.get('stamp', 0.0)))
    locked_ids = [
        int(row.get('target_id', 0)) for row in states
        if row.get('lock_state') == 'TARGET_LOCKED' and int(row.get('target_id', 0)) > 0
    ]
    if not locked_ids:
        return {
            'primary_target_id': None, 'time_sec': [], 'camera_visible': [],
            'lidar_visible': [], 'source_state': [], 'association_state': [],
            'lock_state': [], 'selected_tracklet_id': [],
            'measurement_events': [], 'release_time_sec': None,
        }
    primary = Counter(locked_ids).most_common(1)[0][0]
    start_index = next(
        index for index, row in enumerate(states)
        if int(row.get('target_id', 0)) == primary and row.get('lock_state') == 'TARGET_LOCKED')
    end_index = len(states) - 1
    for index in range(start_index + 1, len(states)):
        if states[index].get('lock_state') == 'NO_TARGET':
            end_index = index
            break
    episode = states[start_index:end_index + 1]
    start = float(episode[0]['stamp'])
    end = float(episode[-1]['stamp'])
    times = [round(float(row['stamp']) - start, 9) for row in episode]
    events = []
    for row in record.get('debug_samples', []):
        stamp = float(row.get('stamp', -math.inf))
        if start <= stamp <= end and isinstance(row.get('measurement_accepted'), bool):
            events.append({
                'time_sec': round(stamp - start, 9),
                'accepted': bool(row['measurement_accepted']),
            })
    released = episode[-1].get('lock_state') == 'NO_TARGET'
    return {
        'primary_target_id': primary,
        'time_sec': times,
        'camera_visible': [int(bool(row.get('camera_visible', False))) for row in episode],
        'lidar_visible': [int(bool(row.get('lidar_visible', False))) for row in episode],
        'source_state': [row.get('source_state', 'NONE') for row in episode],
        'association_state': [row.get('association_state', 'UNBOUND') for row in episode],
        'lock_state': [row.get('lock_state', 'NO_TARGET') for row in episode],
        'selected_tracklet_id': [int(row.get('selected_tracklet_id', -1)) for row in episode],
        'measurement_events': events,
        'release_time_sec': times[-1] if released else None,
    }


def parse_tegrastats_line(line, stamp=None):
    """Parse the stable fields emitted by NVIDIA Jetson tegrastats."""
    ram = re.search(r'RAM\s+(\d+)/(\d+)MB', line)
    cpu = re.search(r'CPU\s+\[([^]]+)\]', line)
    gpu = re.search(r'GR3D_FREQ\s+(\d+(?:\.\d+)?)%', line)
    emc = re.search(r'EMC_FREQ\s+(\d+(?:\.\d+)?)%', line)
    temp = re.search(r'CPU@(-?\d+(?:\.\d+)?)C', line)
    if not all((ram, cpu, gpu, emc)):
        raise ValueError('unsupported tegrastats line')
    percentages = []
    for token in cpu.group(1).split(','):
        match = re.search(r'(\d+(?:\.\d+)?)%', token)
        if match:
            percentages.append(float(match.group(1)))
    if not percentages:
        raise ValueError('tegrastats line has no active CPU cores')
    return {
        'stamp': None if stamp is None else float(stamp),
        'ram_used_mb': float(ram.group(1)),
        'ram_total_mb': float(ram.group(2)),
        'cpu_core_pct': percentages,
        'cpu_mean_pct': float(np.mean(percentages)),
        'gpu_pct': float(gpu.group(1)),
        'emc_pct': float(emc.group(1)),
        'cpu_temp_c': float(temp.group(1)) if temp else None,
    }


def _median(rows, key):
    values = [float(row[key]) for row in rows if _finite(row.get(key))]
    return float(np.median(values)) if values else None


def _percentile(rows, key, percentile):
    values = [float(row[key]) for row in rows if _finite(row.get(key))]
    return float(np.percentile(values, percentile)) if values else None


def summarize_resources(idle_rows, replay_rows, process_rows):
    idle_cpu = _median(idle_rows, 'cpu_mean_pct') or 0.0
    idle_gpu = _median(idle_rows, 'gpu_pct') or 0.0
    idle_ram = _median(idle_rows, 'ram_used_mb') or 0.0
    replay_cpu = _median(replay_rows, 'cpu_mean_pct') or 0.0
    replay_gpu = _median(replay_rows, 'gpu_pct') or 0.0
    replay_ram = _median(replay_rows, 'ram_used_mb') or 0.0
    device = {
        'cpu_median_pct': replay_cpu,
        'cpu_p95_pct': _percentile(replay_rows, 'cpu_mean_pct', 95),
        'cpu_increment_over_idle_pct': replay_cpu - idle_cpu,
        'gpu_median_pct': replay_gpu,
        'gpu_p95_pct': _percentile(replay_rows, 'gpu_pct', 95),
        'gpu_increment_over_idle_pct': replay_gpu - idle_gpu,
        'ram_median_mb': replay_ram,
        'ram_increment_over_idle_mb': replay_ram - idle_ram,
        'cpu_temp_p95_c': _percentile(replay_rows, 'cpu_temp_c', 95),
        'emc_p95_pct': _percentile(replay_rows, 'emc_pct', 95),
    }
    grouped = defaultdict(list)
    for row in process_rows:
        grouped[str(row.get('node', 'unknown'))].append(row)
    nodes = {}
    for name, rows in sorted(grouped.items()):
        rss = [float(row['rss_mb']) for row in rows if _finite(row.get('rss_mb'))]
        nodes[name] = {
            'sample_count': len(rows),
            'cpu_median_pct': _median(rows, 'cpu_pct'),
            'cpu_p95_pct': _percentile(rows, 'cpu_pct', 95),
            'rss_median_mb': float(np.median(rss)) if rss else None,
            'rss_peak_mb': float(np.max(rss)) if rss else None,
        }
    return {'device': device, 'nodes': nodes}


def _numbers(element, attribute, default):
    if element is None or element.get(attribute) is None:
        return np.asarray(default, dtype=float)
    return np.asarray([float(value) for value in element.get(attribute).split()], dtype=float)


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = _rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis, angle):
    norm = np.linalg.norm(axis)
    if norm == 0:
        return np.eye(4)
    x, y, z = axis / norm
    c, s, d = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    rotation = np.asarray([
        [c + x*x*d, x*y*d - z*s, x*z*d + y*s],
        [y*x*d + z*s, c + y*y*d, y*z*d - x*s],
        [z*x*d - y*s, z*y*d + x*s, c + z*z*d],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def _axis_translation(axis, distance):
    result = np.eye(4)
    norm = np.linalg.norm(axis)
    if norm:
        result[:3, 3] = axis / norm * distance
    return result


def sample_urdf_workspace(urdf_xml, root_link, tip_links, sample_count=50000, seed=7):
    """Sample kinematic tip positions from a serial/tree URDF."""
    root = ET.fromstring(urdf_xml)
    by_child = {}
    for element in root.findall('joint'):
        origin = element.find('origin')
        limit = element.find('limit')
        joint = {
            'name': element.get('name'),
            'type': element.get('type', 'fixed'),
            'parent': element.find('parent').get('link'),
            'child': element.find('child').get('link'),
            'xyz': _numbers(origin, 'xyz', (0, 0, 0)),
            'rpy': _numbers(origin, 'rpy', (0, 0, 0)),
            'axis': _numbers(element.find('axis'), 'xyz', (1, 0, 0)),
            'lower': float(limit.get('lower', '0')) if limit is not None else 0.0,
            'upper': float(limit.get('upper', '0')) if limit is not None else 0.0,
        }
        by_child[joint['child']] = joint

    chains = {}
    variable_names = []
    for tip in tip_links:
        chain = []
        child = tip
        visited = set()
        while child != root_link:
            if child in visited or child not in by_child:
                raise ValueError(f'no URDF chain from {root_link} to {tip}')
            visited.add(child)
            joint = by_child[child]
            chain.append(joint)
            child = joint['parent']
        chain.reverse()
        chains[tip] = chain
        for joint in chain:
            if joint['type'] in ('revolute', 'continuous', 'prismatic') and joint['name'] not in variable_names:
                variable_names.append(joint['name'])

    rng = np.random.default_rng(seed)
    values = {}
    all_joints = {joint['name']: joint for chain in chains.values() for joint in chain}
    for name in variable_names:
        joint = all_joints[name]
        values[name] = rng.uniform(joint['lower'], joint['upper'], int(sample_count))

    points = {tip: [] for tip in tip_links}
    for sample_index in range(int(sample_count)):
        for tip, chain in chains.items():
            matrix = np.eye(4)
            for joint in chain:
                matrix = matrix @ _transform(joint['xyz'], joint['rpy'])
                if joint['type'] in ('revolute', 'continuous'):
                    matrix = matrix @ _axis_rotation(joint['axis'], values[joint['name']][sample_index])
                elif joint['type'] == 'prismatic':
                    matrix = matrix @ _axis_translation(joint['axis'], values[joint['name']][sample_index])
            points[tip].append(matrix[:3, 3].astype(float).tolist())
    return {
        'sample_count': int(sample_count),
        'seed': int(seed),
        'joint_names': variable_names,
        'joint_limits': {
            name: [all_joints[name]['lower'], all_joints[name]['upper']]
            for name in variable_names
        },
        'points': points,
    }


class _CvKalman:
    def __init__(self, accel_std=1.2):
        self.accel_std = float(accel_std)
        self.x = None
        self.p = None
        self.stamp = None
        self.last_measurement_stamp = None

    def predict(self, stamp):
        stamp = float(stamp)
        if self.x is None:
            self.stamp = stamp
            return
        dt = max(0.0, stamp - self.stamp)
        f = np.asarray([[1, 0, dt, 0], [0, 1, 0, dt],
                        [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        g = np.asarray([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt],
                        [dt, 0], [0, dt]], dtype=float)
        q = g @ (np.eye(2) * self.accel_std ** 2) @ g.T
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q
        self.stamp = stamp

    def update(self, measurement, variance, stamp):
        z = np.asarray(measurement, dtype=float)[:2]
        if self.x is None:
            self.x = np.asarray([z[0], z[1], 0.0, 0.0])
            self.p = np.diag([0.35, 0.35, 1.0, 1.0])
            self.stamp = float(stamp)
            self.last_measurement_stamp = float(stamp)
            return
        h = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        r = np.eye(2) * float(variance)
        innovation = z - h @ self.x
        innovation_covariance = h @ self.p @ h.T + r
        gain = self.p @ h.T @ np.linalg.inv(innovation_covariance)
        self.x = self.x + gain @ innovation
        self.p = (np.eye(4) - gain @ h) @ self.p
        self.last_measurement_stamp = float(stamp)


def _condition_metrics(times, xy, covariance, valid, production_xy, production_valid):
    valid_array = np.asarray(valid, dtype=bool)
    covariance_array = np.asarray(covariance, dtype=float)
    xy_array = np.asarray(xy, dtype=float)
    comparison = valid_array & np.asarray(production_valid, dtype=bool)
    deviations = np.linalg.norm(xy_array[comparison] - production_xy[comparison], axis=1)
    smoothness = None
    selected_times = np.asarray(times, dtype=float)[valid_array]
    selected_xy = xy_array[valid_array]
    if len(selected_xy) >= 3 and np.all(np.diff(selected_times) > 0):
        velocity = np.diff(selected_xy, axis=0) / np.diff(selected_times)[:, None]
        if len(velocity) >= 2:
            velocity_times = 0.5 * (selected_times[1:] + selected_times[:-1])
            acceleration = np.diff(velocity, axis=0) / np.diff(velocity_times)[:, None]
            smoothness = float(np.sqrt(np.mean(np.sum(acceleration ** 2, axis=1))))
    finite_covariance = covariance_array[np.isfinite(covariance_array) & valid_array]
    return {
        'trajectory_rms_mutual_deviation_m': float(np.sqrt(np.mean(deviations ** 2))) if len(deviations) else None,
        'covariance_trace_median_m2': float(np.median(finite_covariance)) if len(finite_covariance) else None,
        'acceleration_rms_mps2': smoothness,
    }


def run_estimator_ablation(measurements, production_states, timeout_sec=2.0):
    """Compare masked CV-KF inputs with the production IMM output."""
    measurements = sorted(measurements, key=lambda row: float(row['stamp']))
    times = np.asarray(sorted({float(row['stamp']) for row in measurements}), dtype=float)
    if not len(times):
        return {'time_sec': [], 'conditions': {}}
    measurement_by_stamp = {float(row['stamp']): row for row in measurements}
    production = [
        row for row in sorted(production_states, key=lambda row: float(row['stamp']))
        if row.get('position_valid', True) and len(row.get('position', [])) >= 2
    ]
    production_times = np.asarray([float(row['stamp']) for row in production], dtype=float)
    production_xy = np.column_stack([
        np.interp(times, production_times, [float(row['position'][0]) for row in production]),
        np.interp(times, production_times, [float(row['position'][1]) for row in production]),
    ])
    production_cov = np.interp(
        times, production_times,
        [float(row.get('covariance_trace_xy', np.nan)) for row in production])
    production_valid = np.ones(len(times), dtype=bool)

    conditions = {}
    configurations = [
        ('camera_guided_anchor_only', True, False),
        ('selected_lidar_only', False, True),
        ('cv_kf_fusion', True, True),
    ]
    for name, use_camera, use_lidar in configurations:
        filter_ = _CvKalman()
        xy, covariance, valid = [], [], []
        for stamp in times:
            filter_.predict(stamp)
            row = measurement_by_stamp[float(stamp)]
            camera = row.get('camera_anchor')
            lidar = row.get('selected_lidar')
            if use_camera and isinstance(camera, (list, tuple)) and len(camera) >= 2:
                filter_.update(camera, 0.08, stamp)
            if use_lidar and isinstance(lidar, (list, tuple)) and len(lidar) >= 2:
                filter_.update(lidar, 0.18, stamp)
            is_valid = (
                filter_.x is not None and filter_.last_measurement_stamp is not None and
                stamp - filter_.last_measurement_stamp <= float(timeout_sec))
            valid.append(bool(is_valid))
            xy.append(filter_.x[:2].astype(float).tolist() if filter_.x is not None else [np.nan, np.nan])
            covariance.append(float(filter_.p[0, 0] + filter_.p[1, 1]) if filter_.p is not None else np.nan)
        metrics = _condition_metrics(
            times, xy, covariance, valid, production_xy, production_valid)
        conditions[name] = {
            'sample_count': int(len(times)),
            'continuity_fraction': float(np.mean(valid)),
            'xy': xy,
            'covariance_trace_xy': covariance,
            'valid': valid,
            'metrics': metrics,
        }

    production_metrics = _condition_metrics(
        times, production_xy, production_cov, production_valid,
        production_xy, production_valid)
    conditions['production_imm'] = {
        'sample_count': int(len(times)),
        'continuity_fraction': 1.0,
        'xy': production_xy.astype(float).tolist(),
        'covariance_trace_xy': production_cov.astype(float).tolist(),
        'valid': production_valid.tolist(),
        'metrics': production_metrics,
    }
    relative_times = (times - times[0]).astype(float).tolist()
    return {
        'time_sec': relative_times,
        'conditions': conditions,
        'comparison_reference': 'production_imm_mutual_deviation_not_ground_truth',
        'timeout_sec': float(timeout_sec),
    }
