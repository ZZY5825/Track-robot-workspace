import math
from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from human_following_extended_analysis import (
    build_continuity_lanes,
    parse_tegrastats_line,
    run_estimator_ablation,
    sample_urdf_workspace,
    summarize_resources,
)


def test_continuity_lanes_are_clipped_to_primary_lock_and_release():
    record = {
        'states': [
            {'stamp': 0.0, 'target_id': 0, 'lock_state': 'NO_TARGET'},
            {'stamp': 1.0, 'target_id': 7, 'lock_state': 'TARGET_LOCKED',
             'camera_visible': True, 'lidar_visible': True,
             'source_state': 'CAMERA_LIDAR', 'association_state': 'CONFIRMED',
             'selected_tracklet_id': 12},
            {'stamp': 2.0, 'target_id': 7, 'lock_state': 'TARGET_LOST',
             'camera_visible': False, 'lidar_visible': True,
             'source_state': 'LIDAR_ONLY', 'association_state': 'CONFIRMED',
             'selected_tracklet_id': 12},
            {'stamp': 3.0, 'target_id': 0, 'lock_state': 'NO_TARGET',
             'camera_visible': False, 'lidar_visible': False,
             'source_state': 'NONE', 'association_state': 'UNBOUND',
             'selected_tracklet_id': -1},
        ],
        'debug_samples': [
            {'stamp': 1.5, 'measurement_accepted': True},
            {'stamp': 2.5, 'measurement_accepted': False},
        ],
    }
    lanes = build_continuity_lanes(record)
    assert lanes['primary_target_id'] == 7
    assert lanes['time_sec'] == [0.0, 1.0, 2.0]
    assert lanes['camera_visible'] == [1, 0, 0]
    assert lanes['lidar_visible'] == [1, 1, 0]
    assert lanes['release_time_sec'] == 2.0
    assert lanes['measurement_events'] == [
        {'time_sec': 0.5, 'accepted': True},
        {'time_sec': 1.5, 'accepted': False},
    ]


def test_parse_tegrastats_line_ignores_off_cpu_cores():
    line = (
        'RAM 11136/30536MB (lfb 864x4MB) SWAP 0/0MB '
        'CPU [18%@729,off,14%@729,28%@729] EMC_FREQ 12% '
        'GR3D_FREQ 37% CPU@46.656C Tboard@35C')
    row = parse_tegrastats_line(line, stamp=4.0)
    assert row['stamp'] == 4.0
    assert row['ram_used_mb'] == 11136
    assert row['ram_total_mb'] == 30536
    assert row['cpu_mean_pct'] == 20.0
    assert row['gpu_pct'] == 37.0
    assert row['emc_pct'] == 12.0
    assert math.isclose(row['cpu_temp_c'], 46.656)


def test_resource_summary_reports_idle_adjusted_device_load_and_nodes():
    idle = [
        {'cpu_mean_pct': 10.0, 'gpu_pct': 0.0, 'ram_used_mb': 1000.0},
        {'cpu_mean_pct': 14.0, 'gpu_pct': 2.0, 'ram_used_mb': 1010.0},
    ]
    replay = [
        {'cpu_mean_pct': 30.0, 'gpu_pct': 40.0, 'ram_used_mb': 1200.0},
        {'cpu_mean_pct': 34.0, 'gpu_pct': 44.0, 'ram_used_mb': 1240.0},
    ]
    processes = [
        {'node': 'camera', 'cpu_pct': 80.0, 'rss_mb': 500.0},
        {'node': 'camera', 'cpu_pct': 100.0, 'rss_mb': 520.0},
        {'node': 'fusion', 'cpu_pct': 10.0, 'rss_mb': 100.0},
    ]
    summary = summarize_resources(idle, replay, processes)
    assert summary['device']['cpu_median_pct'] == 32.0
    assert summary['device']['cpu_increment_over_idle_pct'] == 20.0
    assert summary['device']['gpu_increment_over_idle_pct'] == 41.0
    assert summary['device']['ram_increment_over_idle_mb'] == 215.0
    assert summary['nodes']['camera']['cpu_median_pct'] == 90.0
    assert summary['nodes']['camera']['rss_peak_mb'] == 520.0


def test_urdf_workspace_follows_fixed_and_revolute_chain_deterministically():
    urdf = '''
    <robot name="tiny">
      <link name="base"/><link name="arm"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
        <limit lower="0" upper="0" effort="1" velocity="1"/>
      </joint>
      <joint name="tool" type="fixed">
        <origin xyz="1 0 0" rpy="0 0 0"/>
        <parent link="arm"/><child link="tip"/>
      </joint>
    </robot>'''
    result = sample_urdf_workspace(urdf, 'base', ['tip'], sample_count=4, seed=9)
    assert result['joint_names'] == ['j1']
    assert np.asarray(result['points']['tip']).shape == (4, 3)
    assert np.allclose(result['points']['tip'], [[2.0, 0.0, 0.0]] * 4)
    assert result['sample_count'] == 4


def test_estimator_ablation_returns_four_finite_conditions():
    measurements = [
        {'stamp': 0.0, 'camera_anchor': [1.0, 0.0], 'selected_lidar': [1.1, 0.0]},
        {'stamp': 1.0, 'camera_anchor': [2.0, 0.0], 'selected_lidar': [2.1, 0.0]},
        {'stamp': 2.0, 'camera_anchor': [3.0, 0.0], 'selected_lidar': [3.1, 0.0]},
    ]
    production = [
        {'stamp': 0.0, 'position': [1.05, 0.0, 0.0], 'position_valid': True,
         'covariance_trace_xy': 0.2},
        {'stamp': 1.0, 'position': [2.05, 0.0, 0.0], 'position_valid': True,
         'covariance_trace_xy': 0.1},
        {'stamp': 2.0, 'position': [3.05, 0.0, 0.0], 'position_valid': True,
         'covariance_trace_xy': 0.1},
    ]
    result = run_estimator_ablation(measurements, production, timeout_sec=2.0)
    assert list(result['conditions']) == [
        'camera_guided_anchor_only',
        'selected_lidar_only',
        'cv_kf_fusion',
        'production_imm',
    ]
    for condition in result['conditions'].values():
        assert condition['sample_count'] == 3
        assert condition['continuity_fraction'] == 1.0
        assert np.isfinite(condition['metrics']['trajectory_rms_mutual_deviation_m'])
        assert np.isfinite(condition['metrics']['covariance_trace_median_m2'])
