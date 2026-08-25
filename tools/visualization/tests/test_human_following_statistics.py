import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from human_following_statistics import (  # noqa: E402
    aggregate_association,
    aggregate_funnel,
    align_repeatability,
    summarize_run,
)


def sample_record(locked=True, run_index=1):
    states = [
        {
            'stamp': 10.0,
            'target_id': 1 if locked else 0,
            'lock_state': 'TARGET_LOCKED' if locked else 'NO_TARGET',
            'association_state': 'CONFIRMED' if locked else 'NONE',
            'source_state': 'CAMERA_LIDAR',
            'selected_tracklet_id': 7 if locked else -1,
            'position_valid': locked,
            'position': [2.0, 0.1, 0.8],
            'distance': 2.0,
            'covariance_trace_xy': 0.2,
        },
        {
            'stamp': 10.5,
            'target_id': 0,
            'lock_state': 'NO_TARGET',
            'association_state': 'NONE',
            'source_state': 'NONE',
            'selected_tracklet_id': -1,
            'position_valid': False,
            'position': [0.0, 0.0, 0.0],
            'distance': 0.0,
            'covariance_trace_xy': 0.0,
        },
    ]
    return {
        'bag': 'bag-a',
        'run_index': run_index,
        'states': states,
        'associations': [{
            'anchor_distance_m': 0.1,
            'range_difference_m': 0.04,
            'projection_center_error_px': 30.0,
            'association_score': 0.81,
            'top_two_margin': 0.21,
        }] if locked else [],
        'debug_samples': [],
        'funnel_updates': [{
            'confirmed_evaluations': 5,
            'anchor_gate_pass': 3,
            'range_gate_pass': 2,
            'valid_projection': 2,
            'published_hypotheses': 1,
            'score_threshold_pass': 1,
            'selected': 1,
        }] if locked else [],
    }


def test_summarize_run_keeps_unsuccessful_bag_visible():
    summary = summarize_run(sample_record(locked=False))
    assert summary['bag'] == 'bag-a'
    assert summary['locked'] is False
    assert summary['confirmed_association_duration_sec'] == 0.0
    assert summary['safe_release'] is False


def test_association_aggregation_uses_finite_samples():
    record = sample_record()
    record['associations'].append({
        'anchor_distance_m': float('nan'),
        'range_difference_m': 0.2,
        'projection_center_error_px': None,
        'association_score': 0.7,
        'top_two_margin': None,
    })
    result = aggregate_association([record])
    assert result['anchor_distance_m'] == [0.1]
    assert result['association_score'] == [0.81, 0.7]


def test_funnel_is_monotonic_and_uses_configured_threshold():
    result = aggregate_funnel([sample_record()], score_threshold=0.65)
    assert result['score_threshold'] == pytest.approx(0.65)
    assert result['counts'] == [5, 3, 2, 2, 1, 1, 1]
    assert all(a >= b for a, b in zip(result['counts'], result['counts'][1:]))


def test_repeatability_alignment_starts_each_locked_run_at_zero():
    runs = align_repeatability([sample_record(run_index=1), sample_record(run_index=2)])
    assert len(runs) == 2
    assert runs[0]['time_sec'][0] == pytest.approx(0.0)
    assert runs[1]['time_sec'][0] == pytest.approx(0.0)
    assert runs[0]['tracklet_ids'] == [7]


def test_summary_and_alignment_use_primary_logical_target_episode():
    record = sample_record()
    record['states'].insert(1, {
        'stamp': 10.25,
        'target_id': 2,
        'lock_state': 'TARGET_LOCKED',
        'association_state': 'CONFIRMED',
        'source_state': 'CAMERA_LIDAR',
        'selected_tracklet_id': 99,
        'position_valid': True,
        'position': [4.0, 1.0, 0.8],
        'distance': 4.1,
        'covariance_trace_xy': 0.4,
    })
    record['states'].insert(1, dict(record['states'][0], stamp=10.1))
    summary = summarize_run(record)
    aligned = align_repeatability([record])[0]
    assert summary['primary_target_id'] == 1
    assert summary['selected_tracklet_ids'] == [7]
    assert aligned['tracklet_ids'] == [7]
