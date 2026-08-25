#!/usr/bin/env python3
"""Pure aggregation helpers for human-following replay evidence."""

from collections import Counter
import math

import numpy as np


FUNNEL_STAGES = [
    'confirmed_evaluations',
    'anchor_gate_pass',
    'range_gate_pass',
    'valid_projection',
    'published_hypotheses',
    'score_threshold_pass',
    'selected',
]
FUNNEL_LABELS = [
    'confirmed candidate\nevaluations',
    'anchor XY\ngate',
    'range\ngate',
    'valid camera\nprojection',
    'published\nhypotheses',
    'score\nthreshold',
    'selected\ntarget',
]


def is_finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def finite_values(rows, key):
    return [float(row[key]) for row in rows if is_finite_number(row.get(key))]


def elapsed_duration(rows, predicate, maximum_step_sec=0.5):
    duration = 0.0
    ordered = sorted(rows, key=lambda row: float(row.get('stamp', 0.0)))
    for current, following in zip(ordered, ordered[1:]):
        step = float(following['stamp']) - float(current['stamp'])
        if predicate(current) and 0.0 <= step <= maximum_step_sec:
            duration += step
    return duration


def summarize_run(record):
    states = sorted(record.get('states', []), key=lambda row: float(row.get('stamp', 0.0)))
    locked_target_counts = Counter(
        int(row.get('target_id', -1)) for row in states
        if row.get('lock_state') == 'TARGET_LOCKED' and int(row.get('target_id', -1)) > 0)
    primary_target_id = (
        locked_target_counts.most_common(1)[0][0] if locked_target_counts else None)
    locked_states = [
        row for row in states
        if row.get('lock_state') == 'TARGET_LOCKED' and
        (primary_target_id is None or int(row.get('target_id', -1)) == primary_target_id)]
    locked = bool(locked_states)
    first_stamp = float(states[0]['stamp']) if states else None
    first_lock_stamp = float(locked_states[0]['stamp']) if locked_states else None
    episode_states = [
        row for row in states
        if primary_target_id is not None and int(row.get('target_id', -1)) == primary_target_id and
        float(row.get('stamp', 0.0)) >= (first_lock_stamp or 0.0)]
    selected_ids = []
    for row in episode_states:
        tracklet_id = int(row.get('selected_tracklet_id', -1))
        if tracklet_id >= 0 and (not selected_ids or tracklet_id != selected_ids[-1]):
            selected_ids.append(tracklet_id)
    source_counts = Counter(row.get('source_state', 'NONE') for row in episode_states)
    source_total = sum(source_counts.values())
    last_lock_stamp = float(locked_states[-1]['stamp']) if locked_states else None
    safe_release = bool(
        locked and any(
            row.get('lock_state') == 'NO_TARGET' and float(row['stamp']) > last_lock_stamp
            for row in states))
    return {
        'bag': record.get('bag', 'unknown'),
        'run_index': int(record.get('run_index', 1)),
        'primary_target_id': primary_target_id,
        'state_samples': len(states),
        'association_samples': len(record.get('associations', [])),
        'debug_samples': len(record.get('debug_samples', [])),
        'locked': locked,
        'first_lock_sec': (
            first_lock_stamp - first_stamp
            if first_lock_stamp is not None and first_stamp is not None else None),
        'locked_duration_sec': elapsed_duration(
            states, lambda row: row.get('lock_state') == 'TARGET_LOCKED'),
        'confirmed_association_duration_sec': elapsed_duration(
            episode_states, lambda row: row.get('association_state') == 'CONFIRMED'),
        'selected_tracklet_switches': max(0, len(selected_ids) - 1),
        'selected_tracklet_ids': selected_ids,
        'safe_release': safe_release,
        'source_fractions': {
            source: count / source_total for source, count in sorted(source_counts.items())
        } if source_total else {},
    }


def aggregate_association(records):
    rows = [row for record in records for row in record.get('associations', [])]
    keys = [
        'anchor_distance_m', 'range_difference_m',
        'projection_center_error_px', 'association_score', 'top_two_margin',
    ]
    result = {key: finite_values(rows, key) for key in keys}
    result['sample_count'] = len(rows)
    result['finite_sample_counts'] = {key: len(result[key]) for key in keys}
    return result


def aggregate_funnel(records, score_threshold=0.65):
    totals = {stage: 0 for stage in FUNNEL_STAGES}
    update_count = 0
    for record in records:
        for update in record.get('funnel_updates', []):
            values = [int(update.get(stage, 0)) for stage in FUNNEL_STAGES]
            if any(value < 0 for value in values):
                raise ValueError('funnel counts must be non-negative')
            if any(left < right for left, right in zip(values, values[1:])):
                raise ValueError(f'non-monotonic funnel update: {values}')
            for stage, value in zip(FUNNEL_STAGES, values):
                totals[stage] += value
            update_count += 1
    counts = [totals[stage] for stage in FUNNEL_STAGES]
    return {
        'stages': list(FUNNEL_STAGES),
        'labels': list(FUNNEL_LABELS),
        'counts': counts,
        'update_count': update_count,
        'score_threshold': float(score_threshold),
    }


def align_repeatability(records):
    aligned = []
    for record in records:
        all_states = sorted(
            record.get('states', []), key=lambda item: float(item.get('stamp', 0.0)))
        locked_target_counts = Counter(
            int(row.get('target_id', -1)) for row in all_states
            if row.get('lock_state') == 'TARGET_LOCKED' and int(row.get('target_id', -1)) > 0)
        if not locked_target_counts:
            continue
        primary_target_id = locked_target_counts.most_common(1)[0][0]
        first_lock = next(
            float(row['stamp']) for row in all_states
            if row.get('lock_state') == 'TARGET_LOCKED' and
            int(row.get('target_id', -1)) == primary_target_id)
        valid = [
            row for row in sorted(
                all_states, key=lambda item: float(item.get('stamp', 0.0)))
            if int(row.get('target_id', -1)) == primary_target_id and
            float(row.get('stamp', 0.0)) >= first_lock and
            bool(row.get('position_valid', False)) and
            len(row.get('position', [])) >= 2 and
            all(is_finite_number(value) for value in row['position'][:2])
        ]
        if not valid:
            continue
        origin = float(valid[0]['stamp'])
        tracklet_ids = []
        for row in valid:
            value = int(row.get('selected_tracklet_id', -1))
            if value >= 0 and value not in tracklet_ids:
                tracklet_ids.append(value)
        aligned.append({
            'bag': record.get('bag', 'unknown'),
            'run_index': int(record.get('run_index', 1)),
            'primary_target_id': primary_target_id,
            'time_sec': [float(row['stamp']) - origin for row in valid],
            'x_m': [float(row['position'][0]) for row in valid],
            'y_m': [float(row['position'][1]) for row in valid],
            'range_m': [float(row.get('distance', np.nan)) for row in valid],
            'covariance_trace_xy': [
                float(row.get('covariance_trace_xy', np.nan)) for row in valid],
            'tracklet_ids': tracklet_ids,
        })
    return aligned


def percentile_summary(values):
    array = np.asarray([value for value in values if is_finite_number(value)], dtype=float)
    if not len(array):
        return {'count': 0, 'median': None, 'p90': None, 'p95': None, 'maximum': None}
    return {
        'count': int(len(array)),
        'median': float(np.median(array)),
        'p90': float(np.percentile(array, 90)),
        'p95': float(np.percentile(array, 95)),
        'maximum': float(np.max(array)),
    }
