#!/usr/bin/env python3

import argparse
import json
import math
import os
from pathlib import Path
import statistics


ANNOTATIONS = {'positive', 'negative', 'unlabelled'}
DECISIONS = {
    'rejected_gate', 'unmatched', 'tentative', 'matched', 'ambiguous'}
MINIMUM_ASSOCIATION_PRECISION = 0.95
MINIMUM_ASSOCIATION_RECALL = 0.80
SOFT_TERM_NAMES = (
    'position_consistency', 'projected_centroid', 'inside_fraction',
    'projected_iou', 'visual_cosine', 'extent_consistency',
    'point_count_consistency', 'motion_continuity',
    'previous_association', 'detector_confidence',
    'geometry_confidence', 'sensor_confidence')


def _read_jsonl(path):
    rows = []
    with Path(path).open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f'invalid JSON at {path}:{line_number}: {error}') from error
            if not isinstance(row, dict):
                raise ValueError(f'row at {path}:{line_number} must be an object')
            _require_finite(row, f'{path}:{line_number}')
            rows.append(row)
    return rows


def _require_finite(value, location):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f'{location} contains a non-finite number')
    if isinstance(value, dict):
        for key, child in value.items():
            _require_finite(child, f'{location}.{key}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _require_finite(child, f'{location}[{index}]')


def _atomic_write_text(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(content, encoding='utf-8')
    os.replace(str(temporary), str(path))


def _annotations(path):
    if path is None:
        return {}
    result = {}
    for row in _read_jsonl(path):
        pair_id = row.get('pair_id')
        annotation = row.get('annotation')
        if not isinstance(pair_id, str) or not pair_id:
            raise ValueError('annotation pair_id must be a non-empty string')
        if annotation not in ANNOTATIONS - {'unlabelled'}:
            raise ValueError(f'invalid annotation for {pair_id}')
        if pair_id in result:
            raise ValueError(f'duplicate annotation for {pair_id}')
        result[pair_id] = annotation
    return result


def _validate_debug_pair(row):
    required = {
        'schema_version', 'pair_id', 'visual_stamp_ns', 'lidar_stamp_ns',
        'memory_epoch_id', 'observation_producer_epoch_id',
        'visual_candidate_id', 'lidar_source_epoch_id', 'lidar_tracklet_id',
        'decision', 'total_score', 'terms'}
    missing = sorted(required - set(row))
    if missing:
        raise ValueError(f'debug pair is missing fields: {missing}')
    if row['schema_version'] != '1.0.0':
        raise ValueError('unsupported association sample schema_version')
    if not isinstance(row['pair_id'], str) or not row['pair_id']:
        raise ValueError('pair_id must be a non-empty string')
    if row['decision'] not in DECISIONS:
        raise ValueError(f'invalid decision for {row["pair_id"]}')
    if not isinstance(row['terms'], list) or len(row['terms']) > 24:
        raise ValueError('terms must be a bounded list of at most 24 items')
    term_names = []
    for term in row['terms']:
        if not isinstance(term, dict) or not isinstance(term.get('name'), str):
            raise ValueError('each association term requires a name')
        term_names.append(term['name'])
    if len(set(term_names)) != len(term_names):
        raise ValueError(f'duplicate term name in {row["pair_id"]}')


def _summary(values):
    if not values:
        return {'count': 0, 'minimum': None, 'median': None, 'maximum': None}
    ordered = sorted(values)
    return {
        'count': len(ordered),
        'minimum': ordered[0],
        'median': statistics.median(ordered),
        'maximum': ordered[-1],
    }


def _term_distributions(samples):
    names = sorted({
        term['name'] for sample in samples for term in sample['terms']})
    result = {}
    for name in names:
        classes = {}
        for annotation in ('positive', 'negative'):
            values = []
            for sample in samples:
                if sample['annotation'] != annotation:
                    continue
                for term in sample['terms']:
                    if (term['name'] == name and term.get('valid') and
                            term.get('raw_value') is not None):
                        values.append(float(term['raw_value']))
            classes[annotation] = _summary(values)
        result[name] = classes
    return result


def _normalized_soft_separations(samples):
    names = sorted({
        term['name'] for sample in samples for term in sample['terms']
        if not term.get('hard_gate', False)})
    result = {}
    for name in names:
        medians = {}
        for annotation in ('positive', 'negative'):
            values = []
            for sample in samples:
                if sample['annotation'] != annotation:
                    continue
                for term in sample['terms']:
                    if (term['name'] == name and not term.get('hard_gate') and
                            term.get('valid') and
                            term.get('normalized_value') is not None):
                        values.append(float(term['normalized_value']))
            medians[annotation] = (
                statistics.median(values) if values else None)
        if medians['positive'] is not None and medians['negative'] is not None:
            result[name] = abs(
                medians['positive'] - medians['negative'])
    return result


def _passes_valid_hard_gates(sample):
    return all(
        term.get('gate_passed', False)
        for term in sample['terms']
        if term.get('hard_gate', False) and term.get('valid', False))


def _selected_parameters(samples):
    separations = _normalized_soft_separations(samples)
    separation_total = sum(separations.values())
    if separation_total > 0.0:
        weights = {
            name: value / separation_total
            for name, value in sorted(separations.items())}
    else:
        weights = {name: 0.0 for name in sorted(separations)}
    calibrated_scores = {'positive': [], 'negative': []}
    for sample in samples:
        annotation = sample['annotation']
        if annotation not in calibrated_scores:
            continue
        score = 0.0
        for term in sample['terms']:
            name = term['name']
            if (name in weights and not term.get('hard_gate') and
                    term.get('valid') and
                    term.get('normalized_value') is not None):
                score += weights[name] * float(term['normalized_value'])
        calibrated_scores[annotation].append(score)
    positive_median = statistics.median(calibrated_scores['positive'])
    negative_median = statistics.median(calibrated_scores['negative'])
    score_gap = positive_median - negative_median
    unique_scores = sorted(set(
        calibrated_scores['positive'] + calibrated_scores['negative']))
    thresholds = [unique_scores[0]]
    thresholds.extend(
        0.5 * (left + right)
        for left, right in zip(unique_scores, unique_scores[1:]))
    threshold_metrics = []
    for threshold in thresholds:
        true_positive = sum(
            score >= threshold for score in calibrated_scores['positive'])
        false_negative = len(calibrated_scores['positive']) - true_positive
        false_positive = sum(
            score >= threshold for score in calibrated_scores['negative'])
        precision = true_positive / (true_positive + false_positive)
        recall = true_positive / (true_positive + false_negative)
        threshold_metrics.append({
            'threshold': threshold,
            'true_positive': true_positive,
            'false_positive': false_positive,
            'false_negative': false_negative,
            'precision': precision,
            'recall': recall,
        })
    qualifying = [
        row for row in threshold_metrics
        if row['precision'] >= MINIMUM_ASSOCIATION_PRECISION and
        row['recall'] >= MINIMUM_ASSOCIATION_RECALL]
    selected_metrics = min(
        qualifying, key=lambda row: row['threshold']) if qualifying else max(
        threshold_metrics,
        key=lambda row: (row['precision'], row['recall'], -row['threshold']))
    return {
        'match_threshold': selected_metrics['threshold'],
        'ambiguity_margin': max(0.01, 0.1 * score_gap),
        'association_metrics': selected_metrics,
        'association_metric_gates': {
            'minimum_precision': MINIMUM_ASSOCIATION_PRECISION,
            'minimum_recall': MINIMUM_ASSOCIATION_RECALL,
        },
        'calibrated_score_summary': {
            annotation: _summary(values)
            for annotation, values in calibrated_scores.items()
        },
        'term_weights_from_median_separation': weights,
        'selection_method': (
            'annotated_normalized_soft_precision_recall_gate_v3'),
    }


def _runtime_contract(selected):
    selected_weights = selected['term_weights_from_median_separation']
    return {
        'scoring_contract_version': 'stage2d_association_v1',
        'camera_calibration_id': 'zed_left_rectified_v1',
        'hard_gates': {
            'max_source_time_delta_s': 0.10,
            'max_evidence_age_s': 0.50,
            'max_position_nis': 9.21,
            'minimum_size_ratio': 0.25,
            'maximum_size_ratio': 40.0,
            'max_relative_speed_mps': 3.0,
            'position_distance_max_m': 3.0,
            'center_distance_max_px': 200.0,
            'descriptor_normalization_tolerance': 0.0001,
            'require_position_nis': False,
            'require_size_ratio': False,
            'require_motion_gate': False,
            'require_descriptors': False,
        },
        'soft_weights': {
            name: selected_weights.get(name, 0.0)
            for name in SOFT_TERM_NAMES
        },
        'confirmation': {
            'confirmation_frames': 3,
            'detach_after_misses': 2,
            'previous_association_hysteresis': 0.02,
            'cooldown_frames': 2,
        },
    }


def export_samples(
        debug_path, annotation_path, output_path, report_path, dataset_id,
        minimum_labeled_per_class=20):
    if not isinstance(dataset_id, str) or not dataset_id:
        raise ValueError('dataset_id must be a non-empty string')
    if minimum_labeled_per_class < 1:
        raise ValueError('minimum_labeled_per_class must be positive')

    annotations = _annotations(annotation_path)
    pairs = {}
    for row in _read_jsonl(debug_path):
        _validate_debug_pair(row)
        pair_id = row['pair_id']
        if pair_id in pairs:
            raise ValueError(f'duplicate pair_id: {pair_id}')
        sample = dict(row)
        sample['dataset_id'] = dataset_id
        sample['annotation'] = annotations.get(pair_id, 'unlabelled')
        pairs[pair_id] = sample

    samples = [pairs[pair_id] for pair_id in sorted(pairs)]
    output = ''.join(
        json.dumps(row, sort_keys=True, allow_nan=False) + '\n'
        for row in samples)
    _atomic_write_text(output_path, output)

    counts = {
        'total': len(samples),
        'positive': sum(row['annotation'] == 'positive' for row in samples),
        'negative': sum(row['annotation'] == 'negative' for row in samples),
        'unlabelled': sum(row['annotation'] == 'unlabelled' for row in samples),
    }
    reasons = []
    if counts['positive'] < minimum_labeled_per_class:
        reasons.append('insufficient_positive_annotations')
    if counts['negative'] < minimum_labeled_per_class:
        reasons.append('insufficient_negative_annotations')
    distributions = _term_distributions(samples)
    hard_gate_counts = {
        annotation: sum(
            row['annotation'] == annotation and _passes_valid_hard_gates(row)
            for row in samples)
        for annotation in ('positive', 'negative')
    }
    if hard_gate_counts['positive'] < minimum_labeled_per_class:
        reasons.append('insufficient_positive_hard_gate_passes')
    selected = None
    if not reasons:
        selected = _selected_parameters(samples)
        positive_median = selected[
            'calibrated_score_summary']['positive']['median']
        negative_median = selected[
            'calibrated_score_summary']['negative']['median']
        if positive_median <= negative_median:
            reasons.append('annotated_score_distributions_not_separable')
            selected = None
        elif selected['association_metrics']['precision'] < \
                MINIMUM_ASSOCIATION_PRECISION:
            reasons.append('association_precision_below_gate')
            selected = None
        elif selected['association_metrics']['recall'] < \
                MINIMUM_ASSOCIATION_RECALL:
            reasons.append('association_recall_below_gate')
            selected = None

    calibrated = not reasons
    report = {
        'schema_version': '1.0.0',
        'dataset_id': dataset_id,
        'status': 'calibrated' if calibrated else 'not_calibrated',
        'camera_attachment_allowed': calibrated,
        'minimum_labeled_per_class': minimum_labeled_per_class,
        'counts': counts,
        'hard_gate_pass_counts': hard_gate_counts,
        'reasons': reasons,
        'term_distributions': distributions,
        'selected_parameters': selected,
        'runtime_contract': _runtime_contract(selected) if calibrated else None,
    }
    _atomic_write_text(
        report_path,
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(
        description='Export and summarize Phase 2 shadow association pairs.')
    parser.add_argument('--debug-jsonl', required=True, type=Path)
    parser.add_argument('--annotations-jsonl', type=Path)
    parser.add_argument('--output-jsonl', required=True, type=Path)
    parser.add_argument('--report-json', required=True, type=Path)
    parser.add_argument('--dataset-id', required=True)
    parser.add_argument('--minimum-labelled-per-class', type=int, default=20)
    arguments = parser.parse_args()
    report = export_samples(
        arguments.debug_jsonl, arguments.annotations_jsonl,
        arguments.output_jsonl, arguments.report_json, arguments.dataset_id,
        arguments.minimum_labelled_per_class)
    return 0 if report['camera_attachment_allowed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
