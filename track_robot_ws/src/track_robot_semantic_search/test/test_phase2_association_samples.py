import importlib.util
import json
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / 'scripts' / 'export_phase2_association_samples.py'
SCHEMA_PATH = (
    PACKAGE_ROOT / 'schemas' / 'phase2_association_samples.schema.json')
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
CALIBRATION_REPORT = (
    WORKSPACE_ROOT / 'rosbags' / 'semantic_search' / 'reports' /
    'phase2_association_calibration_2026-07-16.json')
CALIBRATION_REVIEW = (
    WORKSPACE_ROOT / 'rosbags' / 'semantic_search' / 'calibration' /
    'phase2c_manual_pilot_review.json')
ASSOCIATION_BASELINE = (
    WORKSPACE_ROOT / 'src' / 'track_robot' /
    'track_robot_semantic_memory' / 'config' /
    'phase2_association_baseline.yaml')


def load_exporter():
    spec = importlib.util.spec_from_file_location(
        'export_phase2_association_samples', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def debug_pair(pair_id, score, raw_iou, decision='unmatched'):
    return {
        'schema_version': '1.0.0',
        'pair_id': pair_id,
        'visual_stamp_ns': 1_000_000_000,
        'lidar_stamp_ns': 1_020_000_000,
        'memory_epoch_id': 7,
        'observation_producer_epoch_id': 4,
        'visual_candidate_id': int(pair_id.split('-')[-1]),
        'lidar_source_epoch_id': 9,
        'lidar_tracklet_id': 12,
        'decision': decision,
        'total_score': score,
        'terms': [{
            'name': 'projected_iou',
            'valid': True,
            'hard_gate': False,
            'gate_passed': True,
            'raw_value': raw_iou,
            'normalized_value': raw_iou,
            'weight': 1.0,
            'contribution': raw_iou,
        }],
    }


def hard_gate(name, passed):
    return {
        'name': name,
        'valid': True,
        'hard_gate': True,
        'gate_passed': passed,
        'raw_value': 1.0 if passed else 2.0,
        'normalized_value': 1.0 if passed else 0.0,
        'weight': 0.0,
        'contribution': 0.0,
    }


def write_jsonl(path, rows):
    path.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8')


def test_schema_is_strict_and_preserves_annotation_state():
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))

    assert schema['additionalProperties'] is False
    assert schema['properties']['annotation']['enum'] == [
        'positive', 'negative', 'unlabelled']
    assert {'pair_id', 'terms', 'annotation'} <= set(schema['required'])
    assert schema['properties']['terms']['maxItems'] == 24


def test_export_joins_annotations_and_writes_stable_distributions(tmp_path):
    exporter = load_exporter()
    debug_path = tmp_path / 'debug.jsonl'
    annotation_path = tmp_path / 'annotations.jsonl'
    output_path = tmp_path / 'samples.jsonl'
    report_path = tmp_path / 'report.json'
    write_jsonl(debug_path, [
        debug_pair('pair-2', 0.85, 0.8),
        debug_pair('pair-1', 0.20, 0.1),
        debug_pair('pair-3', 0.95, 0.9),
        debug_pair('pair-4', 0.10, 0.2),
    ])
    write_jsonl(annotation_path, [
        {'pair_id': 'pair-1', 'annotation': 'negative'},
        {'pair_id': 'pair-2', 'annotation': 'positive'},
        {'pair_id': 'pair-3', 'annotation': 'positive'},
        {'pair_id': 'pair-4', 'annotation': 'negative'},
    ])

    report = exporter.export_samples(
        debug_path=debug_path,
        annotation_path=annotation_path,
        output_path=output_path,
        report_path=report_path,
        dataset_id='pilot-a',
        minimum_labeled_per_class=2)

    samples = [json.loads(line) for line in output_path.read_text(
        encoding='utf-8').splitlines()]
    assert [sample['pair_id'] for sample in samples] == [
        'pair-1', 'pair-2', 'pair-3', 'pair-4']
    assert report['status'] == 'calibrated'
    assert report['camera_attachment_allowed'] is True
    assert report['counts'] == {
        'total': 4, 'positive': 2, 'negative': 2, 'unlabelled': 0}
    distribution = report['term_distributions']['projected_iou']
    assert distribution['positive']['median'] == pytest.approx(0.85)
    assert distribution['negative']['median'] == pytest.approx(0.15)
    assert report['selected_parameters']['match_threshold'] == pytest.approx(
        0.5)
    assert report['selected_parameters'][
        'term_weights_from_median_separation'] == {'projected_iou': 1.0}
    assert report['hard_gate_pass_counts'] == {
        'positive': 2, 'negative': 2}
    assert json.loads(report_path.read_text(encoding='utf-8')) == report


def test_unlabelled_or_insufficient_data_cannot_enable_attachment(tmp_path):
    exporter = load_exporter()
    debug_path = tmp_path / 'debug.jsonl'
    output_path = tmp_path / 'samples.jsonl'
    report_path = tmp_path / 'report.json'
    write_jsonl(debug_path, [debug_pair('pair-1', 0.8, 0.7)])

    report = exporter.export_samples(
        debug_path=debug_path,
        annotation_path=None,
        output_path=output_path,
        report_path=report_path,
        dataset_id='pilot-missing',
        minimum_labeled_per_class=2)

    assert report['status'] == 'not_calibrated'
    assert report['camera_attachment_allowed'] is False
    assert report['selected_parameters'] is None
    assert 'insufficient_positive_annotations' in report['reasons']
    assert 'insufficient_negative_annotations' in report['reasons']


def test_duplicate_annotations_and_nonfinite_values_are_rejected(tmp_path):
    exporter = load_exporter()
    debug_path = tmp_path / 'debug.jsonl'
    annotations = tmp_path / 'annotations.jsonl'
    write_jsonl(debug_path, [debug_pair('pair-1', 0.8, 0.7)])
    write_jsonl(annotations, [
        {'pair_id': 'pair-1', 'annotation': 'positive'},
        {'pair_id': 'pair-1', 'annotation': 'negative'},
    ])

    with pytest.raises(ValueError, match='duplicate annotation'):
        exporter.export_samples(
            debug_path, annotations, tmp_path / 'out.jsonl',
            tmp_path / 'report.json', 'pilot')

    row = debug_pair('pair-2', float('nan'), 0.7)
    write_jsonl(debug_path, [row])
    with pytest.raises(ValueError, match='finite'):
        exporter.export_samples(
            debug_path, None, tmp_path / 'out.jsonl',
            tmp_path / 'report.json', 'pilot')


def test_positive_pairs_rejected_by_valid_hard_gates_cannot_calibrate(tmp_path):
    exporter = load_exporter()
    debug_path = tmp_path / 'debug.jsonl'
    annotations = tmp_path / 'annotations.jsonl'
    rows = [
        debug_pair('pair-1', 0.9, 0.8),
        debug_pair('pair-2', 0.8, 0.7),
        debug_pair('pair-3', 0.2, 0.1),
        debug_pair('pair-4', 0.1, 0.2),
    ]
    rows[0]['terms'].append(hard_gate('size_ratio', False))
    rows[1]['terms'].append(hard_gate('size_ratio', True))
    write_jsonl(debug_path, rows)
    write_jsonl(annotations, [
        {'pair_id': 'pair-1', 'annotation': 'positive'},
        {'pair_id': 'pair-2', 'annotation': 'positive'},
        {'pair_id': 'pair-3', 'annotation': 'negative'},
        {'pair_id': 'pair-4', 'annotation': 'negative'},
    ])

    report = exporter.export_samples(
        debug_path, annotations, tmp_path / 'out.jsonl',
        tmp_path / 'report.json', 'hard-gate-pilot',
        minimum_labeled_per_class=2)

    assert report['status'] == 'not_calibrated'
    assert report['hard_gate_pass_counts']['positive'] == 1
    assert 'insufficient_positive_hard_gate_passes' in report['reasons']


def test_checked_in_baseline_is_bound_to_calibrated_pilot_report():
    report = json.loads(CALIBRATION_REPORT.read_text(encoding='utf-8'))
    review = json.loads(CALIBRATION_REVIEW.read_text(encoding='utf-8'))
    baseline = yaml.safe_load(
        ASSOCIATION_BASELINE.read_text(encoding='utf-8'))
    parameters = baseline['semantic_memory']['ros__parameters']
    selected = report['selected_parameters']
    runtime_contract = report['runtime_contract']

    assert report['status'] == 'calibrated'
    assert report['camera_attachment_allowed'] is True
    assert report['counts']['positive'] >= 20
    assert report['counts']['negative'] >= 20
    assert report['hard_gate_pass_counts']['positive'] >= 20
    assert selected['association_metrics']['precision'] >= 0.95
    assert selected['association_metrics']['recall'] >= 0.80
    assert parameters['association_shadow_mode'] is True
    assert parameters['camera_attachment_enabled'] is False
    assert parameters['association_calibration_status'] == 'calibrated'
    assert parameters['association_match_threshold'] == pytest.approx(
        selected['match_threshold'])
    assert parameters['association_ambiguity_margin'] == pytest.approx(
        selected['ambiguity_margin'])
    assert runtime_contract['scoring_contract_version'] == \
        'stage2d_association_v1'
    assert parameters['camera_calibration_id'] == \
        runtime_contract['camera_calibration_id']
    assert parameters['association_maximum_size_ratio'] == pytest.approx(
        review['calibrated_size_ratio_maximum'])
    gate_parameter_names = {
        'max_source_time_delta_s': 'association_max_source_time_delta_sec',
        'max_evidence_age_s': 'association_max_evidence_age_sec',
    }
    for gate_name, gate_value in runtime_contract['hard_gates'].items():
        if gate_name.startswith('require_'):
            continue
        parameter_name = gate_parameter_names.get(
            gate_name, 'association_' + gate_name)
        assert parameters[parameter_name] == pytest.approx(gate_value)
    for term_name, weight in selected[
            'term_weights_from_median_separation'].items():
        assert parameters[
            'association_weight_' + term_name] == pytest.approx(weight)
    for term_name, weight in runtime_contract['soft_weights'].items():
        assert parameters[
            'association_weight_' + term_name] == pytest.approx(weight)
