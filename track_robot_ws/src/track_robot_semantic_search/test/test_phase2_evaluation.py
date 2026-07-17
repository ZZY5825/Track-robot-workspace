import json
from pathlib import Path

import pytest

from track_robot_semantic_search.phase2_evaluation import build_phase2_report
from track_robot_semantic_search.phase2_evaluation_cli import run
from track_robot_semantic_search.task_threshold_calibration import (
    calibrate_task_threshold,
)


SCENARIOS = [
    'static_multi_view',
    'similar_static_objects',
    'moving_human_crossing',
    'camera_occlusion',
    'camera_fov_exit_lidar_visible',
    'both_sensors_exit_reentry',
    'lidar_cluster_split',
    'lidar_cluster_merge',
    'camera_false_positive',
    'lidar_false_cluster',
    'robot_rotation_translation',
    'task_change_without_memory_clear',
]


def annotation(stamp_ns, object_id, x, **extra):
    value = {
        'stamp_ns': stamp_ns,
        'object_id': object_id,
        'visibility': 'visible',
        'ignore': False,
        'position_3d': {'frame_id': 'odom', 'xyz_m': [x, 0.0, 0.0]},
    }
    value.update(extra)
    return value


def prediction(stamp_ns, object_id, global_id, x, **extra):
    value = {
        'stamp_ns': stamp_ns,
        'annotation_object_id': object_id,
        'memory_epoch_id': 7,
        'global_object_id': global_id,
        'association_expected': True,
        'association_decision': True,
        'association_correct': True,
        'position_m': [x, 0.0, 0.0],
    }
    value.update(extra)
    return value


def complete_inputs():
    annotations = []
    predictions = []
    for index in range(60):
        relevant = index < 30
        stamp = index * 1_000_000_000
        object_id = 'object-{}'.format(index)
        query_id = index + 1
        annotations.append(annotation(
            stamp, object_id, float(index), query_id=query_id,
            task_relevant=relevant))
        extra = {
            'query_id': query_id,
            'task_relevant': relevant,
            'task_rank': 1 if relevant else 2,
            'task_selected': relevant,
            'task_relevance': 0.9 if relevant else 0.1,
            'task_threshold': 0.9,
        }
        if index == 0:
            extra.update(
                reidentification_expected=True,
                reidentification_success=True)
        if index == 1:
            extra.update(
                stale_reactivation_expected=True,
                stale_reactivation_success=True)
        predictions.append(prediction(
            stamp, object_id, index + 1, float(index), **extra))
    runtime = {
        'duration_sec': 1800.0,
        'update_stamps_ns': [index * 100_000_000 for index in range(18_001)],
        'module_latency_ms': {
            'semantic_memory_core': [10.0, 20.0, 30.0],
            'phase1_inference_excluded': [100.0],
        },
        'semantic_path_latency_ms': [80.0, 100.0, 120.0],
        'drops': 0,
        'bounded_growth_pass': True,
    }
    resources = {
        'cpu_percent': [20.0, 30.0],
        'gpu_percent': [10.0, 20.0],
        'resident_memory_mb': [300.0, 320.0],
        'cuda_reserved_memory_mib': [800.0, 900.0],
    }
    return annotations, predictions, runtime, resources


def calibrated_threshold():
    samples = []
    for index in range(60):
        samples.append({
            'dataset_id': 'task-calibration-1',
            'split': 'calibration',
            'query_id': index + 1,
            'candidate_id': 'candidate-{}'.format(index),
            'task_relevant': index < 30,
            'relevance_score': 0.9 if index < 30 else 0.1,
        })
    return calibrate_task_threshold('task-calibration-1', samples)


def build_complete_report():
    annotations, predictions, runtime, resources = complete_inputs()
    return build_phase2_report(
        dataset_id='pilot-1',
        manifest_sha256='a' * 64,
        annotations=annotations,
        predictions=predictions,
        runtime=runtime,
        resources=resources,
        covered_scenarios=SCENARIOS,
        deterministic_replay_passed=True,
        human_tracking_regression_passed=True,
        software_revision='abc123',
        task_threshold_calibration=calibrated_threshold())


def test_complete_evidence_computes_identity_association_position_and_runtime():
    report = build_complete_report()

    assert report['metrics']['identity']['id_switches'] == 0
    assert report['metrics']['identity']['duplicate_objects'] == 0
    assert report['metrics']['identity']['incorrect_merges'] == 0
    assert report['metrics']['identity']['continuity_ratio'] == 1.0
    assert report['metrics']['association']['precision'] == 1.0
    assert report['metrics']['association']['recall'] == 1.0
    assert report['metrics']['position']['p95_error_m'] == pytest.approx(0.0)
    assert report['metrics']['reidentification']['rate'] == 1.0
    assert report['metrics']['stale_reactivation']['accuracy'] == 1.0
    assert report['metrics']['task_ranking']['top1_accuracy'] == 1.0
    assert report['metrics']['task_ranking']['candidate_recall'] == 1.0
    assert report['metrics']['task_ranking'][
        'hard_negative_false_confirmation_rate'] == 0.0
    assert report['metrics']['runtime']['update_rate_hz'] == pytest.approx(10.0)
    assert report['metrics']['runtime']['core_latency_p95_ms'] < 50.0
    assert report['metrics']['runtime']['semantic_path_latency_p95_ms'] < 150.0
    assert report['metrics']['runtime']['long_duration_pass'] is True
    assert report['metrics']['resources']['available'] is True
    assert report['metrics']['resources']['cuda_reserved_memory_p95_mib'] < 1536.0
    assert report['gates']['task_threshold_calibrated_and_frozen'] is True
    assert report['missing_scenarios'] == []
    assert report['passed'] is True
    assert report['status'] == 'passed'


def test_missing_annotations_is_unavailable_not_zero_accuracy():
    report = build_phase2_report(
        dataset_id='legacy',
        manifest_sha256='b' * 64,
        annotations=[],
        predictions=[],
        runtime=None,
        resources=None,
        covered_scenarios=[],
        deterministic_replay_passed=False,
        human_tracking_regression_passed=False,
        software_revision='abc123',
        task_threshold_calibration=None)

    assert report['metrics']['association']['available'] is False
    assert report['metrics']['association']['precision'] is None
    assert report['metrics']['association']['recall'] is None
    assert report['metrics']['identity']['id_switches'] is None
    assert report['passed'] is False
    assert report['status'] == 'unavailable'
    assert 'annotations_unavailable' in report['reasons']


def test_switch_duplicate_and_merge_are_counted_and_fail_pilot_identity_gate():
    annotations = [
        annotation(0, 'a', 0.0), annotation(1, 'a', 0.0),
        annotation(1, 'b', 0.0),
    ]
    predictions = [
        prediction(0, 'a', 1, 0.0),
        prediction(1, 'a', 2, 0.0),
        prediction(1, 'a', 3, 0.0),
        prediction(1, 'b', 2, 0.0),
    ]

    report = build_phase2_report(
        'pilot', 'c' * 64, annotations, predictions,
        runtime=None, resources=None, covered_scenarios=[],
        deterministic_replay_passed=False,
        human_tracking_regression_passed=False,
        software_revision='abc123', task_threshold_calibration=None)

    assert report['metrics']['identity']['id_switches'] >= 1
    assert report['metrics']['identity']['duplicate_objects'] == 1
    assert report['metrics']['identity']['incorrect_merges'] == 1
    assert report['gates']['zero_identity_corruption'] is False


def test_ignore_annotations_do_not_contribute_metrics():
    annotations = [annotation(0, 'a', 0.0, ignore=True)]
    predictions = [prediction(
        0, 'a', 1, 50.0,
        association_decision=True, association_correct=False)]

    report = build_phase2_report(
        'pilot', 'd' * 64, annotations, predictions,
        runtime=None, resources=None, covered_scenarios=[],
        deterministic_replay_passed=False,
        human_tracking_regression_passed=False,
        software_revision='abc123', task_threshold_calibration=None)

    assert report['evidence']['evaluated_annotation_count'] == 0
    assert report['metrics']['position']['available'] is False


def test_missing_positive_predictions_reduce_task_candidate_recall():
    annotations, predictions, runtime, resources = complete_inputs()
    predictions = [
        row for row in predictions
        if row['annotation_object_id'] not in {'object-0', 'object-1', 'object-2'}
    ]

    report = build_phase2_report(
        'pilot', 'e' * 64, annotations, predictions, runtime, resources,
        SCENARIOS, True, True, 'abc123', calibrated_threshold())

    assert report['metrics']['task_ranking']['false_negative'] == 3
    assert report['metrics']['task_ranking']['candidate_recall'] == pytest.approx(0.9)


def test_task_selection_must_match_frozen_threshold():
    annotations, predictions, runtime, resources = complete_inputs()
    predictions[0]['task_selected'] = False
    with pytest.raises(ValueError, match='selection'):
        build_phase2_report(
            'pilot', 'f' * 64, annotations, predictions, runtime, resources,
            SCENARIOS, True, True, 'abc123', calibrated_threshold())

    predictions[0]['task_selected'] = True
    predictions[0]['task_threshold'] = 0.8
    with pytest.raises(ValueError, match='frozen threshold'):
        build_phase2_report(
            'pilot', 'f' * 64, annotations, predictions, runtime, resources,
            SCENARIOS, True, True, 'abc123', calibrated_threshold())


def test_complete_but_below_runtime_and_resource_limits_is_failed():
    annotations, predictions, runtime, resources = complete_inputs()
    runtime['duration_sec'] = 1799.0
    runtime['semantic_path_latency_ms'] = [151.0]
    resources['cuda_reserved_memory_mib'] = [1537.0]
    report = build_phase2_report(
        'pilot', '1' * 64, annotations, predictions, runtime, resources,
        SCENARIOS, True, True, 'abc123', calibrated_threshold())

    assert report['status'] == 'failed'
    assert report['gates']['stable_for_at_least_30_minutes'] is False
    assert report['gates']['semantic_path_latency_p95_at_most_150_ms'] is False
    assert report['gates']['cuda_reserved_memory_p95_at_most_1536_mib'] is False


def test_missing_calibration_is_unavailable_even_with_other_complete_evidence():
    annotations, predictions, runtime, resources = complete_inputs()
    report = build_phase2_report(
        'pilot', '2' * 64, annotations, predictions, runtime, resources,
        SCENARIOS, True, True, 'abc123', None)

    assert report['status'] == 'unavailable'
    assert report['gates']['task_threshold_calibrated_and_frozen'] is False
    assert 'task_threshold_calibration_unavailable' in report['reasons']


def test_report_contains_every_top_level_field_required_by_checked_schema():
    root = Path(__file__).resolve().parents[1]
    schema = json.loads(
        (root / 'schemas' / 'phase2_evaluation_report.schema.json').read_text())
    report = build_complete_report()
    assert set(schema['required']) <= set(report)
    assert schema['properties']['schema_version']['const'] == (
        report['schema_version'])
    assert report['status'] in schema['properties']['status']['enum']


def test_cli_writes_deterministic_unavailable_report_and_returns_two(tmp_path):
    manifest = tmp_path / 'manifest.json'
    output = tmp_path / 'report.json'
    manifest.write_text(json.dumps({
        'dataset_id': 'legacy',
        'phase2': {'scenario_ids': []},
    }), encoding='utf-8')

    exit_code = run(
        manifest_path=manifest,
        annotations_path=None,
        predictions_path=None,
        runtime_path=None,
        resources_path=None,
        output_path=output,
        deterministic_replay_passed=False,
        human_tracking_regression_passed=False,
        software_revision='abc123')

    assert exit_code == 2
    first = output.read_bytes()
    assert json.loads(first)['status'] == 'unavailable'
    run(
        manifest, None, None, None, None, output,
        False, False, 'abc123', None)
    assert output.read_bytes() == first


def test_extended_annotation_and_manifest_schemas_declare_phase2_fields():
    root = Path(__file__).resolve().parents[1] / 'schemas'
    annotation_schema = json.loads(
        (root / 'annotation.schema.json').read_text())
    manifest_schema = json.loads(
        (root / 'dataset_manifest.schema.json').read_text())

    properties = annotation_schema['properties']
    assert 'public_object_key' in properties
    assert 'lidar_source_key' in properties
    assert 'support_state' in properties
    assert 'position_3d' in properties
    assert 'task_relevant' in properties
    assert 'ignore' in properties
    assert 'phase2' in manifest_schema['properties']


def test_report_schema_declares_every_exact_gate_and_nested_metric():
    root = Path(__file__).resolve().parents[1] / 'schemas'
    schema = json.loads(
        (root / 'phase2_evaluation_report.schema.json').read_text())
    report = build_complete_report()

    assert schema['properties']['gates']['additionalProperties'] is False
    assert set(schema['properties']['gates']['required']) == set(report['gates'])
    metrics = schema['properties']['metrics']['properties']
    for name, value in report['metrics'].items():
        metric_schema = metrics[name]
        if '$ref' in metric_schema:
            metric_schema = schema['definitions'][
                metric_schema['$ref'].rsplit('/', 1)[-1]]
        assert metric_schema['additionalProperties'] is False
        assert set(metric_schema['required']) == set(value)
