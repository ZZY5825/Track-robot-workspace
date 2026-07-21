import json
from pathlib import Path

import pytest

from track_robot_semantic_search.phase1_baselines import (
    build_baseline_report,
    validate_phase1_report,
)
from track_robot_semantic_search.phase1_baseline_cli import run


CAPABILITIES = {
    'camera': True,
    'lidar': True,
    'imu': False,
    'local_pose': False,
    'world_pose': False,
    'query_events': False,
    'annotations': False,
    'active_motion': False,
}


def record(
        kind, score=0.9, correct=None, latency_ms=20.0,
        output_rate_hz=10.0):
    value = {
        'kind': kind,
        'score': score,
        'latency_ms': latency_ms,
        'output_rate_hz': output_rate_hz,
    }
    if correct is not None:
        value['correct'] = correct
    return value


@pytest.mark.parametrize('baseline_id,kind', [
    ('baseline_1_fixed_detector', 'fixed_detector'),
    ('baseline_2_lidar_geometry', 'lidar_geometry'),
    ('baseline_3_language_camera', 'language_camera'),
])
def test_each_baseline_aggregates_matching_records(baseline_id, kind):
    report = build_baseline_report(
        baseline_id=baseline_id,
        dataset_id='legacy-bag',
        manifest_sha256='a' * 64,
        manifest_capabilities=dict(CAPABILITIES, annotations=True),
        records=[record(kind, correct=True), record(kind, correct=False)],
        software_revision='abc123',
        model_evidence={} if baseline_id == 'baseline_2_lidar_geometry' else {
            'available': True,
            'encoder_id': 'fixed:yolo' if baseline_id == (
                'baseline_1_fixed_detector') else 'openai_clip:ViT-B/32',
            'checkpoint_id': 'sha256:model',
            'licence': 'MIT/code; checkpoint reviewed',
        })

    validate_phase1_report(report)
    assert report['baseline_id'] == baseline_id
    assert report['metrics']['observation_count'] == 2
    assert report['metrics']['labelled_count'] == 2
    assert report['metrics']['precision'] == pytest.approx(0.5)
    assert report['metrics']['phrase_region_recall'] is None
    assert report['metrics']['latency_ms']['p95'] == pytest.approx(20.0)
    assert report['status'] in ('passed', 'failed')


def test_missing_annotations_is_not_evaluated_not_zero_accuracy():
    report = build_baseline_report(
        baseline_id='baseline_1_fixed_detector',
        dataset_id='legacy-bag',
        manifest_sha256='b' * 64,
        manifest_capabilities=CAPABILITIES,
        records=[record('fixed_detector')],
        software_revision='abc123',
        model_evidence={
            'available': True,
            'encoder_id': 'fixed:yolo',
            'checkpoint_id': 'sha256:model',
            'licence': 'AGPL/runtime reviewed',
        })

    assert report['status'] == 'not_evaluated'
    assert report['metrics']['labelled_count'] == 0
    assert report['metrics']['precision'] is None
    assert 'annotations_unavailable' in report['reasons']


def test_low_output_rate_cannot_pass_an_evaluated_baseline():
    report = build_baseline_report(
        baseline_id='baseline_3_language_camera',
        dataset_id='legacy-bag',
        manifest_sha256='f' * 64,
        manifest_capabilities=dict(CAPABILITIES, annotations=True),
        records=[record(
            'language_camera', correct=True, output_rate_hz=0.1)],
        software_revision='abc123',
        model_evidence={
            'available': True,
            'encoder_id': 'openai_clip:ViT-B/32',
            'checkpoint_id': 'sha256:model',
            'licence': 'MIT/code and weights reviewed',
        })

    assert report['gates']['semantic_output_at_least_5_hz'] is False
    assert report['passed'] is False
    assert report['status'] == 'failed'


def test_fixed_detector_requires_model_evidence():
    report = build_baseline_report(
        baseline_id='baseline_1_fixed_detector',
        dataset_id='legacy-bag',
        manifest_sha256='1' * 64,
        manifest_capabilities=CAPABILITIES,
        records=[record('fixed_detector')],
        software_revision='abc123')

    assert report['status'] == 'unavailable'
    assert report['artifacts']['model']['required'] is True
    assert 'model_unavailable' in report['reasons']


def test_missing_language_model_is_unavailable_even_with_records():
    report = build_baseline_report(
        baseline_id='baseline_3_language_camera',
        dataset_id='legacy-bag',
        manifest_sha256='c' * 64,
        manifest_capabilities=dict(CAPABILITIES, annotations=True),
        records=[record('language_camera', correct=True)],
        software_revision='abc123',
        model_evidence={
            'available': False,
            'encoder_id': '',
            'checkpoint_id': '',
            'licence': 'not reviewed',
        })

    assert report['status'] == 'unavailable'
    assert 'model_unavailable' in report['reasons']
    assert report['passed'] is False


def test_empty_records_are_unavailable():
    report = build_baseline_report(
        baseline_id='baseline_2_lidar_geometry',
        dataset_id='legacy-bag',
        manifest_sha256='d' * 64,
        manifest_capabilities=CAPABILITIES,
        records=[],
        software_revision='abc123')

    assert report['status'] == 'unavailable'
    assert report['metrics']['observation_count'] == 0
    assert 'observations_unavailable' in report['reasons']


def test_nonfinite_record_is_rejected():
    with pytest.raises(ValueError, match='finite'):
        build_baseline_report(
            baseline_id='baseline_1_fixed_detector',
            dataset_id='legacy-bag',
            manifest_sha256='e' * 64,
            manifest_capabilities=CAPABILITIES,
            records=[record('fixed_detector', latency_ms=float('nan'))],
            software_revision='abc123')


def test_schema_declares_all_four_statuses():
    schema_path = (
        Path(__file__).resolve().parents[1] /
        'schemas' /
        'phase1_baseline_report.schema.json')
    schema = json.loads(schema_path.read_text(encoding='utf-8'))

    assert set(schema['properties']['status']['enum']) == {
        'passed', 'failed', 'unavailable', 'not_evaluated'}


def test_cli_writes_three_reports_in_stable_order(tmp_path):
    manifest_path = tmp_path / 'manifest.json'
    observations_path = tmp_path / 'observations.jsonl'
    output_dir = tmp_path / 'reports'
    manifest_path.write_text(json.dumps({
        'dataset_id': 'legacy-bag',
        'capabilities': CAPABILITIES,
    }), encoding='utf-8')
    observations_path.write_text(
        json.dumps(record('fixed_detector')) + '\n' +
        json.dumps(record('lidar_geometry')) + '\n',
        encoding='utf-8')

    exit_code = run(
        manifest_path=manifest_path,
        observations_path=observations_path,
        output_dir=output_dir,
        software_revision='abc123',
        model_evidence_path=None)

    assert exit_code == 2
    paths = sorted(output_dir.glob('*.json'))
    assert [path.name for path in paths] == [
        'baseline_1_fixed_detector.json',
        'baseline_2_lidar_geometry.json',
        'baseline_3_language_camera.json',
    ]
    reports = [json.loads(path.read_text(encoding='utf-8')) for path in paths]
    assert [item['baseline_id'] for item in reports] == [
        'baseline_1_fixed_detector',
        'baseline_2_lidar_geometry',
        'baseline_3_language_camera',
    ]
    assert not list(output_dir.glob('*.tmp'))


def test_cli_routes_per_baseline_model_evidence(tmp_path):
    manifest_path = tmp_path / 'manifest.json'
    observations_path = tmp_path / 'observations.jsonl'
    evidence_path = tmp_path / 'models.json'
    output_dir = tmp_path / 'reports'
    manifest_path.write_text(json.dumps({
        'dataset_id': 'legacy-bag',
        'capabilities': CAPABILITIES,
    }), encoding='utf-8')
    observations_path.write_text(
        json.dumps(record('fixed_detector')) + '\n' +
        json.dumps(record('language_camera')) + '\n',
        encoding='utf-8')
    evidence_path.write_text(json.dumps({
        'baseline_1_fixed_detector': {
            'available': True,
            'encoder_id': 'fixed:yolo',
            'checkpoint_id': 'sha256:yolo',
            'licence': 'reviewed',
        },
        'baseline_3_language_camera': {
            'available': True,
            'encoder_id': 'openai_clip:ViT-B/32',
            'checkpoint_id': 'sha256:clip',
            'licence': 'reviewed',
        },
    }), encoding='utf-8')

    run(
        manifest_path=manifest_path,
        observations_path=observations_path,
        output_dir=output_dir,
        software_revision='abc123',
        model_evidence_path=evidence_path)

    baseline_1 = json.loads((
        output_dir / 'baseline_1_fixed_detector.json').read_text())
    baseline_3 = json.loads((
        output_dir / 'baseline_3_language_camera.json').read_text())
    assert baseline_1['artifacts']['model']['encoder_id'] == 'fixed:yolo'
    assert baseline_3['artifacts']['model']['encoder_id'] == (
        'openai_clip:ViT-B/32')


def test_baseline_cli_is_packaged():
    setup_source = (
        Path(__file__).resolve().parents[1] / 'setup.py'
    ).read_text(encoding='utf-8')

    assert 'semantic_search_phase1_baselines' in setup_source
    assert 'track_robot_semantic_search.phase1_baseline_cli:main' in setup_source


def test_replay_probe_exposes_and_records_multiscale_controls():
    package_root = Path(__file__).resolve().parents[1]
    source = (
        package_root / 'scripts' / 'run_phase1_replay_probe.py'
    ).read_text(encoding='utf-8')

    assert "'--window-strategy'" in source
    assert "choices=('grid_only', 'multiscale_v1')" in source
    assert "'--center-window-scale'" in source
    assert 'window_strategy=arguments.window_strategy' in source
    assert 'center_window_scale=arguments.center_window_scale' in source
    assert "'window_strategy': arguments.window_strategy" in source
    assert "'center_window_scale': arguments.center_window_scale" in source
    assert 'score_multiscale_regions(' in source


def test_readme_documents_multiscale_boundary_behavior_and_global_fallback():
    readme = (
        Path(__file__).resolve().parents[1] / 'README.md'
    ).read_text(encoding='utf-8')

    assert 'multiscale_v1' in readme
    assert 'six-window' in readme
    assert 'grid boundary' in readme
    assert 'whole-frame fallback' in readme
