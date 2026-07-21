import hashlib
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from track_robot_semantic_search import (
    compare_reports,
    evaluation as evaluation_module,
    evaluator_node,
)
from track_robot_semantic_search.compare_reports import compare
from track_robot_semantic_search.evaluation import (
    EvaluationAccumulator,
    TopicSeries,
    parse_tegrastats_line,
    percentile,
    summarize_tegrastats,
)
from track_robot_semantic_search.evaluator_node import (
    SemanticSearchEvaluatorNode,
    stamp_ns,
)


def manifest(local_pose=False, world_pose=False):
    return {
        'schema_version': '1.0.0',
        'dataset_id': 'evaluation_test',
        'capabilities': {
            'camera': True,
            'lidar': True,
            'imu': local_pose,
            'local_pose': local_pose,
            'world_pose': world_pose,
            'query_events': False,
            'annotations': False,
            'active_motion': False,
        },
    }


def make_metrics(
        manifest_sha256='a' * 64, run_id='rate-1.0', replay_rate=1.0,
        wall_duration_sec=45.0):
    return EvaluationAccumulator(
        manifest=manifest(),
        manifest_sha256=manifest_sha256,
        run_id=run_id,
        software_revision='unit-test-revision',
        config_sha256='c' * 64,
        replay_rate=replay_rate,
        wall_duration_sec=wall_duration_sec,
        timing_policy='foxy_wall_time_scaled',
        freshness_time_base='arrival_monotonic',
    )


def observe_complete_healthy_replay(metrics):
    receive_step = int(1000000000 / metrics.replay_rate)
    for index in range(42):
        source = index * 1000000000
        receive = index * receive_step
        metrics.observe_topic('image', source, receive)
        metrics.observe_topic('lidar', source + 20000000, receive)
        metrics.observe_pair_offset(20000000)
        metrics.observe_localization('OBSERVATION_ONLY', 1)


def observe_minimum_healthy_replay(metrics):
    observe_complete_healthy_replay(metrics)


def test_percentile_is_deterministic_and_handles_boundaries():
    assert percentile([], 0.5) is None
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.5) == 2.5
    assert percentile([3.0, 1.0, 2.0], -1.0) == 1.0
    assert percentile([3.0, 1.0, 2.0], 2.0) == 3.0


def test_topic_series_handles_empty_single_out_of_order_and_duplicates():
    assert TopicSeries().report()['source_rate_hz'] == 0.0

    series = TopicSeries()
    series.observe(300000000, 600000000)
    assert series.report()['receive_rate_hz'] == 0.0
    series.observe(100000000, 200000000)
    series.observe(100000000, 200000000)
    series.observe(200000000, 400000000)

    report = series.report()
    serialized = json.dumps(
        [300000000, 100000000, 100000000, 200000000],
        separators=(',', ':'),
    ).encode('utf-8')
    assert report == {
        'count': 4,
        'source_start_ns': 100000000,
        'source_end_ns': 300000000,
        'source_span_sec': 0.2,
        'receive_span_sec': 0.4,
        'source_rate_hz': 15.0,
        'receive_rate_hz': 7.5,
        'source_sequence_sha256': hashlib.sha256(serialized).hexdigest(),
    }


def test_capability_aware_legacy_replay_passes_observation_only():
    metrics = make_metrics()
    observe_complete_healthy_replay(metrics)
    for index in range(10):
        metrics.observe_latency('image_callback', 0.001 + index * 0.0001)

    report = metrics.finalize()

    assert report['gates']['required_topic_window_complete'] is True
    assert report['gates']['sync_p95_at_most_80_ms'] is True
    assert report['gates']['manifest_localization_mode_respected'] is True
    assert report['latency_metrics']['image_callback']['count'] == 10
    assert report['latency_metrics']['image_callback']['p95_sec'] < 0.002
    assert report['run']['replay_rate'] == 1.0
    assert report['artifacts']['model_exports'] == []
    assert report['passed'] is True


def test_missing_topics_and_missing_pairs_fail_capability_gates():
    metrics = make_metrics()
    metrics.observe_topic('image', 1, 1)
    metrics.observe_localization('OBSERVATION_ONLY', 1)

    report = metrics.finalize()

    assert report['gates']['required_topic_window_complete'] is False
    assert report['gates']['sync_p95_at_most_80_ms'] is False
    assert report['passed'] is False


def test_one_sample_per_required_topic_cannot_pass():
    metrics = make_metrics()
    metrics.observe_topic('image', 1, 1)
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_pair_offset(0)
    metrics.observe_localization('OBSERVATION_ONLY', 1)

    report = metrics.finalize()

    assert report['gates']['required_topic_window_complete'] is False
    assert report['passed'] is False


def test_required_topic_window_needs_ninety_percent_source_coverage():
    metrics = make_metrics()
    for index in range(41):
        stamp = index * 1000000000
        metrics.observe_topic('image', stamp, stamp)
        metrics.observe_topic('lidar', stamp, stamp)
        metrics.observe_pair_offset(0)
        metrics.observe_localization('OBSERVATION_ONLY', 1)

    report = metrics.finalize()

    assert report['topic_metrics']['image']['source_span_sec'] == 40.0
    assert report['run']['target_source_duration_sec'] == 45.0
    assert report['gates']['required_topic_window_complete'] is False


def test_receive_source_ratio_must_match_replay_rate():
    metrics = make_metrics(replay_rate=2.0, wall_duration_sec=22.5)
    for index in range(42):
        stamp = index * 1000000000
        metrics.observe_topic('image', stamp, stamp)
        metrics.observe_topic('lidar', stamp, stamp)
        metrics.observe_pair_offset(0)
        metrics.observe_localization('OBSERVATION_ONLY', 1)

    assert metrics.finalize()['gates']['replay_rate_consistent'] is False


@pytest.mark.parametrize(
    'observed_rate, expected',
    [(1.71, True), (1.69, False)],
)
def test_replay_rate_uses_fifteen_percent_relative_tolerance(
        observed_rate, expected):
    metrics = make_metrics(replay_rate=2.0, wall_duration_sec=22.5)
    receive_step = int(1000000000 / observed_rate)
    for index in range(42):
        source = index * 1000000000
        receive = index * receive_step
        metrics.observe_topic('image', source, receive)
        metrics.observe_topic('lidar', source, receive)
        metrics.observe_pair_offset(0)
        metrics.observe_localization('OBSERVATION_ONLY', 1)

    assert metrics.finalize()['gates']['replay_rate_consistent'] is expected


def test_forward_intent_is_a_hard_failure():
    metrics = make_metrics(run_id='unsafe')
    observe_minimum_healthy_replay(metrics)
    metrics.observe_motion_intent(forward_permitted=True)

    report = metrics.finalize()

    assert report['safety']['forward_permission_violations'] == 1
    assert report['gates']['no_forward_permission'] is False
    assert report['passed'] is False


@pytest.mark.parametrize(
    ('local_pose', 'world_pose', 'mode', 'expected'),
    [
        (True, False, 'LOCAL_SESSION', True),
        (True, False, 'OBSERVATION_ONLY', False),
        (True, False, 'WORLD', False),
        (True, True, 'WORLD', True),
        (True, True, 'LOCAL_SESSION', False),
        (True, True, 'OBSERVATION_ONLY', False),
    ],
)
def test_pose_capabilities_require_the_corresponding_healthy_mode(
        local_pose, world_pose, mode, expected):
    payload = manifest(local_pose=local_pose, world_pose=world_pose)
    metrics = EvaluationAccumulator(
        payload, 'd' * 64, mode.lower(), 'unit-test-revision',
        'c' * 64, 1.0, 45.0,
        'foxy_wall_time_scaled', 'arrival_monotonic',
    )
    names = ['image', 'lidar', 'imu', 'local_pose']
    if world_pose:
        names.append('world_pose')
    for index in range(42):
        stamp = index * 1000000000
        for name in names:
            metrics.observe_topic(name, stamp, stamp)
    metrics.observe_pair_offset(0)
    metrics.observe_localization(mode, 1)

    assert metrics.finalize()['passed'] is expected


def pose_metrics(world_pose=False):
    payload = manifest(local_pose=True, world_pose=world_pose)
    metrics = EvaluationAccumulator(
        payload, 'd' * 64, 'pose', 'unit-test-revision', 'c' * 64,
        1.0, 45.0, 'foxy_wall_time_scaled', 'arrival_monotonic')
    names = ['image', 'lidar', 'imu', 'local_pose']
    if world_pose:
        names.append('world_pose')
    for index in range(42):
        stamp = index * 1000000000
        for name in names:
            metrics.observe_topic(name, stamp, stamp)
    metrics.observe_pair_offset(0)
    return metrics


@pytest.mark.parametrize(
    ('world_pose', 'modes'),
    [
        (False, ('LOCAL_SESSION', 'OBSERVATION_ONLY')),
        (False, ('LOCAL_SESSION', 'WORLD')),
        (True, ('LOCAL_SESSION', 'WORLD', 'LOCAL_SESSION')),
        (True, ('WORLD', 'UNKNOWN')),
    ],
)
def test_localization_rejects_mixed_or_regressing_mode_sequences(
        world_pose, modes):
    metrics = pose_metrics(world_pose=world_pose)
    for mode in modes:
        metrics.observe_localization(mode, 1)

    assert metrics.finalize()['gates'][
        'manifest_localization_mode_respected'] is False


def test_world_localization_allows_startup_transition_to_world():
    metrics = pose_metrics(world_pose=True)
    metrics.observe_localization('OBSERVATION_ONLY', 1)
    metrics.observe_localization('LOCAL_SESSION', 1)
    metrics.observe_localization('WORLD', 2)
    metrics.observe_localization('WORLD', 2)

    assert metrics.finalize()['gates'][
        'manifest_localization_mode_respected'] is True


def test_local_startup_transition_must_reach_local_session():
    metrics = pose_metrics(world_pose=False)
    metrics.observe_localization('OBSERVATION_ONLY', 1)
    metrics.observe_localization('LOCAL_SESSION', 1)

    assert metrics.finalize()['gates'][
        'manifest_localization_mode_respected'] is True


@pytest.mark.parametrize(
    'world_pose, modes',
    [
        (False, ('OBSERVATION_ONLY',)),
        (True, ('OBSERVATION_ONLY', 'LOCAL_SESSION')),
    ],
)
def test_startup_localization_must_reach_declared_final_mode(
        world_pose, modes):
    metrics = pose_metrics(world_pose=world_pose)
    for mode in modes:
        metrics.observe_localization(mode, 1)

    assert metrics.finalize()['gates'][
        'manifest_localization_mode_respected'] is False


@pytest.mark.parametrize('epoch_id', [0, -1, True, 1.0, 'bad', None])
def test_localization_rejects_non_positive_or_non_integer_epochs(epoch_id):
    metrics = make_metrics()
    metrics.observe_topic('image', 1, 1)
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_pair_offset(0)

    metrics.observe_localization('OBSERVATION_ONLY', epoch_id)

    report = metrics.finalize()
    assert report['gates']['manifest_localization_mode_respected'] is False
    assert 0 not in report['localization']['epoch_ids']


def test_localization_rejects_decreasing_epoch_sequence():
    metrics = make_metrics()
    metrics.observe_topic('image', 1, 1)
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_pair_offset(0)
    metrics.observe_localization('OBSERVATION_ONLY', 2)
    metrics.observe_localization('OBSERVATION_ONLY', 1)

    assert metrics.finalize()['gates'][
        'manifest_localization_mode_respected'] is False


def test_invalid_latency_and_resource_samples_are_ignored():
    metrics = make_metrics()
    for value in (-1.0, math.nan, math.inf, -math.inf):
        metrics.observe_latency('callback', value)
        metrics.observe_resource(value, value, value, value)
    metrics.observe_latency('callback', 0.25)
    metrics.observe_resource(10.0, 20.0, 30.0, 40.0)

    report = metrics.finalize()

    assert report['latency_metrics']['callback'] == {
        'count': 1,
        'mean_sec': 0.25,
        'p95_sec': 0.25,
        'maximum_sec': 0.25,
    }
    assert report['resources']['evaluator_cpu_percent']['mean'] == 10.0
    assert report['resources']['evaluator_rss_mb']['mean'] == 20.0
    assert report['resources']['system_cpu_percent']['mean'] == 30.0
    assert report['resources']['system_ram_used_mb']['mean'] == 40.0


@pytest.mark.parametrize(
    ('keyword', 'value'),
    [
        ('manifest_sha256', 'bad'),
        ('config_sha256', 'BAD'),
        ('run_id', ''),
        ('software_revision', ''),
        ('replay_rate', 0.0),
        ('replay_rate', math.nan),
        ('replay_rate', math.inf),
        ('wall_duration_sec', 0.0),
        ('wall_duration_sec', math.nan),
        ('wall_duration_sec', math.inf),
        ('timing_policy', 'other'),
        ('freshness_time_base', 'bag_clock'),
    ],
)
def test_accumulator_rejects_values_that_cannot_match_report_schema(
        keyword, value):
    arguments = {
        'manifest': manifest(),
        'manifest_sha256': 'a' * 64,
        'run_id': 'unit',
        'software_revision': 'unit-test-revision',
        'config_sha256': 'c' * 64,
        'replay_rate': 1.0,
        'wall_duration_sec': 45.0,
        'timing_policy': 'foxy_wall_time_scaled',
        'freshness_time_base': 'arrival_monotonic',
    }
    arguments[keyword] = value

    with pytest.raises(ValueError):
        EvaluationAccumulator(**arguments)


def test_report_has_evaluation_schema_shape_and_json_safe_values():
    metrics = make_metrics()
    observe_minimum_healthy_replay(metrics)

    report = metrics.finalize()

    assert set(report) == {
        'schema_version', 'dataset_id', 'manifest_sha256', 'run',
        'manifest_capabilities',
        'artifacts', 'coverage', 'topic_metrics', 'synchronization',
        'latency_metrics', 'localization', 'semantic_counts', 'resources',
        'safety', 'gates', 'passed',
    }
    assert report['schema_version'] == '1.1.0'
    assert report['manifest_capabilities'] == manifest()['capabilities']
    assert set(report['run']) == {
        'run_id', 'phase', 'replay_rate', 'timing_policy',
        'wall_duration_sec', 'target_source_duration_sec',
        'minimum_source_coverage_ratio', 'freshness_time_base',
    }
    assert set(report['artifacts']) == {
        'software_revision', 'config_sha256', 'model_exports'}
    assert all(isinstance(value, bool) for value in report['gates'].values())
    assert set(report['gates']) == {
        'required_topic_window_complete',
        'sync_p95_at_most_80_ms',
        'manifest_localization_mode_respected',
        'replay_rate_consistent',
        'no_forward_permission',
    }

    def assert_finite_numbers(value):
        if type(value) is float:
            assert math.isfinite(value)
        elif isinstance(value, dict):
            for child in value.values():
                assert_finite_numbers(child)
        elif isinstance(value, list):
            for child in value:
                assert_finite_numbers(child)

    assert_finite_numbers(report)
    json.dumps(report, allow_nan=False)


def test_evaluation_report_schema_is_strict_for_updated_objects():
    schema_path = (
        Path(__file__).parents[1] / 'schemas' /
        'evaluation_report.schema.json')
    schema = json.loads(schema_path.read_text(encoding='utf-8'))
    capability_keys = set(manifest()['capabilities'])
    gate_keys = {
        'required_topic_window_complete',
        'sync_p95_at_most_80_ms',
        'manifest_localization_mode_respected',
        'replay_rate_consistent',
        'no_forward_permission',
    }
    run_keys = {
        'run_id', 'phase', 'replay_rate', 'timing_policy',
        'wall_duration_sec', 'target_source_duration_sec',
        'minimum_source_coverage_ratio', 'freshness_time_base',
    }

    assert schema['properties']['schema_version'] == {'const': '1.1.0'}
    assert schema['additionalProperties'] is False
    assert set(schema['required']) == set(schema['properties'])
    assert set(schema['properties']['run']['required']) == run_keys
    assert schema['properties']['run']['additionalProperties'] is False
    capabilities = schema['properties']['manifest_capabilities']
    assert set(capabilities['required']) == capability_keys
    assert set(capabilities['properties']) == capability_keys
    assert capabilities['additionalProperties'] is False
    gates = schema['properties']['gates']
    assert set(gates['required']) == gate_keys
    assert set(gates['properties']) == gate_keys
    assert gates['additionalProperties'] is False
    topic_value = schema['properties']['topic_metrics'][
        'patternProperties']['^.+$']
    assert topic_value['additionalProperties'] is False
    assert set(topic_value['required']) == set(topic_value['properties'])


def test_finalize_uses_shared_hard_gates_and_recomputes_passed(monkeypatch):
    metrics = make_metrics()
    observe_complete_healthy_replay(metrics)
    expected = {
        'required_topic_window_complete': True,
        'sync_p95_at_most_80_ms': True,
        'manifest_localization_mode_respected': True,
        'replay_rate_consistent': False,
        'no_forward_permission': True,
    }
    calls = []

    def fake_compute(report, capabilities):
        calls.append((report, capabilities))
        return dict(expected)

    monkeypatch.setattr(
        evaluation_module, 'compute_hard_gates', fake_compute,
        raising=False)

    report = metrics.finalize()

    assert calls[0][1] == manifest()['capabilities']
    assert report['gates'] == expected
    assert report['passed'] is False
    assert report['passed'] == all(report['gates'].values())


def test_shared_hard_gate_recomputation_matches_report():
    helper = getattr(evaluation_module, 'compute_hard_gates', None)
    assert callable(helper)
    metrics = make_metrics()
    observe_complete_healthy_replay(metrics)

    report = metrics.finalize()

    assert helper(report, report['manifest_capabilities']) == report['gates']
    assert report['passed'] == all(report['gates'].values())


def test_hard_gates_recompute_target_and_fixed_coverage_policy():
    truncated = make_metrics()
    for stamp in (0, 900000000):
        truncated.observe_topic('image', stamp, stamp)
        truncated.observe_topic('lidar', stamp, stamp)
        truncated.observe_pair_offset(0)
        truncated.observe_localization('OBSERVATION_ONLY', 1)
    forged_target = truncated.finalize()
    forged_target['run']['target_source_duration_sec'] = 1.0

    target_gates = evaluation_module.compute_hard_gates(
        forged_target, forged_target['manifest_capabilities'])

    assert target_gates['required_topic_window_complete'] is False

    complete = make_metrics()
    observe_complete_healthy_replay(complete)
    forged_coverage = complete.finalize()
    forged_coverage['run']['minimum_source_coverage_ratio'] = 0.5

    coverage_gates = evaluation_module.compute_hard_gates(
        forged_coverage, forged_coverage['manifest_capabilities'])

    assert coverage_gates['required_topic_window_complete'] is False


def test_no_required_sensor_topics_never_becomes_ready_or_complete():
    payload = manifest()
    for capability in ('camera', 'lidar', 'imu', 'local_pose', 'world_pose'):
        payload['capabilities'][capability] = False
    metrics = EvaluationAccumulator(
        payload, 'a' * 64, 'empty', 'unit-test-revision', 'c' * 64,
        1.0, 45.0, 'foxy_wall_time_scaled', 'arrival_monotonic')

    assert metrics.required_topics_ready() is False
    gates = metrics.finalize()['gates']
    assert gates['required_topic_window_complete'] is False
    assert gates['replay_rate_consistent'] is False
    assert gates['sync_p95_at_most_80_ms'] is False


def test_hard_gates_fail_closed_without_overflow_for_huge_json_integers():
    metrics = make_metrics()
    observe_complete_healthy_replay(metrics)
    report = metrics.finalize()
    report['run']['wall_duration_sec'] = 10 ** 400
    report['run']['target_source_duration_sec'] = 10 ** 400
    report['topic_metrics']['image']['source_span_sec'] = 10 ** 400

    gates = evaluation_module.compute_hard_gates(
        report, report['manifest_capabilities'])

    assert all(value is False for value in gates.values())


def test_tegrastats_parser_extracts_report_only_resources():
    values = parse_tegrastats_line(
        'RAM 4000/31919MB CPU [10%@1200,off] GR3D_FREQ 21% '
        'CPU@48.5C GPU@46.0C VDD_IN 12200mW/11800mW')
    assert values == {
        'ram_used_mb': 4000.0,
        'gpu_utilization_percent': 21.0,
        'cpu_temperature_c': 48.5,
        'gpu_temperature_c': 46.0,
        'input_power_mw': 12200.0,
    }
    assert parse_tegrastats_line('malformed and partial RAM ???') == {}


def test_tegrastats_summary_tolerates_missing_and_malformed_input(tmp_path):
    assert summarize_tegrastats(tmp_path / 'missing.log') == {}
    path = tmp_path / 'tegrastats.log'
    path.write_text(
        'unrelated\nRAM 100/1000MB GR3D_FREQ 10%\n'
        'RAM 300/1000MB GR3D_FREQ 30%\n',
        encoding='utf-8',
    )

    summary = summarize_tegrastats(path)

    assert summary['ram_used_mb'] == {
        'mean': 200.0, 'p95': 290.0, 'maximum': 300.0}
    assert summary['gpu_utilization_percent'] == {
        'mean': 20.0, 'p95': 29.0, 'maximum': 30.0}


def test_percentile_and_tegrastats_reject_nonfinite_values():
    with pytest.raises(ValueError):
        percentile([], math.nan)
    for value in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            percentile([1.0, value], 0.5)
    with pytest.raises(ValueError):
        percentile([1.0], math.nan)

    assert parse_tegrastats_line(
        'RAM nan/1000MB GR3D_FREQ inf% CPU@1e309C GPU@-infC') == {}


def test_tegrastats_summary_only_reads_samples_after_first_sensor(tmp_path):
    path = tmp_path / 'tegrastats.log'
    path.write_text('RAM 900/1000MB GR3D_FREQ 90%\n', encoding='utf-8')
    metrics = EvaluationAccumulator(
        manifest(), 'a' * 64, 'window', 'unit-test-revision',
        'c' * 64, 1.0, 45.0,
        'foxy_wall_time_scaled', 'arrival_monotonic',
        tegrastats_path=path)

    metrics.observe_topic('image', 1, 1)
    with path.open('a', encoding='utf-8') as stream:
        stream.write('RAM 100/1000MB GR3D_FREQ 10%\n')
        stream.write('RAM 300/1000MB GR3D_FREQ 30%\n')
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_pair_offset(0)
    metrics.observe_localization('OBSERVATION_ONLY', 1)

    summary = metrics.finalize()['resources']['tegrastats']
    assert summary['ram_used_mb']['mean'] == 200.0
    assert summary['gpu_utilization_percent']['mean'] == 20.0


@pytest.mark.parametrize('replace_kind', ['truncate', 'rotate'])
def test_tegrastats_window_fails_safe_on_truncation_or_rotation(
        tmp_path, replace_kind):
    path = tmp_path / 'tegrastats.log'
    path.write_text(
        ('RAM 900/1000MB GR3D_FREQ 90%\n' * 8), encoding='utf-8')
    metrics = EvaluationAccumulator(
        manifest(), 'a' * 64, 'window', 'unit-test-revision',
        'c' * 64, 1.0, 45.0,
        'foxy_wall_time_scaled', 'arrival_monotonic',
        tegrastats_path=path)
    metrics.observe_topic('image', 1, 1)
    if replace_kind == 'rotate':
        path.rename(tmp_path / 'tegrastats.old')
    path.write_text('RAM 100/1000MB GR3D_FREQ 10%\n', encoding='utf-8')
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_pair_offset(0)
    metrics.observe_localization('OBSERVATION_ONLY', 1)

    assert metrics.finalize()['resources']['tegrastats'] == {}


def formal_manifest():
    return {
        'schema_version': '1.0.0',
        'dataset_id': 'evaluation_test',
        'split': 'legacy_replay_only',
        'bag': {
            'relative_path': 'bags/evaluation_test',
            'sha256': '0' * 64,
            'storage_id': 'sqlite3',
            'start_time_ns': 1,
            'duration_ns': 45000000000,
            'topics': [
                {
                    'name': '/camera',
                    'type': 'sensor_msgs/msg/Image',
                    'count': 42,
                },
                {
                    'name': '/lidar',
                    'type': 'sensor_msgs/msg/PointCloud2',
                    'count': 42,
                },
            ],
        },
        'capabilities': dict(manifest()['capabilities']),
        'calibration': {
            'camera_intrinsics_id': 'unknown',
            'camera_lidar_extrinsics_id': 'unknown',
            'lidar_imu_extrinsics_id': 'unknown',
            'localization_config_id': 'none',
        },
        'environment': {
            'site_id': 'legacy_unknown',
            'session_id': 'unit',
            'lighting': 'unknown',
            'surface': 'unknown',
            'weather': 'unknown',
        },
        'queries': [],
        'annotation_files': [],
        'objects': [],
        'trials': [],
        'provenance': {
            'created_at': '2026-07-13T00:00:00Z',
            'created_by': 'unit_test',
            'notes': 'legacy replay only',
        },
    }


def write_formal_reports(tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    manifest_path = tmp_path / 'manifest.json'
    manifest_path.write_text(
        json.dumps(formal_manifest(), sort_keys=True), encoding='utf-8')
    manifest_sha256 = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    reports = []
    for replay_rate, wall_duration in (
            (0.5, 90.0), (1.0, 45.0), (2.0, 22.5)):
        metrics = make_metrics(
            manifest_sha256=manifest_sha256,
            run_id='rate-{}'.format(replay_rate),
            replay_rate=replay_rate,
            wall_duration_sec=wall_duration,
        )
        observe_complete_healthy_replay(metrics)
        path = tmp_path / 'rate-{}.json'.format(replay_rate)
        path.write_text(
            json.dumps(metrics.finalize(), allow_nan=False),
            encoding='utf-8')
        reports.append(path)
    return manifest_path, reports


def mutate_report(path, mutation):
    report = json.loads(path.read_text(encoding='utf-8'))
    mutation(report)
    path.write_text(json.dumps(report), encoding='utf-8')


def test_compare_accepts_exact_formal_report_set(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)

    result = compare(manifest_path, list(reversed(reports)))

    assert result['passed'] is True
    assert result['reports'] == [str(path) for path in reports]
    assert list(result['replay_rates'].values()) == [0.5, 1.0, 2.0]
    assert set(result['recomputed_gates']) == {
        str(path) for path in reports}
    assert all(all(gates.values())
               for gates in result['recomputed_gates'].values())


def test_compare_requires_list_and_exact_three_unique_paths_and_rates(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)

    assert compare(manifest_path, tuple(reports))['passed'] is False
    assert compare(manifest_path, reports[:1])['passed'] is False
    duplicate = compare(
        manifest_path, [reports[0], reports[0], reports[2]])
    assert duplicate['passed'] is False
    assert any('duplicate report path' in item
               for item in duplicate['failures'])
    mutate_report(
        reports[2], lambda report: report['run'].update(replay_rate=1.0))
    repeated_rate = compare(manifest_path, reports)
    assert repeated_rate['passed'] is False
    assert any('rate set' in item for item in repeated_rate['failures'])


@pytest.mark.parametrize(
    'field, value, failure',
    [
        ('timing_policy', 'other', 'timing policy'),
        ('wall_duration_sec', 44.0, 'wall duration'),
        ('target_source_duration_sec', 44.0, 'target source duration'),
        ('minimum_source_coverage_ratio', 0.5, 'coverage ratio'),
        ('freshness_time_base', 'source_clock', 'freshness time base'),
    ],
)
def test_compare_rejects_wrong_formal_policy(
        tmp_path, field, value, failure):
    manifest_path, reports = write_formal_reports(tmp_path)
    mutate_report(
        reports[1], lambda report: report['run'].update({field: value}))

    result = compare(manifest_path, reports)

    assert result['passed'] is False
    assert any(failure in item for item in result['failures'])


def test_compare_rejects_manifest_checksum_and_capability_mismatch(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)
    mutate_report(
        reports[0], lambda report: report.update(manifest_sha256='f' * 64))
    checksum = compare(manifest_path, reports)
    assert checksum['passed'] is False
    assert any('manifest checksum' in item for item in checksum['failures'])

    manifest_path, reports = write_formal_reports(tmp_path / 'capabilities')
    mutate_report(
        reports[0],
        lambda report: report['manifest_capabilities'].update(imu=True))
    capabilities = compare(manifest_path, reports)
    assert capabilities['passed'] is False
    assert any('manifest capabilities' in item
               for item in capabilities['failures'])


def test_compare_recomputes_and_rejects_forged_gates_and_passed(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)

    def forge(report):
        report['topic_metrics']['image']['source_span_sec'] = 1.0
        report['gates'] = {name: True for name in report['gates']}
        report['passed'] = True

    mutate_report(reports[0], forge)
    result = compare(manifest_path, reports)

    assert result['passed'] is False
    assert any('stored gates differ' in item for item in result['failures'])

    manifest_path, reports = write_formal_reports(tmp_path / 'passed')
    mutate_report(reports[0], lambda report: report.update(passed=False))
    result = compare(manifest_path, reports)
    assert result['passed'] is False
    assert any('stored passed differs' in item for item in result['failures'])


def test_compare_recomputes_receive_source_scaling_gate(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)

    def forge(report):
        report['topic_metrics']['image']['receive_span_sec'] = 1.0
        report['gates'] = {name: True for name in report['gates']}
        report['passed'] = True

    mutate_report(reports[0], forge)
    result = compare(manifest_path, reports)

    assert result['passed'] is False
    assert result['recomputed_gates'][str(reports[0])][
        'replay_rate_consistent'] is False
    assert any('stored gates differ' in item for item in result['failures'])


@pytest.mark.parametrize(
    'mutation',
    [
        lambda metric: metric.update(count=metric['count'] + 1),
        lambda metric: metric.update(count=1),
        lambda metric: metric.update(
            source_start_ns=metric['source_start_ns'] + 1000000000),
        lambda metric: metric.update(
            source_start_ns=metric['source_end_ns'] + 1),
        lambda metric: metric.update(
            source_rate_hz=metric['source_rate_hz'] + 0.125),
        lambda metric: metric.update(
            receive_rate_hz=metric['receive_rate_hz'] + 0.125),
    ],
)
def test_compare_rejects_internally_inconsistent_topic_metrics(
        tmp_path, mutation):
    manifest_path, reports = write_formal_reports(tmp_path)
    mutate_report(
        reports[0],
        lambda report: mutation(report['topic_metrics']['image']))

    result = compare(manifest_path, reports)

    assert result['passed'] is False
    assert any(
        '{}: malformed report: topic_metrics.image'.format(reports[0])
        in failure
        for failure in result['failures'])


def test_compare_allows_internally_consistent_zero_span_topic_metric(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)
    duplicate_metric = {
        'count': 2,
        'source_start_ns': 1000000000,
        'source_end_ns': 1000000000,
        'source_span_sec': 0.0,
        'receive_span_sec': 0.0,
        'source_rate_hz': 0.0,
        'receive_rate_hz': 0.0,
        'source_sequence_sha256': 'e' * 64,
    }
    for report in reports:
        mutate_report(
            report,
            lambda payload: payload['topic_metrics'].update(
                diagnostic=duplicate_metric))

    assert compare(manifest_path, reports)['passed'] is True


def test_compare_rejects_nonfinite_or_unbounded_nested_metric(tmp_path):
    for name, value in (('nan', math.nan), ('huge', 10 ** 400)):
        manifest_path, reports = write_formal_reports(tmp_path / name)
        mutate_report(
            reports[0],
            lambda report, value=value: report['topic_metrics'][
                'image'].update(source_rate_hz=value))

        result = compare(manifest_path, reports)

        assert result['passed'] is False
        assert any('malformed report' in item
                   for item in result['failures'])


@pytest.mark.parametrize(
    'mutation, failure',
    [
        (lambda report: report.update(dataset_id='other'), 'dataset_id'),
        (lambda report: report['artifacts'].update(
            software_revision='other'), 'software_revision'),
        (lambda report: report['artifacts'].update(
            config_sha256='d' * 64), 'config_sha256'),
        (lambda report: report['artifacts'].update(
            model_exports=['model']), 'model_exports'),
        (lambda report: report['coverage'].update(
            accuracy='other'), 'coverage.accuracy'),
    ],
)
def test_compare_requires_matching_provenance(
        tmp_path, mutation, failure):
    manifest_path, reports = write_formal_reports(tmp_path)
    mutate_report(reports[1], mutation)

    result = compare(manifest_path, reports)

    assert result['passed'] is False
    assert any(failure in item for item in result['failures'])


@pytest.mark.parametrize(
    'section, value',
    [
        ('run', []),
        ('topic_metrics', []),
        ('synchronization', []),
        ('localization', []),
        ('safety', []),
        ('gates', []),
    ],
)
def test_compare_reports_malformed_sections_as_failures(
        tmp_path, section, value):
    manifest_path, reports = write_formal_reports(tmp_path)
    mutate_report(reports[0], lambda report: report.update({section: value}))

    result = compare(manifest_path, reports)

    assert result['passed'] is False
    assert any('malformed report' in item for item in result['failures'])


def test_compare_and_cli_turn_load_errors_into_nonzero_failures(
        tmp_path, capsys):
    manifest_path, reports = write_formal_reports(tmp_path)
    reports[0].write_text('{not json', encoding='utf-8')

    result = compare(manifest_path, reports)

    assert result['passed'] is False
    assert any('could not load report' in item for item in result['failures'])
    arguments = ['--manifest', str(manifest_path)] + [
        str(path) for path in reports]
    assert compare_reports.main(arguments) == 1
    output = json.loads(capsys.readouterr().out)
    assert output['passed'] is False


def localization_diagnostic(mode='OBSERVATION_ONLY', epoch='1'):
    status = SimpleNamespace(
        name='semantic_search/localization',
        values=[
            SimpleNamespace(key='memory_mode', value=mode),
            SimpleNamespace(key='epoch_id', value=epoch),
        ],
    )
    return SimpleNamespace(status=[status])


def semantic_message(field):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0)),
        **{field: [object()]}
    )


def test_accumulator_readiness_requires_every_manifest_topic():
    metrics = make_metrics()

    assert metrics.required_topics_ready() is False
    metrics.observe_topic('image', 0, 0)
    assert metrics.required_topics_ready() is False
    metrics.observe_topic('lidar', 20000000, 20000000)
    assert metrics.required_topics_ready() is True


def test_node_excludes_pre_ready_diagnostics_and_semantics_but_not_safety():
    node = object.__new__(SemanticSearchEvaluatorNode)
    node.metrics = make_metrics()
    node.started_ros_ns = 0

    node.diagnostic_callback(localization_diagnostic())
    node.region_callback(semantic_message('regions'))
    node.observation_callback(semantic_message('observations'))
    node.tracked_callback(semantic_message('objects'))
    node.intent_callback(SimpleNamespace(forward_permitted=True))

    assert node.metrics.localization_mode_sequence == []
    assert node.metrics.semantic_region_count == 0
    assert node.metrics.observation_count == 0
    assert node.metrics.tracked_object_count == 0
    assert node.metrics.forward_permission_violations == 1
    assert 'semantic_regions' not in node.metrics.topics

    node.metrics.observe_topic('image', 0, 0)
    node.metrics.observe_topic('lidar', 20000000, 20000000)
    node.diagnostic_callback(localization_diagnostic())
    node.region_callback(semantic_message('regions'))
    node.observation_callback(semantic_message('observations'))
    node.tracked_callback(semantic_message('objects'))

    assert node.metrics.localization_mode_sequence == ['OBSERVATION_ONLY']
    assert node.metrics.semantic_region_count == 1
    assert node.metrics.observation_count == 1
    assert node.metrics.tracked_object_count == 1


def test_node_declares_and_passes_explicit_replay_policy_parameters():
    source = inspect.getsource(SemanticSearchEvaluatorNode.__init__)

    assert "'duration_sec'" in source
    assert "'timing_policy'" in source
    assert "'freshness_time_base'" in source
    assert 'wall_duration_sec=self.duration_sec' in source
    assert 'timing_policy=timing_policy' in source
    assert 'freshness_time_base=freshness_time_base' in source


def test_node_converts_stamps_and_records_monotonic_receive_time(monkeypatch):
    metrics = make_metrics()
    fake = SimpleNamespace(
        metrics=metrics,
        started_ros_ns=None,
        process=SimpleNamespace(cpu_percent=lambda interval=None: None),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=900)),
    )
    message_stamp = SimpleNamespace(sec=2, nanosec=3)
    monkeypatch.setattr(
        'track_robot_semantic_search.evaluator_node.time.monotonic_ns',
        lambda: 700,
    )
    monkeypatch.setattr(
        'track_robot_semantic_search.evaluator_node.psutil.cpu_percent',
        lambda interval=None: None,
    )

    SemanticSearchEvaluatorNode._observe(fake, 'image', message_stamp)

    assert stamp_ns(message_stamp) == 2000000003
    assert metrics.topics['image'].source_stamps == [2000000003]
    assert metrics.topics['image'].receive_stamps == [700]
    assert fake.started_ros_ns == 900


def test_semantic_output_does_not_start_the_sensor_evaluation_window(
        monkeypatch):
    metrics = make_metrics()
    fake = SimpleNamespace(
        metrics=metrics,
        started_ros_ns=None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=900)),
    )
    message_stamp = SimpleNamespace(sec=2, nanosec=3)
    monkeypatch.setattr(
        'track_robot_semantic_search.evaluator_node.time.monotonic_ns',
        lambda: 700,
    )

    SemanticSearchEvaluatorNode._observe(
        fake, 'semantic_regions', message_stamp)

    assert metrics.topics['semantic_regions'].source_stamps == [2000000003]
    assert fake.started_ros_ns is None


def stamp_pairer():
    return evaluator_node.StampPairMatcher()


def collect_pairs(events):
    matcher = stamp_pairer()
    offsets = []
    for topic, stamp in events:
        offsets.extend(matcher.observe(topic, stamp))
    offsets.extend(matcher.flush())
    return offsets


def test_stamp_pairing_is_delayed_and_consumes_both_stamps_once():
    matcher = stamp_pairer()

    assert matcher.observe('image', 0) == []
    assert matcher.observe('lidar', 1) == []
    assert matcher.observe('lidar', 2) == []
    assert matcher.observe('image', 10) == []
    assert matcher.flush() == [1, 8]
    assert matcher.flush() == []


def test_stamp_pairing_is_independent_of_cross_topic_arrival_order():
    image_first = [
        ('image', 0), ('lidar', 9), ('image', 10),
        ('lidar', 10), ('image', 30), ('lidar', 100),
    ]
    lidar_first = [
        ('lidar', 9), ('image', 0), ('lidar', 10),
        ('image', 10), ('lidar', 100), ('image', 30),
    ]

    assert collect_pairs(image_first) == [0, 9, 70]
    assert collect_pairs(lidar_first) == [0, 9, 70]


def test_stamp_pairing_handles_duplicates_without_reuse():
    events = [
        ('image', 10), ('image', 10), ('lidar', 10),
        ('image', 20), ('lidar', 20),
    ]

    assert collect_pairs(events) == [0, 0]


def test_stamp_pairing_flushes_unmatched_unequal_frequency_tail():
    matcher = stamp_pairer()
    for stamp in (0, 5, 10):
        assert matcher.observe('image', stamp) == []
    assert matcher.observe('lidar', 6) == []

    assert matcher.flush() == [1]
    assert matcher.flush() == []


def pairing_report_for_lidar_arrival_order(lidar_stamps):
    matcher = stamp_pairer()
    for stamp in (0, 246000000):
        assert matcher.observe('image', stamp) == []
    for stamp in lidar_stamps:
        assert matcher.observe('lidar', stamp) == []
    offsets = matcher.flush()

    metrics = make_metrics()
    metrics.observe_topic('image', 1, 1)
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_localization('OBSERVATION_ONLY', 1)
    for offset in offsets:
        metrics.observe_pair_offset(offset)
    return offsets, metrics.finalize()


def test_stamp_pairing_is_independent_of_late_source_stamp_arrival():
    first_offsets, first_report = pairing_report_for_lidar_arrival_order(
        (82000000, 328000000, 0))
    second_offsets, second_report = pairing_report_for_lidar_arrival_order(
        (82000000, 0, 328000000))

    assert first_offsets == second_offsets == [0, 82000000]
    assert first_report['synchronization']['p95_sec'] == \
        second_report['synchronization']['p95_sec'] == 0.0779
    assert first_report['gates']['sync_p95_at_most_80_ms'] is True
    assert second_report['gates']['sync_p95_at_most_80_ms'] is True


def test_bad_diagnostic_epoch_is_recorded_as_a_gate_violation():
    metrics = make_metrics()
    observe_minimum_healthy_replay(metrics)
    fake = SimpleNamespace(metrics=metrics)
    message = SimpleNamespace(status=[SimpleNamespace(
        name='semantic_search/localization',
        values=[
            SimpleNamespace(key='memory_mode', value='OBSERVATION_ONLY'),
            SimpleNamespace(key='epoch_id', value='not-an-int'),
        ],
    )])

    SemanticSearchEvaluatorNode.diagnostic_callback(fake, message)

    report = metrics.finalize()
    assert report['localization']['epoch_ids'] == [1]
    assert report['gates']['manifest_localization_mode_respected'] is False


def test_resources_are_sampled_only_inside_the_sensor_window(monkeypatch):
    samples = []
    fake = SimpleNamespace(
        started_ros_ns=None,
        finished=False,
        metrics=SimpleNamespace(
            observe_resource=lambda *values: samples.append(values)),
        process=SimpleNamespace(
            cpu_percent=lambda interval=None: 10.0,
            memory_info=lambda: SimpleNamespace(rss=20 * 1024 * 1024)),
    )
    monkeypatch.setattr(
        evaluator_node.psutil, 'cpu_percent', lambda interval=None: 30.0)
    monkeypatch.setattr(
        evaluator_node.psutil, 'virtual_memory',
        lambda: SimpleNamespace(used=40 * 1024 * 1024))

    SemanticSearchEvaluatorNode.resource_callback(fake)
    fake.started_ros_ns = 1
    SemanticSearchEvaluatorNode.resource_callback(fake)
    fake.finished = True
    SemanticSearchEvaluatorNode.resource_callback(fake)

    assert samples == [(10.0, 20.0, 30.0, 40.0)]


def test_first_sensor_reprimes_cpu_before_the_first_window_sample(monkeypatch):
    process_cpu_values = iter((91.0, 82.0, 11.0))
    system_cpu_values = iter((93.0, 84.0, 13.0))
    metrics = make_metrics()
    fake = SimpleNamespace(
        started_ros_ns=None,
        finished=False,
        metrics=metrics,
        process=SimpleNamespace(
            cpu_percent=lambda interval=None: next(process_cpu_values),
            memory_info=lambda: SimpleNamespace(rss=20 * 1024 * 1024)),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=900)),
    )
    monkeypatch.setattr(
        evaluator_node.psutil, 'cpu_percent',
        lambda interval=None: next(system_cpu_values))
    monkeypatch.setattr(
        evaluator_node.psutil, 'virtual_memory',
        lambda: SimpleNamespace(used=40 * 1024 * 1024))
    monkeypatch.setattr(evaluator_node.time, 'monotonic_ns', lambda: 700)

    fake.process.cpu_percent(interval=None)
    evaluator_node.psutil.cpu_percent(interval=None)
    stamp = SimpleNamespace(sec=1, nanosec=0)
    SemanticSearchEvaluatorNode._observe(fake, 'semantic_regions', stamp)
    SemanticSearchEvaluatorNode._observe(fake, 'image', stamp)
    SemanticSearchEvaluatorNode._observe(fake, 'lidar', stamp)
    SemanticSearchEvaluatorNode.resource_callback(fake)

    assert metrics.process_cpu_percent == [11.0]
    assert metrics.system_cpu_percent == [13.0]


def test_node_finishes_after_duration_and_writes_exactly_once(
        monkeypatch, tmp_path):
    writes = []
    shutdowns = []
    now = SimpleNamespace(nanoseconds=1099)
    fake = SimpleNamespace(
        finished=False,
        started_ros_ns=1000,
        duration_sec=0.0000001,
        output_path=tmp_path / 'report.json',
        metrics=SimpleNamespace(finalize=lambda: {'passed': True}),
        pair_matcher=SimpleNamespace(flush=lambda: []),
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    monkeypatch.setattr(
        'track_robot_semantic_search.evaluator_node.write_json_atomic',
        lambda path, report: writes.append((path, report)),
    )
    monkeypatch.setattr(
        'track_robot_semantic_search.evaluator_node.rclpy.shutdown',
        lambda: shutdowns.append(True),
    )

    SemanticSearchEvaluatorNode.finish_callback(fake)
    assert writes == []
    now.nanoseconds = 1100
    SemanticSearchEvaluatorNode.finish_callback(fake)
    SemanticSearchEvaluatorNode.finish_callback(fake)

    assert writes == [(fake.output_path, {'passed': True})]
    assert shutdowns == [True]
    assert fake.finished is True


def test_node_flushes_pair_tail_before_final_report(monkeypatch, tmp_path):
    writes = []
    pair_offsets = []
    fake = SimpleNamespace(
        finished=False,
        started_ros_ns=1000,
        duration_sec=0.0000001,
        output_path=tmp_path / 'report.json',
        metrics=SimpleNamespace(
            observe_pair_offset=lambda value: pair_offsets.append(value),
            finalize=lambda: {'pair_offsets': list(pair_offsets)}),
        pair_matcher=SimpleNamespace(flush=lambda: [7, 11]),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1100)),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    monkeypatch.setattr(
        evaluator_node, 'write_json_atomic',
        lambda path, report: writes.append((path, report)))
    monkeypatch.setattr(evaluator_node.rclpy, 'shutdown', lambda: None)

    SemanticSearchEvaluatorNode.finish_callback(fake)

    assert pair_offsets == [7, 11]
    assert writes == [(fake.output_path, {'pair_offsets': [7, 11]})]


def test_node_rejects_nonfinite_report_before_atomic_write(
        monkeypatch, tmp_path):
    writes = []
    fake = SimpleNamespace(
        finished=False,
        started_ros_ns=1000,
        duration_sec=0.0000001,
        output_path=tmp_path / 'report.json',
        metrics=SimpleNamespace(
            finalize=lambda: {'nonfinite': math.nan}),
        pair_matcher=SimpleNamespace(flush=lambda: []),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1100)),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    monkeypatch.setattr(
        evaluator_node, 'write_json_atomic',
        lambda path, report: writes.append((path, report)))

    with pytest.raises(ValueError):
        SemanticSearchEvaluatorNode.finish_callback(fake)

    assert writes == []
    assert fake.finished is False


def test_evaluator_node_defines_no_application_publisher():
    source = inspect.getsource(SemanticSearchEvaluatorNode)

    assert 'create_publisher' not in source
