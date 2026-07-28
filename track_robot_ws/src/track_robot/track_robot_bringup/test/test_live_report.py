import json
import math
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import track_robot_bringup.live_test as live_test
from track_robot_bringup.live_test import (
    _BoundedCollector,
    LiveTestSummary,
    Phase2Sample,
    RegionSample,
    build_report,
    run_live_test,
    write_report,
)


def summary(**overrides):
    values = {
        'stage': 'phase1',
        'query': 'blue chair',
        'duration_sec': 2.0,
        'frames': 10,
        'nonempty_frames': 8,
        'scores': [0.2, 0.4],
        'query_ids': {7},
        'query_versions': {1},
        'expected_query_id': 7,
        'expected_query_version': 1,
        'failures': [],
    }
    values.update(overrides)
    return LiveTestSummary(**values)


def test_report_separates_pipeline_from_semantic_correctness():
    report = build_report(summary())

    assert report['pipeline']['status'] == 'PASS'
    assert report['semantic_result']['status'] == 'REVIEW REQUIRED'
    assert report['metrics']['nonempty_frame_ratio'] == 0.8
    assert report['metrics']['score']['minimum'] == 0.2
    assert report['metrics']['score']['maximum'] == 0.4


def test_nonfinite_score_fails_pipeline():
    report = build_report(summary(scores=[float('nan')]))

    assert report['pipeline']['status'] == 'FAIL'
    assert any('non-finite' in item for item in report['pipeline']['failures'])
    assert report['metrics']['score'] == {
        'count': 1,
        'minimum': None,
        'maximum': None,
        'mean': None,
    }


def test_zero_frames_fails_pipeline_without_dividing_by_zero():
    report = build_report(summary(frames=0, nonempty_frames=0, scores=[]))

    assert report['pipeline']['status'] == 'FAIL'
    assert report['metrics']['nonempty_frame_ratio'] == 0.0


def test_inconsistent_query_ids_or_versions_fail_pipeline():
    identifiers = build_report(summary(query_ids={7, 8}))
    versions = build_report(summary(query_versions={1, 2}))
    unexpected = build_report(summary(query_ids={8}, query_versions={2}))

    assert identifiers['pipeline']['status'] == 'FAIL'
    assert versions['pipeline']['status'] == 'FAIL'
    assert unexpected['pipeline']['status'] == 'FAIL'


def test_phase2_metrics_are_bounded_counts_not_message_history():
    report = build_report(summary(
        stage='phase2',
        phase2=Phase2Sample(
            tracklet_messages=3,
            tracklet_count=5,
            localization_messages=2,
            object_messages=4,
            object_count=6,
            association_messages=7,
            association_matches=2,
            calibration_mode='prototype',
        ),
    ))

    assert report['metrics']['phase2'] == {
        'tracklet_messages': 3,
        'tracklet_count': 5,
        'localization_messages': 2,
        'object_messages': 4,
        'object_count': 6,
        'association_messages': 7,
        'association_matches': 2,
        'diagnostic_ranking_messages': 0,
        'diagnostic_candidate_count': 0,
        'latest_memory_mode': None,
        'latest_localization_reason': '',
    }
    assert report['calibration']['mode'] == 'prototype'


def test_phase2_requires_each_critical_stream_during_test_interval():
    ready = Phase2Sample(
        tracklet_messages=1,
        localization_messages=1,
        object_messages=1,
    )
    assert build_report(summary(stage='phase2', phase2=ready))[
        'pipeline']['status'] == 'PASS'

    for missing in (
            'tracklet_messages',
            'localization_messages',
            'object_messages'):
        values = {
            'tracklet_messages': 1,
            'localization_messages': 1,
            'object_messages': 1,
            missing: 0,
        }
        report = build_report(summary(
            stage='phase2',
            phase2=Phase2Sample(**values),
        ))
        assert report['pipeline']['status'] == 'FAIL'
        assert any(
            missing.split('_')[0] in failure
            for failure in report['pipeline']['failures']
        )


def test_phase2_association_may_be_absent_and_latest_localization_is_reported():
    report = build_report(summary(
        stage='phase2',
        phase2=Phase2Sample(
            tracklet_messages=1,
            localization_messages=1,
            object_messages=1,
            association_messages=0,
            latest_memory_mode=2,
            latest_localization_reason='world localization healthy',
        ),
    ))

    assert report['pipeline']['status'] == 'PASS'
    assert report['metrics']['phase2']['association_messages'] == 0
    assert report['metrics']['phase2']['latest_memory_mode'] == 2
    assert report['metrics']['phase2']['latest_localization_reason'] == (
        'world localization healthy')


def test_phase3_requires_diagnostic_ranking_but_accepts_empty_abstention():
    sample = Phase2Sample(
        tracklet_messages=1,
        localization_messages=1,
        object_messages=1,
        diagnostic_ranking_messages=1,
        diagnostic_candidate_count=0,
    )

    report = build_report(summary(stage='phase3', phase2=sample))

    assert report['pipeline']['status'] == 'PASS'
    assert report['semantic_result']['status'] == 'REVIEW REQUIRED'
    assert report['metrics']['phase2']['diagnostic_ranking_messages'] == 1
    assert report['metrics']['phase2']['diagnostic_candidate_count'] == 0
    assert report['calibration']['state'] == 'UNCALIBRATED'


def test_write_report_is_strict_json_and_uses_explicit_output_directory(
        tmp_path):
    result = write_report(summary(), tmp_path)

    assert result.directory == tmp_path
    assert result.report_path == tmp_path / 'report.json'
    assert json.loads(result.report_path.read_text(encoding='utf-8'))[
        'pipeline']['status'] == 'PASS'
    assert list(tmp_path.glob('*.tmp')) == []


def test_write_report_rejects_nonfinite_json_even_if_nested(tmp_path):
    invalid = summary(readiness_snapshot={'rate': math.inf})

    try:
        write_report(invalid, tmp_path)
    except ValueError as error:
        assert 'JSON' in str(error) or 'range' in str(error)
    else:
        raise AssertionError('strict JSON must reject Infinity')


def test_overlay_failure_still_writes_passing_json_with_warning(
        tmp_path, monkeypatch):
    value = summary(
        best_region=RegionSample(
            query_id=7,
            query_version=1,
            score=0.9,
            x_offset=1,
            y_offset=2,
            width=3,
            height=4,
            stamp_ns=5,
        ),
        latest_image=object(),
    )
    monkeypatch.setitem(sys.modules, 'cv_bridge', None)

    result = write_report(value, tmp_path)
    payload = json.loads(result.report_path.read_text(encoding='utf-8'))

    assert result.overlay_path is None
    assert payload['pipeline']['status'] == 'PASS'
    assert any(
        'overlay not written' in warning
        for warning in payload['pipeline']['warnings']
    )


def test_run_live_test_queries_with_argv_before_collecting(tmp_path):
    calls = []
    collected = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        return subprocess.CompletedProcess(
            argv, 0,
            "ACCEPTED query_id=19 version=3 query='blue chair': reason='ok'\n",
            '')

    def collector(**kwargs):
        collected.append(kwargs)
        return LiveTestSummary(
            stage=kwargs['stage'],
            query=kwargs['query'],
            duration_sec=kwargs['duration_sec'],
            frames=1,
            nonempty_frames=1,
            scores=[0.5],
            query_ids={19},
            query_versions={3},
            expected_query_id=kwargs['query_id'],
            expected_query_version=kwargs['query_version'],
        )

    result = run_live_test(
        'phase1',
        'blue chair',
        0.1,
        tmp_path,
        {'ROS_DOMAIN_ID': '20'},
        runner=runner,
        collector=collector,
        query_timeout=0.2,
    )

    assert result.exit_code == 0
    assert calls[0][0] == [
        'ros2', 'run', 'track_robot_semantic_search',
        'semantic_search_query', 'blue chair',
    ]
    assert calls[0][1]['shell'] is False
    assert calls[0][1]['timeout'] == 0.2
    assert calls[0][1]['env']['ROS_DOMAIN_ID'] == '20'
    assert collected[0]['query_id'] == 19
    assert collected[0]['query_version'] == 3
    assert collected[0]['environment'] == {'ROS_DOMAIN_ID': '20'}


def test_run_live_test_does_not_collect_when_query_is_not_accepted(tmp_path):
    def collector(**kwargs):
        raise AssertionError('collection must follow a parsed ACCEPTED result')

    result = run_live_test(
        'phase1',
        'blue chair',
        0.1,
        tmp_path,
        {'ROS_DOMAIN_ID': '20'},
        runner=lambda argv, **kwargs: subprocess.CompletedProcess(
            argv, 0, 'unparseable success\n', ''),
        collector=collector,
    )

    assert result.exit_code == 4
    assert result.report_path is None


def test_region_sample_keeps_only_overlay_data():
    region = RegionSample(
        query_id=7,
        query_version=1,
        score=0.9,
        x_offset=1,
        y_offset=2,
        width=3,
        height=4,
        stamp_ns=5,
    )

    assert region.width == 3
    assert not hasattr(region, 'message_history')


def test_package_registers_live_test_and_runtime_dependencies():
    package_root = Path(__file__).resolve().parents[1]
    cmake = (package_root / 'CMakeLists.txt').read_text(encoding='utf-8')
    manifest = (package_root / 'package.xml').read_text(encoding='utf-8')

    assert 'test_live_report.py' in cmake
    assert '<exec_depend>cv_bridge</exec_depend>' in manifest
    assert '<exec_depend>python3-opencv</exec_depend>' in manifest


def test_streaming_score_stats_count_each_nonfinite_sample_once():
    collector = _BoundedCollector('phase1', 'target', 1.0, 7, 1)
    roi = type('Roi', (), {
        'x_offset': 0, 'y_offset': 0, 'width': 1, 'height': 1,
    })()
    region = type('Region', (), {
        'query_id': 7,
        'query_version': 1,
        'fused_score': float('nan'),
        'roi': roi,
        'header': None,
    })()
    message = type('Regions', (), {
        'query_id': 7,
        'query_version': 1,
        'regions': [region],
        'header': None,
    })()

    collector.regions(message)
    report = build_report(collector.summary)

    assert report['metrics']['score']['count'] == 1
    assert report['pipeline']['failures'].count(
        '1 non-finite score value(s) observed') == 1


def test_image_subscription_uses_sensor_data_qos_for_zed_best_effort():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_bringup'
        / 'live_test.py'
    ).read_text(encoding='utf-8')

    assert 'from rclpy.qos import qos_profile_sensor_data' in source
    image_subscription = source.split(
        "Image,\n            '/zed/zed_node/left/image_rect_color',", 1)[1]
    assert 'qos_profile_sensor_data' in image_subscription.split(')', 1)[0]


def test_private_ros_context_uses_an_executor_bound_to_the_same_context():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_bringup'
        / 'live_test.py'
    ).read_text(encoding='utf-8')

    assert 'SingleThreadedExecutor(context=context)' in source
    assert 'executor.spin_once(' in source
    assert 'rclpy.spin_once(node' not in source


def test_stamp_ns_preserves_header_time_for_overlay_correlation():
    message = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=12, nanosec=345),
    ))

    assert live_test._stamp_ns(message) == 12_000_000_345
    assert live_test._stamp_ns(SimpleNamespace(header=None)) is None


class FakeContext:
    def __init__(self):
        self.initialized = False
        self.shutdown_calls = 0

    def ok(self):
        return self.initialized and self.shutdown_calls == 0

    def try_shutdown(self):
        self.shutdown_calls += 1


class FakeRclpy:
    def __init__(self, fail_node=False):
        self.context = SimpleNamespace(Context=FakeContext)
        self.fail_node = fail_node
        self.seen = []
        self.last_context = None

    def init(self, *, args, context):
        self.seen.append(('init', dict(os.environ)))
        self.last_context = context
        context.initialized = True

    def create_node(self, name, *, context):
        self.seen.append(('node', dict(os.environ)))
        if self.fail_node:
            raise RuntimeError('node creation failed')
        return SimpleNamespace(name=name)


def test_isolated_context_creation_temporarily_applies_managed_ros_environment(
        monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '30')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_shell')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    before = dict(os.environ)
    managed = {
        'ROS_DOMAIN_ID': '20',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': '/managed/fastdds.xml',
        'ROS_LOCALHOST_ONLY': '0',
        'CYCLONEDDS_URI': 'file:///managed/cyclonedds.xml',
    }
    fake = FakeRclpy()

    context, node = live_test._create_isolated_node(fake, managed)

    assert node.name == 'semantic_search_live_test_collector'
    for _, environment in fake.seen:
        assert environment['ROS_DOMAIN_ID'] == '20'
        assert environment['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
        assert environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == (
            '/managed/fastdds.xml')
        assert environment['ROS_LOCALHOST_ONLY'] == '0'
        assert environment['CYCLONEDDS_URI'] == (
            'file:///managed/cyclonedds.xml')
    assert os.environ == before
    assert context.shutdown_calls == 0


def test_isolated_context_creation_restores_environment_on_node_error(
        monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '30')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '/shell/profile.xml')
    before = dict(os.environ)
    fake = FakeRclpy(fail_node=True)

    with pytest.raises(RuntimeError, match='node creation failed'):
        live_test._create_isolated_node(
            fake,
            {'ROS_DOMAIN_ID': '20'},
        )

    assert fake.seen[0][1]['ROS_DOMAIN_ID'] == '20'
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in fake.seen[0][1]
    assert os.environ == before
    assert fake.last_context.shutdown_calls == 1
