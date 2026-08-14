import hashlib
import json
import subprocess
from pathlib import Path

import pytest
import yaml

from track_robot_bringup.control_config import HardwareSelection
from track_robot_bringup.readiness import (
    CheckResult,
    CheckStatus,
    ReadinessReport,
    RosCliProbe,
    StaticProbe,
    check_stage,
)


@pytest.fixture
def paths(tmp_path):
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    checkpoint = tmp_path / 'checkpoint.pt'
    checkpoint.write_bytes(b'checkpoint')
    yolo_runtime = tmp_path / 'yolo_runtime'
    yolo_runtime.mkdir()
    yolo_checkpoint = tmp_path / 'yolo.pt'
    yolo_checkpoint.write_bytes(b'yolo')
    dino_repo = tmp_path / 'dino_repo'
    dino_repo.mkdir()
    dino_checkpoint = tmp_path / 'dino.pth'
    dino_checkpoint.write_bytes(b'dino')
    cuda_runtime = tmp_path / 'cuda'
    cuda_runtime.mkdir()
    dds_profile = tmp_path / 'dds.xml'
    dds_profile.write_text('<profiles/>', encoding='utf-8')
    calibration = tmp_path / 'calibration.yaml'
    calibration.write_text(yaml.safe_dump({
        'calibration_id': 'measured_2026_07_23',
        'parent_frame': 'base_link',
        'child_frame': 'zed_camera_link',
        'translation': {'x': 0.27, 'y': 0.0, 'z': 0.62},
        'rotation_rpy': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    }), encoding='utf-8')
    return {
        'runtime_path': runtime,
        'checkpoint_path': checkpoint,
        'checkpoint_sha256': hashlib.sha256(b'checkpoint').hexdigest(),
        'yolo_runtime_path': yolo_runtime,
        'yolo_checkpoint_path': yolo_checkpoint,
        'yolo_checkpoint_sha256': hashlib.sha256(b'yolo').hexdigest(),
        'dino_repo_path': dino_repo,
        'dino_checkpoint_path': dino_checkpoint,
        'dino_checkpoint_sha256': hashlib.sha256(b'dino').hexdigest(),
        'cuda_runtime_path': cuda_runtime,
        'phase3_profile_path': tmp_path / 'phase123_test.yaml',
        'dds_profile': dds_profile,
        'extrinsic_mode': 'measured',
        'extrinsic_file': calibration,
        'allow_degraded': False,
    }


class FakeProbe:
    def __init__(self):
        self.topic_calls = []
        self.publisher_calls = []
        self.tf_calls = []

    def static(self, paths, stage):
        return StaticProbe(paths).check(stage)

    def topic(self, name, topic):
        self.topic_calls.append((name, topic))
        return CheckResult.pass_(name, '{} at 15.0 Hz'.format(topic))

    def publisher(self, name, topic):
        self.publisher_calls.append((name, topic))
        return CheckResult.pass_(name, '{} has 1 publisher'.format(topic))

    def transform(self, name, target, source):
        self.tf_calls.append((name, target, source))
        return CheckResult.pass_(name, '{} -> {}'.format(target, source))

    def cmd_vel(self):
        return CheckResult.pass_('cmd_vel', 'zero publishers')


@pytest.fixture
def fake_probe():
    return FakeProbe()


def selection():
    return HardwareSelection(camera=True, lidar=True, base=True, imu=True)


def test_not_ready_dominates_degraded_and_pass():
    report = ReadinessReport([
        CheckResult.pass_('camera', '15 Hz'),
        CheckResult.degraded('calibration', 'prototype'),
        CheckResult.not_ready('tf', 'base_link -> zed_camera_link missing'),
    ])

    assert report.overall is CheckStatus.NOT_READY


def test_fail_dominates_all_other_readiness_states():
    report = ReadinessReport([
        CheckResult.not_ready('camera', 'missing'),
        CheckResult.fail('cmd_vel', 'publisher detected'),
    ])

    assert report.overall is CheckStatus.FAIL


def test_phase1_does_not_check_lidar_imu_or_base(paths, fake_probe):
    report = check_stage('phase1', selection(), paths, fake_probe)

    assert 'lidar' not in report.names
    assert 'imu' not in report.names
    assert 'odometry' not in report.names
    assert 'tf_lidar' not in report.names
    assert 'tf_camera_optical' not in report.names
    assert report.names == (
        'yolo_runtime', 'yolo_checkpoint', 'clip_runtime', 'clip_checkpoint',
        'dino_repo', 'dino_checkpoint', 'cuda_runtime', 'dds_profile',
        'camera', 'camera_info', 'regions', 'cmd_vel')
    assert fake_probe.publisher_calls == [
        ('regions', '/semantic_search/regions'),
    ]
    assert ('regions', '/semantic_search/regions') not in fake_probe.topic_calls
    assert ('camera', '/zed/zed_node/left/image_rect_color') in (
        fake_probe.topic_calls)


def test_phase2_checks_required_topics_and_tf_paths(paths, fake_probe):
    report = check_stage('phase2', selection(), paths, fake_probe)

    assert report.names == (
        'yolo_runtime', 'yolo_checkpoint', 'clip_runtime', 'clip_checkpoint',
        'dino_repo', 'dino_checkpoint', 'cuda_runtime', 'dds_profile',
        'calibration', 'camera', 'camera_info', 'lidar', 'imu', 'odometry',
        'regions', 'tracklets', 'localization', 'memory',
        'tf_camera_optical', 'tf_lidar', 'cmd_vel')
    assert ('tf_camera_optical', 'base_link',
            'zed_left_camera_optical_frame') in fake_probe.tf_calls
    assert ('tf_lidar', 'base_link', 'lidar_link') in fake_probe.tf_calls


def test_phase0_checks_no_sensor_model_or_tf(paths, fake_probe):
    report = check_stage('phase0', selection(), paths, fake_probe)

    assert report.names == ('dds_profile', 'cmd_vel')
    assert fake_probe.topic_calls == []
    assert fake_probe.publisher_calls == []
    assert fake_probe.tf_calls == []


def test_phase3_requires_diagnostic_ranking_and_uncalibrated_safe_profile(
        paths, fake_probe):
    profile = Path(paths['phase3_profile_path'] or '')
    profile.write_text(yaml.safe_dump({
        'semantic_memory': {
            'ros__parameters': {
                'camera_only_memory_enabled': True,
                'publish_diagnostic_ranking': True,
                'best_candidate_threshold_calibrated': False,
                'reidentification_mutation_enabled': False,
            },
        },
    }), encoding='utf-8')

    report = check_stage('phase3', selection(), paths, fake_probe)

    assert 'phase3_safe_profile' in report.names
    assert fake_probe.publisher_calls[-2:] == [
        ('diagnostic_ranking', '/semantic_memory/diagnostic_ranking'),
        ('best_candidate', '/semantic_memory/best_candidate'),
    ]


def test_prototype_calibration_is_degraded_only_when_explicitly_allowed(paths):
    paths.update(extrinsic_mode='prototype', allow_degraded=True)

    result = StaticProbe(paths).check('phase2')

    assert result[-1] == CheckResult.degraded(
        'calibration', 'prototype camera extrinsic; not calibrated')


def test_unapproved_prototype_calibration_fails_closed(paths):
    paths.update(extrinsic_mode='prototype', allow_degraded=False)

    result = StaticProbe(paths).check('phase2')

    assert result[-1].status is CheckStatus.FAIL
    assert 'allow_degraded' in result[-1].action


def test_string_false_does_not_authorize_prototype_calibration(paths):
    paths.update(extrinsic_mode='prototype', allow_degraded='false')

    result = StaticProbe(paths).check('phase2')

    assert result[-1].status is CheckStatus.FAIL


def test_measured_calibration_requires_task2_schema_and_finite_values(paths):
    Path(paths['extrinsic_file']).write_text(yaml.safe_dump({
        'calibration_id': 'replace_with_measured_calibration_id',
        'parent_frame': 'map',
        'child_frame': 'zed_camera_link',
        'translation': {'x': 'nan', 'y': 0.0, 'z': 0.0},
        'rotation_rpy': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    }), encoding='utf-8')

    result = StaticProbe(paths).check('phase2')

    assert result[-1].status is CheckStatus.FAIL
    assert result[-1].name == 'calibration'


def test_ros_cli_topic_echo_parses_foxy_output_after_bounded_timeout():
    calls = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        if argv[:3] == ['ros2', 'topic', 'info']:
            return subprocess.CompletedProcess(argv, 0, 'Publisher count: 1\n', '')
        if argv[:3] == ['ros2', 'topic', 'echo']:
            raise subprocess.TimeoutExpired(
                argv, kwargs['timeout'],
                output='header:\n  stamp:\n    sec: 42\n---\n')
        raise AssertionError(argv)

    result = RosCliProbe(runner=runner, topic_timeout=0.2).topic('camera', '/camera')

    assert result.status is CheckStatus.PASS
    assert 'publishing sensor messages' in result.detail
    assert all(kwargs['shell'] is False for _, kwargs in calls)
    assert calls[-1][0] == [
        'ros2', 'topic', 'echo', '/camera',
        '--qos-profile', 'sensor_data', '--no-arr', '--no-str',
    ]


def test_ros_cli_topic_uses_sensor_qos_echo_when_foxy_hz_cannot_discover():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        if argv == ['ros2', 'topic', 'info', '/camera']:
            return subprocess.CompletedProcess(argv, 0, 'Publisher count: 1\n', '')
        if argv == [
                'ros2', 'topic', 'echo', '/camera',
                '--qos-profile', 'sensor_data', '--no-arr', '--no-str']:
            raise subprocess.TimeoutExpired(
                argv, kwargs['timeout'],
                output='header:\n  stamp:\n    sec: 42\n---\n')
        raise AssertionError(argv)

    result = RosCliProbe(
        runner=runner, topic_timeout=0.2).topic('camera', '/camera')

    assert result == CheckResult.pass_(
        'camera', '/camera is publishing sensor messages')
    assert calls[-1] == [
        'ros2', 'topic', 'echo', '/camera',
        '--qos-profile', 'sensor_data', '--no-arr', '--no-str',
    ]


def test_ros_cli_topic_timeout_names_exact_missing_topic():
    def runner(argv, **kwargs):
        assert argv == ['ros2', 'topic', 'info', '/missing']
        return subprocess.CompletedProcess(argv, 1, '', 'Unknown topic /missing')

    result = RosCliProbe(runner=runner, topic_timeout=0.2).topic('camera', '/missing')

    assert result.status is CheckStatus.NOT_READY
    assert result.detail == 'topic /missing is absent'


def test_ros_cli_tf_requires_a_transform_line_after_timeout():
    def runner(argv, **kwargs):
        assert argv == [
            'ros2', 'run', 'tf2_ros', 'tf2_echo',
            'base_link', 'zed_left_camera_optical_frame']
        raise subprocess.TimeoutExpired(
            argv, kwargs['timeout'],
            output='At time 42.0\n- Translation: [0.27, 0.0, 0.62]\n')

    result = RosCliProbe(runner=runner, tf_timeout=0.2).transform(
        'tf_camera_optical', 'base_link', 'zed_left_camera_optical_frame')

    assert result.status is CheckStatus.PASS


def test_ros_cli_tf_startup_without_transform_is_not_ready():
    def runner(argv, **kwargs):
        return subprocess.CompletedProcess(argv, 0, 'Waiting for transform\n', '')

    result = RosCliProbe(runner=runner, tf_timeout=0.2).transform(
        'tf_lidar', 'base_link', 'lidar_link')

    assert result.status is CheckStatus.NOT_READY
    assert result.detail == 'timeout waiting for transform base_link -> lidar_link'


def test_cmd_vel_absent_or_zero_publishers_passes_and_any_publisher_fails():
    outputs = iter((
        subprocess.CompletedProcess([], 1, '', 'Unknown topic'),
        subprocess.CompletedProcess([], 0, 'Publisher count: 0\n', ''),
        subprocess.CompletedProcess([], 0, 'Publisher count: 2\n', ''),
    ))

    def runner(argv, **kwargs):
        assert argv == ['ros2', 'topic', 'info', '/cmd_vel']
        return next(outputs)

    probe = RosCliProbe(runner=runner)

    assert probe.cmd_vel().status is CheckStatus.PASS
    assert probe.cmd_vel().status is CheckStatus.PASS
    assert probe.cmd_vel().status is CheckStatus.FAIL


def test_ros_cli_probe_forces_domain_20_and_applies_optional_dds_profile():
    captured_environments = []

    def runner(argv, **kwargs):
        captured_environments.append(kwargs['env'])
        return subprocess.CompletedProcess(argv, 1, '', 'Unknown topic')

    result = RosCliProbe(
        runner=runner,
        environment={'PATH': '/bin', 'ROS_DOMAIN_ID': '99'},
        dds_profile='/tmp/readiness-dds.xml',
    ).cmd_vel()

    assert result.status is CheckStatus.PASS
    assert captured_environments == [{
        'PATH': '/bin',
        'ROS_DOMAIN_ID': '20',
        'FASTRTPS_DEFAULT_PROFILES_FILE': '/tmp/readiness-dds.xml',
        'PYTHONUNBUFFERED': '1',
    }]


def test_ros_cli_topic_info_transport_failure_is_fail():
    def runner(argv, **kwargs):
        assert argv == ['ros2', 'topic', 'info', '/camera']
        return subprocess.CompletedProcess(argv, 1, '', 'daemon unavailable')

    result = RosCliProbe(runner=runner).topic('camera', '/camera')

    assert result.status is CheckStatus.FAIL
    assert 'topic info failed' in result.detail


def test_ros_cli_topic_info_timeout_is_transient_not_ready():
    def runner(argv, **kwargs):
        assert argv == ['ros2', 'topic', 'info', '/camera']
        raise subprocess.TimeoutExpired(
            argv,
            kwargs['timeout'],
            stderr=(
                '/opt/ros/foxy/bin/ros2: DeprecationWarning: '
                'pkg_resources is deprecated'),
        )

    result = RosCliProbe(
        runner=runner, topic_timeout=0.2).publisher('camera', '/camera')

    assert result.status is CheckStatus.NOT_READY
    assert result.detail == 'timeout inspecting topic /camera'


def test_ros_cli_topic_info_absence_is_not_ready():
    def runner(argv, **kwargs):
        assert argv == ['ros2', 'topic', 'info', '/camera']
        return subprocess.CompletedProcess(argv, 1, '', 'Unknown topic /camera')

    result = RosCliProbe(runner=runner).topic('camera', '/camera')

    assert result.status is CheckStatus.NOT_READY
    assert result.detail == 'topic /camera is absent'


def test_ros_cli_topic_info_unknown_topic_is_not_ready():
    def runner(argv, **kwargs):
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(argv, 0, '/camera [sensor_msgs/msg/Image]\n', '')
        assert argv == ['ros2', 'topic', 'info', '/camera']
        return subprocess.CompletedProcess(argv, 1, '', 'Unknown topic /camera')

    result = RosCliProbe(runner=runner).topic('camera', '/camera')

    assert result.status is CheckStatus.NOT_READY
    assert result.detail == 'topic /camera is absent'


def test_ros_cli_topic_info_failure_is_fail_unless_topic_is_explicitly_absent():
    def runner(argv, **kwargs):
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(argv, 0, '/camera [sensor_msgs/msg/Image]\n', '')
        assert argv == ['ros2', 'topic', 'info', '/camera']
        return subprocess.CompletedProcess(argv, 1, '', 'permission denied')

    result = RosCliProbe(runner=runner).topic('camera', '/camera')

    assert result.status is CheckStatus.FAIL
    assert 'topic info failed' in result.detail


def test_ros_cli_topic_info_success_without_publisher_count_is_fail():
    def runner(argv, **kwargs):
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(argv, 0, '/camera [sensor_msgs/msg/Image]\n', '')
        assert argv == ['ros2', 'topic', 'info', '/camera']
        return subprocess.CompletedProcess(argv, 0, 'Topic type: sensor_msgs/msg/Image\n', '')

    result = RosCliProbe(runner=runner).topic('camera', '/camera')

    assert result.status is CheckStatus.FAIL
    assert 'could not determine publisher count' in result.detail


def test_ros_cli_publisher_check_does_not_wait_for_topic_messages():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(
                argv, 0,
                '/semantic_search/regions '
                '[track_robot_interfaces/msg/SemanticRegionArray]\n',
                '')
        assert argv == [
            'ros2', 'topic', 'info', '/semantic_search/regions',
        ]
        return subprocess.CompletedProcess(
            argv, 0, 'Publisher count: 1\n', '')

    result = RosCliProbe(runner=runner).publisher(
        'regions', '/semantic_search/regions')

    assert result == CheckResult.pass_(
        'regions', '/semantic_search/regions has 1 publisher')
    assert calls == [
        ['ros2', 'topic', 'info', '/semantic_search/regions'],
    ]
    assert not any(call[:3] == ['ros2', 'topic', 'hz'] for call in calls)


def test_ros_cli_publisher_uses_topic_info_when_foxy_topic_list_is_stale():
    """Foxy may omit a live remote topic from list while info can resolve it."""
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(
                argv, 0, '/rosout [rcl_interfaces/msg/Log]\n', '')
        assert argv == [
            'ros2', 'topic', 'info', '/zed/zed_node/left/image_rect_color',
        ]
        return subprocess.CompletedProcess(
            argv, 0, 'Type: sensor_msgs/msg/Image\nPublisher count: 1\n', '')

    result = RosCliProbe(runner=runner).publisher(
        'camera', '/zed/zed_node/left/image_rect_color')

    assert result == CheckResult.pass_(
        'camera', '/zed/zed_node/left/image_rect_color has 1 publisher')
    assert calls[-1] == [
        'ros2', 'topic', 'info', '/zed/zed_node/left/image_rect_color',
    ]


def test_cmd_vel_info_failure_and_unparseable_success_are_fail():
    outputs = iter((
        subprocess.CompletedProcess([], 1, '', 'permission denied'),
        subprocess.CompletedProcess([], 0, 'Topic type: geometry_msgs/msg/Twist\n', ''),
    ))

    def runner(argv, **kwargs):
        assert argv == ['ros2', 'topic', 'info', '/cmd_vel']
        return next(outputs)

    probe = RosCliProbe(runner=runner)

    assert probe.cmd_vel().status is CheckStatus.FAIL
    assert probe.cmd_vel().status is CheckStatus.FAIL


def test_cmd_vel_topic_info_timeout_is_fail_even_with_absence_like_output():
    def runner(argv, **kwargs):
        assert argv == ['ros2', 'topic', 'info', '/cmd_vel']
        raise subprocess.TimeoutExpired(
            argv, kwargs['timeout'], output='Unknown topic /cmd_vel')

    result = RosCliProbe(runner=runner).cmd_vel()

    assert result.status is CheckStatus.FAIL


def test_ros_cli_probe_composes_static_checks_without_importing_ros(paths):
    def runner(argv, **kwargs):
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(
                argv, 0,
                '/zed/zed_node/left/image_rect_color [sensor_msgs/msg/Image]\n'
                '/zed/zed_node/left/camera_info [sensor_msgs/msg/CameraInfo]\n'
                '/semantic_search/regions [track_robot_interfaces/msg/SemanticRegions]\n',
                '')
        if argv[:3] == ['ros2', 'topic', 'info']:
            if argv[-1] == '/cmd_vel':
                return subprocess.CompletedProcess(argv, 1, '', 'Unknown topic')
            return subprocess.CompletedProcess(argv, 0, 'Publisher count: 1\n', '')
        if argv[:3] == ['ros2', 'topic', 'echo']:
            raise subprocess.TimeoutExpired(
                argv, kwargs['timeout'],
                output='header:\n  stamp:\n    sec: 42\n---\n')
        raise AssertionError(argv)

    report = check_stage(
        'phase1', selection(), paths, RosCliProbe(runner=runner, topic_timeout=0.2))

    assert report.overall is CheckStatus.PASS


def test_report_text_and_json_are_stable_and_complete():
    report = ReadinessReport([
        CheckResult.pass_('camera', '15 Hz'),
        CheckResult.degraded('calibration', 'prototype'),
    ], stage='phase2', timestamp='2026-07-23T12:00:00Z', ros_domain_id='20')

    text = report.render_text()
    data = json.loads(report.render_json())

    assert text.splitlines() == [
        'PASS      camera       15 Hz',
        'DEGRADED  calibration  prototype',
        'Overall: DEGRADED',
    ]
    assert data == {
        'stage': 'phase2',
        'overall': 'DEGRADED',
        'checks': [
            {'status': 'PASS', 'name': 'camera', 'detail': '15 Hz', 'action': ''},
            {'status': 'DEGRADED', 'name': 'calibration', 'detail': 'prototype', 'action': ''},
        ],
        'timestamp': '2026-07-23T12:00:00Z',
        'ros_domain_id': '20',
    }
