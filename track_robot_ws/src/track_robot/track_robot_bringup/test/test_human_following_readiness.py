import subprocess

import pytest

from track_robot_bringup import human_following_readiness
from track_robot_bringup.readiness import CheckStatus


TOPICS = (
    '/zed/zed_node/left/image_rect_color',
    '/zed/zed_node/left/camera_info',
    '/zed/zed_node/imu/data',
    '/rslidar_points',
    '/bunker_status',
    '/bunker_rc_state',
    '/odom',
    '/human_tracking/target_state',
    '/perception/health',
    '/follow/avoidance_state',
    '/safety/state',
)


class HealthyRunner:
    def __init__(
            self,
            runtime_mode='active',
            publisher_overrides=None,
            stale_topic=None,
            missing_service=None,
            missing_tf=None,
            bunker_mode=1,
            vehicle_state=0,
            error_code=0,
            cmd_vel_owner='cmd_vel_gate'):
        self.runtime_mode = runtime_mode
        self.publisher_overrides = publisher_overrides or {}
        self.stale_topic = stale_topic
        self.missing_service = missing_service
        self.missing_tf = missing_tf
        self.bunker_mode = bunker_mode
        self.vehicle_state = vehicle_state
        self.error_code = error_code
        self.cmd_vel_owner = cmd_vel_owner
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        if argv[:3] == ['ros2', 'topic', 'info']:
            topic = argv[3]
            if topic == '/cmd_vel':
                count = 1 if self.runtime_mode == 'active' else 0
                output = 'Publisher count: {}\n'.format(count)
                if count:
                    output += (
                        'Node name: {}\nNode namespace: /\n'.format(
                            self.cmd_vel_owner))
                return subprocess.CompletedProcess(argv, 0, output, '')
            count = self.publisher_overrides.get(topic, 1)
            return subprocess.CompletedProcess(
                argv, 0, 'Publisher count: {}\n'.format(count), '')
        if argv[:3] == ['ros2', 'topic', 'echo']:
            topic = argv[3]
            if topic == self.stale_topic:
                return subprocess.CompletedProcess(argv, 0, '', '')
            if topic == '/bunker_status':
                output = (
                    'header:\n  stamp:\n    sec: 1\n'
                    'vehicle_state: {}\nerror_code: {}\n'
                    'control_mode: {}\n---\n'
                    .format(
                        self.vehicle_state, self.error_code,
                        self.bunker_mode))
            else:
                output = 'header:\n  stamp:\n    sec: 1\n---\n'
            return subprocess.CompletedProcess(argv, 0, output, '')
        if argv[:4] == ['ros2', 'run', 'tf2_ros', 'tf2_echo']:
            pair = (argv[4], argv[5])
            output = (
                '' if pair == self.missing_tf
                else 'Translation: [0, 0, 0]\n')
            return subprocess.CompletedProcess(argv, 0, output, '')
        if argv[:3] == ['ros2', 'service', 'type']:
            service = argv[3]
            if service == self.missing_service:
                return subprocess.CompletedProcess(
                    argv, 1, '', 'The service type is invalid\n')
            return subprocess.CompletedProcess(
                argv, 0, 'std_srvs/srv/Trigger\n', '')
        raise AssertionError(argv)


@pytest.mark.parametrize('runtime_mode', ['shadow', 'active'])
def test_complete_graph_passes_with_exact_publishers_and_fresh_messages(
        runtime_mode):
    runner = HealthyRunner(runtime_mode)

    report = human_following_readiness.check_human_following(
        runtime_mode,
        runner=runner,
        environment={'PATH': '/bin'},
        topic_timeout=0.2,
        tf_timeout=0.1,
    )

    assert report.overall is CheckStatus.PASS
    assert report.names == (
        'image', 'camera_info', 'imu', 'cloud', 'bunker_status',
        'rc_state', 'odometry', 'target_state', 'perception_health',
        'avoidance', 'safety', 'tf_camera', 'tf_lidar', 'safety_arm',
        'safety_disarm', 'reset_target', 'cmd_vel',
    )
    assert all(
        call[1]['env']['ROS_DOMAIN_ID'] == '20'
        for call in runner.calls)
    assert all(call[1]['shell'] is False for call in runner.calls)
    assert all(call[1]['timeout'] <= 0.2 for call in runner.calls)


@pytest.mark.parametrize('topic', TOPICS)
def test_active_fails_on_missing_or_duplicate_topic_publisher(topic):
    for count in (0, 2):
        report = human_following_readiness.check_human_following(
            'active',
            runner=HealthyRunner(publisher_overrides={topic: count}),
            environment={},
            topic_timeout=0.1,
        )

        assert report.overall is CheckStatus.FAIL
        matching = [check for check in report.checks if topic in check.detail]
        assert matching and matching[0].status is CheckStatus.FAIL


def test_active_fails_when_a_required_topic_is_stale():
    report = human_following_readiness.check_human_following(
        'active',
        runner=HealthyRunner(stale_topic='/perception/health'),
        environment={},
        topic_timeout=0.1,
    )

    assert report.overall is CheckStatus.FAIL
    assert next(
        check for check in report.checks
        if check.name == 'perception_health').status is CheckStatus.FAIL


@pytest.mark.parametrize('service', [
    '/safety/arm',
    '/safety/disarm',
    '/human_tracking/reset_target',
])
def test_active_fails_when_required_trigger_service_is_missing(service):
    report = human_following_readiness.check_human_following(
        'active',
        runner=HealthyRunner(missing_service=service),
        environment={},
        topic_timeout=0.1,
    )

    assert report.overall is CheckStatus.FAIL
    assert any(
        check.status is CheckStatus.FAIL and service in check.detail
        for check in report.checks)


@pytest.mark.parametrize('pair', [
    ('base_link', 'zed_left_camera_optical_frame'),
    ('base_link', 'rslidar'),
])
def test_active_fails_when_required_transform_is_missing(pair):
    report = human_following_readiness.check_human_following(
        'active',
        runner=HealthyRunner(missing_tf=pair),
        environment={},
        topic_timeout=0.1,
        tf_timeout=0.05,
    )

    assert report.overall is CheckStatus.FAIL
    assert any(
        check.status is CheckStatus.FAIL and 'transform' in check.detail
        for check in report.checks)


@pytest.mark.parametrize('kwargs', [
    {'bunker_mode': 3},
    {'vehicle_state': 1},
    {'error_code': 8},
])
def test_active_fails_on_non_can_mode_or_base_error(kwargs):
    report = human_following_readiness.check_human_following(
        'active',
        runner=HealthyRunner(**kwargs),
        environment={},
        topic_timeout=0.1,
    )

    assert report.overall is CheckStatus.FAIL
    bunker = next(
        check for check in report.checks if check.name == 'bunker_status')
    assert bunker.status is CheckStatus.FAIL


@pytest.mark.parametrize('runtime_mode,owner', [
    ('shadow', 'cmd_vel_gate'),
    ('active', 'other_node'),
])
def test_cmd_vel_publisher_count_and_active_owner_are_fail_closed(
        runtime_mode, owner):
    runner = HealthyRunner(runtime_mode, cmd_vel_owner=owner)
    if runtime_mode == 'shadow':
        runner.runtime_mode = 'active'

    report = human_following_readiness.check_human_following(
        runtime_mode,
        runner=runner,
        environment={},
        topic_timeout=0.1,
    )

    assert report.overall is CheckStatus.FAIL
    cmd_vel = next(check for check in report.checks if check.name == 'cmd_vel')
    assert cmd_vel.status is CheckStatus.FAIL


def test_active_does_not_mistake_cmd_vel_gate_subscriber_for_publisher():
    class MisleadingRunner(HealthyRunner):
        def __call__(self, argv, **kwargs):
            if argv[:4] == ['ros2', 'topic', 'info', '/cmd_vel']:
                self.calls.append((argv, kwargs))
                return subprocess.CompletedProcess(
                    argv,
                    0,
                    'Publisher count: 1\n'
                    'Node name: other_node\n'
                    'Node namespace: /\n'
                    'Subscription count: 1\n'
                    'Node name: cmd_vel_gate\n'
                    'Node namespace: /\n',
                    '',
                )
            return super().__call__(argv, **kwargs)

    report = human_following_readiness.check_human_following(
        'active',
        runner=MisleadingRunner(),
        environment={},
        topic_timeout=0.1,
    )

    assert report.overall is CheckStatus.FAIL
    cmd_vel = next(check for check in report.checks if check.name == 'cmd_vel')
    assert cmd_vel.status is CheckStatus.FAIL
