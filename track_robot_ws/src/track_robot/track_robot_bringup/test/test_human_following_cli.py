import io
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from track_robot_bringup import human_following_cli
from track_robot_bringup.control_config import HardwareSelection


def test_parser_supports_independent_commands_with_shadow_default():
    start = human_following_cli.build_parser().parse_args(['start'])
    doctor = human_following_cli.build_parser().parse_args(['doctor'])

    assert start.command == 'start'
    assert start.runtime_mode == 'shadow'
    assert start.hardware == 'auto'
    assert start.confirm_motion is False
    assert start.domain == 20
    assert doctor.runtime_mode == 'shadow'
    assert doctor.hardware == 'external'
    for command in ('status', 'stop'):
        parsed = human_following_cli.build_parser().parse_args([command])
        assert parsed.command == command


def test_active_requires_explicit_confirmation():
    args = human_following_cli.build_parser().parse_args([
        'start', '--runtime-mode', 'active',
    ])

    with pytest.raises(ValueError, match='confirm-motion'):
        human_following_cli.validate_motion_request(args)


def test_shadow_rejects_motion_confirmation():
    args = human_following_cli.build_parser().parse_args([
        'start', '--confirm-motion',
    ])

    with pytest.raises(ValueError, match='active'):
        human_following_cli.validate_motion_request(args)


def test_parser_rejects_ros_domain_other_than_20():
    with pytest.raises(SystemExit):
        human_following_cli.build_parser().parse_args([
            '--domain', '19', 'start',
        ])


@pytest.mark.parametrize('runtime_mode,confirmed,profile', [
    ('shadow', False, 'human_following_shadow.yaml'),
    ('active', True, 'human_following_supervised_test.yaml'),
])
def test_launch_vector_is_exact_and_selects_runtime_profile(
        tmp_path, runtime_mode, confirmed, profile):
    argv = ['start', '--runtime-mode', runtime_mode,
            '--workspace-root', str(tmp_path)]
    if confirmed:
        argv.append('--confirm-motion')
    args = human_following_cli.build_parser().parse_args(argv)
    selection = HardwareSelection(True, False, True, False)

    command = human_following_cli.build_launch_argv(args, selection)

    assert command == [
        'ros2', 'launch', 'track_robot_bringup',
        'human_following_live.launch.py',
        'runtime_mode:={}'.format(runtime_mode),
        'motion_confirmed:={}'.format(str(confirmed).lower()),
        'start_camera:=true',
        'start_lidar:=false',
        'start_base:=true',
        'start_imu:=false',
        'profile_config:={}'.format(
            tmp_path / 'src' / 'track_robot' / 'track_robot_bringup'
            / 'config' / profile),
    ]


def test_default_state_path_is_independent_from_semantic_search(tmp_path):
    path = human_following_cli.default_state_path({
        'ROS_HOME': str(tmp_path),
    })

    assert path == (
        tmp_path / 'track_robot_human_following' / 'managed_process.json')
    assert 'semantic_search' not in str(path)


def test_managed_spawn_reuses_sensor_hardware_selection_and_domain20(
        monkeypatch, tmp_path):
    args = human_following_cli.build_parser().parse_args([
        'start', '--hardware', 'auto', '--workspace-root', str(tmp_path),
    ])
    selection_calls = []
    selection = HardwareSelection(False, True, False, True)

    def select(stage, mode, **kwargs):
        selection_calls.append((stage, mode, kwargs))
        return selection

    monkeypatch.setattr(human_following_cli, 'select_hardware', select)

    class Manager:
        def __init__(self):
            self.spawn = None

        def verified_state(self):
            return None

        def spawn_verified(self, command, **kwargs):
            self.spawn = (command, kwargs)
            return SimpleNamespace(pid=31), SimpleNamespace(stage='shadow')

    manager = Manager()
    child, state = human_following_cli._spawn(
        args,
        manager,
        lambda *a, **k: None,
        {'ROS_DOMAIN_ID': '20'},
        deadline=10.0,
        monotonic=lambda: 1.0,
    )

    assert child.pid == 31
    assert state.stage == 'shadow'
    assert selection_calls[0][:2] == ('sensors', 'auto')
    assert selection_calls[0][2]['environment']['ROS_DOMAIN_ID'] == '20'
    assert manager.spawn[1] == {
        'stage': 'shadow',
        'owned_modules': ('lidar', 'imu'),
        'environment': {'ROS_DOMAIN_ID': '20'},
    }
    assert manager.spawn[0] == human_following_cli.build_launch_argv(
        args, selection)


def test_ctl_script_and_cmake_registration_are_exact():
    package_root = Path(__file__).resolve().parents[1]
    script = package_root / 'scripts' / 'human_following_ctl'

    assert script.read_text(encoding='utf-8') == (
        '#!/usr/bin/env python3\n'
        'from track_robot_bringup.human_following_cli import main\n'
        '\n'
        'raise SystemExit(main())\n'
    )
    cmake = (package_root / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'scripts/human_following_ctl' in cmake
    assert 'test/test_human_following_cli.py' in cmake
    assert 'test/test_human_following_readiness.py' in cmake


def _zero_twist():
    return (
        'linear:\n'
        '  x: 0.0\n'
        '  y: 0.0\n'
        '  z: 0.0\n'
        'angular:\n'
        '  x: 0.0\n'
        '  y: 0.0\n'
        '  z: 0.0\n'
        '---\n'
    )


def test_zero_observation_uses_latest_complete_twist():
    nonzero = _zero_twist().replace('x: 0.0', 'x: 0.2', 1)

    def runner(argv, **kwargs):
        return subprocess.CompletedProcess(
            argv, 0, _zero_twist() + nonzero, '')

    assert human_following_cli.observe_zero_twist(
        '/follow/cmd_vel_safe',
        runner,
        {'ROS_DOMAIN_ID': '20'},
        timeout=1.0,
    ) is False


class _StopManager:
    def __init__(self, timeline, stopped=True):
        self.timeline = timeline
        self.stopped = stopped
        self.last_stop_error = None

    def stop_owned(self):
        self.timeline.append(('stop_owned',))
        return self.stopped


def test_active_stop_orders_disarm_zero_gate_shutdown_then_process_signal():
    timeline = []

    def runner(argv, **kwargs):
        timeline.append(('run', tuple(argv), kwargs['timeout']))
        if argv[:3] == ['ros2', 'service', 'call']:
            return subprocess.CompletedProcess(argv, 0, 'success: true\n', '')
        if argv[:3] == ['ros2', 'topic', 'echo']:
            return subprocess.CompletedProcess(argv, 0, _zero_twist(), '')
        if argv[:4] == ['ros2', 'topic', 'info', '/cmd_vel']:
            return subprocess.CompletedProcess(
                argv, 0, 'Publisher count: 0\n', '')
        raise AssertionError(argv)

    manager = _StopManager(timeline)
    state = SimpleNamespace(stage='active', owned_modules=('camera', 'base'))

    stopped = human_following_cli.ordered_stop(
        manager, state, runner, {'ROS_DOMAIN_ID': '20'})

    assert stopped is True
    commands = [list(event[1]) for event in timeline if event[0] == 'run']
    assert commands == [
        ['ros2', 'service', 'call', '/safety/disarm',
         'std_srvs/srv/Trigger', '{}'],
        ['ros2', 'topic', 'echo', '/follow/cmd_vel_safe',
         '--qos-profile', 'default', '--no-arr', '--no-str'],
        ['ros2', 'service', 'call', '/cmd_vel_gate/shutdown',
         'std_srvs/srv/Trigger', '{}'],
        ['ros2', 'topic', 'info', '/cmd_vel'],
    ]
    assert [event[2] for event in timeline if event[0] == 'run'] == [
        2.0, 1.0, 2.0, 2.0,
    ]
    assert timeline[-1] == ('stop_owned',)


def test_stop_emergency_stops_after_disarm_or_zero_failure_before_gate():
    timeline = []

    def runner(argv, **kwargs):
        timeline.append(tuple(argv))
        if '/safety/disarm' in argv:
            return subprocess.CompletedProcess(argv, 0, 'success: false\n', '')
        if argv[:3] == ['ros2', 'topic', 'echo']:
            nonzero = _zero_twist().replace('x: 0.0', 'x: 0.1', 1)
            return subprocess.CompletedProcess(argv, 0, nonzero, '')
        if ('/safety/emergency_stop' in argv
                or '/cmd_vel_gate/shutdown' in argv):
            return subprocess.CompletedProcess(argv, 0, 'success: true\n', '')
        if argv[:4] == ['ros2', 'topic', 'info', '/cmd_vel']:
            return subprocess.CompletedProcess(
                argv, 0, 'Publisher count: 0\n', '')
        raise AssertionError(argv)

    manager = _StopManager(timeline)

    assert human_following_cli.ordered_stop(
        manager,
        SimpleNamespace(stage='active', owned_modules=()),
        runner,
        {'ROS_DOMAIN_ID': '20'},
    ) is True
    assert timeline.index((
        'ros2', 'service', 'call', '/safety/emergency_stop',
        'std_srvs/srv/Trigger', '{}')) < timeline.index((
            'ros2', 'service', 'call', '/cmd_vel_gate/shutdown',
            'std_srvs/srv/Trigger', '{}'))


def test_stop_never_signals_while_cmd_vel_still_has_a_publisher():
    timeline = []

    def runner(argv, **kwargs):
        timeline.append(tuple(argv))
        if argv[:3] == ['ros2', 'topic', 'echo']:
            return subprocess.CompletedProcess(argv, 0, _zero_twist(), '')
        if '/cmd_vel_gate/shutdown' in argv:
            return subprocess.CompletedProcess(argv, 0, 'success: false\n', '')
        if '/safety/emergency_stop' in argv or '/safety/disarm' in argv:
            return subprocess.CompletedProcess(argv, 0, 'success: true\n', '')
        if argv[:4] == ['ros2', 'topic', 'info', '/cmd_vel']:
            return subprocess.CompletedProcess(
                argv, 0, 'Publisher count: 1\n', '')
        raise AssertionError(argv)

    manager = _StopManager(timeline)

    assert human_following_cli.ordered_stop(
        manager,
        SimpleNamespace(stage='active', owned_modules=()),
        runner,
        {'ROS_DOMAIN_ID': '20'},
    ) is False
    assert ('stop_owned',) not in timeline
    assert timeline.count((
        'ros2', 'service', 'call', '/safety/emergency_stop',
        'std_srvs/srv/Trigger', '{}')) == 1


def test_stop_with_unverified_state_calls_no_ros_service_and_sends_no_signal():
    class Manager:
        def verified_state(self):
            return None

        def read_state(self):
            return SimpleNamespace(pid=123)

        def stop_owned(self):
            raise AssertionError('unverified process must not be signalled')

    stdout = io.StringIO()
    code = human_following_cli.main(
        ['stop'],
        manager=Manager(),
        runner=lambda *a, **k: (_ for _ in ()).throw(
            AssertionError(
                'unverified process must not trigger ROS activity')),
        stdout=stdout,
        environ={'PATH': '/bin'},
    )

    assert code == 2
    assert 'stale or unverified' in stdout.getvalue()
