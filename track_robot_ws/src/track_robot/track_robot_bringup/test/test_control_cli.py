import io
import subprocess
from contextlib import contextmanager
from types import SimpleNamespace

import pytest

from track_robot_bringup import control_cli
from track_robot_bringup.control_config import HardwareSelection
from track_robot_bringup.readiness import CheckResult, CheckStatus, ReadinessReport


@pytest.mark.parametrize('command', [
    ['doctor', 'sensors'],
    ['doctor', 'phase1'],
    ['doctor', 'phase2'],
    ['start', 'phase1'],
    ['start', 'phase2'],
    ['status', 'phase1'],
    ['status', 'phase2'],
    ['query', 'blue chair'],
    ['visualize', 'phase1'],
    ['visualize', 'phase2'],
    ['test', 'phase1', 'blue chair'],
    ['run', 'phase4b'],
    ['run', 'phase5a'],
    ['run', 'phase5a', '--search-shadow'],
    ['run', 'phase5a', '--rotation-supervised'],
    ['stop'],
])
def test_parser_supports_every_command_and_fixed_domain(command):
    args = control_cli.build_parser().parse_args(command)

    assert args.command == command[0]
    assert args.domain == 20


def test_parser_supports_control_options():
    args = control_cli.build_parser().parse_args([
        'test', 'phase2', 'blue chair',
        '--hardware', 'external',
        '--allow-degraded',
        '--extrinsic-mode', 'prototype',
        '--start-stack',
    ])

    assert args.hardware == 'external'
    assert args.allow_degraded is True
    assert args.extrinsic_mode == 'prototype'
    assert args.start_stack is True
    assert args.readiness_timeout == 30.0
    assert args.probe_timeout == 3.0


def test_phase4b_run_builds_the_single_supervised_command_without_imu(tmp_path):
    args = control_cli.build_parser().parse_args([
        'run', 'phase4b',
        '--workspace-root', str(tmp_path),
        '--extrinsic-mode', 'prototype',
    ])

    command = control_cli.build_phase4b_launch_argv(
        args, control_cli.default_workspace_paths(tmp_path))

    assert command[:4] == [
        'ros2', 'launch', 'track_robot_bringup',
        'semantic_search_phase4b.launch.py',
    ]
    assert 'runtime_mode:=SEMANTIC_ACTIVE' in command
    assert 'enable_semantic_execution:=true' in command
    assert 'start_base:=true' in command
    assert 'start_phase4b_rviz:=true' in command
    assert 'configure_network:=false' in command
    assert 'start_rviz:=true' not in command
    assert 'extrinsic_mode:=prototype' in command
    assert 'dino_enabled:=true' in command
    assert 'physical_recovery_enabled:=false' in command
    assert not any('imu' in value.lower() for value in command)


def test_phase4b_physical_recovery_requires_an_explicit_cli_flag(tmp_path):
    args = control_cli.build_parser().parse_args([
        'run', 'phase4b', '--physical-recovery',
        '--workspace-root', str(tmp_path),
    ])

    command = control_cli.build_phase4b_launch_argv(
        args, control_cli.default_workspace_paths(tmp_path))

    assert 'physical_recovery_enabled:=true' in command


def test_phase4b_run_allows_explicit_dino_fallback(tmp_path):
    args = control_cli.build_parser().parse_args([
        'run', 'phase4b',
        '--workspace-root', str(tmp_path),
        '--no-dino',
    ])

    command = control_cli.build_phase4b_launch_argv(
        args, control_cli.default_workspace_paths(tmp_path))

    assert 'dino_enabled:=false' in command


def test_phase5a_run_is_passive_and_stationary_by_default(tmp_path):
    args = control_cli.build_parser().parse_args([
        'run', 'phase5a', '--workspace-root', str(tmp_path),
    ])

    command = control_cli.build_phase5a_launch_argv(
        args, control_cli.default_workspace_paths(tmp_path))

    assert command[:4] == [
        'ros2', 'launch', 'track_robot_bringup',
        'semantic_search_phase5a.launch.py',
    ]
    assert 'search_mode:=PASSIVE_ONLY' in command
    assert 'rotation_runtime_mode:=PLANNING_ONLY' in command
    assert 'enable_rotation_execution:=false' in command
    assert 'start_base:=false' in command
    assert 'start_phase5a_rviz:=true' in command
    assert 'configure_network:=false' in command
    assert 'physical_recovery_enabled:=false' in command


def test_phase5a_shadow_is_stationary_and_supervised_rotation_is_explicit(
        tmp_path):
    shadow = control_cli.build_parser().parse_args([
        'run', 'phase5a', '--search-shadow',
        '--workspace-root', str(tmp_path),
    ])
    active = control_cli.build_parser().parse_args([
        'run', 'phase5a', '--rotation-supervised',
        '--workspace-root', str(tmp_path),
    ])

    shadow_command = control_cli.build_phase5a_launch_argv(
        shadow, control_cli.default_workspace_paths(tmp_path))
    active_command = control_cli.build_phase5a_launch_argv(
        active, control_cli.default_workspace_paths(tmp_path))

    assert 'search_mode:=SEARCH_SHADOW' in shadow_command
    assert 'enable_rotation_execution:=false' in shadow_command
    assert 'start_base:=false' in shadow_command
    assert 'search_mode:=ROTATION_SUPERVISED' in active_command
    assert 'rotation_runtime_mode:=SEMANTIC_ACTIVE' in active_command
    assert 'enable_rotation_execution:=true' in active_command
    assert 'start_base:=true' in active_command


def test_phase5a_physical_recovery_is_opt_in_and_does_not_enable_rotation(
        tmp_path):
    passive = control_cli.build_parser().parse_args([
        'run', 'phase5a', '--physical-recovery',
        '--workspace-root', str(tmp_path),
    ])
    active = control_cli.build_parser().parse_args([
        'run', 'phase5a', '--rotation-supervised', '--physical-recovery',
        '--workspace-root', str(tmp_path),
    ])

    passive_command = control_cli.build_phase5a_launch_argv(
        passive, control_cli.default_workspace_paths(tmp_path))
    active_command = control_cli.build_phase5a_launch_argv(
        active, control_cli.default_workspace_paths(tmp_path))

    assert 'physical_recovery_enabled:=true' in passive_command
    assert 'rotation_runtime_mode:=PLANNING_ONLY' in passive_command
    assert 'enable_rotation_execution:=false' in passive_command
    assert 'start_base:=false' in passive_command
    assert 'physical_recovery_enabled:=true' in active_command


def test_phase4b_shutdown_requests_cancel_and_disarm_in_domain20(tmp_path):
    calls = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        return subprocess.CompletedProcess(argv, 0, 'success: true', '')

    result = control_cli.request_phase4b_cancel_disarm(
        runner,
        {'PATH': '/bin', 'ROS_DOMAIN_ID': '20'},
        timeout=2.0,
    )

    assert result is True
    assert calls[0][0] == [
        'ros2', 'service', 'call',
        '/semantic_navigation/cancel_and_disarm',
        'std_srvs/srv/Trigger', '{}',
    ]
    assert calls[0][1]['env']['ROS_DOMAIN_ID'] == '20'
    assert calls[0][1]['timeout'] == 2.0
    assert calls[0][1]['shell'] is False


def test_parser_rejects_a_domain_other_than_20():
    parser = control_cli.build_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(['--domain', '19', 'status', 'phase1'])


def test_parser_rejects_live_test_for_sensor_only_stage():
    with pytest.raises(SystemExit):
        control_cli.build_parser().parse_args([
            'test', 'sensors', 'blue chair',
        ])


def test_auto_selection_lists_graph_once_and_reuses_any_existing_publisher():
    calls = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(
                argv, 0,
                '/zed/zed_node/left/image_rect_color [sensor_msgs/msg/Image]\n'
                '/rslidar_points [sensor_msgs/msg/PointCloud2]\n',
                '')
        if argv[-1] == '/zed/zed_node/left/image_rect_color':
            return subprocess.CompletedProcess(argv, 0, 'Publisher count: 1\n', '')
        if argv[-1] == '/rslidar_points':
            return subprocess.CompletedProcess(argv, 0, 'Publisher count: 1\n', '')
        raise AssertionError(argv)

    selection = control_cli.select_hardware(
        'phase2',
        'auto',
        runner=runner,
        environment={'ROS_DOMAIN_ID': '20'},
        timeout=0.2,
    )

    assert selection == HardwareSelection(
        camera=False, lidar=False, base=True, imu=True)
    assert calls[0][0] == ['ros2', 'topic', 'list', '-t']
    assert sum(argv[:3] == ['ros2', 'topic', 'list']
               for argv, _ in calls) == 1
    assert all(argv[:3] != ['ros2', 'topic', 'hz'] for argv, _ in calls)
    assert all(kwargs['shell'] is False for _, kwargs in calls)
    assert all(kwargs['timeout'] == 0.2 for _, kwargs in calls)


def test_auto_reuses_stale_publisher_without_running_frequency_probe():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        if argv[:3] == ['ros2', 'topic', 'list']:
            return subprocess.CompletedProcess(
                argv, 0,
                '/zed/zed_node/left/image_rect_color [sensor_msgs/msg/Image]\n',
                '')
        return subprocess.CompletedProcess(argv, 0, 'Publisher count: 1\n', '')

    selection = control_cli.select_hardware(
        'phase1', 'auto', runner=runner,
        environment={'ROS_DOMAIN_ID': '20'}, timeout=0.2)

    assert selection.camera is False
    assert not any(argv[:3] == ['ros2', 'topic', 'hz'] for argv in calls)


def test_external_selection_starts_no_hardware_and_runs_no_probe():
    def forbidden_runner(*args, **kwargs):
        raise AssertionError('external mode must not probe for launch selection')

    selection = control_cli.select_hardware(
        'phase2', 'external', runner=forbidden_runner,
        environment={'ROS_DOMAIN_ID': '20'})

    assert selection == HardwareSelection(False, False, False, False)


def test_launch_argv_has_explicit_module_booleans():
    args = control_cli.build_parser().parse_args([
        'start', 'phase1', '--extrinsic-mode', 'none'])
    selection = HardwareSelection(True, False, False, False)

    argv = control_cli.build_launch_argv(args, selection, {
        'runtime_path': '/workspace/runtime',
        'checkpoint_path': '/workspace/checkpoint.pt',
    })

    assert argv[:5] == [
        'ros2', 'launch', 'track_robot_bringup',
        'semantic_search_live.launch.py', 'stage:=phase1']
    assert 'start_camera:=true' in argv
    assert 'start_lidar:=false' in argv
    assert 'start_base:=false' in argv
    assert 'start_imu:=false' in argv
    assert 'extrinsic_mode:=none' in argv


def test_doctor_is_passive_and_maps_not_ready_exit_code(monkeypatch, tmp_path):
    popen_calls = []
    report = ReadinessReport([
        CheckResult.not_ready('camera', 'topic absent'),
    ], stage='phase1')
    monkeypatch.setattr(control_cli, '_readiness_report', lambda *a, **k: report)

    stdout = io.StringIO()
    code = control_cli.main(
        ['doctor', 'phase1', '--workspace-root', str(tmp_path)],
        popen_factory=lambda *a, **k: popen_calls.append((a, k)),
        stdout=stdout,
        environ={'PATH': '/bin'},
    )

    assert code == 2
    assert 'Overall: NOT READY' in stdout.getvalue()
    assert popen_calls == []


def test_start_refuses_to_overwrite_a_live_managed_process(
        monkeypatch, tmp_path):
    monkeypatch.setattr(
        control_cli,
        '_static_report',
        lambda *a, **k: ReadinessReport([], stage='phase1'),
    )

    class Manager:
        def verified_state(self):
            return SimpleNamespace(pid=42, pgid=42, stage='phase1')

        def spawn(self, *args, **kwargs):
            raise AssertionError('must not replace live ownership state')

    stderr = io.StringIO()
    code = control_cli.main(
        ['start', 'phase1', '--workspace-root', str(tmp_path)],
        manager=Manager(),
        runner=lambda *a, **k: (_ for _ in ()).throw(
            AssertionError('must not inspect or launch over managed process')),
        stderr=stderr,
        environ={'PATH': '/bin'},
    )

    assert code == 4
    assert 'already manages pid 42' in stderr.getvalue()


def test_start_readiness_uses_one_shared_deadline(monkeypatch):
    clock = {'now': 0.0}
    subprocess_timeouts = []
    not_ready = ReadinessReport([
        CheckResult.not_ready('camera', 'not publishing'),
    ], stage='phase1')

    def runner(argv, **kwargs):
        timeout = kwargs['timeout']
        subprocess_timeouts.append(timeout)
        clock['now'] += timeout
        return subprocess.CompletedProcess(argv, 1, '', 'timed out')

    def readiness(
            stage, selection, paths, environment, bounded_runner, timeout):
        for _ in range(2):
            try:
                bounded_runner(
                    ['ros2', 'topic', 'hz', '/camera'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=timeout,
                    env=environment,
                    shell=False,
                )
            except subprocess.TimeoutExpired:
                pass
        return not_ready

    monkeypatch.setattr(control_cli, '_readiness_report', readiness)

    class Child:
        returncode = None

        def poll(self):
            return None

    args = SimpleNamespace(
        stage='phase1', readiness_timeout=0.5, probe_timeout=1.0)
    result = control_cli._wait_for_readiness(
        args,
        Child(),
        HardwareSelection(True, False, False, False),
        {},
        {'ROS_DOMAIN_ID': '20'},
        runner,
        lambda: clock['now'],
        lambda duration: clock.__setitem__('now', clock['now'] + duration),
    )

    assert result.overall is CheckStatus.NOT_READY
    assert sum(subprocess_timeouts) <= 0.5
    assert clock['now'] <= 0.5


def test_start_phase2_auto_selection_consumes_one_total_deadline(
        monkeypatch, tmp_path):
    clock = {'now': 0.0}
    timeouts = []
    spawn_calls = []
    all_topics = '\n'.join(
        '{} [test/msg/Type]'.format(topic)
        for topic in control_cli._MODULE_TOPICS.values())

    monkeypatch.setattr(
        control_cli,
        '_static_report',
        lambda *a, **k: ReadinessReport([], stage='phase2'),
    )

    def runner(argv, **kwargs):
        timeout = kwargs['timeout']
        timeouts.append(timeout)
        clock['now'] += timeout
        if argv == ['ros2', 'topic', 'list', '-t']:
            return subprocess.CompletedProcess(argv, 0, all_topics, '')
        return subprocess.CompletedProcess(argv, 0, 'Publisher count: 1\n', '')

    class Manager:
        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            spawn_calls.append((args, kwargs))
            raise RuntimeError('must not spawn after deadline')

    stderr = io.StringIO()
    code = control_cli.main(
        [
            'start', 'phase2',
            '--hardware', 'auto',
            '--readiness-timeout', '1.0',
            '--probe-timeout', '0.4',
            '--workspace-root', str(tmp_path),
        ],
        manager=Manager(),
        runner=runner,
        stderr=stderr,
        environ={'PATH': '/bin'},
        monotonic=lambda: clock['now'],
    )

    assert code == 4
    assert sum(timeouts) <= 1.0
    assert clock['now'] <= 1.0
    assert spawn_calls == []
    assert 'deadline' in stderr.getvalue()


def test_start_checks_deadline_after_static_preflight(monkeypatch, tmp_path):
    clock = {'now': 0.0}
    spawn_calls = []

    def static_report(*args, **kwargs):
        clock['now'] = 0.6
        return ReadinessReport([], stage='phase1')

    monkeypatch.setattr(control_cli, '_static_report', static_report)

    class Manager:
        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            spawn_calls.append((args, kwargs))
            raise RuntimeError('must not spawn after deadline')

    stderr = io.StringIO()
    code = control_cli.main(
        [
            'start', 'phase1',
            '--hardware', 'external',
            '--readiness-timeout', '0.5',
            '--workspace-root', str(tmp_path),
        ],
        manager=Manager(),
        stderr=stderr,
        environ={'PATH': '/bin'},
        monotonic=lambda: clock['now'],
    )

    assert code == 4
    assert spawn_calls == []
    assert 'deadline' in stderr.getvalue()


def test_start_interrupt_during_readiness_stops_owned_group(
        monkeypatch, tmp_path):
    monkeypatch.setattr(
        control_cli,
        '_static_report',
        lambda *a, **k: ReadinessReport([], stage='phase1'),
    )
    monkeypatch.setattr(
        control_cli,
        '_wait_for_readiness',
        lambda *a, **k: (_ for _ in ()).throw(KeyboardInterrupt()),
    )

    class Child:
        pid = 42

        def poll(self):
            return None

    class Manager:
        def __init__(self):
            self.stop_calls = 0
            self.state = SimpleNamespace(pid=42)

        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            return Child()

        def read_state(self):
            return self.state

        def stop_owned(self):
            self.stop_calls += 1
            return True

    manager = Manager()
    code = control_cli.main(
        [
            'start', 'phase1',
            '--hardware', 'external',
            '--workspace-root', str(tmp_path),
        ],
        manager=manager,
        environ={'PATH': '/bin'},
    )

    assert code == 130
    assert manager.stop_calls == 1


def test_start_returns_degraded_after_clean_foreground_exit(
        monkeypatch, tmp_path):
    degraded = ReadinessReport([
        CheckResult.degraded('calibration', 'prototype'),
    ], stage='phase1')
    monkeypatch.setattr(
        control_cli,
        '_static_report',
        lambda *a, **k: ReadinessReport([], stage='phase1'),
    )
    monkeypatch.setattr(
        control_cli, '_wait_for_readiness', lambda *a, **k: degraded)

    class Child:
        pid = 42
        returncode = 0

        def poll(self):
            return self.returncode

        def wait(self):
            return self.returncode

    class Manager:
        def __init__(self):
            self.state = SimpleNamespace(pid=42)
            self.cleared = []

        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            return Child()

        def read_state(self):
            return self.state

        def clear_if_owned(self, state):
            self.cleared.append(state)

    manager = Manager()
    code = control_cli.main(
        [
            'start', 'phase1',
            '--hardware', 'external',
            '--workspace-root', str(tmp_path),
        ],
        manager=manager,
        environ={'PATH': '/bin'},
    )

    assert code == 3
    assert manager.cleared == [manager.state]


def test_start_interrupt_still_precedes_degraded_exit(
        monkeypatch, tmp_path):
    degraded = ReadinessReport([
        CheckResult.degraded('calibration', 'prototype'),
    ], stage='phase1')
    forward_calls = []
    monkeypatch.setattr(
        control_cli,
        '_static_report',
        lambda *a, **k: ReadinessReport([], stage='phase1'),
    )
    monkeypatch.setattr(
        control_cli, '_wait_for_readiness', lambda *a, **k: degraded)

    @contextmanager
    def forward(_manager):
        forward_calls.append(None)
        yield {'value': len(forward_calls) == 2}

    monkeypatch.setattr(control_cli, '_forward_signals', forward)

    class Child:
        pid = 42
        returncode = 0

        def poll(self):
            return self.returncode

        def wait(self):
            return self.returncode

    class Manager:
        state = SimpleNamespace(pid=42)

        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            return Child()

        def read_state(self):
            return self.state

        def clear_if_owned(self, state):
            pass

    code = control_cli.main(
        [
            'start', 'phase1',
            '--hardware', 'external',
            '--workspace-root', str(tmp_path),
        ],
        manager=Manager(),
        environ={'PATH': '/bin'},
    )

    assert code == 130
    assert len(forward_calls) == 2


def test_partial_signal_handler_install_restores_already_replaced_handler(
        monkeypatch):
    original = {
        control_cli.signal.SIGINT: object(),
        control_cli.signal.SIGTERM: object(),
    }
    current = dict(original)
    calls = []
    failed_once = {'value': False}

    def getsignal(signum):
        return original[signum]

    def install(signum, handler):
        calls.append((signum, handler))
        if (
                signum == control_cli.signal.SIGTERM
                and callable(handler)
                and not failed_once['value']):
            failed_once['value'] = True
            raise ValueError('not in main thread')
        current[signum] = handler

    monkeypatch.setattr(control_cli.signal, 'getsignal', getsignal)
    monkeypatch.setattr(control_cli.signal, 'signal', install)

    class Manager:
        def stop_owned(self):
            raise AssertionError('no signal should be forwarded')

    with control_cli._forward_signals(Manager()):
        pass

    assert current == original
    assert calls[-2:] == [
        (control_cli.signal.SIGINT, original[control_cli.signal.SIGINT]),
        (control_cli.signal.SIGTERM, original[control_cli.signal.SIGTERM]),
    ]


def test_status_reports_managed_identity_without_starting(monkeypatch, tmp_path):
    report = ReadinessReport([
        CheckResult.pass_('camera', '15 Hz'),
    ], stage='phase1')
    monkeypatch.setattr(control_cli, '_readiness_report', lambda *a, **k: report)

    class Manager:
        def verified_state(self):
            return None

    stdout = io.StringIO()
    code = control_cli.main(
        ['status', 'phase1', '--workspace-root', str(tmp_path)],
        manager=Manager(),
        popen_factory=lambda *a, **k: (_ for _ in ()).throw(
            AssertionError('status must not start')),
        stdout=stdout,
        environ={'PATH': '/bin'},
    )

    assert code == 0
    assert 'Managed process: none' in stdout.getvalue()


def test_query_execs_existing_portal_with_managed_domain():
    calls = []

    class OsApi:
        def execvpe(self, executable, argv, environment):
            calls.append((executable, argv, environment))
            return 17

    code = control_cli.main(
        ['query', 'blue chair'],
        os_api=OsApi(),
        environ={
            'PATH': '/bin',
            'ROS_DOMAIN_ID': '99',
            'ROS_LOCALHOST_ONLY': '0',
            'FASTRTPS_DEFAULT_PROFILES_FILE': '/stale/remote-profile.xml',
        },
    )

    assert code == 17
    assert calls[0][0:2] == (
        'ros2',
        [
            'ros2', 'run', 'track_robot_semantic_search',
            'semantic_search_query', 'blue chair',
        ],
    )
    assert calls[0][2]['PATH'] == '/bin'
    assert calls[0][2]['ROS_DOMAIN_ID'] == '20'
    assert calls[0][2]['ROS_LOCALHOST_ONLY'] == '1'
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in calls[0][2]


def test_visualize_execs_foreground_launch_with_managed_domain():
    calls = []

    class OsApi:
        def execvpe(self, executable, argv, environment):
            calls.append((executable, argv, environment))
            return 23

    code = control_cli.main(
        ['visualize', 'phase2'],
        os_api=OsApi(),
        environ={
            'PATH': '/bin',
            'ROS_DOMAIN_ID': '99',
            'ROS_LOCALHOST_ONLY': '0',
            'FASTRTPS_DEFAULT_PROFILES_FILE': '/stale/remote-profile.xml',
        },
    )

    assert code == 23
    assert calls[0][0:2] == (
        'ros2',
        [
            'ros2', 'launch', 'track_robot_bringup',
            'semantic_search_visualization.launch.py',
            'stage:=phase2',
        ],
    )
    assert calls[0][2]['ROS_DOMAIN_ID'] == '20'
    assert calls[0][2]['ROS_LOCALHOST_ONLY'] == '1'
    assert calls[0][2]['PATH'] == '/bin'
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in calls[0][2]


def test_visualize_rejects_unknown_stage():
    with pytest.raises(SystemExit):
        control_cli.build_parser().parse_args(['visualize', 'sensors'])


def test_test_command_fails_clearly_until_task5_exists(monkeypatch):
    monkeypatch.setattr(control_cli, '_load_live_test', lambda: None)
    stderr = io.StringIO()

    code = control_cli.main(
        ['test', 'phase1', 'blue chair'],
        stderr=stderr,
        environ={'PATH': '/bin'},
    )

    assert code == 4
    assert 'Task 5 live-test support is not installed' in stderr.getvalue()


def test_test_checks_readiness_and_does_not_query_when_not_ready(
        monkeypatch, tmp_path):
    not_ready = ReadinessReport([
        CheckResult.not_ready('regions', 'topic absent'),
    ], stage='phase1')
    monkeypatch.setattr(
        control_cli, '_bounded_readiness_report',
        lambda *args, **kwargs: not_ready)

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            raise AssertionError('NOT READY must not submit a query')

    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)

    code = control_cli.main(
        [
            'test', 'phase1', 'blue chair',
            '--workspace-root', str(tmp_path),
        ],
        environ={'PATH': '/bin'},
    )

    assert code == 2


def test_test_reuses_ready_external_stack_without_process_ownership(
        monkeypatch, tmp_path):
    ready = ReadinessReport([], stage='phase1')
    monkeypatch.setattr(
        control_cli, '_bounded_readiness_report',
        lambda *args, **kwargs: ready)
    live_calls = []

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            live_calls.append((args, kwargs))
            return SimpleNamespace(
                exit_code=0,
                report_path=tmp_path / 'report.json',
                error='',
            )

    class Manager:
        def verified_state(self):
            raise AssertionError('a ready external stack needs no ownership')

        def stop_owned(self):
            raise AssertionError('a reused stack must not be stopped')

    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)
    stdout = io.StringIO()

    code = control_cli.main(
        [
            'test', 'phase1', 'blue chair', '--start-stack',
            '--workspace-root', str(tmp_path),
        ],
        manager=Manager(),
        stdout=stdout,
        environ={'PATH': '/bin'},
    )

    assert code == 0
    assert live_calls
    assert live_calls[0][1]['readiness_snapshot']['overall'] == 'PASS'
    assert str(tmp_path / 'report.json') in stdout.getvalue()


def test_test_can_run_degraded_but_returns_degraded_after_live_pass(
        monkeypatch, tmp_path):
    degraded = ReadinessReport([
        CheckResult.degraded('calibration', 'prototype'),
    ], stage='phase2')
    monkeypatch.setattr(
        control_cli, '_bounded_readiness_report',
        lambda *args, **kwargs: degraded)

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            assert kwargs['calibration_mode'] == 'prototype'
            return SimpleNamespace(exit_code=0, report_path=None, error='')

    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)

    code = control_cli.main(
        [
            'test', 'phase2', 'blue chair',
            '--allow-degraded', '--extrinsic-mode', 'prototype',
            '--workspace-root', str(tmp_path),
        ],
        environ={'PATH': '/bin'},
    )

    assert code == 3


def test_test_start_stack_stops_only_the_process_it_spawned(
        monkeypatch, tmp_path):
    reports = [
        ReadinessReport([
            CheckResult.not_ready('regions', 'topic absent'),
        ], stage='phase1'),
        ReadinessReport([], stage='phase1'),
    ]
    monkeypatch.setattr(
        control_cli, '_bounded_readiness_report',
        lambda *args, **kwargs: reports.pop(0))
    monkeypatch.setattr(
        control_cli, '_wait_for_readiness',
        lambda *args, **kwargs: reports.pop(0))
    monkeypatch.setattr(
        control_cli,
        'select_hardware',
        lambda *args, **kwargs: HardwareSelection(
            True, False, False, False),
    )

    class Child:
        pid = 42
        returncode = None

        def poll(self):
            return None

    class Manager:
        def __init__(self):
            self.spawn_calls = []
            self.stop_calls = 0
            self.state = SimpleNamespace(pid=42)

        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            self.spawn_calls.append((args, kwargs))
            return Child()

        def read_state(self):
            return self.state

        def stop_owned(self):
            self.stop_calls += 1
            return True

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            return SimpleNamespace(exit_code=0, report_path=None, error='')

    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)
    manager = Manager()

    code = control_cli.main(
        [
            'test', 'phase1', 'blue chair', '--start-stack',
            '--hardware', 'auto',
            '--workspace-root', str(tmp_path),
        ],
        manager=manager,
        environ={'PATH': '/bin'},
    )

    assert code == 0
    assert len(manager.spawn_calls) == 1
    assert manager.stop_calls == 1


def test_test_start_stack_gets_fresh_post_launch_readiness_budget(
        monkeypatch, tmp_path):
    clock = {'now': 0.0}
    deadlines = []
    not_ready = ReadinessReport([
        CheckResult.not_ready('regions', 'topic absent'),
    ], stage='phase1')
    ready = ReadinessReport([], stage='phase1')

    def initial_readiness(*args, **kwargs):
        deadlines.append(('initial', args[6]))
        clock['now'] = 29.0
        return not_ready

    class Child:
        returncode = None

        def poll(self):
            return None

    def spawn(*args, **kwargs):
        deadlines.append(('spawn', args[5]))
        return (
            Child(),
            SimpleNamespace(pid=42),
            HardwareSelection(True, False, False, False),
        )

    def wait(*args, **kwargs):
        deadlines.append(('wait', kwargs['deadline']))
        return ready

    class Manager:
        def __init__(self):
            self.stop_calls = 0

        def stop_owned(self):
            self.stop_calls += 1
            return True

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            return SimpleNamespace(exit_code=0, report_path=None, error='')

    monkeypatch.setattr(
        control_cli, '_bounded_readiness_report', initial_readiness)
    monkeypatch.setattr(control_cli, '_spawn_managed_stack', spawn)
    monkeypatch.setattr(control_cli, '_wait_for_readiness', wait)
    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)
    manager = Manager()

    code = control_cli.main(
        [
            'test', 'phase1', 'blue chair', '--start-stack',
            '--readiness-timeout', '30',
            '--workspace-root', str(tmp_path),
        ],
        manager=manager,
        monotonic=lambda: clock['now'],
        environ={'PATH': '/bin'},
    )

    assert code == 0
    assert deadlines == [
        ('initial', 30.0),
        ('spawn', 59.0),
        ('wait', 59.0),
    ]
    assert manager.stop_calls == 1


def test_test_interrupt_has_priority_and_cleans_up_owned_stack(
        monkeypatch, tmp_path):
    reports = [
        ReadinessReport([
            CheckResult.not_ready('regions', 'topic absent'),
        ], stage='phase1'),
        ReadinessReport([], stage='phase1'),
    ]
    monkeypatch.setattr(
        control_cli, '_bounded_readiness_report',
        lambda *args, **kwargs: reports.pop(0))
    monkeypatch.setattr(
        control_cli, '_wait_for_readiness',
        lambda *args, **kwargs: reports.pop(0))
    monkeypatch.setattr(
        control_cli,
        'select_hardware',
        lambda *args, **kwargs: HardwareSelection(
            False, False, False, False),
    )

    class Child:
        pid = 42
        returncode = None

        def poll(self):
            return None

    class Manager:
        state = SimpleNamespace(pid=42)

        def __init__(self):
            self.stop_calls = 0

        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            return Child()

        def read_state(self):
            return self.state

        def stop_owned(self):
            self.stop_calls += 1
            return True

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            raise KeyboardInterrupt

    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)
    manager = Manager()

    code = control_cli.main(
        [
            'test', 'phase1', 'blue chair', '--start-stack',
            '--hardware', 'external',
            '--workspace-root', str(tmp_path),
        ],
        manager=manager,
        environ={'PATH': '/bin'},
    )

    assert code == 130
    # The owning test lifecycle forwards once, then makes an idempotent
    # cleanup attempt from finally.
    assert manager.stop_calls == 2


def test_test_start_stack_signal_forwarding_covers_collector_and_final_cleanup(
        monkeypatch, tmp_path):
    reports = [
        ReadinessReport([
            CheckResult.not_ready('regions', 'topic absent'),
        ], stage='phase1'),
        ReadinessReport([], stage='phase1'),
    ]
    monkeypatch.setattr(
        control_cli, '_bounded_readiness_report',
        lambda *args, **kwargs: reports.pop(0))
    monkeypatch.setattr(
        control_cli, '_wait_for_readiness',
        lambda *args, **kwargs: reports.pop(0))
    monkeypatch.setattr(
        control_cli,
        'select_hardware',
        lambda *args, **kwargs: HardwareSelection(
            False, False, False, False),
    )

    class Child:
        pid = 42
        returncode = None

        def poll(self):
            return None

    class Manager:
        state = SimpleNamespace(pid=42)

        def __init__(self):
            self.stop_calls = 0
            self.forward_state = None

        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            assert self.forward_state is not None
            return Child()

        def read_state(self):
            return self.state

        def stop_owned(self):
            self.stop_calls += 1
            return True

    @contextmanager
    def forward(manager):
        interrupted = {'value': False}
        manager.forward_state = interrupted
        yield interrupted

    manager = Manager()

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            assert manager.forward_state is not None
            # Simulate the existing handler: mark interrupted and forward to
            # the owned process group. The command must still clean in finally.
            manager.forward_state['value'] = True
            manager.stop_owned()
            return SimpleNamespace(exit_code=0, report_path=None, error='')

    monkeypatch.setattr(control_cli, '_forward_signals', forward)
    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)

    code = control_cli.main(
        [
            'test', 'phase1', 'blue chair', '--start-stack',
            '--hardware', 'external',
            '--workspace-root', str(tmp_path),
        ],
        manager=manager,
        environ={'PATH': '/bin'},
    )

    assert code == 130
    assert manager.stop_calls == 2


def test_test_reused_external_stack_never_installs_owned_signal_forwarding(
        monkeypatch, tmp_path):
    monkeypatch.setattr(
        control_cli,
        '_bounded_readiness_report',
        lambda *args, **kwargs: ReadinessReport([], stage='phase1'),
    )
    monkeypatch.setattr(
        control_cli,
        '_forward_signals',
        lambda manager: (_ for _ in ()).throw(
            AssertionError('external stack must not receive owned signals')),
    )

    class LiveTest:
        @staticmethod
        def run_live_test(*args, **kwargs):
            return SimpleNamespace(exit_code=0, report_path=None, error='')

    monkeypatch.setattr(control_cli, '_load_live_test', lambda: LiveTest)

    assert control_cli.main(
        [
            'test', 'phase1', 'blue chair', '--start-stack',
            '--workspace-root', str(tmp_path),
        ],
        environ={'PATH': '/bin'},
    ) == 0


def test_spawn_helper_cleans_up_when_owned_state_cannot_be_read(
        monkeypatch):
    monkeypatch.setattr(
        control_cli,
        'select_hardware',
        lambda *args, **kwargs: HardwareSelection(
            False, False, False, False),
    )

    class Manager:
        def __init__(self):
            self.stop_calls = 0

        def verified_state(self):
            return None

        def spawn(self, *args, **kwargs):
            return SimpleNamespace(pid=42)

        def read_state(self):
            raise OSError('state vanished')

        def stop_owned(self):
            self.stop_calls += 1
            return True

    manager = Manager()
    args = control_cli.build_parser().parse_args([
        'test', 'phase1', 'target', '--hardware', 'external',
    ])

    with pytest.raises(RuntimeError, match='state vanished'):
        control_cli._spawn_managed_stack(
            args,
            manager,
            {
                'runtime_path': '/runtime',
                'checkpoint_path': '/checkpoint',
            },
            {'ROS_DOMAIN_ID': '20'},
            lambda *a, **k: None,
            1.0,
            lambda: 0.0,
        )

    assert manager.stop_calls == 1


def test_spawn_helper_cleans_captured_identity_when_state_returns_none(
        monkeypatch):
    monkeypatch.setattr(
        control_cli,
        'select_hardware',
        lambda *args, **kwargs: HardwareSelection(
            False, False, False, False),
    )
    captured = SimpleNamespace(pid=42)

    class Manager:
        def __init__(self):
            self.captured_calls = []

        def verified_state(self):
            return None

        def spawn_verified(self, *args, **kwargs):
            return SimpleNamespace(pid=42), captured

        def read_state(self):
            return None

        def stop_captured(self, state):
            self.captured_calls.append(state)
            return True

    manager = Manager()
    args = control_cli.build_parser().parse_args([
        'test', 'phase1', 'target', '--hardware', 'external',
    ])

    with pytest.raises(RuntimeError, match='ownership state.*unavailable'):
        control_cli._spawn_managed_stack(
            args,
            manager,
            {
                'runtime_path': '/runtime',
                'checkpoint_path': '/checkpoint',
            },
            {'ROS_DOMAIN_ID': '20'},
            lambda *a, **k: None,
            1.0,
            lambda: 0.0,
        )

    assert manager.captured_calls == [captured]


def test_spawn_helper_reports_orphan_risk_when_captured_cleanup_fails(
        monkeypatch):
    monkeypatch.setattr(
        control_cli,
        'select_hardware',
        lambda *args, **kwargs: HardwareSelection(
            False, False, False, False),
    )
    captured = SimpleNamespace(pid=42)

    class Manager:
        last_stop_error = 'captured identity changed before cleanup'

        def verified_state(self):
            return None

        def spawn_verified(self, *args, **kwargs):
            return SimpleNamespace(pid=42), captured

        def read_state(self):
            return None

        def stop_captured(self, state):
            assert state is captured
            return False

    args = control_cli.build_parser().parse_args([
        'test', 'phase1', 'target', '--hardware', 'external',
    ])

    with pytest.raises(RuntimeError) as raised:
        control_cli._spawn_managed_stack(
            args,
            Manager(),
            {
                'runtime_path': '/runtime',
                'checkpoint_path': '/checkpoint',
            },
            {'ROS_DOMAIN_ID': '20'},
            lambda *a, **k: None,
            1.0,
            lambda: 0.0,
        )

    assert 'orphan risk' in str(raised.value)
    assert 'captured identity changed before cleanup' in str(raised.value)


def test_spawn_helper_cleans_captured_identity_when_state_read_raises(
        monkeypatch):
    monkeypatch.setattr(
        control_cli,
        'select_hardware',
        lambda *args, **kwargs: HardwareSelection(
            False, False, False, False),
    )
    captured = SimpleNamespace(pid=42)

    class Manager:
        def __init__(self):
            self.captured_calls = []

        def verified_state(self):
            return None

        def spawn_verified(self, *args, **kwargs):
            return SimpleNamespace(pid=42), captured

        def read_state(self):
            raise OSError('state vanished')

        def stop_captured(self, state):
            self.captured_calls.append(state)
            return True

    manager = Manager()
    args = control_cli.build_parser().parse_args([
        'test', 'phase1', 'target', '--hardware', 'external',
    ])

    with pytest.raises(RuntimeError, match='state vanished'):
        control_cli._spawn_managed_stack(
            args,
            manager,
            {
                'runtime_path': '/runtime',
                'checkpoint_path': '/checkpoint',
            },
            {'ROS_DOMAIN_ID': '20'},
            lambda *a, **k: None,
            1.0,
            lambda: 0.0,
        )

    assert manager.captured_calls == [captured]


def test_stop_does_not_report_success_when_verified_process_survives():
    class Manager:
        last_stop_error = 'verified process group remains alive'

        def stop_owned(self):
            return False

    stdout = io.StringIO()
    stderr = io.StringIO()
    code = control_cli.main(
        ['stop'],
        manager=Manager(),
        stdout=stdout,
        stderr=stderr,
    )

    assert code == 4
    assert 'Stopped verified managed process group' not in stdout.getvalue()
    assert 'remains alive' in stderr.getvalue()


def test_phase5a_passive_stop_does_not_wait_for_absent_motion_services():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 0, 'success: true', '')

    class Manager:
        last_stop_error = None

        def verified_state(self):
            return SimpleNamespace(
                stage='phase5a', owned_modules=('camera', 'lidar'))

        def stop_owned(self):
            return True

    code = control_cli.main(
        ['stop'], manager=Manager(), runner=runner,
        environ={'PATH': '/bin'},
    )

    assert code == 0
    assert calls == []


def test_phase5a_active_stop_cancels_and_disarms_before_process_stop():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 0, 'success: true', '')

    class Manager:
        last_stop_error = None

        def verified_state(self):
            return SimpleNamespace(
                stage='phase5a',
                owned_modules=('camera', 'lidar', 'base'))

        def stop_owned(self):
            return True

    code = control_cli.main(
        ['stop'], manager=Manager(), runner=runner,
        environ={'PATH': '/bin'},
    )

    assert code == 0
    assert [call[3] for call in calls] == [
        '/semantic_search/active_search/cancel', '/safety/disarm']


@pytest.mark.parametrize('status, expected', [
    (CheckStatus.PASS, 0),
    (CheckStatus.NOT_READY, 2),
    (CheckStatus.DEGRADED, 3),
    (CheckStatus.FAIL, 4),
])
def test_readiness_exit_codes(status, expected):
    assert control_cli.exit_code(status) == expected
