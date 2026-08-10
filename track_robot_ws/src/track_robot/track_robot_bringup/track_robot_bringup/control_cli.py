"""Beginner-facing control for passive stages and supervised Phase 4B."""

import argparse
import importlib
import os
import re
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path

from .control_config import (
    HardwareSelection,
    default_workspace_paths,
    managed_environment,
    resolve_stage,
)
from .process_control import ProcessManager
from .readiness import (
    CheckResult,
    CheckStatus,
    ReadinessReport,
    RosCliProbe,
    StaticProbe,
    check_stage,
)


_PUBLISHERS = re.compile(r'Publisher count:\s*(\d+)', re.IGNORECASE)
_ABSENT = re.compile(r'\bunknown\s+topic\b|\bnot\s+found\b', re.IGNORECASE)
_MODULE_TOPICS = {
    'camera': '/zed/zed_node/left/image_rect_color',
    'lidar': '/rslidar_points',
    'base': '/odom',
    'imu': '/imu/data_raw',
}
_EXIT_CODES = {
    CheckStatus.PASS: 0,
    CheckStatus.NOT_READY: 2,
    CheckStatus.DEGRADED: 3,
    CheckStatus.FAIL: 4,
}


class HardwareProbeError(RuntimeError):
    """Auto selection could not safely determine existing publishers."""


def _default_workspace_root():
    configured = os.environ.get('TRACK_ROBOT_WS')
    return configured or str(Path.home() / 'track_robot_ws')


def _add_stage_options(
        parser,
        hardware=False,
        stages=('sensors', 'phase0', 'phase1', 'phase2', 'phase3')):
    parser.add_argument('stage', choices=stages)
    if hardware:
        parser.add_argument(
            '--hardware', choices=('auto', 'external'), default='auto')
    else:
        parser.add_argument(
            '--hardware', choices=('auto', 'external'), default='external')
    parser.add_argument('--allow-degraded', action='store_true')
    parser.add_argument(
        '--extrinsic-mode',
        choices=('none', 'prototype', 'measured'),
        default='none',
    )
    parser.add_argument('--extrinsic-file', default='')
    parser.add_argument(
        '--workspace-root', default=_default_workspace_root())
    parser.add_argument('--probe-timeout', type=float, default=3.0)


def build_parser():
    """Build the control parser; the managed domain is fixed at 20."""

    parser = argparse.ArgumentParser(
        prog='semantic_search_ctl',
        description=(
            'Semantic-search bringup and diagnosis on fixed ROS Domain 20. '
            'Supervised motion is available only through explicit Phase 4B '
            'or Phase 5A execution modes.'),
    )
    parser.add_argument(
        '--domain',
        type=int,
        choices=(20,),
        default=20,
        help='managed ROS domain (fixed at 20)',
    )
    commands = parser.add_subparsers(dest='command', required=True)

    doctor = commands.add_parser(
        'doctor', help='passively check static inputs and the external graph')
    _add_stage_options(doctor)

    start = commands.add_parser(
        'start', help='start a passive stack in the foreground')
    _add_stage_options(start, hardware=True)
    start.add_argument('--readiness-timeout', type=float, default=30.0)

    status = commands.add_parser(
        'status', help='passively check graph and managed-process identity')
    _add_stage_options(status)

    query = commands.add_parser(
        'query', help='exec the existing semantic_search_query portal')
    query.add_argument('query', nargs='?')
    query.add_argument('query_args', nargs=argparse.REMAINDER)
    query.add_argument(
        '--workspace-root', default=_default_workspace_root(),
        help=argparse.SUPPRESS)

    visualize = commands.add_parser(
        'visualize', help='open the passive RViz test console in foreground')
    visualize.add_argument('stage', choices=('phase1', 'phase2', 'phase3'))
    visualize.add_argument(
        '--workspace-root', default=_default_workspace_root(),
        help=argparse.SUPPRESS)

    test = commands.add_parser(
        'test', help='delegate a bounded live test to Task 5 support')
    _add_stage_options(
        test, hardware=True, stages=('phase1', 'phase2', 'phase3'))
    test.add_argument('query')
    test.add_argument('--start-stack', action='store_true')
    test.add_argument('--duration-sec', type=float, default=10.0)
    test.add_argument('--output-dir', default=None)
    test.add_argument('--readiness-timeout', type=float, default=30.0)

    run = commands.add_parser(
        'run', help='run an explicitly supervised end-to-end workflow')
    run.add_argument('stage', choices=('phase4b', 'phase5a'))
    run.add_argument(
        '--workspace-root', default=_default_workspace_root())
    run.add_argument(
        '--extrinsic-mode',
        choices=('prototype', 'measured'),
        default='prototype',
    )
    run.add_argument('--extrinsic-file', default='')
    dino = run.add_mutually_exclusive_group()
    dino.add_argument(
        '--dino-enabled', dest='dino_enabled', action='store_true')
    dino.add_argument(
        '--no-dino', dest='dino_enabled', action='store_false')
    run.set_defaults(dino_enabled=True)
    run.add_argument(
        '--physical-recovery',
        action='store_true',
        help=(
            'enable bounded Nav2 Spin/BackUp recovery for an authorized '
            'SEMANTIC_ACTIVE mission'),
    )
    phase5a_mode = run.add_mutually_exclusive_group()
    phase5a_mode.add_argument(
        '--search-shadow',
        dest='phase5a_mode',
        action='store_const',
        const='SEARCH_SHADOW',
        help='record bounded search decisions but publish no executable motion',
    )
    phase5a_mode.add_argument(
        '--rotation-supervised',
        dest='phase5a_mode',
        action='store_const',
        const='ROTATION_SUPERVISED',
        help='enable operator-supervised, rotation-only Nav2 Spin execution',
    )
    run.set_defaults(phase5a_mode='PASSIVE_ONLY')

    stop = commands.add_parser(
        'stop', help='stop only a verified managed process group')
    stop.add_argument(
        '--workspace-root', default=_default_workspace_root(),
        help=argparse.SUPPRESS)
    return parser


def exit_code(status):
    """Map a readiness status to the stable command exit code."""

    return _EXIT_CODES[status]


def _run(runner, argv, environment, timeout):
    try:
        completed = runner(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=float(timeout),
            env=environment,
            shell=False,
        )
        return completed.returncode, str(completed.stdout or ''), str(
            completed.stderr or ''), False
    except subprocess.TimeoutExpired as error:
        stdout = error.output.decode(errors='replace') if isinstance(
            error.output, bytes) else str(error.output or '')
        stderr = error.stderr.decode(errors='replace') if isinstance(
            error.stderr, bytes) else str(error.stderr or '')
        return None, stdout, stderr, True
    except OSError as error:
        return 1, '', str(error), False


def select_hardware(
        stage,
        mode,
        runner=None,
        environment=None,
        timeout=1.0,
        deadline=None,
        monotonic=None):
    """Select owned modules without ever using message frequency as a gate.

    Auto mode lists the graph exactly once. Every required topic found in that
    snapshot is inspected for publisher existence. A publisher is reused even
    when it may later fail readiness frequency checks, preventing a duplicate
    driver from being started for a stale publisher.
    """

    spec = resolve_stage(stage)
    if mode == 'external':
        return HardwareSelection(False, False, False, False)
    if mode != 'auto':
        raise ValueError('hardware mode must be auto or external')

    runner = runner or subprocess.run
    environment = managed_environment(
        os.environ if environment is None else environment)
    monotonic = monotonic or time.monotonic

    def remaining_timeout():
        if deadline is None:
            return timeout
        remaining = float(deadline) - monotonic()
        if remaining <= 0.0:
            raise HardwareProbeError(
                'start deadline exhausted during hardware selection')
        return min(timeout, remaining)

    code, output, stderr, timed_out = _run(
        runner,
        ['ros2', 'topic', 'list', '-t'],
        environment,
        remaining_timeout(),
    )
    if code != 0:
        detail = stderr.strip() or output.strip() or 'no diagnostic output'
        if timed_out:
            detail = 'timed out: {}'.format(detail)
        raise HardwareProbeError(
            'cannot safely inspect ROS graph: {}'.format(detail))

    listed = {
        line.split()[0] for line in output.splitlines() if line.split()
    }
    start = {}
    for module in ('camera', 'lidar', 'base', 'imu'):
        required = bool(getattr(spec, module))
        topic = _MODULE_TOPICS[module]
        if not required:
            start[module] = False
            continue
        if topic not in listed:
            start[module] = True
            continue

        argv = ['ros2', 'topic', 'info', topic]
        code, info, info_error, timed_out = _run(
            runner, argv, environment, remaining_timeout())
        combined = info + '\n' + info_error
        if code != 0:
            if code is not None and _ABSENT.search(combined):
                start[module] = True
                continue
            detail = info_error.strip() or info.strip() or 'no diagnostic output'
            if timed_out:
                detail = 'timed out: {}'.format(detail)
            raise HardwareProbeError(
                'cannot safely inspect {}: {}'.format(topic, detail))
        publishers = _PUBLISHERS.search(combined)
        if publishers is None:
            raise HardwareProbeError(
                'cannot determine publisher count for {}'.format(topic))
        start[module] = int(publishers.group(1)) == 0

    return HardwareSelection(**start)


def build_launch_argv(args, selection, paths):
    """Build an explicit argument vector for the aggregate passive launch."""

    argv = [
        'ros2',
        'launch',
        'track_robot_bringup',
        'semantic_search_live.launch.py',
        'stage:={}'.format(args.stage),
        'start_camera:={}'.format(str(selection.camera).lower()),
        'start_lidar:={}'.format(str(selection.lidar).lower()),
        'start_base:={}'.format(str(selection.base).lower()),
        'start_imu:={}'.format(str(selection.imu).lower()),
        'runtime_path:={}'.format(paths['runtime_path']),
        'checkpoint_path:={}'.format(paths['checkpoint_path']),
        'yolo_runtime_path:={}'.format(
            paths.get('yolo_runtime_path', paths['runtime_path'])),
        'yolo_checkpoint_path:={}'.format(
            paths.get('yolo_checkpoint_path', paths['checkpoint_path'])),
        'dino_repo_path:={}'.format(
            paths.get('dino_repo_path', paths['runtime_path'])),
        'dino_checkpoint_path:={}'.format(
            paths.get('dino_checkpoint_path', paths['checkpoint_path'])),
        'extrinsic_mode:={}'.format(args.extrinsic_mode),
        'allow_degraded:={}'.format(str(args.allow_degraded).lower()),
    ]
    if args.extrinsic_file:
        argv.append('extrinsic_file:={}'.format(args.extrinsic_file))
    return argv


def build_phase4b_launch_argv(args, paths):
    """Build the fixed supervised Phase 0-4B command.

    The launch itself hard-disables IMU. Motion remains behind the semantic
    authorization service, safety supervisor, and final velocity gate.
    """

    argv = [
        'ros2',
        'launch',
        'track_robot_bringup',
        'semantic_search_phase4b.launch.py',
        'runtime_mode:=SEMANTIC_ACTIVE',
        'enable_semantic_execution:=true',
        'physical_recovery_enabled:={}'.format(
            str(bool(args.physical_recovery)).lower()),
        'start_base:=true',
        'start_phase4b_rviz:=true',
        'configure_network:=false',
        'extrinsic_mode:={}'.format(args.extrinsic_mode),
        'dino_enabled:={}'.format(
            str(bool(args.dino_enabled)).lower()),
        'yolo_runtime_path:={}'.format(paths['yolo_runtime_path']),
        'clip_runtime_path:={}'.format(paths['runtime_path']),
        'yolo_checkpoint:={}'.format(paths['yolo_checkpoint_path']),
        'clip_checkpoint:={}'.format(paths['checkpoint_path']),
        'dino_repo_path:={}'.format(paths['dino_repo_path']),
        'dino_checkpoint:={}'.format(paths['dino_checkpoint_path']),
    ]
    if args.extrinsic_file:
        argv.append('extrinsic_file:={}'.format(args.extrinsic_file))
    return argv


def build_phase5a_launch_argv(args, paths):
    """Build the Phase 0-5A command with fail-closed mode pairing."""

    search_mode = str(args.phase5a_mode)
    active = search_mode == 'ROTATION_SUPERVISED'
    runtime_mode = 'SEMANTIC_ACTIVE' if active else 'PLANNING_ONLY'
    argv = [
        'ros2',
        'launch',
        'track_robot_bringup',
        'semantic_search_phase5a.launch.py',
        'search_mode:={}'.format(search_mode),
        'rotation_runtime_mode:={}'.format(runtime_mode),
        'enable_rotation_execution:={}'.format(str(active).lower()),
        'physical_recovery_enabled:={}'.format(
            str(bool(args.physical_recovery)).lower()),
        'start_base:={}'.format(str(active).lower()),
        'start_phase5a_rviz:=true',
        'configure_network:=false',
        'extrinsic_mode:={}'.format(args.extrinsic_mode),
        'dino_enabled:={}'.format(str(bool(args.dino_enabled)).lower()),
        'yolo_runtime_path:={}'.format(paths['yolo_runtime_path']),
        'clip_runtime_path:={}'.format(paths['runtime_path']),
        'yolo_checkpoint:={}'.format(paths['yolo_checkpoint_path']),
        'clip_checkpoint:={}'.format(paths['checkpoint_path']),
        'dino_repo_path:={}'.format(paths['dino_repo_path']),
        'dino_checkpoint:={}'.format(paths['dino_checkpoint_path']),
    ]
    if args.extrinsic_file:
        argv.append('extrinsic_file:={}'.format(args.extrinsic_file))
    return argv


def request_phase4b_cancel_disarm(runner, environment, timeout=2.0):
    """Best-effort bounded cancellation before stopping the owned launch."""

    command = [
        'ros2',
        'service',
        'call',
        '/semantic_navigation/cancel_and_disarm',
        'std_srvs/srv/Trigger',
        '{}',
    ]
    code, output, _stderr, timed_out = _run(
        runner, command, environment, timeout)
    return (
        not timed_out
        and code == 0
        and 'success: true' in output.lower())


class _Phase4BStopProxy:
    """Add one bounded cancel/disarm request before verified process stop."""

    def __init__(self, manager, runner, environment):
        self._manager = manager
        self._runner = runner
        self._environment = environment
        self._requested = False

    def stop_owned(self):
        if not self._requested:
            self._requested = True
            request_phase4b_cancel_disarm(
                self._runner, self._environment)
        return self._manager.stop_owned()

    def clear_if_owned(self, state):
        return self._manager.clear_if_owned(state)


def request_phase5a_cancel_disarm(runner, environment, timeout=2.0):
    """Best-effort bounded stop for rotation-only active search."""

    commands = (
        [
            'ros2', 'service', 'call',
            '/semantic_search/active_search/cancel',
            'std_srvs/srv/Trigger', '{}',
        ],
        [
            'ros2', 'service', 'call',
            '/safety/disarm',
            'std_srvs/srv/Trigger', '{}',
        ],
    )
    results = []
    for command in commands:
        code, output, _stderr, timed_out = _run(
            runner, command, environment, timeout)
        results.append(
            not timed_out
            and code == 0
            and 'success: true' in output.lower())
    return all(results)


class _Phase5AStopProxy:
    """Cancel rotation and disarm before stopping a verified Phase 5A run."""

    def __init__(self, manager, runner, environment, active):
        self._manager = manager
        self._runner = runner
        self._environment = environment
        self._active = bool(active)
        self._requested = False

    def stop_owned(self):
        if self._active and not self._requested:
            self._requested = True
            request_phase5a_cancel_disarm(
                self._runner, self._environment)
        return self._manager.stop_owned()

    def clear_if_owned(self, state):
        return self._manager.clear_if_owned(state)


def _paths_for(args):
    paths = default_workspace_paths(args.workspace_root)
    paths.update({
        'extrinsic_mode': args.extrinsic_mode,
        'extrinsic_file': args.extrinsic_file,
        'allow_degraded': args.allow_degraded,
    })
    return paths


def _environment(base, paths):
    environment = managed_environment(base)
    # Foxy/Fast DDS on this Jetson fails to deliver transient-local /tf_static
    # reliably across the managed multi-process stack with localhost-only
    # discovery. Keep Domain 20 isolated by convention, but allow normal local
    # interface discovery so RViz and robot_state_publisher share the same
    # complete TF graph.
    environment['ROS_LOCALHOST_ONLY'] = '0'
    # The retired remote-panel profile also causes partial graph discovery and
    # must not leak into the managed local stack from an older shell.
    environment.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    return environment


def _static_report(stage, paths, environment):
    return ReadinessReport(
        StaticProbe(paths).check(stage),
        stage=stage,
        ros_domain_id=environment['ROS_DOMAIN_ID'],
    )


def _readiness_report(
        stage,
        selection,
        paths,
        environment,
        runner,
        timeout):
    probe = RosCliProbe(
        runner=runner,
        environment=environment,
        topic_timeout=timeout,
        tf_timeout=min(0.2, timeout),
        dds_profile=paths['dds_profile'],
    )
    return check_stage(stage, selection, paths, probe)


def _bounded_readiness_report(
        stage,
        selection,
        paths,
        environment,
        runner,
        timeout,
        deadline,
        monotonic):
    """Run one readiness snapshot within a caller-owned total deadline."""

    def bounded_runner(argv, **kwargs):
        remaining = float(deadline) - monotonic()
        if remaining <= 0.0:
            raise subprocess.TimeoutExpired(argv, 0.0)
        requested = float(kwargs.get('timeout', remaining))
        kwargs['timeout'] = min(requested, remaining)
        return runner(argv, **kwargs)

    remaining = max(0.0, float(deadline) - monotonic())
    probe_timeout = min(max(0.01, float(timeout)), max(0.01, remaining))
    return _readiness_report(
        stage,
        selection,
        paths,
        environment,
        bounded_runner,
        probe_timeout,
    )


def _render_report(report, stream):
    stream.write(report.render_text())
    stream.write('\n')


def _managed_status(manager):
    verified = manager.verified_state()
    if verified is not None:
        modules = ','.join(verified.owned_modules) or 'none'
        return (
            'Managed process: verified pid={pid} pgid={pgid} stage={stage} '
            'modules={modules}'.format(
                pid=verified.pid,
                pgid=verified.pgid,
                stage=verified.stage,
                modules=modules,
            ))
    read_state = getattr(manager, 'read_state', None)
    recorded = read_state() if read_state is not None else None
    if recorded is not None:
        return (
            'Managed process: stale or unverified pid={}; no signal will be '
            'sent'.format(recorded.pid))
    return 'Managed process: none'


def _manager(manager, popen_factory, os_api):
    if manager is not None:
        return manager
    return ProcessManager(os_api=os_api, popen_factory=popen_factory)


def _cleanup_spawned(process_manager, captured_state):
    """Attempt verified cleanup and return explicit ownership-risk detail."""

    try:
        if captured_state is not None:
            stopped = process_manager.stop_captured(captured_state)
        else:
            stopped = process_manager.stop_owned()
    except Exception as error:
        return 'orphan risk: cleanup raised {}'.format(error)
    if stopped:
        return 'verified cleanup completed'
    stop_error = getattr(process_manager, 'last_stop_error', None)
    return 'orphan risk: cleanup failed{}'.format(
        ': {}'.format(stop_error) if stop_error else '')


def _spawn_managed_stack(
        args,
        process_manager,
        paths,
        environment,
        runner,
        deadline,
        monotonic):
    """Select, spawn, and persist a stack through the verified manager."""

    existing = process_manager.verified_state()
    if existing is not None:
        raise RuntimeError(
            'Controller already manages pid {} (stage {}); stop it before '
            'starting another stack.'.format(existing.pid, existing.stage))
    try:
        selection = select_hardware(
            args.stage,
            args.hardware,
            runner=runner,
            environment=environment,
            timeout=max(0.01, args.probe_timeout),
            deadline=deadline,
            monotonic=monotonic,
        )
    except HardwareProbeError as error:
        raise RuntimeError(str(error)) from error
    if monotonic() >= deadline:
        raise RuntimeError('Start deadline exhausted before launch.')

    command = build_launch_argv(args, selection, paths)
    modules = tuple(
        name for name in ('camera', 'lidar', 'base', 'imu')
        if getattr(selection, name))
    captured_state = None
    spawn_verified = getattr(process_manager, 'spawn_verified', None)
    used_captured_api = spawn_verified is not None
    try:
        if used_captured_api:
            child, captured_state = spawn_verified(
                command,
                stage=args.stage,
                owned_modules=modules,
                environment=environment,
            )
        else:
            child = process_manager.spawn(
                command,
                stage=args.stage,
                owned_modules=modules,
                environment=environment,
            )
    except (OSError, RuntimeError) as error:
        raise RuntimeError(
            'Failed to start semantic-search stack: {}'.format(error)) from error
    if used_captured_api and captured_state is None:
        cleanup_detail = _cleanup_spawned(process_manager, None)
        raise RuntimeError(
            'Spawned process did not return captured ownership state; {}.'.format(
                cleanup_detail))
    try:
        owned_state = process_manager.read_state()
    except Exception as error:
        cleanup_detail = _cleanup_spawned(
            process_manager, captured_state)
        raise RuntimeError(
            'Failed to verify spawned ownership state: {}; {}.'.format(
                error, cleanup_detail)) from error
    if owned_state is None:
        cleanup_detail = _cleanup_spawned(
            process_manager, captured_state)
        raise RuntimeError(
            'Spawned ownership state is unavailable; {}.'.format(
                cleanup_detail))
    if (captured_state is not None
            and owned_state.identity != captured_state.identity):
        cleanup_detail = _cleanup_spawned(
            process_manager, captured_state)
        raise RuntimeError(
            'Spawned ownership state does not match captured identity; '
            '{}.'.format(cleanup_detail))
    if monotonic() >= deadline:
        cleanup_detail = _cleanup_spawned(
            process_manager, captured_state)
        raise RuntimeError(
            'Start deadline exhausted during launch; {}.'.format(
                cleanup_detail))
    return child, owned_state, selection


def _wait_for_readiness(
        args,
        child,
        selection,
        paths,
        environment,
        runner,
        monotonic,
        sleeper,
        deadline=None):
    if deadline is None:
        timeout = max(0.0, float(args.readiness_timeout))
        deadline = monotonic() + timeout
    last_report = None

    def bounded_runner(argv, **kwargs):
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            raise subprocess.TimeoutExpired(argv, 0.0)
        requested = float(kwargs.get('timeout', remaining))
        kwargs['timeout'] = min(requested, remaining)
        return runner(argv, **kwargs)

    while True:
        if child.poll() is not None:
            return ReadinessReport([
                CheckResult.fail(
                    'managed_process',
                    'launch exited before readiness with code {}'.format(
                        child.returncode)),
            ], stage=args.stage)
        remaining = max(0.0, deadline - monotonic())
        probe_timeout = min(
            max(0.01, float(args.probe_timeout)),
            max(0.01, remaining),
        )
        last_report = _readiness_report(
            args.stage,
            selection,
            paths,
            environment,
            bounded_runner,
            probe_timeout,
        )
        if last_report.overall in (CheckStatus.PASS, CheckStatus.DEGRADED):
            return last_report
        remaining = max(0.0, deadline - monotonic())
        if last_report.overall is CheckStatus.FAIL or remaining <= 0.0:
            return last_report
        sleeper(min(0.25, remaining))


@contextmanager
def _forward_signals(manager):
    interrupted = {'value': False}
    previous = {}

    def forward(signum, _frame):
        if interrupted['value']:
            return
        interrupted['value'] = True
        manager.stop_owned()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, forward)
    except ValueError:
        pass

    try:
        try:
            yield interrupted
        except KeyboardInterrupt:
            interrupted['value'] = True
            manager.stop_owned()
    finally:
        for signum, handler in previous.items():
            try:
                signal.signal(signum, handler)
            except ValueError:
                pass


def _foreground_wait(
        child,
        manager,
        owned_state,
        readiness_status=CheckStatus.PASS):
    return_code = None
    with _forward_signals(manager) as interrupted:
        return_code = child.wait()

    if interrupted['value']:
        if child.poll() is not None:
            manager.clear_if_owned(owned_state)
        return 130
    manager.clear_if_owned(owned_state)
    return exit_code(readiness_status) if return_code == 0 else 4


def _load_live_test():
    try:
        return importlib.import_module('.live_test', package=__package__)
    except ModuleNotFoundError as error:
        expected = '{}.live_test'.format(__package__)
        if error.name == expected:
            return None
        raise


def _execute_live_test_command(
        live_test,
        args,
        environment,
        report,
        stdout,
        stderr):
    """Run collection and apply pipeline/readiness exit-code precedence."""

    result = live_test.run_live_test(
        args.stage,
        args.query,
        args.duration_sec,
        args.output_dir,
        environment,
        calibration_mode=args.extrinsic_mode,
        readiness_snapshot=report.as_dict(),
    )
    live_code = (
        int(result) if isinstance(result, int)
        else int(getattr(result, 'exit_code', 0))
    )
    error = getattr(result, 'error', '')
    if error:
        stderr.write('{}\n'.format(error))
    report_path = getattr(result, 'report_path', None)
    if report_path is not None:
        stdout.write('Report: {}\n'.format(report_path))
    if live_code != 0:
        return live_code
    if report.overall is CheckStatus.DEGRADED:
        return 3
    return 0


def main(
        argv=None,
        *,
        runner=None,
        popen_factory=None,
        os_api=None,
        manager=None,
        stdout=None,
        stderr=None,
        environ=None,
        monotonic=None,
        sleeper=None):
    """Run a control command. Optional dependencies support hardware-free tests."""

    args = build_parser().parse_args(argv)
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    runner = runner or subprocess.run
    popen_factory = popen_factory or subprocess.Popen
    base_environment = os.environ if environ is None else environ
    monotonic = monotonic or time.monotonic
    sleeper = sleeper or time.sleep
    start_deadline = None
    if args.command in ('start', 'test'):
        start_deadline = (
            monotonic() + max(0.0, float(args.readiness_timeout)))

    if args.command == 'stop':
        process_manager = _manager(manager, popen_factory, os_api)
        verified_state = getattr(process_manager, 'verified_state', None)
        state = verified_state() if verified_state is not None else None
        if state is not None and state.stage in ('phase4b', 'phase5a'):
            paths = default_workspace_paths(args.workspace_root)
            environment = _environment(base_environment, paths)
            if state.stage == 'phase4b':
                request_phase4b_cancel_disarm(runner, environment)
            elif 'base' in getattr(state, 'owned_modules', ()):
                request_phase5a_cancel_disarm(runner, environment)
        stopped = process_manager.stop_owned()
        if stopped:
            stdout.write('Stopped verified managed process group.\n')
            return 0
        stop_error = getattr(process_manager, 'last_stop_error', None)
        if stop_error:
            stderr.write('Failed to stop managed process: {}\n'.format(
                stop_error))
            return 4
        stdout.write('No verified managed process group found.\n')
        return 2

    if args.command == 'run':
        paths = default_workspace_paths(args.workspace_root)
        environment = _environment(base_environment, paths)
        process_manager = _manager(manager, popen_factory, os_api)
        if process_manager.verified_state() is not None:
            stderr.write(
                'A verified managed process is already running; stop it '
                'before {}.\n'.format(args.stage.upper()))
            return 4
        if args.stage == 'phase4b':
            command = build_phase4b_launch_argv(args, paths)
            active = True
        else:
            command = build_phase5a_launch_argv(args, paths)
            active = args.phase5a_mode == 'ROTATION_SUPERVISED'
        try:
            child, owned_state = process_manager.spawn_verified(
                command,
                stage=args.stage,
                owned_modules=(
                    ('camera', 'lidar', 'base') if active
                    else ('camera', 'lidar')),
                environment=environment,
            )
        except (OSError, RuntimeError) as error:
            stderr.write('Failed to start {}: {}\n'.format(
                args.stage.upper(), error))
            return 4
        if args.stage == 'phase4b':
            stdout.write(
                'Phase 4B started in ROS Domain 20 (IMU disabled).\n'
                '1. Enter an English target in RViz.\n'
                '2. Confirm one stable best candidate and a valid path.\n'
                '3. Click Start Approach; motion cannot start before this.\n'
                '4. Use Cancel & Disarm, RC override, or E-stop to stop.\n')
            proxy = _Phase4BStopProxy(
                process_manager, runner, environment)
        else:
            stdout.write(
                'Phase 5A started in {} on ROS Domain 20 (IMU disabled).\n'
                'Enter an English target in RViz. Passive and shadow modes '
                'cannot move; supervised mode permits bounded rotation only.\n'
                .format(args.phase5a_mode))
            proxy = _Phase5AStopProxy(
                process_manager, runner, environment, active)
        return _foreground_wait(
            child, proxy, owned_state, CheckStatus.PASS)

    if args.command == 'query':
        paths = default_workspace_paths(args.workspace_root)
        environment = _environment(base_environment, paths)
        command = [
            'ros2',
            'run',
            'track_robot_semantic_search',
            'semantic_search_query',
        ]
        if args.query is not None:
            command.append(args.query)
        command.extend(args.query_args)
        exec_api = os_api or os
        try:
            result = exec_api.execvpe(command[0], command, environment)
            return int(result) if result is not None else 0
        except OSError as error:
            stderr.write('Failed to execute semantic_search_query: {}\n'.format(
                error))
            return 4

    if args.command == 'visualize':
        paths = default_workspace_paths(args.workspace_root)
        environment = _environment(base_environment, paths)
        command = [
            'ros2',
            'launch',
            'track_robot_bringup',
            'semantic_search_visualization.launch.py',
            'stage:={}'.format(args.stage),
        ]
        exec_api = os_api or os
        try:
            result = exec_api.execvpe(command[0], command, environment)
            return int(result) if result is not None else 0
        except OSError as error:
            stderr.write(
                'Failed to execute semantic-search visualization: {}\n'.format(
                    error))
            return 4

    paths = _paths_for(args)
    environment = _environment(base_environment, paths)

    if args.command == 'test':
        live_test = _load_live_test()
        if live_test is None:
            stderr.write(
                'Task 5 live-test support is not installed; '
                'track_robot_bringup.live_test is unavailable.\n')
            return 4
        external = HardwareSelection(False, False, False, False)
        try:
            report = _bounded_readiness_report(
                args.stage,
                external,
                paths,
                environment,
                runner,
                args.probe_timeout,
                start_deadline,
                monotonic,
            )
        except KeyboardInterrupt:
            return 130
        except Exception as error:
            stderr.write('Readiness check failed: {}\n'.format(error))
            return 4
        _render_report(report, stdout)
        if report.overall is CheckStatus.FAIL:
            return 4
        if report.overall is CheckStatus.NOT_READY and not args.start_stack:
            return 2

        if report.overall is not CheckStatus.NOT_READY:
            try:
                return _execute_live_test_command(
                    live_test, args, environment, report, stdout, stderr)
            except KeyboardInterrupt:
                return 130
            except Exception as error:
                stderr.write('Live test failed: {}\n'.format(error))
                return 4

        # The pre-start graph snapshot answers only whether an external stack
        # can be reused. It must not consume the budget intended for hardware
        # startup, model loading, and post-launch readiness.
        start_deadline = (
            monotonic() + max(0.0, float(args.readiness_timeout)))
        process_manager = _manager(manager, popen_factory, os_api)
        result_code = 4
        owns_stack = False
        try:
            with _forward_signals(process_manager) as interrupted:
                try:
                    child, _owned_state, selection = _spawn_managed_stack(
                        args,
                        process_manager,
                        paths,
                        environment,
                        runner,
                        start_deadline,
                        monotonic,
                    )
                    owns_stack = True
                except Exception as error:
                    stderr.write(
                        'Failed to start semantic-search stack: {}\n'.format(
                            error))
                if owns_stack and not interrupted['value']:
                    report = _wait_for_readiness(
                        args,
                        child,
                        selection,
                        paths,
                        environment,
                        runner,
                        monotonic,
                        sleeper,
                        deadline=start_deadline,
                    )
                    _render_report(report, stdout)
                    if report.overall in (
                            CheckStatus.NOT_READY, CheckStatus.FAIL):
                        result_code = exit_code(report.overall)
                    else:
                        result_code = _execute_live_test_command(
                            live_test,
                            args,
                            environment,
                            report,
                            stdout,
                            stderr,
                        )
            if interrupted['value']:
                result_code = 130
        except KeyboardInterrupt:
            result_code = 130
        except Exception as error:
            stderr.write('Live test failed: {}\n'.format(error))
            result_code = 4
        finally:
            if owns_stack:
                stopped = process_manager.stop_owned()
                if not stopped and result_code != 130:
                    stop_error = getattr(
                        process_manager, 'last_stop_error', None)
                    stderr.write(
                        'Failed to stop managed test stack: {}\n'.format(
                            stop_error or 'ownership verification failed'))
                    result_code = 4
        return result_code

    external = HardwareSelection(False, False, False, False)
    if args.command in ('doctor', 'status'):
        report = _readiness_report(
            args.stage,
            external,
            paths,
            environment,
            runner,
            max(0.01, args.probe_timeout),
        )
        _render_report(report, stdout)
        if args.command == 'status':
            process_manager = _manager(manager, popen_factory, os_api)
            stdout.write(_managed_status(process_manager))
            stdout.write('\n')
        return exit_code(report.overall)

    static = _static_report(args.stage, paths, environment)
    _render_report(static, stdout)
    if static.overall in (CheckStatus.NOT_READY, CheckStatus.FAIL):
        return exit_code(static.overall)
    if monotonic() >= start_deadline:
        stderr.write('Start deadline exhausted after static preflight.\n')
        return 4

    process_manager = _manager(manager, popen_factory, os_api)
    try:
        child, owned_state, selection = _spawn_managed_stack(
            args,
            process_manager,
            paths,
            environment,
            runner,
            start_deadline,
            monotonic,
        )
    except RuntimeError as error:
        stderr.write('{}\n'.format(error))
        return 4
    report = None
    try:
        with _forward_signals(process_manager) as interrupted:
            report = _wait_for_readiness(
                args,
                child,
                selection,
                paths,
                environment,
                runner,
                monotonic,
                sleeper,
                deadline=start_deadline,
            )
    except Exception as error:
        process_manager.stop_owned()
        stderr.write('Readiness check failed: {}\n'.format(error))
        return 4
    if interrupted['value']:
        child.poll()
        return 130
    _render_report(report, stdout)
    if report.overall in (CheckStatus.NOT_READY, CheckStatus.FAIL):
        process_manager.stop_owned()
        return exit_code(report.overall)
    return _foreground_wait(
        child, process_manager, owned_state, report.overall)


__all__ = [
    'build_phase4b_launch_argv',
    'build_phase5a_launch_argv',
    'build_launch_argv',
    'build_parser',
    'exit_code',
    'main',
    'request_phase4b_cancel_disarm',
    'request_phase5a_cancel_disarm',
    'select_hardware',
]
