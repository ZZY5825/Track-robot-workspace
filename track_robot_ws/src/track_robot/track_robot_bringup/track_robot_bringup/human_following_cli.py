"""Independent one-command operation for live human following."""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import yaml

from .control_cli import HardwareProbeError, select_hardware
from .control_config import managed_environment
from .human_following_readiness import check_human_following
from .process_control import ProcessManager
from .readiness import CheckResult, CheckStatus, ReadinessReport


_PUBLISHERS = re.compile(r'Publisher count:\s*(\d+)', re.IGNORECASE)
_TOPIC_ABSENT = re.compile(
    r'\bunknown\s+topic\b|\bnot\s+found\b', re.IGNORECASE)


def _default_workspace_root():
    return os.environ.get(
        'TRACK_ROBOT_WS', str(Path.home() / 'track_robot_ws'))


def default_state_path(environment=None):
    """Return human-following ownership state, separate from every feature."""

    environment = os.environ if environment is None else environment
    ros_home = environment.get('ROS_HOME')
    root = Path(ros_home).expanduser() if ros_home else Path.home() / '.ros'
    return root / 'track_robot_human_following' / 'managed_process.json'


def _add_runtime_options(parser, default_hardware):
    parser.add_argument(
        '--runtime-mode', choices=('shadow', 'active'), default='shadow')
    parser.add_argument(
        '--hardware', choices=('auto', 'external'),
        default=default_hardware)
    parser.add_argument(
        '--workspace-root', default=_default_workspace_root(),
        help=argparse.SUPPRESS)
    parser.add_argument('--probe-timeout', type=float, default=1.0)


def build_parser():
    """Build the human-following parser with ROS Domain 20 fixed."""

    parser = argparse.ArgumentParser(
        prog='human_following_ctl',
        description='Fail-closed human-following operation on ROS Domain 20.',
    )
    parser.add_argument(
        '--domain', type=int, choices=(20,), default=20,
        help='managed ROS domain (fixed at 20)')
    commands = parser.add_subparsers(dest='command', required=True)

    doctor = commands.add_parser(
        'doctor', help='passively inspect human-following readiness')
    _add_runtime_options(doctor, 'external')

    start = commands.add_parser(
        'start', help='start an owned human-following stack in the foreground')
    _add_runtime_options(start, 'auto')
    start.add_argument('--confirm-motion', action='store_true')
    start.add_argument('--readiness-timeout', type=float, default=30.0)

    status = commands.add_parser(
        'status', help='inspect the verified managed run and live readiness')
    status.add_argument('--probe-timeout', type=float, default=1.0)

    commands.add_parser(
        'stop', help='perform ordered shutdown of a verified managed run')
    return parser


def validate_motion_request(args):
    """Reject every ambiguous request that could create command ownership."""

    if args.command != 'start':
        return
    if args.runtime_mode == 'active' and not args.confirm_motion:
        raise ValueError(
            'active human following requires --confirm-motion')
    if args.runtime_mode != 'active' and args.confirm_motion:
        raise ValueError('--confirm-motion is valid only with active mode')


def _profile_path(args):
    profile = (
        'human_following_supervised_test.yaml'
        if args.runtime_mode == 'active'
        else 'human_following_shadow.yaml')
    return (
        Path(args.workspace_root) / 'src' / 'track_robot'
        / 'track_robot_bringup' / 'config' / profile)


def build_launch_argv(args, selection):
    """Build the plan's exact explicit aggregate launch vector."""

    return [
        'ros2', 'launch', 'track_robot_bringup',
        'human_following_live.launch.py',
        'runtime_mode:={}'.format(args.runtime_mode),
        'motion_confirmed:={}'.format(
            str(bool(args.confirm_motion)).lower()),
        'start_camera:={}'.format(str(selection.camera).lower()),
        'start_lidar:={}'.format(str(selection.lidar).lower()),
        'start_base:={}'.format(str(selection.base).lower()),
        'start_imu:={}'.format(str(selection.imu).lower()),
        'profile_config:={}'.format(_profile_path(args)),
    ]


def _environment(base):
    environment = managed_environment(base)
    environment['ROS_LOCALHOST_ONLY'] = '0'
    environment.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    return environment


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
        return (
            completed.returncode,
            _text(completed.stdout),
            _text(completed.stderr),
            False,
        )
    except subprocess.TimeoutExpired as error:
        return None, _text(error.output), _text(error.stderr), True
    except OSError as error:
        return 1, '', str(error), False


def _text(value):
    if value is None:
        return ''
    if isinstance(value, bytes):
        return value.decode(errors='replace')
    return str(value)


def call_trigger(service, runner, environment, timeout):
    """Call one Trigger service through a bounded, non-shell subprocess."""

    code, output, stderr, timed_out = _run(runner, [
        'ros2', 'service', 'call', service,
        'std_srvs/srv/Trigger', '{}',
    ], environment, timeout)
    return (
        not timed_out
        and code == 0
        and re.search(
            r'\bsuccess:\s*true\b', '{}\n{}'.format(output, stderr),
            re.IGNORECASE) is not None
    )


def _twist_is_zero(payload):
    try:
        documents = [
            document for document in yaml.safe_load_all(payload)
            if isinstance(document, dict)
        ]
    except yaml.YAMLError:
        return False
    latest_is_zero = None
    for message in documents:
        linear = message.get('linear')
        angular = message.get('angular')
        if not isinstance(linear, dict) or not isinstance(angular, dict):
            continue
        try:
            values = [
                float(linear[axis]) for axis in ('x', 'y', 'z')
            ] + [
                float(angular[axis]) for axis in ('x', 'y', 'z')
            ]
        except (KeyError, TypeError, ValueError):
            continue
        latest_is_zero = all(
            math.isfinite(value) and abs(value) <= 1.0e-9
            for value in values)
    return latest_is_zero is True


def observe_zero_twist(topic, runner, environment, timeout=1.0):
    """Observe a fresh, complete zero Twist within a bounded echo."""

    _code, output, stderr, _timed_out = _run(runner, [
        'ros2', 'topic', 'echo', topic,
        '--qos-profile', 'default', '--no-arr', '--no-str',
    ], environment, timeout)
    return _twist_is_zero('{}\n{}'.format(output, stderr))


def cmd_vel_has_zero_publishers(runner, environment, timeout=2.0):
    """Return true only when the graph proves no /cmd_vel publisher exists."""

    code, output, stderr, timed_out = _run(runner, [
        'ros2', 'topic', 'info', '/cmd_vel',
    ], environment, timeout)
    combined = '{}\n{}'.format(output, stderr)
    if timed_out:
        return False
    if code != 0:
        return bool(_TOPIC_ABSENT.search(combined))
    publishers = _PUBLISHERS.search(combined)
    return publishers is not None and int(publishers.group(1)) == 0


def ordered_stop(
        process_manager,
        state,
        runner,
        environment,
        *,
        sleeper=None):
    """Prove motion is stopped before signalling owned feature processes."""

    sleeper = sleeper or time.sleep
    disarmed = call_trigger(
        '/safety/disarm', runner, environment, timeout=2.0)
    zero = observe_zero_twist(
        '/follow/cmd_vel_safe', runner, environment, timeout=1.0)
    emergency_ok = True
    if not disarmed or not zero:
        emergency_ok = call_trigger(
            '/safety/emergency_stop', runner, environment, timeout=2.0)

    gate_ok = True
    if state.stage == 'active':
        gate_ok = call_trigger(
            '/cmd_vel_gate/shutdown', runner, environment, timeout=2.0)
        if not gate_ok:
            emergency_ok = call_trigger(
                '/safety/emergency_stop', runner, environment,
                timeout=2.0) and emergency_ok
        else:
            # The gate responds before its delayed shutdown so DDS has a brief
            # chance to remove publisher ownership before the proof query.
            sleeper(0.1)

    no_command_publisher = cmd_vel_has_zero_publishers(
        runner, environment, timeout=2.0)
    if not emergency_ok or not no_command_publisher:
        return False
    return bool(process_manager.stop_owned())


def _render_report(report, stream):
    stream.write(report.render_text())
    stream.write('\n')


def _exit_code(report):
    return 0 if report.overall is CheckStatus.PASS else 4


def _manager(manager, popen_factory, os_api, environment):
    if manager is not None:
        return manager
    return ProcessManager(
        state_path=default_state_path(environment),
        popen_factory=popen_factory,
        os_api=os_api,
    )


def _status_line(process_manager):
    state = process_manager.verified_state()
    if state is not None:
        modules = ','.join(state.owned_modules) or 'none'
        return (
            'Managed human-following process: verified pid={} pgid={} '
            'mode={} modules={}'.format(
                state.pid, state.pgid, state.stage, modules))
    recorded = process_manager.read_state()
    if recorded is not None:
        return (
            'Managed human-following process: stale or unverified pid={}; '
            'no signal will be sent'.format(recorded.pid))
    return 'Managed human-following process: none'


def _bounded_readiness(
        runtime_mode, runner, environment, probe_timeout, deadline,
        monotonic):
    def bounded_runner(argv, **kwargs):
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            raise subprocess.TimeoutExpired(argv, 0.0)
        kwargs['timeout'] = min(float(kwargs['timeout']), remaining)
        return runner(argv, **kwargs)

    remaining = max(0.01, deadline - monotonic())
    return check_human_following(
        runtime_mode,
        runner=bounded_runner,
        environment=environment,
        topic_timeout=min(max(0.01, probe_timeout), remaining),
        tf_timeout=min(0.2, max(0.01, remaining)),
    )


def _wait_for_readiness(
        runtime_mode,
        child,
        runner,
        environment,
        probe_timeout,
        deadline,
        monotonic,
        sleeper):
    last_report = None
    while True:
        if child.poll() is not None:
            return ReadinessReport([
                CheckResult.fail(
                    'managed_process',
                    'launch exited before readiness with code {}'.format(
                        child.returncode)),
            ], stage=runtime_mode)
        last_report = _bounded_readiness(
            runtime_mode, runner, environment, probe_timeout, deadline,
            monotonic)
        remaining = deadline - monotonic()
        if last_report.overall is CheckStatus.PASS or remaining <= 0.0:
            return last_report
        sleeper(min(0.25, remaining))


@contextmanager
def _forward_signals(stop_callback):
    interrupted = {'value': False}
    previous = {}

    def forward(_signum, _frame):
        if interrupted['value']:
            return
        interrupted['value'] = True
        stop_callback()

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
            stop_callback()
    finally:
        for signum, handler in previous.items():
            try:
                signal.signal(signum, handler)
            except ValueError:
                pass


def _spawn(
        args, process_manager, runner, environment, deadline, monotonic):
    existing = process_manager.verified_state()
    if existing is not None:
        raise RuntimeError(
            'human_following_ctl already manages pid {} in {} mode'.format(
                existing.pid, existing.stage))
    selection = select_hardware(
        'sensors',
        args.hardware,
        runner=runner,
        environment=environment,
        timeout=max(0.01, args.probe_timeout),
        deadline=deadline,
        monotonic=monotonic,
    )
    command = build_launch_argv(args, selection)
    modules = tuple(
        name for name in ('camera', 'lidar', 'base', 'imu')
        if getattr(selection, name))
    child, state = process_manager.spawn_verified(
        command,
        stage=args.runtime_mode,
        owned_modules=modules,
        environment=environment,
    )
    return child, state


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
    """Run one human-following command with injectable process boundaries."""

    args = build_parser().parse_args(argv)
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    runner = runner or subprocess.run
    popen_factory = popen_factory or subprocess.Popen
    base_environment = os.environ if environ is None else environ
    environment = _environment(base_environment)
    monotonic = monotonic or time.monotonic
    sleeper = sleeper or time.sleep
    process_manager = _manager(
        manager, popen_factory, os_api, base_environment)

    if args.command == 'stop':
        state = process_manager.verified_state()
        if state is None:
            stdout.write(_status_line(process_manager))
            stdout.write('\n')
            return 2
        if state.stage not in ('shadow', 'active'):
            stderr.write(
                'Managed state has invalid runtime mode {!r}; no signal '
                'sent.\n'.format(state.stage))
            return 4
        if ordered_stop(
                process_manager, state, runner, environment,
                sleeper=sleeper):
            stdout.write('Stopped verified managed human-following process.\n')
            return 0
        stderr.write(
            'Fail-closed stop did not signal the managed process; verify '
            'disarm, emergency stop, /cmd_vel ownership, and process '
            'identity.\n')
        return 4

    if args.command == 'status':
        state = process_manager.verified_state()
        stdout.write(_status_line(process_manager))
        stdout.write('\n')
        if state is None:
            return 2
        if state.stage not in ('shadow', 'active'):
            return 4
        report = check_human_following(
            state.stage,
            runner=runner,
            environment=environment,
            topic_timeout=max(0.01, args.probe_timeout),
        )
        _render_report(report, stdout)
        return _exit_code(report)

    if args.command == 'doctor':
        report = check_human_following(
            args.runtime_mode,
            runner=runner,
            environment=environment,
            topic_timeout=max(0.01, args.probe_timeout),
        )
        _render_report(report, stdout)
        return _exit_code(report)

    try:
        validate_motion_request(args)
    except ValueError as error:
        stderr.write('{}\n'.format(error))
        return 4

    deadline = monotonic() + max(0.0, args.readiness_timeout)
    try:
        child, state = _spawn(
            args, process_manager, runner, environment, deadline, monotonic)
    except (HardwareProbeError, OSError, RuntimeError) as error:
        stderr.write('Failed to start human-following stack: {}\n'.format(
            error))
        return 4

    def stop_callback():
        return ordered_stop(
            process_manager, state, runner, environment, sleeper=sleeper)

    try:
        with _forward_signals(stop_callback) as interrupted:
            report = _wait_for_readiness(
                args.runtime_mode,
                child,
                runner,
                environment,
                args.probe_timeout,
                deadline,
                monotonic,
                sleeper,
            )
            if interrupted['value']:
                return 130
            _render_report(report, stdout)
            if report.overall is not CheckStatus.PASS:
                if not stop_callback():
                    stderr.write(
                        'Readiness failed and fail-closed cleanup could not '
                        'prove safe process termination.\n')
                return 4
            return_code = child.wait()
        if interrupted['value']:
            return 130
        process_manager.clear_if_owned(state)
        return 0 if return_code == 0 else 4
    except Exception as error:
        stopped = stop_callback()
        stderr.write('Human-following operation failed: {}'.format(error))
        if not stopped:
            stderr.write('; fail-closed cleanup did not signal the process')
        stderr.write('\n')
        return 4


__all__ = [
    'build_launch_argv',
    'build_parser',
    'call_trigger',
    'cmd_vel_has_zero_publishers',
    'default_state_path',
    'main',
    'observe_zero_twist',
    'ordered_stop',
    'validate_motion_request',
]
