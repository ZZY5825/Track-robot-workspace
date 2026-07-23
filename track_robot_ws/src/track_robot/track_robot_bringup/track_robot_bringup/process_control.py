"""Verified ownership and bounded cleanup for managed bringup processes."""

import fcntl
import json
import os
import re
import signal
import subprocess
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from contextlib import contextmanager
from pathlib import Path


@dataclass(frozen=True)
class ProcessIdentity:
    """Kernel identity and exact requested argv for one managed process."""

    pid: int
    pgid: int
    start_ticks: int
    argv: tuple

    @property
    def command(self):
        """Compatibility name for callers that describe argv as a command."""

        return self.argv


@dataclass(frozen=True)
class OwnedProcessState:
    """Persistent ownership record for one process group."""

    identity: ProcessIdentity
    stage: str
    owned_modules: tuple
    started_at: str

    @property
    def pid(self):
        return self.identity.pid

    @property
    def pgid(self):
        return self.identity.pgid

    @property
    def start_ticks(self):
        return self.identity.start_ticks

    @property
    def argv(self):
        return self.identity.argv

    def as_dict(self):
        return {
            'pid': self.pid,
            'pgid': self.pgid,
            'start_ticks': self.start_ticks,
            'argv': list(self.argv),
            'stage': self.stage,
            'owned_modules': list(self.owned_modules),
            'started_at': self.started_at,
        }


class _RealOs:
    """Small injectable boundary around process-table and signalling calls."""

    @staticmethod
    def process_start_ticks(pid):
        stat = Path('/proc/{}/stat'.format(pid)).read_text(encoding='utf-8')
        command_end = stat.rfind(')')
        if command_end < 0:
            raise ValueError('invalid process stat for pid {}'.format(pid))
        fields_from_state = stat[command_end + 2:].split()
        return int(fields_from_state[19])

    @staticmethod
    def process_argv(pid):
        raw = Path('/proc/{}/cmdline'.format(pid)).read_bytes()
        return [
            value.decode(errors='replace')
            for value in raw.split(b'\0')
            if value
        ]

    @staticmethod
    def getpgid(pid):
        return os.getpgid(pid)

    @staticmethod
    def killpg(pgid, signum):
        os.killpg(pgid, signum)

    @staticmethod
    def process_exists(pid):
        try:
            stat = Path('/proc/{}/stat'.format(pid)).read_text(
                encoding='utf-8')
            command_end = stat.rfind(')')
            if command_end < 0:
                return False
            # A child that has exited but has not yet been waited for is no
            # longer signalable in a useful sense. Treat the zombie as gone so
            # a signal handler can return and reap it promptly.
            return stat[command_end + 2:].split()[0] != 'Z'
        except (OSError, IndexError):
            return False

    @staticmethod
    def monotonic():
        return time.monotonic()

    @staticmethod
    def sleep(duration):
        time.sleep(duration)


def default_state_path(environment=None):
    """Return the ownership file under ROS_HOME, or the standard ROS home."""

    environment = os.environ if environment is None else environment
    ros_home = environment.get('ROS_HOME')
    root = Path(ros_home).expanduser() if ros_home else Path.home() / '.ros'
    return root / 'track_robot_semantic_search' / 'managed_process.json'


class ProcessManager:
    """Start, persist, verify, and stop only the process group we own."""

    def __init__(
            self,
            state_path=None,
            os_api=None,
            popen_factory=None,
            interrupt_timeout=5.0,
            terminate_timeout=2.0,
            poll_interval=0.05):
        self.state_path = (
            Path(state_path) if state_path is not None else default_state_path())
        self.os_api = os_api or _RealOs()
        self.popen_factory = popen_factory or subprocess.Popen
        self.interrupt_timeout = max(0.0, float(interrupt_timeout))
        self.terminate_timeout = max(0.0, float(terminate_timeout))
        self.poll_interval = max(0.001, float(poll_interval))
        self.last_stop_error = None

    def _start_ticks(self, pid):
        function = getattr(self.os_api, 'process_start_ticks', None)
        if function is not None:
            return int(function(pid))
        return int(self.os_api.start_ticks[pid])

    def _argv(self, pid):
        function = getattr(self.os_api, 'process_argv', None)
        if function is not None:
            return tuple(str(value) for value in function(pid))
        return tuple(str(value) for value in self.os_api.cmdlines[pid])

    def _pgid(self, pid):
        function = getattr(self.os_api, 'getpgid', None)
        if function is not None:
            return int(function(pid))
        return int(self.os_api.process_groups[pid])

    def _process_exists(self, pid):
        function = getattr(self.os_api, 'process_exists', None)
        if function is not None:
            return bool(function(pid))
        try:
            self._start_ticks(pid)
            return True
        except (KeyError, OSError, ProcessLookupError):
            return False

    def _monotonic(self):
        function = getattr(self.os_api, 'monotonic', None)
        return float(function()) if function is not None else time.monotonic()

    def _sleep(self, duration):
        function = getattr(self.os_api, 'sleep', None)
        if function is not None:
            function(duration)
        else:
            time.sleep(duration)

    @contextmanager
    def _ownership_lock(self):
        """Serialize the ownership check and state-producing spawn."""

        self.state_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        lock_path = self.state_path.with_name(
            '{}.lock'.format(self.state_path.name))
        flags = os.O_CREAT | os.O_RDWR
        flags |= getattr(os, 'O_CLOEXEC', 0)
        flags |= getattr(os, 'O_NOFOLLOW', 0)
        descriptor = os.open(str(lock_path), flags, 0o600)
        try:
            os.fchmod(descriptor, 0o600)
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)

    @staticmethod
    def _semantic_command_matches(expected, actual):
        """Accept an exact argv or the normal Python wrapper around ``ros2``."""

        expected = tuple(expected)
        actual = tuple(actual)
        if not expected or not actual:
            return False
        if actual == expected:
            return True
        expected_program = os.path.basename(expected[0])
        python_program = os.path.basename(actual[0])
        return (
            expected_program == 'ros2'
            and re.fullmatch(
                r'python(?:\d+(?:\.\d+)*)?', python_program) is not None
            and len(actual) >= 2
            and os.path.basename(actual[1]) == expected_program
            and actual[2:] == expected[1:]
        )

    @staticmethod
    def _state_from_dict(payload):
        required = {
            'pid', 'pgid', 'start_ticks', 'argv', 'stage',
            'owned_modules', 'started_at',
        }
        if not isinstance(payload, dict) or set(payload) != required:
            raise ValueError('managed process state has an invalid schema')
        if (type(payload['pid']) is not int
                or type(payload['pgid']) is not int
                or type(payload['start_ticks']) is not int):
            raise ValueError('managed process identity must use integers')
        if (not isinstance(payload['argv'], list) or not payload['argv']
                or any(not isinstance(value, str) for value in payload['argv'])):
            raise ValueError('managed process argv must be a non-empty string list')
        if not isinstance(payload['stage'], str):
            raise ValueError('managed process stage must be a string')
        if (not isinstance(payload['owned_modules'], list)
                or any(not isinstance(value, str)
                       for value in payload['owned_modules'])):
            raise ValueError('managed process modules must be a string list')
        if not isinstance(payload['started_at'], str):
            raise ValueError('managed process start time must be a string')
        return OwnedProcessState(
            identity=ProcessIdentity(
                pid=payload['pid'],
                pgid=payload['pgid'],
                start_ticks=payload['start_ticks'],
                argv=tuple(payload['argv']),
            ),
            stage=payload['stage'],
            owned_modules=tuple(payload['owned_modules']),
            started_at=payload['started_at'],
        )

    def read_state(self):
        """Read a valid ownership record, returning ``None`` when unavailable."""

        try:
            payload = json.loads(self.state_path.read_text(encoding='utf-8'))
            return self._state_from_dict(payload)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return None

    def write_state(
            self,
            pid,
            start_ticks,
            command=None,
            *,
            argv=None,
            pgid=None,
            stage='',
            owned_modules=(),
            started_at=None):
        """Atomically persist all fields needed to prove later ownership."""

        requested = argv if argv is not None else command
        if requested is None:
            raise ValueError('managed process argv is required')
        requested = tuple(str(value) for value in requested)
        if not requested:
            raise ValueError('managed process argv is required')
        identity = ProcessIdentity(
            pid=int(pid),
            pgid=self._pgid(pid) if pgid is None else int(pgid),
            start_ticks=int(start_ticks),
            argv=requested,
        )
        state = OwnedProcessState(
            identity=identity,
            stage=str(stage),
            owned_modules=tuple(str(value) for value in owned_modules),
            started_at=started_at or datetime.now(timezone.utc).replace(
                microsecond=0).isoformat().replace('+00:00', 'Z'),
        )

        self.state_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix='{}.'.format(self.state_path.name),
            suffix='.tmp',
            dir=str(self.state_path.parent),
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(state.as_dict(), stream, sort_keys=True)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(str(temporary), 0o600)
            os.replace(str(temporary), str(self.state_path))
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        return state

    def spawn_verified(self, argv, stage, owned_modules, environment):
        """Spawn and return both the child and its captured ownership state."""

        with self._ownership_lock():
            existing = self.verified_state()
            if existing is not None:
                raise RuntimeError(
                    'controller already manages pid {} (stage {})'.format(
                        existing.pid, existing.stage))
            command = [str(value) for value in argv]
            child = self.popen_factory(
                command,
                env=dict(environment),
                shell=False,
                start_new_session=True,
            )
            try:
                identity = ProcessIdentity(
                    pid=child.pid,
                    pgid=self._pgid(child.pid),
                    start_ticks=self._start_ticks(child.pid),
                    argv=tuple(command),
                )
                if not self._semantic_command_matches(
                        identity.argv, self._argv(identity.pid)):
                    raise RuntimeError(
                        'spawned process command does not match requested argv')
            except (
                    KeyError, OSError, ProcessLookupError,
                    RuntimeError, ValueError) as error:
                raise RuntimeError(
                    'spawned pid {} but complete identity capture failed; '
                    'orphan risk: no signal sent'.format(child.pid)
                ) from error

            try:
                state = self.write_state(
                    pid=identity.pid,
                    pgid=identity.pgid,
                    start_ticks=identity.start_ticks,
                    argv=identity.argv,
                    stage=stage,
                    owned_modules=owned_modules,
                )
            except Exception as error:
                try:
                    current_matches = (
                        self._start_ticks(identity.pid) == identity.start_ticks
                        and self._pgid(identity.pid) == identity.pgid
                        and self._semantic_command_matches(
                            identity.argv, self._argv(identity.pid))
                    )
                except (KeyError, OSError, ProcessLookupError, ValueError):
                    current_matches = False
                if not current_matches:
                    raise RuntimeError(
                        'managed process state persistence failed for pid {}; '
                        'orphan risk: identity could not be reverified, so no '
                        'signal was sent'.format(identity.pid)
                    ) from error
                try:
                    self.os_api.killpg(identity.pgid, signal.SIGINT)
                except (OSError, ProcessLookupError) as signal_error:
                    raise RuntimeError(
                        'managed process state persistence failed for pid {}; '
                        'orphan risk: verified cleanup signal failed: {}'.format(
                            identity.pid, signal_error)
                    ) from error
                raise RuntimeError(
                    'managed process state persistence failed for pid {}; '
                    'verified process group received SIGINT'.format(
                        identity.pid)
                ) from error
            return child, state

    def spawn(self, argv, stage, owned_modules, environment):
        """Backward-compatible child-only wrapper around ``spawn_verified``."""

        child, _state = self.spawn_verified(
            argv,
            stage=stage,
            owned_modules=owned_modules,
            environment=environment,
        )
        return child

    def verified_state(self):
        """Return state only when the live process still has the saved identity."""

        state = self.read_state()
        if state is None:
            return None
        try:
            if self._start_ticks(state.pid) != state.start_ticks:
                return None
            if self._pgid(state.pid) != state.pgid:
                return None
            if not self._semantic_command_matches(
                    state.argv, self._argv(state.pid)):
                return None
            return state
        except (KeyError, OSError, ProcessLookupError, ValueError):
            return None

    def _same_saved_state(self, expected):
        current = self.read_state()
        return current is not None and current.identity == expected.identity

    def _remove_state_if(self, expected):
        with self._ownership_lock():
            if not self._same_saved_state(expected):
                return
            try:
                self.state_path.unlink()
            except FileNotFoundError:
                pass

    def _wait_until_identity_gone(self, state, timeout):
        deadline = self._monotonic() + timeout
        while True:
            if not self._process_exists(state.pid):
                return True
            current = self.verified_state()
            if current is None or current.identity != state.identity:
                return True
            now = self._monotonic()
            if now >= deadline:
                return False
            self._sleep(min(self.poll_interval, deadline - now))

    def _identity_matches(self, state):
        """Revalidate a captured identity directly against the process table."""

        try:
            return (
                self._process_exists(state.pid)
                and self._start_ticks(state.pid) == state.start_ticks
                and self._pgid(state.pid) == state.pgid
                and self._semantic_command_matches(
                    state.argv, self._argv(state.pid))
            )
        except (KeyError, OSError, ProcessLookupError, ValueError):
            return False

    def _wait_until_captured_gone(self, state, timeout):
        deadline = self._monotonic() + timeout
        while True:
            if not self._identity_matches(state):
                return True
            now = self._monotonic()
            if now >= deadline:
                return False
            self._sleep(min(self.poll_interval, deadline - now))

    def stop_captured(self, state):
        """Stop a captured identity without trusting the persistent state file."""

        self.last_stop_error = None
        if state is None or not self._identity_matches(state):
            return False
        try:
            self.os_api.killpg(state.pgid, signal.SIGINT)
        except (OSError, ProcessLookupError) as error:
            if self._identity_matches(state):
                self.last_stop_error = (
                    'failed to send SIGINT to captured process group {}: {}; '
                    'process remains alive'.format(state.pgid, error))
                return False
            self._remove_state_if(state)
            return True

        gone = self._wait_until_captured_gone(
            state, self.interrupt_timeout)
        terminate_error = None
        if not gone:
            if not self._identity_matches(state):
                self._remove_state_if(state)
                return True
            try:
                self.os_api.killpg(state.pgid, signal.SIGTERM)
            except (OSError, ProcessLookupError) as error:
                terminate_error = error
            gone = self._wait_until_captured_gone(
                state, self.terminate_timeout)

        if gone:
            self._remove_state_if(state)
            return True
        detail = (
            '; SIGTERM failed: {}'.format(terminate_error)
            if terminate_error is not None else '')
        self.last_stop_error = (
            'captured process group {} remains alive after SIGINT and '
            'SIGTERM waits{}'.format(state.pgid, detail))
        return False

    def clear_if_owned(self, state):
        """Remove state after a child has exited, without deleting newer state."""

        self._remove_state_if(state)

    def stop_owned(self):
        """Verify ownership, then use SIGINT and bounded SIGTERM escalation."""

        self.last_stop_error = None
        state = self.verified_state()
        if state is None:
            return False
        try:
            self.os_api.killpg(state.pgid, signal.SIGINT)
        except (OSError, ProcessLookupError) as error:
            current = self.verified_state()
            if current is not None and current.identity == state.identity:
                self.last_stop_error = (
                    'failed to send SIGINT to verified process group {}: {}; '
                    'process remains alive'.format(state.pgid, error))
                return False
            self._remove_state_if(state)
            return False

        gone = self._wait_until_identity_gone(state, self.interrupt_timeout)
        terminate_error = None
        if not gone:
            # Re-verify immediately before escalating. PID/PGID reuse must never
            # turn cleanup into a signal sent to an unrelated process.
            current = self.verified_state()
            if current is None or current.identity != state.identity:
                self._remove_state_if(state)
                return True
            try:
                self.os_api.killpg(state.pgid, signal.SIGTERM)
            except (OSError, ProcessLookupError) as error:
                terminate_error = error
            gone = self._wait_until_identity_gone(
                state, self.terminate_timeout)

        if gone or self.verified_state() is None:
            self._remove_state_if(state)
            return True
        detail = (
            '; SIGTERM failed: {}'.format(terminate_error)
            if terminate_error is not None else '')
        self.last_stop_error = (
            'verified process group {} remains alive after SIGINT and '
            'SIGTERM waits{}'.format(state.pgid, detail))
        return False


__all__ = [
    'OwnedProcessState',
    'ProcessIdentity',
    'ProcessManager',
    'default_state_path',
]
