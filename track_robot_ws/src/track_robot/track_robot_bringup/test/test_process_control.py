import json
import signal
import subprocess
import threading

import pytest

from track_robot_bringup.process_control import ProcessManager


EXPECTED = [
    'ros2', 'launch', 'track_robot_bringup',
    'semantic_search_live.launch.py', 'stage:=phase1',
]


class FakeOs:
    def __init__(self):
        self.start_ticks = {}
        self.cmdlines = {}
        self.process_groups = {}
        self.signals = []
        self.alive = set()
        self.now = 0.0
        self.stop_on_sigint = True

    def process_start_ticks(self, pid):
        return self.start_ticks[pid]

    def process_argv(self, pid):
        return self.cmdlines[pid]

    def getpgid(self, pid):
        return self.process_groups[pid]

    def killpg(self, pgid, signum):
        self.signals.append((pgid, signum))
        if signum == signal.SIGINT and self.stop_on_sigint:
            self.alive.clear()
        if signum == signal.SIGTERM:
            self.alive.clear()

    def process_exists(self, pid):
        return pid in self.alive

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


def _manager(tmp_path, fake_os, **kwargs):
    return ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, poll_interval=0.01, **kwargs)


def _write_live_state(manager, fake_os, pid=42, pgid=41):
    fake_os.start_ticks[pid] = 100
    fake_os.cmdlines[pid] = list(EXPECTED)
    fake_os.process_groups[pid] = pgid
    fake_os.alive.add(pid)
    return manager.write_state(
        pid=pid,
        pgid=pgid,
        start_ticks=100,
        command=EXPECTED,
        stage='phase1',
        owned_modules=('camera',),
        started_at='2026-07-23T12:00:00Z',
    )


def test_state_is_atomic_and_records_complete_identity(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)

    state = _write_live_state(manager, fake_os)

    payload = json.loads((tmp_path / 'state.json').read_text(encoding='utf-8'))
    assert payload == {
        'argv': EXPECTED,
        'owned_modules': ['camera'],
        'pgid': 41,
        'pid': 42,
        'stage': 'phase1',
        'start_ticks': 100,
        'started_at': '2026-07-23T12:00:00Z',
    }
    assert state.identity.pid == 42
    assert state.identity.argv == tuple(EXPECTED)
    assert list(tmp_path.glob('*.tmp')) == []


def test_stale_pid_is_not_signalled(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    _write_live_state(manager, fake_os)
    fake_os.start_ticks[42] = 101

    assert manager.stop_owned() is False
    assert fake_os.signals == []


def test_changed_cmdline_is_not_signalled(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    _write_live_state(manager, fake_os)
    fake_os.cmdlines[42] = ['sleep', '999']

    assert manager.stop_owned() is False
    assert fake_os.signals == []


def test_changed_process_group_is_not_signalled(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    _write_live_state(manager, fake_os)
    fake_os.process_groups[42] = 99

    assert manager.stop_owned() is False
    assert fake_os.signals == []


def test_cleanup_uses_actual_verified_process_group_sigint_first(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    _write_live_state(manager, fake_os)

    assert manager.stop_owned() is True
    assert fake_os.signals == [(41, signal.SIGINT)]
    assert not (tmp_path / 'state.json').exists()


def test_cleanup_waits_then_escalates_to_sigterm(tmp_path):
    fake_os = FakeOs()
    fake_os.stop_on_sigint = False
    manager = _manager(tmp_path, fake_os, interrupt_timeout=0.03)
    _write_live_state(manager, fake_os)

    assert manager.stop_owned() is True
    assert fake_os.signals == [
        (41, signal.SIGINT),
        (41, signal.SIGTERM),
    ]
    assert fake_os.now >= 0.03


def test_sigterm_permission_failure_returns_false_and_retains_state(tmp_path):
    fake_os = FakeOs()
    fake_os.stop_on_sigint = False

    def killpg(pgid, signum):
        fake_os.signals.append((pgid, signum))
        if signum == signal.SIGTERM:
            raise PermissionError('operation not permitted')

    fake_os.killpg = killpg
    manager = _manager(
        tmp_path,
        fake_os,
        interrupt_timeout=0.02,
        terminate_timeout=0.02,
    )
    _write_live_state(manager, fake_os)

    assert manager.stop_owned() is False
    assert fake_os.signals == [
        (41, signal.SIGINT),
        (41, signal.SIGTERM),
    ]
    assert (tmp_path / 'state.json').exists()
    assert 'operation not permitted' in manager.last_stop_error


def test_process_surviving_both_waits_returns_false_and_retains_state(tmp_path):
    fake_os = FakeOs()

    def survive_all_signals(pgid, signum):
        fake_os.signals.append((pgid, signum))

    fake_os.killpg = survive_all_signals
    manager = _manager(
        tmp_path,
        fake_os,
        interrupt_timeout=0.02,
        terminate_timeout=0.03,
    )
    _write_live_state(manager, fake_os)

    assert manager.stop_owned() is False
    assert fake_os.signals == [
        (41, signal.SIGINT),
        (41, signal.SIGTERM),
    ]
    assert fake_os.now >= 0.05
    assert (tmp_path / 'state.json').exists()
    assert 'remains alive' in manager.last_stop_error


def test_python_ros2_wrapper_is_the_same_semantic_command(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    _write_live_state(manager, fake_os)
    fake_os.cmdlines[42] = [
        '/usr/bin/python3', '/opt/ros/foxy/bin/ros2',
        *EXPECTED[1:],
    ]

    assert manager.stop_owned() is True
    assert fake_os.signals[0] == (41, signal.SIGINT)


@pytest.mark.parametrize('cmdline', [
    [
        '/usr/bin/env', 'python3', '/opt/ros/foxy/bin/ros2',
        *EXPECTED[1:],
    ],
    [
        '/usr/bin/bash', '/opt/ros/foxy/bin/ros2',
        *EXPECTED[1:],
    ],
    [
        '/usr/bin/python3', '-E', '/opt/ros/foxy/bin/ros2',
        *EXPECTED[1:],
    ],
])
def test_arbitrary_ros2_prefix_or_wrapper_is_not_owned(tmp_path, cmdline):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    _write_live_state(manager, fake_os)
    fake_os.cmdlines[42] = cmdline

    assert manager.stop_owned() is False
    assert fake_os.signals == []


def test_spawn_uses_argv_no_shell_and_a_new_session(tmp_path):
    fake_os = FakeOs()
    calls = []

    class Child:
        pid = 42

    def popen(argv, **kwargs):
        calls.append((argv, kwargs))
        fake_os.start_ticks[42] = 100
        fake_os.cmdlines[42] = list(argv)
        fake_os.process_groups[42] = 42
        fake_os.alive.add(42)
        return Child()

    manager = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=popen)
    child = manager.spawn(
        EXPECTED,
        stage='phase1',
        owned_modules=('camera',),
        environment={'PATH': '/bin', 'ROS_DOMAIN_ID': '20'},
    )

    assert child.pid == 42
    assert calls == [(
        EXPECTED,
        {
            'env': {'PATH': '/bin', 'ROS_DOMAIN_ID': '20'},
            'shell': False,
            'start_new_session': True,
        },
    )]
    assert manager.read_state().identity.pgid == 42


def test_spawn_verified_returns_child_and_captured_non_none_state(tmp_path):
    fake_os = FakeOs()

    class Child:
        pid = 42

    def popen(argv, **kwargs):
        fake_os.start_ticks[42] = 100
        fake_os.cmdlines[42] = list(argv)
        fake_os.process_groups[42] = 42
        fake_os.alive.add(42)
        return Child()

    manager = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=popen)

    child, state = manager.spawn_verified(
        EXPECTED,
        stage='phase1',
        owned_modules=('camera',),
        environment={'ROS_DOMAIN_ID': '20'},
    )

    assert child.pid == 42
    assert state is not None
    assert state.pid == 42
    assert state.pgid == 42
    assert state.start_ticks == 100
    assert state.argv == tuple(EXPECTED)


def test_stop_captured_cleans_up_after_state_file_disappears(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    state = _write_live_state(manager, fake_os)
    (tmp_path / 'state.json').unlink()

    assert manager.stop_captured(state) is True
    assert fake_os.signals == [(41, signal.SIGINT)]


@pytest.mark.parametrize('replacement', ['start_ticks', 'pgid', 'cmdline'])
def test_stop_captured_never_signals_changed_identity(tmp_path, replacement):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    state = _write_live_state(manager, fake_os)
    (tmp_path / 'state.json').unlink()
    if replacement == 'start_ticks':
        fake_os.start_ticks[42] = 101
    elif replacement == 'pgid':
        fake_os.process_groups[42] = 99
    else:
        fake_os.cmdlines[42] = ['sleep', '999']

    assert manager.stop_captured(state) is False
    assert fake_os.signals == []


def test_spawn_persistence_failure_only_signals_the_captured_identity(tmp_path):
    fake_os = FakeOs()

    class Child:
        pid = 42

    def popen(argv, **kwargs):
        fake_os.start_ticks[42] = 100
        fake_os.cmdlines[42] = list(argv)
        fake_os.process_groups[42] = 42
        fake_os.alive.add(42)
        return Child()

    manager = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=popen)

    def fail_write(**kwargs):
        raise OSError('disk full')

    manager.write_state = fail_write

    with pytest.raises(RuntimeError, match='state persistence failed'):
        manager.spawn(
            EXPECTED,
            stage='phase1',
            owned_modules=('camera',),
            environment={'ROS_DOMAIN_ID': '20'},
        )

    assert fake_os.signals == [(42, signal.SIGINT)]


@pytest.mark.parametrize('replacement', ['pid', 'pgid', 'cmdline'])
def test_spawn_persistence_failure_does_not_signal_reused_identity(
        tmp_path, replacement):
    fake_os = FakeOs()

    class Child:
        pid = 42

    def popen(argv, **kwargs):
        fake_os.start_ticks[42] = 100
        fake_os.cmdlines[42] = list(argv)
        fake_os.process_groups[42] = 42
        fake_os.alive.add(42)
        return Child()

    manager = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=popen)

    def fail_write(**kwargs):
        if replacement == 'pid':
            fake_os.start_ticks[42] = 101
        elif replacement == 'pgid':
            fake_os.process_groups[42] = 99
        else:
            fake_os.cmdlines[42] = ['sleep', '999']
        raise OSError('disk full')

    manager.write_state = fail_write

    with pytest.raises(RuntimeError, match='orphan risk'):
        manager.spawn(
            EXPECTED,
            stage='phase1',
            owned_modules=('camera',),
            environment={'ROS_DOMAIN_ID': '20'},
        )

    assert fake_os.signals == []


def test_spawn_persistence_failure_does_not_signal_when_identity_reread_fails(
        tmp_path):
    fake_os = FakeOs()

    class Child:
        pid = 42

    def popen(argv, **kwargs):
        fake_os.start_ticks[42] = 100
        fake_os.cmdlines[42] = list(argv)
        fake_os.process_groups[42] = 42
        fake_os.alive.add(42)
        return Child()

    manager = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=popen)

    def fail_write(**kwargs):
        fake_os.process_argv = lambda _pid: (_ for _ in ()).throw(
            OSError('proc read failed'))
        raise OSError('disk full')

    manager.write_state = fail_write

    with pytest.raises(RuntimeError, match='orphan risk'):
        manager.spawn(
            EXPECTED,
            stage='phase1',
            owned_modules=('camera',),
            environment={'ROS_DOMAIN_ID': '20'},
        )

    assert fake_os.signals == []


def test_spawn_does_not_signal_when_complete_identity_cannot_be_captured(
        tmp_path):
    fake_os = FakeOs()

    class Child:
        pid = 42

    def popen(argv, **kwargs):
        fake_os.start_ticks[42] = 100
        fake_os.cmdlines[42] = ['unexpected-wrapper', *argv]
        fake_os.process_groups[42] = 42
        fake_os.alive.add(42)
        return Child()

    manager = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=popen)

    with pytest.raises(RuntimeError, match='orphan risk'):
        manager.spawn(
            EXPECTED,
            stage='phase1',
            owned_modules=('camera',),
            environment={'ROS_DOMAIN_ID': '20'},
        )

    assert fake_os.signals == []
    assert not (tmp_path / 'state.json').exists()


def test_spawn_refuses_to_overwrite_live_owned_state(tmp_path):
    fake_os = FakeOs()
    manager = _manager(tmp_path, fake_os)
    _write_live_state(manager, fake_os)

    manager.popen_factory = lambda *a, **k: (_ for _ in ()).throw(
        AssertionError('must not spawn over live state'))

    with pytest.raises(RuntimeError, match='already manages pid 42'):
        manager.spawn(
            EXPECTED,
            stage='phase1',
            owned_modules=('camera',),
            environment={'ROS_DOMAIN_ID': '20'},
        )


def test_concurrent_spawn_cannot_pass_ownership_check_twice(tmp_path):
    fake_os = FakeOs()
    first_in_popen = threading.Event()
    second_started = threading.Event()
    release_first = threading.Event()
    second_spawned = threading.Event()
    errors = []

    class Child:
        def __init__(self, pid):
            self.pid = pid

    def first_popen(argv, **kwargs):
        first_in_popen.set()
        assert release_first.wait(1.0)
        fake_os.start_ticks[42] = 100
        fake_os.cmdlines[42] = list(argv)
        fake_os.process_groups[42] = 42
        fake_os.alive.add(42)
        return Child(42)

    def second_popen(argv, **kwargs):
        second_spawned.set()
        fake_os.start_ticks[43] = 101
        fake_os.cmdlines[43] = list(argv)
        fake_os.process_groups[43] = 43
        fake_os.alive.add(43)
        return Child(43)

    first = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=first_popen)
    second = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=second_popen)

    def run(manager, started=None):
        if started is not None:
            started.set()
        try:
            manager.spawn(
                EXPECTED,
                stage='phase1',
                owned_modules=('camera',),
                environment={'ROS_DOMAIN_ID': '20'},
            )
        except RuntimeError as error:
            errors.append(str(error))

    thread1 = threading.Thread(target=run, args=(first,))
    thread2 = threading.Thread(target=run, args=(second, second_started))
    thread1.start()
    assert first_in_popen.wait(1.0)
    thread2.start()
    assert second_started.wait(1.0)

    spawned_while_first_owned_check_open = second_spawned.wait(0.05)
    release_first.set()
    thread1.join(1.0)
    thread2.join(1.0)
    assert spawned_while_first_owned_check_open is False
    assert not thread1.is_alive()
    assert not thread2.is_alive()
    assert errors == ['controller already manages pid 42 (stage phase1)']
    assert second_spawned.is_set() is False
    assert first.read_state().pid == 42


def test_old_cleanup_cannot_unlink_state_written_by_new_spawn(tmp_path):
    fake_os = FakeOs()
    old_manager = _manager(tmp_path, fake_os)
    old_state = _write_live_state(old_manager, fake_os)
    fake_os.start_ticks[42] = 101

    cleanup_compared = threading.Event()
    release_cleanup = threading.Event()
    new_spawned = threading.Event()
    errors = []

    original_same_saved_state = old_manager._same_saved_state

    def pause_after_compare(expected):
        matches = original_same_saved_state(expected)
        cleanup_compared.set()
        assert release_cleanup.wait(1.0)
        return matches

    old_manager._same_saved_state = pause_after_compare

    class Child:
        pid = 43

    def popen(argv, **kwargs):
        fake_os.start_ticks[43] = 200
        fake_os.cmdlines[43] = list(argv)
        fake_os.process_groups[43] = 43
        fake_os.alive.add(43)
        new_spawned.set()
        return Child()

    new_manager = ProcessManager(
        tmp_path / 'state.json', os_api=fake_os, popen_factory=popen)

    def cleanup():
        old_manager.clear_if_owned(old_state)

    def spawn():
        try:
            new_manager.spawn(
                EXPECTED,
                stage='phase1',
                owned_modules=('camera',),
                environment={'ROS_DOMAIN_ID': '20'},
            )
        except Exception as error:
            errors.append(error)

    cleanup_thread = threading.Thread(target=cleanup)
    spawn_thread = threading.Thread(target=spawn)
    cleanup_thread.start()
    assert cleanup_compared.wait(1.0)
    spawn_thread.start()

    spawned_while_cleanup_compare_open = new_spawned.wait(0.05)
    release_cleanup.set()
    cleanup_thread.join(1.0)
    spawn_thread.join(1.0)

    assert spawned_while_cleanup_compare_open is False
    assert not cleanup_thread.is_alive()
    assert not spawn_thread.is_alive()
    assert errors == []
    assert new_manager.read_state().pid == 43


def test_default_state_path_uses_ros_home(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros-home'))

    manager = ProcessManager()

    assert manager.state_path == (
        tmp_path / 'ros-home' / 'track_robot_semantic_search'
        / 'managed_process.json')
