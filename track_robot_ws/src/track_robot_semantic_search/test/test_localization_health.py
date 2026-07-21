import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time as TimeMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from track_robot_interfaces.msg import SemanticLocalizationState

import track_robot_semantic_search.localization_health_node as node_module
from track_robot_semantic_search.localization_health import (
    LocalizationDecision,
    LocalizationHealthEvaluator,
    LocalizationSample,
    MemoryMode,
)
from track_robot_semantic_search.localization_health_node import (
    LocalizationHealthNode,
    message_is_fresh,
    odometry_transform_available,
    stamp_ns,
    world_values,
)


def sample(stamp_ns, **overrides):
    values = {
        'stamp_ns': stamp_ns,
        'local_pose_fresh': True,
        'imu_fresh': True,
        'local_tf_available': True,
        'world_pose_fresh': False,
        'world_tf_available': False,
        'world_pose_stamp_ns': -1,
        'world_covariance_xy_m2': math.inf,
        'world_yaw_variance_rad2': math.inf,
        'world_x': math.nan,
        'world_y': math.nan,
        'world_yaw': math.nan,
    }
    values.update(overrides)
    return LocalizationSample(**values)


def world_sample(
        stamp_ns, x=0.0, yaw=0.0, world_stamp_ns=None, **overrides):
    values = {
        'world_pose_fresh': True,
        'world_tf_available': True,
        'world_pose_stamp_ns': (
            stamp_ns if world_stamp_ns is None else world_stamp_ns),
        'world_covariance_xy_m2': 0.04,
        'world_yaw_variance_rad2': 0.01,
        'world_x': x,
        'world_y': 0.0,
        'world_yaw': yaw,
    }
    values.update(overrides)
    return sample(stamp_ns, **values)


def evaluator(world_enabled=True):
    return LocalizationHealthEvaluator(
        world_enabled=world_enabled,
        world_stable_samples=3,
        maximum_world_xy_variance_m2=0.25,
        maximum_world_yaw_variance_rad2=0.12,
        maximum_world_jump_m=0.50,
        maximum_world_yaw_jump_rad=0.26,
    )


def odometry(
        stamp, parent_frame='map', child_frame='base_link',
        x_variance=0.04, y_variance=0.04):
    message = Odometry()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp, 1000000000)
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = x_variance
    message.pose.covariance[7] = y_variance
    message.pose.covariance[35] = 0.01
    return message


class RecordingTransformBuffer:
    def __init__(self, available_stamps):
        self.available_stamps = set(available_stamps)
        self.calls = []

    def can_transform(self, target_frame, source_frame, query_time):
        call = (target_frame, source_frame, query_time.nanoseconds)
        self.calls.append(call)
        return query_time.nanoseconds in self.available_stamps


class FixedNow:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        message = TimeMessage()
        message.sec, message.nanosec = divmod(
            self.nanoseconds, 1000000000)
        return message


class FixedClock:
    def __init__(self, nanoseconds):
        self.current = FixedNow(nanoseconds)

    def now(self):
        return self.current


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PublishHarness:
    def __init__(self, now_ns):
        self.clock = FixedClock(now_ns)
        self.evaluator = evaluator()
        for stamp in (1, 2, 3):
            self.evaluator.update(world_sample(stamp))
        self.imu_timeout = 0.25
        self.local_timeout = 0.30
        self.world_timeout = 0.30
        self.freshness_time_base = 'source_clock'
        self._source_stamps = {
            'imu': None,
            'local': None,
            'world': None,
        }
        self._source_timestamp_rollback = False
        self._world_source_timestamp_rollback = False
        self.local_frame = 'odom'
        self.world_frame = 'map'
        self.base_frame = 'base_link'
        local_pose = odometry(
            now_ns, parent_frame='odom', child_frame='base_link')
        imu = Imu()
        imu.header.stamp = local_pose.header.stamp
        world_pose = odometry(0)
        world_pose.header.stamp.sec = -1
        self.local_pose = (local_pose, now_ns)
        self.imu = (imu, now_ns)
        self.world_pose = (world_pose, now_ns)
        self.tf_buffer = RecordingTransformBuffer({now_ns})
        self.publisher = CapturingPublisher()
        self.state_publisher = CapturingPublisher()
        self._last_published_mode = None

    def get_clock(self):
        return self.clock

    def _fresh(self, received, timeout, now_ns):
        if received is None:
            return False
        message, arrival_ns = received
        return message_is_fresh(
            message, now_ns, timeout,
            time_base=self.freshness_time_base,
            arrival_ns=arrival_ns)

    def _world_values(self):
        return world_values(self.world_pose[0])


def update_from_world_odometry(health, message, transform_buffer):
    covariance, yaw_variance, x, y, yaw = world_values(message)
    return health.update(world_sample(
        stamp_ns(message.header.stamp),
        world_tf_available=odometry_transform_available(
            transform_buffer, message, 'map', 'base_link'),
        world_covariance_xy_m2=covariance,
        world_yaw_variance_rad2=yaw_variance,
        world_x=x,
        world_y=y,
        world_yaw=yaw,
    ))


def test_missing_local_pose_is_observation_only():
    decision = evaluator().update(
        sample(1, local_pose_fresh=False))
    assert decision.mode == MemoryMode.OBSERVATION_ONLY
    assert decision.reason == 'local_pose_stale'


def test_healthy_local_pose_without_world_is_local_session():
    decision = evaluator().update(sample(1))
    assert decision.mode == MemoryMode.LOCAL_SESSION
    assert decision.epoch_id == 1


def test_world_requires_explicit_enable_and_three_stable_samples():
    disabled = evaluator(world_enabled=False)
    assert disabled.update(world_sample(1)).mode == MemoryMode.LOCAL_SESSION
    enabled = evaluator()
    assert enabled.update(world_sample(1)).reason == 'world_stabilizing'
    assert enabled.update(world_sample(1)).mode == MemoryMode.LOCAL_SESSION
    assert enabled.update(world_sample(2)).mode == MemoryMode.LOCAL_SESSION
    assert enabled.update(world_sample(3)).mode == MemoryMode.WORLD


def test_world_jump_closes_epoch_and_does_not_auto_associate():
    health = evaluator()
    for stamp in (1, 2, 3):
        decision = health.update(world_sample(stamp))
    assert decision.mode == MemoryMode.WORLD
    jumped = health.update(world_sample(4, x=1.0))
    assert jumped.mode == MemoryMode.LOCAL_SESSION
    assert jumped.epoch_changed is True
    assert jumped.epoch_id == 2
    assert jumped.reason == 'world_pose_jump'


def test_timestamp_rollback_forces_one_observation_only_sample():
    health = evaluator()
    assert health.update(sample(10)).mode == MemoryMode.LOCAL_SESSION
    rolled = health.update(sample(9))
    assert rolled.mode == MemoryMode.OBSERVATION_ONLY
    assert rolled.epoch_id == 2
    assert rolled.epoch_changed is True
    assert rolled.reason == 'timestamp_rollback'


def test_explicit_source_rollback_starts_new_epoch_in_arrival_mode():
    health = evaluator()
    assert health.update(sample(
        100, source_timestamp_rollback=False)).mode == (
            MemoryMode.LOCAL_SESSION)

    rolled = health.update(sample(
        200,
        source_timestamp_rollback=True,
        world_source_timestamp_rollback=True,
    ))

    assert rolled.mode == MemoryMode.OBSERVATION_ONLY
    assert rolled.reason == 'timestamp_rollback'
    assert rolled.epoch_changed is True
    assert rolled.epoch_id == 2


@pytest.mark.parametrize(
    'world_enabled, overrides, expected_reason',
    [
        (False, {}, 'world_disabled'),
        (True, {'world_pose_fresh': False}, 'world_pose_unhealthy'),
    ],
)
def test_world_source_rollback_does_not_preempt_unhealthy_world(
        world_enabled, overrides, expected_reason):
    decision = evaluator(world_enabled=world_enabled).update(world_sample(
        1,
        world_source_timestamp_rollback=True,
        **overrides
    ))

    assert decision.mode == MemoryMode.LOCAL_SESSION
    assert decision.reason == expected_reason
    assert decision.epoch_id == 1
    assert decision.epoch_changed is False


def test_repeated_world_stamp_preserves_world_without_counting_again():
    health = evaluator()
    for stamp in (1, 2, 3):
        decision = health.update(world_sample(stamp))
    assert decision.mode == MemoryMode.WORLD

    repeated = health.update(world_sample(3))

    assert repeated.mode == MemoryMode.WORLD
    assert repeated.epoch_id == 1
    assert repeated.epoch_changed is False


def test_world_timestamp_rollback_closes_epoch():
    health = evaluator()
    for stamp in (1, 2, 3):
        health.update(world_sample(stamp))

    rolled = health.update(world_sample(4, world_stamp_ns=2))

    assert rolled.mode == MemoryMode.LOCAL_SESSION
    assert rolled.reason == 'world_timestamp_rollback'
    assert rolled.epoch_id == 2
    assert rolled.epoch_changed is True


def test_world_jump_stamp_is_not_counted_in_new_epoch_stability():
    health = evaluator()
    for stamp in (1, 2, 3):
        health.update(world_sample(stamp))
    health.update(world_sample(4, x=1.0))

    assert health.update(world_sample(4, x=1.0)).mode == (
        MemoryMode.LOCAL_SESSION)
    assert health.update(world_sample(5, x=1.0)).mode == (
        MemoryMode.LOCAL_SESSION)
    assert health.update(world_sample(6, x=1.0)).mode == (
        MemoryMode.LOCAL_SESSION)
    assert health.update(world_sample(7, x=1.0)).mode == MemoryMode.WORLD


def test_world_timestamp_rollback_immediately_after_jump_is_detected():
    health = evaluator()
    for stamp in (1, 2, 3):
        health.update(world_sample(stamp))
    jumped = health.update(world_sample(4, x=1.0))
    assert jumped.reason == 'world_pose_jump'

    rolled = health.update(
        world_sample(5, x=1.0, world_stamp_ns=3))

    assert rolled.reason == 'world_timestamp_rollback'
    assert rolled.epoch_id == 3
    assert rolled.epoch_changed is True


def test_negative_world_stamp_during_stabilizing_starts_new_epoch():
    health = evaluator()
    assert health.update(world_sample(1)).reason == 'world_stabilizing'

    invalid = health.update(world_sample(2, world_stamp_ns=-1))

    assert invalid.mode == MemoryMode.LOCAL_SESSION
    assert invalid.reason == 'world_stamp_invalid'
    assert invalid.epoch_id == 2
    assert invalid.epoch_changed is True
    assert health.update(world_sample(3)).mode == MemoryMode.LOCAL_SESSION
    assert health.update(world_sample(4)).mode == MemoryMode.LOCAL_SESSION
    assert health.update(world_sample(5)).mode == MemoryMode.WORLD


def test_negative_world_stamp_from_world_starts_new_epoch():
    health = evaluator()
    for stamp in (1, 2, 3):
        health.update(world_sample(stamp))

    invalid = health.update(world_sample(4, world_stamp_ns=-1))

    assert invalid.mode == MemoryMode.LOCAL_SESSION
    assert invalid.reason == 'world_stamp_invalid'
    assert invalid.epoch_id == 2
    assert invalid.epoch_changed is True
    assert health.update(world_sample(5)).mode == MemoryMode.LOCAL_SESSION


@pytest.mark.parametrize(
    'overrides, reason',
    [
        ({'imu_fresh': False}, 'imu_stale'),
        ({'local_tf_available': False}, 'local_tf_unavailable'),
    ],
)
def test_local_health_failures_are_observation_only(overrides, reason):
    decision = evaluator().update(sample(1, **overrides))

    assert decision.mode == MemoryMode.OBSERVATION_ONLY
    assert decision.reason == reason


def test_world_yaw_jump_uses_wrapped_angle_difference():
    health = evaluator()

    assert health.update(world_sample(1, yaw=math.pi - 0.01)).mode == (
        MemoryMode.LOCAL_SESSION)
    assert health.update(world_sample(2, yaw=-math.pi + 0.01)).mode == (
        MemoryMode.LOCAL_SESSION)
    assert health.update(world_sample(3, yaw=-math.pi + 0.02)).mode == (
        MemoryMode.WORLD)


@pytest.mark.parametrize('covariance', [math.nan, math.inf, -0.01])
def test_invalid_world_covariance_never_unlocks_world(covariance):
    health = evaluator()

    for stamp in (1, 2, 3):
        decision = health.update(world_sample(
            stamp, world_covariance_xy_m2=covariance))

    assert decision.mode == MemoryMode.LOCAL_SESSION
    assert decision.reason == 'world_pose_unhealthy'


@pytest.mark.parametrize(
    'axis, invalid_variance',
    [
        ('y', math.nan),
        ('y', -math.inf),
        ('y', -0.01),
        ('x', math.nan),
        ('x', -math.inf),
        ('x', -0.01),
    ],
)
def test_node_invalid_xy_diagonal_never_unlocks_world(
        axis, invalid_variance):
    health = evaluator()
    transform_buffer = RecordingTransformBuffer({1, 2, 3})

    for stamp in (1, 2, 3):
        variances = {'x_variance': 0.04, 'y_variance': 0.04}
        variances[axis + '_variance'] = invalid_variance
        message = odometry(stamp, **variances)
        decision = update_from_world_odometry(
            health, message, transform_buffer)

    assert decision.mode == MemoryMode.LOCAL_SESSION
    assert decision.reason == 'world_pose_unhealthy'


@pytest.mark.parametrize(
    'parent_frame, child_frame',
    [('wrong_map', 'base_link'), ('map', 'wrong_base')],
)
def test_wrong_world_odometry_frame_never_unlocks_world(
        parent_frame, child_frame):
    health = evaluator()
    transform_buffer = RecordingTransformBuffer({1, 2, 3})

    for stamp in (1, 2, 3):
        message = odometry(
            stamp,
            parent_frame=parent_frame,
            child_frame=child_frame,
        )
        decision = update_from_world_odometry(
            health, message, transform_buffer)

    assert decision.mode == MemoryMode.LOCAL_SESSION
    assert decision.reason == 'world_pose_unhealthy'
    assert transform_buffer.calls == []


def test_latest_tf_does_not_substitute_for_exact_world_pose_stamp():
    health = evaluator()
    transform_buffer = RecordingTransformBuffer({0})

    for stamp in (1, 2, 3):
        decision = update_from_world_odometry(
            health, odometry(stamp), transform_buffer)

    assert decision.mode == MemoryMode.LOCAL_SESSION
    assert decision.reason == 'world_pose_unhealthy'
    assert transform_buffer.calls == [
        ('map', 'base_link', 1),
        ('map', 'base_link', 2),
        ('map', 'base_link', 3),
    ]


def test_exact_stamp_tf_and_matching_frames_can_unlock_world():
    health = evaluator()
    transform_buffer = RecordingTransformBuffer({
        2000000003, 2000000004, 2000000005})

    for stamp in (2000000003, 2000000004, 2000000005):
        decision = update_from_world_odometry(
            health, odometry(stamp), transform_buffer)

    assert decision.mode == MemoryMode.WORLD
    assert transform_buffer.calls[0] == (
        'map', 'base_link', 2000000003)


def test_message_freshness_uses_message_stamp_and_rejects_old_or_future():
    message = odometry(2000000003)

    assert message_is_fresh(message, 2250000003, 0.25) is True
    assert message_is_fresh(message, 2250000004, 0.25) is False
    assert message_is_fresh(message, 2000000002, 0.25) is False

    stale = evaluator().update(sample(
        2250000004,
        local_pose_fresh=message_is_fresh(
            message, 2250000004, 0.25),
    ))
    assert stale.mode == MemoryMode.OBSERVATION_ONLY
    assert stale.reason == 'local_pose_stale'


def test_arrival_freshness_accepts_historical_source_stamp():
    message = odometry(100)

    assert message_is_fresh(
        message,
        now_ns=10200000000,
        timeout_sec=0.25,
        time_base='arrival_monotonic',
        arrival_ns=10000000000,
    ) is True


def test_arrival_freshness_rejects_missing_future_or_stale_arrival():
    message = odometry(100)

    assert message_is_fresh(
        message, 10, 0.25, 'arrival_monotonic', None) is False
    assert message_is_fresh(
        message, 9, 0.25, 'arrival_monotonic', 10) is False
    assert message_is_fresh(
        message, 300000001, 0.25,
        'arrival_monotonic', 1) is False


def test_message_freshness_rejects_unknown_time_base():
    with pytest.raises(ValueError, match='freshness_time_base'):
        message_is_fresh(
            odometry(100), 100, 0.25,
            time_base='bag_clock', arrival_ns=100)


def test_node_freshness_parameter_validation_rejects_unknown_mode():
    validator = getattr(
        node_module, '_validate_freshness_time_base', None)
    assert validator is not None
    assert validator('source_clock') == 'source_clock'
    assert validator('arrival_monotonic') == 'arrival_monotonic'
    with pytest.raises(ValueError, match='freshness_time_base'):
        validator('bag_clock')


def callback_harness():
    return SimpleNamespace(
        imu=None,
        local_pose=None,
        world_pose=None,
        _source_stamps={'imu': None, 'local': None, 'world': None},
        _source_timestamp_rollback=False,
        _world_source_timestamp_rollback=False,
    )


def imu_message(stamp):
    message = Imu()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp, 1000000000)
    return message


def test_callbacks_capture_monotonic_arrival_for_every_topic(monkeypatch):
    arrivals = iter((101, 102, 103))
    monkeypatch.setattr(
        node_module, 'time',
        SimpleNamespace(monotonic_ns=lambda: next(arrivals)),
        raising=False)
    node = callback_harness()
    imu = imu_message(10)
    local = odometry(20)
    world = odometry(30)

    LocalizationHealthNode._imu(node, imu)
    LocalizationHealthNode._local(node, local)
    LocalizationHealthNode._world(node, world)

    assert node.imu == (imu, 101)
    assert node.local_pose == (local, 102)
    assert node.world_pose == (world, 103)
    assert node._source_stamps == {
        'imu': 10, 'local': 20, 'world': 30}


def test_callbacks_detect_imu_and_local_source_rollback_without_stealing_world(
        monkeypatch):
    monkeypatch.setattr(
        node_module, 'time',
        SimpleNamespace(monotonic_ns=lambda: 1000),
        raising=False)
    node = callback_harness()

    LocalizationHealthNode._world(node, odometry(30))
    LocalizationHealthNode._world(node, odometry(29))
    assert node._source_stamps['world'] == 29
    assert node._source_timestamp_rollback is False
    assert node._world_source_timestamp_rollback is True

    LocalizationHealthNode._imu(node, imu_message(20))
    LocalizationHealthNode._imu(node, imu_message(19))
    assert node._source_timestamp_rollback is True
    assert node._world_source_timestamp_rollback is True

    node._source_timestamp_rollback = False
    LocalizationHealthNode._local(node, odometry(10))
    LocalizationHealthNode._local(node, odometry(9))
    assert node._source_timestamp_rollback is True


@pytest.mark.parametrize(
    'parent_frame, stamp_sec, stamp_nanosec',
    [
        ('map', -1, 0),
        ('odom', -1, 0),
        ('map', 0, 1000000000),
    ],
)
def test_invalid_odometry_stamp_rejects_tf_without_query(
        parent_frame, stamp_sec, stamp_nanosec):
    message = odometry(
        0, parent_frame=parent_frame, child_frame='base_link')
    message.header.stamp.sec = stamp_sec
    message.header.stamp.nanosec = stamp_nanosec
    transform_buffer = RecordingTransformBuffer({0})

    available = odometry_transform_available(
        transform_buffer, message, parent_frame, 'base_link')

    assert available is False
    assert transform_buffer.calls == []


def test_publish_negative_world_stamp_safely_downgrades_epoch():
    node = PublishHarness(now_ns=10000000000)

    LocalizationHealthNode._publish(node)

    assert node.tf_buffer.calls == [
        ('odom', 'base_link', 10000000000),
    ]
    assert len(node.publisher.messages) == 1
    status = node.publisher.messages[0].status[0]
    values = {item.key: item.value for item in status.values}
    assert status.message == 'world_pose_unhealthy'
    assert values['memory_mode'] == 'LOCAL_SESSION'
    assert values['epoch_id'] == '2'
    assert values['epoch_changed'] == 'true'
    assert len(node.state_publisher.messages) == 1
    state = node.state_publisher.messages[0]
    assert isinstance(state, SemanticLocalizationState)
    assert state.memory_mode == SemanticLocalizationState.MEMORY_LOCAL_SESSION
    assert state.localization_epoch_id == 2
    assert state.canonical_frame_id == 'odom'
    assert state.local_healthy is True
    assert state.world_healthy is False
    assert state.epoch_changed is True
    assert state.mode_changed is False
    assert state.reason == 'world_pose_unhealthy'


def test_publish_consumes_world_rollback_between_timer_ticks(monkeypatch):
    now_ns = 10000000000
    node = PublishHarness(now_ns=now_ns)
    monkeypatch.setattr(
        node_module, 'time',
        SimpleNamespace(monotonic_ns=lambda: now_ns),
        raising=False)

    LocalizationHealthNode._world(node, odometry(10000000000))
    LocalizationHealthNode._world(node, odometry(9900000000))
    node.tf_buffer.available_stamps.add(9900000000)

    LocalizationHealthNode._publish(node)

    status = node.publisher.messages[0].status[0]
    values = {item.key: item.value for item in status.values}
    assert status.message == 'world_timestamp_rollback'
    assert values['memory_mode'] == 'LOCAL_SESSION'
    assert values['epoch_id'] == '2'
    assert values['epoch_changed'] == 'true'
    state = node.state_publisher.messages[0]
    assert state.memory_mode == SemanticLocalizationState.MEMORY_LOCAL_SESSION
    assert state.epoch_changed is True


def test_typed_localization_state_marks_mode_change_after_first_publish():
    node = PublishHarness(now_ns=10000000000)

    LocalizationHealthNode._publish(node)
    assert node.state_publisher.messages[-1].mode_changed is False

    node.imu = None
    LocalizationHealthNode._publish(node)

    state = node.state_publisher.messages[-1]
    assert state.memory_mode == SemanticLocalizationState.MEMORY_OBSERVATION_ONLY
    assert state.canonical_frame_id == 'base_link'
    assert state.local_healthy is False
    assert state.mode_changed is True


def test_publish_clears_rollback_only_after_evaluator_consumes_sample():
    node = PublishHarness(now_ns=10000000000)
    consumed = []

    class ConsumingEvaluator:
        world_enabled = False

        def update(self, localization_sample):
            consumed.append((
                localization_sample,
                node._source_timestamp_rollback,
                node._world_source_timestamp_rollback,
            ))
            return LocalizationDecision(
                mode=MemoryMode.OBSERVATION_ONLY,
                epoch_id=2,
                epoch_changed=True,
                reason='timestamp_rollback',
            )

    node.evaluator = ConsumingEvaluator()
    node._source_timestamp_rollback = True
    node._world_source_timestamp_rollback = True

    LocalizationHealthNode._publish(node)

    assert consumed[0][0].source_timestamp_rollback is True
    assert consumed[0][0].world_source_timestamp_rollback is True
    assert consumed[0][1] is True
    assert consumed[0][2] is True
    assert node._source_timestamp_rollback is False
    assert node._world_source_timestamp_rollback is False


def test_publish_preserves_rollback_when_evaluator_rejects_sample():
    node = PublishHarness(now_ns=10000000000)

    class RejectingEvaluator:
        world_enabled = False

        def update(self, localization_sample):
            raise RuntimeError('not consumed')

    node.evaluator = RejectingEvaluator()
    node._source_timestamp_rollback = True
    node._world_source_timestamp_rollback = True

    with pytest.raises(RuntimeError, match='not consumed'):
        LocalizationHealthNode._publish(node)

    assert node._source_timestamp_rollback is True
    assert node._world_source_timestamp_rollback is True
