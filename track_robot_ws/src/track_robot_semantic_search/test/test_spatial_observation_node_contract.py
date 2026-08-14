from pathlib import Path
import threading
from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMessage
import numpy as np

from track_robot_semantic_search.depth_frame_buffer import (
    DepthFrame,
    DepthFrameBuffer,
)
from track_robot_semantic_search.phase4a_depth import CameraIntrinsics
from track_robot_semantic_search.spatial_observation import (
    SpatialObservationConfig,
)
from track_robot_semantic_search.spatial_observation_node import (
    SpatialObservationNode,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    def now(self):
        return SimpleNamespace(
            to_msg=lambda: TimeMessage(sec=9, nanosec=123))


class FakeTransformBuffer:
    def __init__(self, failing_stamps=()):
        self.calls = []
        self._failing_stamps = set(failing_stamps)

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append((target, source, stamp.nanoseconds, timeout))
        if stamp.nanoseconds in self._failing_stamps:
            raise RuntimeError('transform unavailable')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ))


class FakeBridge:
    def __init__(self):
        self.calls = []

    def imgmsg_to_cv2(self, message, desired_encoding):
        self.calls.append((message, desired_encoding))
        return np.full((4, 4), 2.0, dtype=np.float32)


def observation(observation_id, stamp_ns):
    return SimpleNamespace(
        observation_id=observation_id,
        camera_stamp_valid=True,
        camera_stamp=TimeMessage(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000),
        roi=SimpleNamespace(x_offset=0, y_offset=0, width=4, height=4),
        position_valid=False,
        position_frame_id='',
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        position_covariance=[0.0] * 9,
        localization_epoch_id=0,
        pose_stamp_valid=False,
        pose_stamp=TimeMessage(),
        tf_stamp_valid=False,
        tf_stamp=TimeMessage(),
        geometry_confidence=0.0,
        evidence_flags=0,
    )


def observation_array(*observations):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=TimeMessage(sec=1)),
        observations=list(observations),
    )


def make_node(depth_buffer, *, failing_tf_stamps=()):
    node = SpatialObservationNode.__new__(SpatialObservationNode)
    node._depth_buffer = depth_buffer
    node._depth_buffer_lock = threading.Lock()
    node._maximum_depth_delta_ns = 50
    node._tf_timeout_sec = 0.05
    node._config = SpatialObservationConfig(
        minimum_samples=4, inner_fraction=1.0)
    node._intrinsics = CameraIntrinsics(
        fx=100.0, fy=100.0, cx=1.5, cy=1.5)
    node._localization_epoch_id = 7
    node._tf_buffer = FakeTransformBuffer(failing_tf_stamps)
    node._publisher = FakePublisher()
    node._diagnostics_publisher = FakePublisher()
    node._counters = {key: 0 for key in node._COUNTER_KEYS}
    node.get_clock = lambda: FakeClock()
    return node


def depth_frame(stamp_ns):
    return DepthFrame(
        stamp_ns=stamp_ns,
        frame_id='zed_left_camera_optical_frame',
        image=np.full((4, 4), 2.0, dtype=np.float32),
    )


def depth_message(stamp_ns):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=TimeMessage(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000),
            frame_id='zed_left_camera_optical_frame',
        ),
    )


def diagnostic_values(message):
    return {item.key: item.value for item in message.status[0].values}


def localization_state(*, healthy=True, frame_id='base_link', epoch_id=7):
    return SimpleNamespace(
        local_healthy=healthy,
        canonical_frame_id=frame_id,
        localization_epoch_id=epoch_id,
    )


def test_enricher_matches_observation_stamp_and_uses_exact_depth_tf():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_semantic_search'
        / 'spatial_observation_node.py'
    ).read_text()
    assert 'DepthFrameBuffer' in source
    assert 'closest(source_stamp_ns' in source
    assert 'Time(nanoseconds=match.frame.stamp_ns)' in source
    assert 'Time()' not in source
    assert 'DiagnosticArray' in source


def test_depth_and_observation_callbacks_use_independent_executor_workers():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_semantic_search'
        / 'spatial_observation_node.py'
    ).read_text()

    assert 'MutuallyExclusiveCallbackGroup' in source
    assert 'self._depth_callback_group' in source
    assert 'self._observation_callback_group' in source
    assert 'MultiThreadedExecutor(num_threads=2)' in source
    assert 'callback_group=self._depth_callback_group' in source
    assert 'callback_group=self._observation_callback_group' in source
    assert 'with self._depth_buffer_lock:' in source


def test_enricher_diagnostics_use_only_fixed_reason_counters():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_semantic_search'
        / 'spatial_observation_node.py'
    ).read_text()
    expected_keys = (
        'matched_depth',
        'no_matching_depth',
        'depth_delta_exceeded',
        'insufficient_depth_samples',
        'depth_out_of_range',
        'tf_unavailable',
        'invalid_transformed_position',
        'camera_info_unavailable',
        'localization_unavailable',
    )

    assert '_COUNTER_KEYS = (' in source
    for key in expected_keys:
        assert "'{}'".format(key) in source
    assert "self._counters[latest_reason] += 1" in source


def test_enricher_configures_bounded_depth_matching():
    config = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'semantic_search_phase4a.yaml'
    ).read_text()

    assert ('diagnostics_topic: '
            '/semantic_search/spatial_observation_diagnostics') in config
    assert 'depth_buffer_frames: 16' in config
    assert 'depth_buffer_max_age_sec: 2.0' in config
    assert 'depth_processing_rate_hz: 5.0' in config
    assert 'maximum_depth_delta_sec: 0.20' in config


def test_depth_callback_drops_dense_frames_before_cv_bridge_conversion():
    node = SpatialObservationNode.__new__(SpatialObservationNode)
    node._bridge = FakeBridge()
    node._depth_buffer = DepthFrameBuffer(
        max_frames=4, max_age_ns=1_000_000_000)
    node._depth_buffer_lock = threading.Lock()
    node._minimum_depth_interval_ns = 200_000_000
    node._latest_depth_stamp_ns = 0

    node._on_depth(depth_message(1_000_000_000))
    node._on_depth(depth_message(1_050_000_000))
    node._on_depth(depth_message(1_200_000_000))

    assert len(node._bridge.calls) == 2
    assert node._depth_buffer.size == 2
    assert node._latest_depth_stamp_ns == 1_200_000_000


def test_unhealthy_localization_clears_previously_accepted_epoch():
    node = make_node(DepthFrameBuffer(max_frames=4, max_age_ns=10_000))
    node._on_localization(localization_state(epoch_id=41))
    assert node._localization_epoch_id == 41

    node._on_localization(localization_state(healthy=False, epoch_id=41))

    assert node._localization_epoch_id == 0


def test_frame_mismatch_clears_previously_accepted_epoch():
    node = make_node(DepthFrameBuffer(max_frames=4, max_age_ns=10_000))
    node._on_localization(localization_state(epoch_id=41))
    assert node._localization_epoch_id == 41

    node._on_localization(localization_state(
        frame_id='odom', epoch_id=41))

    assert node._localization_epoch_id == 0


def test_nonpositive_localization_epoch_is_not_accepted():
    node = make_node(DepthFrameBuffer(max_frames=4, max_age_ns=10_000))
    node._on_localization(localization_state(epoch_id=41))

    node._on_localization(localization_state(epoch_id=0))

    assert node._localization_epoch_id == 0


def test_missing_camera_info_preserves_each_2d_observation_once():
    depth_buffer = DepthFrameBuffer(max_frames=4, max_age_ns=10_000)
    depth_buffer.push(depth_frame(100))
    node = make_node(depth_buffer)
    node._intrinsics = None
    message = observation_array(
        observation('first', 100), observation('second', 100))

    node._on_observations(message)

    assert len(node._publisher.messages) == 1
    assert [item.observation_id for item in
            node._publisher.messages[0].observations] == ['first', 'second']
    assert all(not item.position_valid for item in
               node._publisher.messages[0].observations)
    assert node._tf_buffer.calls == []
    assert node._counters['camera_info_unavailable'] == 2
    assert len(node._diagnostics_publisher.messages) == 1
    values = diagnostic_values(node._diagnostics_publisher.messages[0])
    assert values['latest_reason'] == 'camera_info_unavailable'
    assert values['camera_info_unavailable'] == '2'


def test_missing_localization_preserves_each_2d_observation_once():
    depth_buffer = DepthFrameBuffer(max_frames=4, max_age_ns=10_000)
    depth_buffer.push(depth_frame(100))
    node = make_node(depth_buffer)
    node._localization_epoch_id = 0
    message = observation_array(
        observation('first', 100), observation('second', 100))

    node._on_observations(message)

    assert len(node._publisher.messages) == 1
    assert [item.observation_id for item in
            node._publisher.messages[0].observations] == ['first', 'second']
    assert all(not item.position_valid for item in
               node._publisher.messages[0].observations)
    assert node._tf_buffer.calls == []
    assert node._counters['localization_unavailable'] == 2
    assert len(node._diagnostics_publisher.messages) == 1
    values = diagnostic_values(node._diagnostics_publisher.messages[0])
    assert values['latest_reason'] == 'localization_unavailable'
    assert values['localization_unavailable'] == '2'


def test_callback_keeps_mixed_results_and_uses_matched_depth_tf():
    depth_buffer = DepthFrameBuffer(max_frames=4, max_age_ns=10_000)
    depth_buffer.push(depth_frame(100))
    depth_buffer.push(depth_frame(200))
    node = make_node(depth_buffer, failing_tf_stamps=(200,))
    message = observation_array(
        observation('matched', 100),
        observation('no-match', 1_000),
        observation('tf-failure', 200),
    )

    node._on_observations(message)

    assert len(node._publisher.messages) == 1
    output = node._publisher.messages[0]
    assert [item.observation_id for item in output.observations] == [
        'matched', 'no-match', 'tf-failure']
    assert [item.position_valid for item in output.observations] == [
        True, False, False]
    assert len(node._tf_buffer.calls) == 2
    assert [call[:3] for call in node._tf_buffer.calls] == [
        ('base_link', 'zed_left_camera_optical_frame', 100),
        ('base_link', 'zed_left_camera_optical_frame', 200),
    ]
    assert node._counters['matched_depth'] == 1
    assert node._counters['depth_delta_exceeded'] == 1
    assert node._counters['tf_unavailable'] == 1
    assert len(node._diagnostics_publisher.messages) == 1
    values = diagnostic_values(node._diagnostics_publisher.messages[0])
    assert values['matched_depth'] == '1'
    assert values['depth_delta_exceeded'] == '1'
    assert values['tf_unavailable'] == '1'


def test_callback_distinguishes_empty_and_outside_delta_buffers():
    depth_buffer = DepthFrameBuffer(max_frames=4, max_age_ns=10_000)
    node = make_node(depth_buffer)

    node._on_observations(observation_array(observation('empty', 100)))
    depth_buffer.push(depth_frame(100))
    node._on_observations(observation_array(
        observation('outside', 1_000_100)))

    assert len(node._publisher.messages) == 2
    assert [
        message.observations[0].observation_id
        for message in node._publisher.messages
    ] == ['empty', 'outside']
    assert node._counters['no_matching_depth'] == 1
    assert node._counters['depth_delta_exceeded'] == 1
    assert len(node._diagnostics_publisher.messages) == 2
    first_values = diagnostic_values(
        node._diagnostics_publisher.messages[0])
    second_values = diagnostic_values(
        node._diagnostics_publisher.messages[1])
    assert first_values['latest_reason'] == 'no_matching_depth'
    assert second_values['latest_reason'] == 'depth_delta_exceeded'
    assert first_values['depth_delta_valid'] == 'false'
    assert first_values['depth_delta_ms'] == '0.000'
    assert second_values['depth_delta_valid'] == 'true'
    assert second_values['depth_delta_ms'] == '1.000'
