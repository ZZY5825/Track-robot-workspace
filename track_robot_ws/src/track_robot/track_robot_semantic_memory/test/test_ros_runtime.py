import json
import os
from pathlib import Path
import subprocess
import time

import pytest
import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from track_robot_interfaces.msg import (
    LidarTracklet,
    SemanticLidarTrackletArray,
    SemanticLocalizationState,
    SemanticMemoryEvent,
    SemanticObservation,
    SemanticObservationArray,
    SemanticObject,
    SemanticObjectArray,
    SemanticTask,
)
from track_robot_interfaces.srv import (
    GetSemanticObject,
    MarkSemanticObjectInspected,
    QuerySemanticObjects,
    ResetSemanticMemory,
)
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import MarkerArray


def reliable(depth=10, transient=False):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient else DurabilityPolicy.VOLATILE),
    )


def best_effort(depth=1):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_localization():
    message = SemanticLocalizationState()
    message.header.stamp.sec = 100
    message.header.frame_id = 'odom'
    message.memory_mode = SemanticLocalizationState.MEMORY_LOCAL_SESSION
    message.localization_epoch_id = 7
    message.canonical_frame_id = 'odom'
    message.local_frame_id = 'odom'
    message.world_frame_id = 'map'
    message.base_frame_id = 'base_link'
    message.local_healthy = True
    message.reason = 'runtime_test'
    return message


def make_tracklet(tracklet_id, x):
    tracklet = LidarTracklet()
    tracklet.tracklet_id = tracklet_id
    tracklet.position.x = x
    tracklet.size.x = 1.0
    tracklet.size.y = 0.5
    tracklet.size.z = 0.5
    tracklet.confidence = 0.9
    tracklet.observation_quality = 0.8
    tracklet.position_covariance_xy = [0.1, 0.0, 0.0, 0.1]
    tracklet.last_measurement_stamp.sec = 100
    tracklet.active = True
    return tracklet


def make_lidar_batch():
    message = SemanticLidarTrackletArray()
    message.header.stamp.sec = 100
    message.header.frame_id = 'odom'
    message.source_epoch_id = 10
    message.tracklets = [make_tracklet(1, 1.0), make_tracklet(2, 2.0)]
    return message


class RuntimeProbe(Node):
    def __init__(self):
        super().__init__('semantic_memory_stage2b_runtime_probe')
        self.localization = self.create_publisher(
            SemanticLocalizationState,
            '/semantic_memory/localization_state',
            reliable(depth=1),
        )
        self.lidar = self.create_publisher(
            SemanticLidarTrackletArray,
            '/semantic_memory/lidar_tracklets',
            best_effort(),
        )
        self.observations = self.create_publisher(
            SemanticObservationArray,
            '/semantic_memory/observations',
            reliable(depth=1),
        )
        self.camera_info = self.create_publisher(
            CameraInfo,
            '/zed/zed_node/left/camera_info',
            best_effort(),
        )
        self.tasks = self.create_publisher(
            SemanticTask,
            '/semantic_memory/tasks',
            reliable(depth=1),
        )
        self.snapshots = []
        self.best_candidates = []
        self.events = []
        self.markers = []
        self.create_subscription(
            SemanticObjectArray,
            '/semantic_memory/active_objects',
            self.snapshots.append,
            reliable(depth=1, transient=True),
        )
        self.create_subscription(
            SemanticObjectArray,
            '/semantic_memory/best_candidate',
            self.best_candidates.append,
            reliable(depth=1, transient=True),
        )
        self.create_subscription(
            SemanticMemoryEvent,
            '/semantic_memory/events',
            self.events.append,
            reliable(depth=64),
        )
        self.create_subscription(
            MarkerArray,
            '/semantic_memory/markers',
            self.markers.append,
            reliable(depth=1, transient=True),
        )
        self.get_object = self.create_client(
            GetSemanticObject, '/semantic_memory/get_object')
        self.query_objects = self.create_client(
            QuerySemanticObjects, '/semantic_memory/query_objects')
        self.mark_inspected = self.create_client(
            MarkSemanticObjectInspected, '/semantic_memory/mark_inspected')
        self.reset_memory = self.create_client(
            ResetSemanticMemory, '/semantic_memory/reset')


def wait_until(probe, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if predicate():
            return True
    return False


def call_service(probe, client, request, timeout=8.0):
    future = client.call_async(request)
    assert wait_until(probe, future.done, timeout), 'service call timed out'
    assert future.exception() is None
    return future.result()


def set_stamp(stamp, nanoseconds):
    stamp.sec = nanoseconds // 1_000_000_000
    stamp.nanosec = nanoseconds % 1_000_000_000


def stamp_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def object_with_lidar(snapshot, tracklet_id):
    return next(
        (
            item for item in snapshot.objects
            if item.lidar_tracklet_id_valid and
            item.lidar_tracklet_id == tracklet_id
        ),
        None,
    )


def runtime_localization(stamp_ns):
    message = make_localization()
    set_stamp(message.header.stamp, stamp_ns)
    message.header.frame_id = 'camera_optical'
    message.canonical_frame_id = 'camera_optical'
    message.local_frame_id = 'camera_optical'
    return message


def runtime_lidar_batch(stamp_ns, tracklet_id=None):
    message = SemanticLidarTrackletArray()
    set_stamp(message.header.stamp, stamp_ns)
    message.header.frame_id = 'camera_optical'
    message.source_epoch_id = 10
    if tracklet_id is not None:
        tracklet = make_tracklet(tracklet_id, 0.0)
        tracklet.position.z = 3.0
        set_stamp(tracklet.last_measurement_stamp, stamp_ns)
        message.tracklets = [tracklet]
    return message


def runtime_duplicate_lidar_batch(stamp_ns, tracklet_id):
    message = runtime_lidar_batch(stamp_ns, tracklet_id)
    duplicate = make_tracklet(tracklet_id, 0.2)
    duplicate.position.z = 3.2
    set_stamp(duplicate.last_measurement_stamp, stamp_ns)
    message.tracklets.append(duplicate)
    return message


def runtime_camera_info(stamp_ns):
    message = CameraInfo()
    set_stamp(message.header.stamp, stamp_ns)
    message.header.frame_id = 'camera_optical'
    message.width = 640
    message.height = 480
    message.k = [100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0]
    message.p = [
        100.0, 0.0, 320.0, 0.0,
        0.0, 100.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def runtime_observations(stamp_ns, observation_id, candidate_id,
                         camera_track_id, lidar_tracklet_id):
    observation = SemanticObservation()
    set_stamp(observation.header.stamp, stamp_ns)
    observation.header.frame_id = 'camera_optical'
    observation.producer_epoch_id = 20
    observation.observation_id = observation_id
    observation.visual_candidate_id = candidate_id
    observation.camera_track_id_valid = True
    observation.camera_track_id = camera_track_id
    observation.lidar_tracklet_id_valid = True
    observation.lidar_source_epoch_id = 10
    observation.lidar_tracklet_id = lidar_tracklet_id
    observation.camera_stamp_valid = True
    set_stamp(observation.camera_stamp, stamp_ns)
    observation.image_width = 640
    observation.image_height = 480
    observation.roi.x_offset = 300
    observation.roi.y_offset = 220
    observation.roi.width = 40
    observation.roi.height = 40
    observation.calibration_id = 'zed_left_rectified_v1'
    observation.evidence_flags = SemanticObservation.EVIDENCE_CAMERA
    observation.detector_confidence = 0.9
    observation.appearance_confidence = 0.9
    observation.geometry_confidence = 0.9
    observation.overall_confidence = 0.9
    observation.appearance_descriptor.encoder_id = 'runtime-test'
    observation.appearance_descriptor.checkpoint_id = 'runtime-test-v1'
    observation.appearance_descriptor.version = 1
    observation.appearance_descriptor.dimension = 2
    observation.appearance_descriptor.l2_normalized = True
    observation.appearance_descriptor.quality = 0.9
    observation.appearance_descriptor.values = [1.0, 0.0]
    batch = SemanticObservationArray()
    set_stamp(batch.header.stamp, stamp_ns)
    batch.header.frame_id = 'camera_optical'
    batch.producer_epoch_id = 20
    batch.observations = [observation]
    return batch


def runtime_task(stamp_ns, query_id=41, query_version=3):
    task = SemanticTask()
    set_stamp(task.header.stamp, stamp_ns)
    task.header.frame_id = 'camera_optical'
    task.producer_epoch_id = 30
    task.query_id = query_id
    task.query_version = query_version
    task.query_text = 'runtime target'
    task.task_descriptor.encoder_id = 'runtime-test'
    task.task_descriptor.checkpoint_id = 'runtime-test-v1'
    task.task_descriptor.version = 1
    task.task_descriptor.dimension = 2
    task.task_descriptor.l2_normalized = True
    task.task_descriptor.quality = 1.0
    task.task_descriptor.values = [1.0, 0.0]
    return task


def write_stage2e_runtime_config(tmp_path, share):
    association_report = {
        'status': 'calibrated',
        'camera_attachment_allowed': True,
        'counts': {'positive': 20, 'negative': 20},
        'hard_gate_pass_counts': {'positive': 20, 'negative': 20},
        'selected_parameters': {
            'match_threshold': 0.5,
            'ambiguity_margin': 0.01,
            'association_metrics': {
                'precision': 1.0, 'recall': 1.0, 'threshold': 0.5},
            'term_weights_from_median_separation': {
                'sensor_confidence': 1.0},
        },
        'runtime_contract': {
            'scoring_contract_version': 'stage2d_association_v1',
            'camera_calibration_id': 'zed_left_rectified_v1',
            'hard_gates': {
                'max_source_time_delta_s': 0.1,
                'max_evidence_age_s': 0.5,
                'max_position_nis': 9.21,
                'minimum_size_ratio': 0.25,
                'maximum_size_ratio': 40.0,
                'max_relative_speed_mps': 3.0,
                'position_distance_max_m': 3.0,
                'center_distance_max_px': 200.0,
                'descriptor_normalization_tolerance': 0.0001,
                'require_position_nis': False,
                'require_size_ratio': False,
                'require_motion_gate': False,
                'require_descriptors': False,
            },
            'soft_weights': {
                'position_consistency': 0.0,
                'projected_centroid': 0.0,
                'inside_fraction': 0.0,
                'projected_iou': 0.0,
                'visual_cosine': 0.0,
                'extent_consistency': 0.0,
                'point_count_consistency': 0.0,
                'motion_continuity': 0.0,
                'previous_association': 0.0,
                'detector_confidence': 0.0,
                'geometry_confidence': 0.0,
                'sensor_confidence': 1.0,
            },
            'confirmation': {
                'confirmation_frames': 1,
                'detach_after_misses': 2,
                'previous_association_hysteresis': 0.02,
                'cooldown_frames': 2,
            },
        },
    }
    reidentification_report = {
        'contract_version': 'stage2e_reidentification_v1',
        'status': 'calibrated',
        'reidentification_allowed': True,
        'selected_parameters': {
            'maximum_age_ns': 5_000_000_000,
            'maximum_spatial_distance_m': 3.0,
            'minimum_appearance_similarity': 0.75,
            'minimum_combined_score': 0.70,
            'ambiguity_margin': 0.05,
            'confirmation_frames': 3,
        },
    }
    association_path = tmp_path / 'association.json'
    reidentification_path = tmp_path / 'reidentification.json'
    association_path.write_text(json.dumps(association_report))
    reidentification_path.write_text(json.dumps(reidentification_report))
    config = yaml.safe_load(
        (share / 'config' / 'semantic_memory.yaml').read_text())
    parameters = config['semantic_memory']['ros__parameters']
    parameters.update({
        'association_shadow_mode': False,
        'camera_attachment_enabled': True,
        'association_calibration_status': 'calibrated',
        'association_calibration_report': str(association_path),
        'association_match_threshold': 0.5,
        'association_ambiguity_margin': 0.01,
        'association_confirmation_frames': 1,
        'association_weight_projected_centroid': 0.0,
        'association_weight_inside_fraction': 0.0,
        'association_weight_projected_iou': 0.0,
        'association_weight_extent_consistency': 0.0,
        'association_weight_sensor_confidence': 1.0,
        'reidentification_shadow_mode': False,
        'reidentification_mutation_enabled': True,
        'reidentification_calibration_status': 'calibrated',
        'reidentification_calibration_report': str(reidentification_path),
        'static_confirmation_hits': 1,
        'static_stale_after_sec': 0.01,
        'static_lost_after_sec': 0.02,
        'static_archive_after_sec': 10.0,
        'initial_memory_epoch_id': 100,
        'localization_state_timeout_sec': 1.0,
    })
    config_path = tmp_path / 'stage2e-runtime.yaml'
    config_path.write_text(yaml.safe_dump(config))
    return config_path


def write_stage2f_runtime_config(tmp_path, share):
    config_path = write_stage2e_runtime_config(tmp_path, share)
    config = yaml.safe_load(config_path.read_text())
    parameters = config['semantic_memory']['ros__parameters']
    parameters.update({
        'best_candidate_threshold_calibrated': True,
        'best_candidate_minimum_relevance': 0.8,
    })
    config_path = tmp_path / 'stage2f-runtime.yaml'
    config_path.write_text(yaml.safe_dump(config))
    return config_path


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_RUNTIME_TESTS') != '1',
    reason='requires local DDS interface access; set RUN_ROS_RUNTIME_TESTS=1',
)
def test_stage2b_nodes_publish_memory_events_and_bounded_markers(tmp_path):
    os.environ.setdefault('ROS_DOMAIN_ID', '20')
    ros_log_dir = tmp_path / 'ros-log'
    ros_log_dir.mkdir()
    os.environ['ROS_LOG_DIR'] = str(ros_log_dir)
    probe = None
    processes = []
    rclpy_initialized = False
    try:
        rclpy.init()
        rclpy_initialized = True
        probe = RuntimeProbe()
        prefix = Path(get_package_prefix('track_robot_semantic_memory'))
        share = Path(get_package_share_directory('track_robot_semantic_memory'))
        config = share / 'config' / 'semantic_memory.yaml'
        commands = [
            prefix / 'lib' / 'track_robot_semantic_memory' /
            'semantic_memory_node',
            prefix / 'lib' / 'track_robot_semantic_memory' /
            'semantic_memory_visualizer_node',
        ]
        for command in commands:
            processes.append(subprocess.Popen(
                [str(command), '--ros-args', '--params-file', str(config)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            ))

        discovered = wait_until(
            probe,
            lambda: (
                probe.localization.get_subscription_count() == 1 and
                probe.lidar.get_subscription_count() == 1),
        )
        assert discovered, 'semantic-memory input subscriptions not discovered'
        probe.localization.publish(make_localization())
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.lidar.publish(make_lidar_batch())

        received = wait_until(
            probe,
            lambda: bool(
                probe.snapshots and probe.markers and any(
                    event.event_type ==
                    SemanticMemoryEvent.EVENT_OBJECT_CREATED
                    for event in probe.events)),
        )
        assert received, 'semantic-memory runtime outputs were not received'
        snapshot = probe.snapshots[-1]
        assert snapshot.header.frame_id == 'odom'
        assert len(snapshot.objects) == 2
        keys = {
            (item.memory_epoch_id, item.global_object_id)
            for item in snapshot.objects
        }
        assert len(keys) == 2
        assert all(item.velocity.x == 0.0 for item in snapshot.objects)
        assert any(
            event.event_type == SemanticMemoryEvent.EVENT_OBJECT_CREATED
            for event in probe.events)
        marker_array = probe.markers[-1]
        assert len(marker_array.markers) == 4
        assert sum(
            marker.ns == 'semantic_memory_objects'
            for marker in marker_array.markers
        ) == 2
        assert sum(
            marker.ns == 'semantic_memory_labels'
            for marker in marker_array.markers
        ) == 2
        assert len({marker.id for marker in marker_array.markers}) == 2
        assert all(process.poll() is None for process in processes)
    finally:
        if probe is not None:
            probe.destroy_node()
        if rclpy_initialized:
            rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            if process.poll() is None:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_RUNTIME_TESTS') != '1',
    reason='requires local DDS interface access; set RUN_ROS_RUNTIME_TESTS=1',
)
def test_duplicate_lidar_ids_do_not_terminate_semantic_memory(tmp_path):
    os.environ.setdefault('ROS_DOMAIN_ID', '20')
    ros_log_dir = tmp_path / 'ros-log-duplicate-lidar'
    ros_log_dir.mkdir()
    os.environ['ROS_LOG_DIR'] = str(ros_log_dir)
    probe = None
    process = None
    rclpy_initialized = False
    try:
        rclpy.init()
        rclpy_initialized = True
        probe = RuntimeProbe()
        prefix = Path(get_package_prefix('track_robot_semantic_memory'))
        share = Path(get_package_share_directory('track_robot_semantic_memory'))
        config = write_stage2e_runtime_config(tmp_path, share)
        command = (
            prefix / 'lib' / 'track_robot_semantic_memory' /
            'semantic_memory_node')
        process = subprocess.Popen(
            [str(command), '--ros-args', '--params-file', str(config)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        discovered = wait_until(
            probe,
            lambda: (
                probe.localization.get_subscription_count() == 1 and
                probe.lidar.get_subscription_count() == 1 and
                probe.observations.get_subscription_count() == 1 and
                probe.camera_info.get_subscription_count() == 1),
        )
        assert discovered, 'Phase 3 input subscriptions not discovered'

        base = 100_000_000_000
        camera_info = runtime_camera_info(base)
        for _ in range(3):
            probe.camera_info.publish(camera_info)
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.localization.publish(runtime_localization(base))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)

        probe.lidar.publish(runtime_duplicate_lidar_batch(base, 1))
        probe.observations.publish(
            runtime_observations(base, 1, 101, 1001, 1))
        alive_after_duplicate = wait_until(
            probe,
            lambda: bool(probe.snapshots),
            timeout=2.0,
        )
        assert alive_after_duplicate
        assert process.poll() is None, (
            'duplicate LiDAR IDs terminated semantic memory')

        valid_stamp = base + 50_000_000
        snapshot_count = len(probe.snapshots)
        probe.localization.publish(runtime_localization(valid_stamp))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.lidar.publish(runtime_lidar_batch(valid_stamp, 2))
        probe.observations.publish(
            runtime_observations(valid_stamp, 2, 102, 1002, 2))
        assert wait_until(
            probe,
            lambda: (
                len(probe.snapshots) > snapshot_count and
                process.poll() is None),
        ), 'semantic memory did not process valid input after duplicate IDs'
    finally:
        if probe is not None:
            probe.destroy_node()
        if rclpy_initialized:
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_RUNTIME_TESTS') != '1',
    reason='requires local DDS interface access; set RUN_ROS_RUNTIME_TESTS=1',
)
def test_stage2e_leave_and_reentry_preserves_one_global_identity(tmp_path):
    os.environ.setdefault('ROS_DOMAIN_ID', '20')
    ros_log_dir = tmp_path / 'ros-log-stage2e'
    ros_log_dir.mkdir()
    os.environ['ROS_LOG_DIR'] = str(ros_log_dir)
    probe = None
    process = None
    rclpy_initialized = False
    try:
        rclpy.init()
        rclpy_initialized = True
        probe = RuntimeProbe()
        prefix = Path(get_package_prefix('track_robot_semantic_memory'))
        share = Path(get_package_share_directory('track_robot_semantic_memory'))
        config = write_stage2e_runtime_config(tmp_path, share)
        command = (
            prefix / 'lib' / 'track_robot_semantic_memory' /
            'semantic_memory_node')
        process = subprocess.Popen(
            [str(command), '--ros-args', '--params-file', str(config)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        discovered = wait_until(
            probe,
            lambda: (
                probe.localization.get_subscription_count() == 1 and
                probe.lidar.get_subscription_count() == 1 and
                probe.observations.get_subscription_count() == 1 and
                probe.camera_info.get_subscription_count() == 1),
        )
        assert discovered, 'Stage 2E input subscriptions not discovered'

        base = 100_000_000_000
        camera_info = runtime_camera_info(base)
        for _ in range(3):
            probe.camera_info.publish(camera_info)
            rclpy.spin_once(probe, timeout_sec=0.05)

        snapshot_count = len(probe.snapshots)
        probe.localization.publish(runtime_localization(base))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.lidar.publish(runtime_lidar_batch(base, 1))
        assert wait_until(
            probe,
            lambda: (
                len(probe.snapshots) > snapshot_count and
                object_with_lidar(probe.snapshots[-1], 1) is not None
            ),
        )
        probe.observations.publish(runtime_observations(base, 1, 101, 1001, 1))
        assert wait_until(
            probe,
            lambda: bool(
                probe.snapshots and
                probe.snapshots[-1].objects and
                probe.snapshots[-1].objects[0].appearance_prototype_count == 1),
        )
        old_key = (
            probe.snapshots[-1].objects[0].memory_epoch_id,
            probe.snapshots[-1].objects[0].global_object_id,
        )

        absent = base + 1_000_000_000
        snapshot_count = len(probe.snapshots)
        probe.localization.publish(runtime_localization(absent))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.lidar.publish(runtime_lidar_batch(absent))
        assert wait_until(
            probe,
            lambda: (
                len(probe.snapshots) > snapshot_count and
                any(
                    event.event_type ==
                    SemanticMemoryEvent.EVENT_LIFECYCLE_CHANGED and
                    event.current_lifecycle_state ==
                    SemanticObject.LIFECYCLE_LOST and
                    (event.memory_epoch_id, event.global_object_id) == old_key
                    for event in probe.events
                )
            ),
        )

        for index in range(3):
            stamp = base + 2_000_000_000 + index * 50_000_000
            snapshot_count = len(probe.snapshots)
            probe.localization.publish(runtime_localization(stamp))
            for _ in range(3):
                rclpy.spin_once(probe, timeout_sec=0.05)
            probe.lidar.publish(runtime_lidar_batch(stamp, 2))
            assert wait_until(
                probe,
                lambda: (
                    len(probe.snapshots) > snapshot_count and
                    object_with_lidar(probe.snapshots[-1], 2) is not None
                ),
            )
            snapshot_count = len(probe.snapshots)
            probe.observations.publish(runtime_observations(
                stamp, 10 + index, 201 + index, 2002, 2))
            assert wait_until(
                probe,
                lambda: (
                    len(probe.snapshots) > snapshot_count and
                    (candidate := object_with_lidar(
                        probe.snapshots[-1], 2)) is not None and
                    stamp_nanoseconds(candidate.last_camera_seen) == stamp and
                    candidate.appearance_prototype_count == 1
                ),
            )

        transferred = wait_until(
            probe,
            lambda: sum(
                event.event_type == SemanticMemoryEvent.EVENT_REIDENTIFIED
                for event in probe.events) == 1,
        )
        assert transferred, 'Stage 2E re-identification event not received'
        assert probe.snapshots
        final = probe.snapshots[-1]
        assert len(final.objects) == 1
        assert (
            final.objects[0].memory_epoch_id,
            final.objects[0].global_object_id,
        ) == old_key
        assert final.objects[0].lidar_tracklet_id == 2
        assert sum(
            event.event_type == SemanticMemoryEvent.EVENT_REIDENTIFIED
            for event in probe.events) == 1
        assert process.poll() is None
    finally:
        if probe is not None:
            probe.destroy_node()
        if rclpy_initialized:
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)


@pytest.mark.skipif(
    os.environ.get('RUN_ROS_RUNTIME_TESTS') != '1',
    reason='requires local DDS interface access; set RUN_ROS_RUNTIME_TESTS=1',
)
def test_stage2f_task_services_inspection_and_reset_are_epoch_safe(tmp_path):
    os.environ.setdefault('ROS_DOMAIN_ID', '20')
    ros_log_dir = tmp_path / 'ros-log-stage2f'
    ros_log_dir.mkdir()
    os.environ['ROS_LOG_DIR'] = str(ros_log_dir)
    probe = None
    process = None
    rclpy_initialized = False
    try:
        rclpy.init()
        rclpy_initialized = True
        probe = RuntimeProbe()
        prefix = Path(get_package_prefix('track_robot_semantic_memory'))
        share = Path(get_package_share_directory('track_robot_semantic_memory'))
        config = write_stage2f_runtime_config(tmp_path, share)
        command = (
            prefix / 'lib' / 'track_robot_semantic_memory' /
            'semantic_memory_node')
        process = subprocess.Popen(
            [str(command), '--ros-args', '--params-file', str(config)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        discovered = wait_until(
            probe,
            lambda: (
                probe.localization.get_subscription_count() == 1 and
                probe.lidar.get_subscription_count() == 1 and
                probe.observations.get_subscription_count() == 1 and
                probe.camera_info.get_subscription_count() == 1 and
                probe.tasks.get_subscription_count() == 1 and
                probe.get_object.service_is_ready() and
                probe.query_objects.service_is_ready() and
                probe.mark_inspected.service_is_ready() and
                probe.reset_memory.service_is_ready()),
        )
        assert discovered, 'Stage 2F DDS endpoints not discovered'

        base = 100_000_000_000
        probe.tasks.publish(runtime_task(base - 1))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        camera_info = runtime_camera_info(base)
        for _ in range(3):
            probe.camera_info.publish(camera_info)
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.localization.publish(runtime_localization(base))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.lidar.publish(runtime_lidar_batch(base, 1))
        assert wait_until(
            probe,
            lambda: bool(
                probe.snapshots and
                object_with_lidar(probe.snapshots[-1], 1) is not None),
        )
        probe.observations.publish(runtime_observations(base, 1, 101, 1001, 1))
        assert wait_until(
            probe,
            lambda: bool(
                probe.snapshots and probe.snapshots[-1].objects and
                probe.snapshots[-1].objects[0].appearance_prototype_count == 1),
        )
        object_key = (
            probe.snapshots[-1].objects[0].memory_epoch_id,
            probe.snapshots[-1].objects[0].global_object_id,
        )

        assert wait_until(
            probe,
            lambda: bool(
                probe.best_candidates and
                len(probe.best_candidates[-1].objects) == 1 and
                probe.best_candidates[-1].objects[0].active_query_id == 41),
        )
        candidate = probe.best_candidates[-1].objects[0]
        assert (candidate.memory_epoch_id, candidate.global_object_id) == object_key
        assert candidate.active_query_version == 3
        assert candidate.task_relevance >= 0.8

        get_request = GetSemanticObject.Request()
        get_request.memory_epoch_id = object_key[0]
        get_request.global_object_id = object_key[1]
        get_response = call_service(probe, probe.get_object, get_request)
        assert get_response.found
        assert get_response.reason == 'ok'
        assert get_response.object.active_query_id == 41

        query_request = QuerySemanticObjects.Request()
        query_request.query_mode = QuerySemanticObjects.Request.QUERY_ACTIVE_TASK
        query_request.query_id = 41
        query_request.query_version = 3
        query_request.page_size = 8
        query_response = call_service(
            probe, probe.query_objects, query_request)
        assert query_response.accepted
        assert query_response.reason == 'ok'
        assert len(query_response.objects) == 1
        assert query_response.objects[0].global_object_id == object_key[1]

        inspect_request = MarkSemanticObjectInspected.Request()
        inspect_request.memory_epoch_id = object_key[0]
        inspect_request.global_object_id = object_key[1]
        inspect_request.inspection_state = SemanticObject.INSPECTION_COMPLETE
        inspect_response = call_service(
            probe, probe.mark_inspected, inspect_request)
        assert inspect_response.updated
        assert inspect_response.reason == 'ok'
        assert inspect_response.object.inspection_state == (
            SemanticObject.INSPECTION_COMPLETE)
        assert wait_until(
            probe,
            lambda: bool(
                probe.best_candidates and
                len(probe.best_candidates[-1].objects) == 0 and
                any(
                    event.event_type ==
                    SemanticMemoryEvent.EVENT_INSPECTION_CHANGED
                    for event in probe.events)),
        )

        reset_request = ResetSemanticMemory.Request()
        reset_request.expected_memory_epoch_id = object_key[0]
        reset_request.require_epoch_match = True
        reset_request.reason = 'stage2f_runtime_test'
        reset_response = call_service(
            probe, probe.reset_memory, reset_request)
        assert reset_response.reset
        assert reset_response.result_reason == 'ok'
        assert reset_response.new_memory_epoch_id != object_key[0]
        assert wait_until(
            probe,
            lambda: bool(
                probe.snapshots and
                probe.snapshots[-1].memory_epoch_id ==
                reset_response.new_memory_epoch_id and
                len(probe.snapshots[-1].objects) == 0 and
                probe.best_candidates and
                probe.best_candidates[-1].memory_epoch_id ==
                reset_response.new_memory_epoch_id and
                len(probe.best_candidates[-1].objects) == 0 and
                any(
                    event.event_type ==
                    SemanticMemoryEvent.EVENT_MEMORY_RESET and
                    event.memory_epoch_id == reset_response.new_memory_epoch_id
                    for event in probe.events)),
        )

        stale_response = call_service(probe, probe.get_object, get_request)
        assert not stale_response.found
        assert stale_response.reason == 'stale_epoch'

        next_stamp = base + 10_000_000_000
        probe.localization.publish(runtime_localization(next_stamp))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.lidar.publish(runtime_lidar_batch(next_stamp, 5))
        assert wait_until(
            probe,
            lambda: bool(
                probe.snapshots and
                object_with_lidar(probe.snapshots[-1], 5) is not None),
        )
        domain_key = (
            probe.snapshots[-1].objects[0].memory_epoch_id,
            probe.snapshots[-1].objects[0].global_object_id,
        )
        domain_get = GetSemanticObject.Request()
        domain_get.memory_epoch_id = domain_key[0]
        domain_get.global_object_id = domain_key[1]
        assert call_service(probe, probe.get_object, domain_get).found

        snapshot_count = len(probe.snapshots)
        unhealthy = runtime_localization(next_stamp + 1)
        unhealthy.local_healthy = False
        probe.localization.publish(unhealthy)
        assert wait_until(
            probe,
            lambda: (
                len(probe.snapshots) > snapshot_count and
                len(probe.snapshots[-1].objects) == 0 and
                probe.best_candidates and
                len(probe.best_candidates[-1].objects) == 0),
        )
        unavailable = call_service(probe, probe.get_object, domain_get)
        assert not unavailable.found
        assert unavailable.reason == 'not_found'

        probe.tasks.publish(runtime_task(next_stamp + 10, 50, 1))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.tasks.publish(runtime_task(next_stamp + 9, 51, 1))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        recovery_stamp = next_stamp + 20
        probe.localization.publish(runtime_localization(recovery_stamp))
        for _ in range(3):
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.lidar.publish(runtime_lidar_batch(recovery_stamp, 5))
        assert wait_until(
            probe,
            lambda: bool(
                probe.snapshots and
                object_with_lidar(probe.snapshots[-1], 5) is not None),
        )
        rollback_query = QuerySemanticObjects.Request()
        rollback_query.query_mode = (
            QuerySemanticObjects.Request.QUERY_ACTIVE_TASK)
        rollback_query.query_id = 51
        rollback_query.query_version = 1
        rollback_query.page_size = 8
        rollback_response = call_service(
            probe, probe.query_objects, rollback_query)
        assert not rollback_response.accepted
        assert rollback_response.reason == 'invalid_request'

        snapshot_count = len(probe.snapshots)
        next_domain = runtime_localization(recovery_stamp + 1)
        next_domain.localization_epoch_id = 8
        probe.localization.publish(next_domain)
        assert wait_until(
            probe,
            lambda: (
                len(probe.snapshots) > snapshot_count and
                len(probe.snapshots[-1].objects) == 0),
        )
        domain_unavailable = call_service(probe, probe.get_object, domain_get)
        assert not domain_unavailable.found
        assert domain_unavailable.reason == 'not_found'
        assert process.poll() is None
    finally:
        if probe is not None:
            probe.destroy_node()
        if rclpy_initialized:
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
