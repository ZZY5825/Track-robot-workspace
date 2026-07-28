"""ROS adapter for the conservative Phase 3A test selector."""

import copy
from dataclasses import dataclass

from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from track_robot_interfaces.msg import (
    SemanticLocalizationState,
    SemanticObject,
    SemanticObjectArray,
    SemanticObservationArray,
    SemanticTask,
)

from .phase4a_depth import (
    CameraIntrinsics,
    estimate_depth_point,
    transform_point,
)
from .phase4a_selector import (
    FixedBaseTargetSelector,
    ObjectCandidate,
    SelectionSnapshot,
    SelectorConfig,
)


@dataclass(frozen=True)
class _DepthGeometry:
    stamp_ns: int
    x: float
    y: float
    z: float
    quality: float


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Phase4ASelectorNode(Node):
    """Select one test target without enabling production calibration."""

    def __init__(self):
        super().__init__('phase4a_target_selector')
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._depth_image = None
        self._depth_stamp_ns = 0
        self._depth_frame_id = ''
        self._intrinsics = None
        self._depth_geometry = {}
        self._localization_epoch_id = 0
        ranking_topic = self.declare_parameter(
            'ranking_topic', '/semantic_memory/diagnostic_ranking').value
        tasks_topic = self.declare_parameter(
            'tasks_topic', '/semantic_memory/tasks').value
        output_topic = self.declare_parameter(
            'selected_target_topic',
            '/semantic_search/phase4a/selected_target').value
        spatial_objects_topic = self.declare_parameter(
            'spatial_objects_topic',
            '/semantic_search/phase4a/spatial_objects').value
        diagnostics_topic = self.declare_parameter(
            'diagnostics_topic',
            '/semantic_search/phase4a/selector_diagnostics').value
        observations_topic = self.declare_parameter(
            'observations_topic',
            '/semantic_memory/observations').value
        depth_topic = self.declare_parameter(
            'depth_topic',
            '/zed/zed_node/depth/depth_registered').value
        camera_info_topic = self.declare_parameter(
            'camera_info_topic',
            '/zed/zed_node/left/camera_info').value
        localization_topic = self.declare_parameter(
            'localization_topic',
            '/semantic_search/phase4a/localization_state').value
        self._frame_id = str(self.declare_parameter(
            'frame_id', 'base_link').value)
        self._maximum_depth_age_ns = int(float(self.declare_parameter(
            'maximum_depth_age_sec', 0.5).value) * 1_000_000_000)
        self._minimum_depth_samples = int(self.declare_parameter(
            'minimum_depth_samples', 20).value)
        self._minimum_depth_m = float(self.declare_parameter(
            'minimum_depth_m', 0.3).value)
        self._maximum_depth_m = float(self.declare_parameter(
            'maximum_depth_m', 8.0).value)
        self._depth_inner_fraction = float(self.declare_parameter(
            'depth_inner_fraction', 0.5).value)
        self._selector = FixedBaseTargetSelector(SelectorConfig(
            minimum_relevance=float(self.declare_parameter(
                'minimum_relevance', 0.50).value),
            minimum_margin=float(self.declare_parameter(
                'minimum_margin', 0.08).value),
            maximum_uncertainty=float(self.declare_parameter(
                'maximum_uncertainty', 0.50).value),
            confirmation_snapshots=int(self.declare_parameter(
                'confirmation_snapshots', 3).value),
            position_window=int(self.declare_parameter(
                'position_window', 5).value),
            maximum_xy_spread_m=float(self.declare_parameter(
                'maximum_xy_spread_m', 0.35).value),
            maximum_age_ns=int(float(self.declare_parameter(
                'maximum_age_sec', 1.0).value) * 1_000_000_000),
            frame_id=self._frame_id,
        ))
        self._query_id = 0
        self._query_version = 0

        selected_qos = QoSProfile(depth=1)
        selected_qos.reliability = ReliabilityPolicy.RELIABLE
        selected_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._selected_publisher = self.create_publisher(
            SemanticObjectArray, output_topic, selected_qos)
        self._spatial_objects_publisher = self.create_publisher(
            SemanticObjectArray, spatial_objects_topic, 10)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10)
        self._task_subscription = self.create_subscription(
            SemanticTask, tasks_topic, self._on_task, 10)
        self._ranking_subscription = self.create_subscription(
            SemanticObjectArray, ranking_topic, self._on_ranking, 10)
        self._observation_subscription = self.create_subscription(
            SemanticObservationArray,
            observations_topic,
            self._on_observations,
            10)
        self._depth_subscription = self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data)
        self._localization_subscription = self.create_subscription(
            SemanticLocalizationState,
            localization_topic,
            self._on_localization,
            10)
        self.get_logger().warn(
            'Phase 3A selector is test-only and does not calibrate '
            '/semantic_memory/best_candidate')

    def _on_task(self, message):
        self._query_id = int(message.query_id)
        self._query_version = int(message.query_version)

    def _on_localization(self, message):
        if (
                message.local_healthy
                and str(message.canonical_frame_id) == self._frame_id):
            self._localization_epoch_id = int(message.localization_epoch_id)

    def _on_camera_info(self, message):
        try:
            self._intrinsics = CameraIntrinsics(
                fx=float(message.k[0]),
                fy=float(message.k[4]),
                cx=float(message.k[2]),
                cy=float(message.k[5]),
            )
            self._intrinsics.validate()
        except (IndexError, TypeError, ValueError):
            self._intrinsics = None

    def _on_depth(self, message):
        try:
            depth = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='32FC1')
            if depth.ndim != 2:
                return
            self._depth_image = depth
            self._depth_stamp_ns = _stamp_ns(message.header.stamp)
            self._depth_frame_id = str(message.header.frame_id)
        except (CvBridgeError, TypeError, ValueError):
            self._depth_image = None
            self._depth_stamp_ns = 0
            self._depth_frame_id = ''

    def _on_observations(self, message):
        now_ns = int(self.get_clock().now().nanoseconds)
        if (
                self._depth_image is None
                or self._intrinsics is None
                or not self._depth_frame_id
                or self._depth_stamp_ns <= 0
                or now_ns - self._depth_stamp_ns < 0
                or now_ns - self._depth_stamp_ns > self._maximum_depth_age_ns):
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._depth_frame_id,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:  # tf2 exception classes vary across ROS 2 Foxy builds.
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        for observation in message.observations:
            if (
                    not observation.camera_track_id_valid
                    or observation.camera_track_id < 0):
                continue
            roi = observation.roi
            try:
                estimate = estimate_depth_point(
                    self._depth_image,
                    roi=(
                        roi.x_offset,
                        roi.y_offset,
                        roi.width,
                        roi.height,
                    ),
                    intrinsics=self._intrinsics,
                    minimum_samples=self._minimum_depth_samples,
                    minimum_depth_m=self._minimum_depth_m,
                    maximum_depth_m=self._maximum_depth_m,
                    inner_fraction=self._depth_inner_fraction,
                )
                x, y, z = transform_point(
                    (estimate.x, estimate.y, estimate.z),
                    translation=(translation.x, translation.y, translation.z),
                    quaternion=(rotation.x, rotation.y, rotation.z, rotation.w),
                )
            except (TypeError, ValueError):
                continue
            key = (
                int(observation.producer_epoch_id),
                int(observation.camera_track_id),
            )
            self._depth_geometry[key] = _DepthGeometry(
                stamp_ns=now_ns,
                x=x,
                y=y,
                z=z,
                quality=estimate.quality,
            )
        oldest = now_ns - 2 * self._maximum_depth_age_ns
        self._depth_geometry = {
            key: value
            for key, value in self._depth_geometry.items()
            if value.stamp_ns >= oldest
        }

    def _geometry(self, message, now_ns):
        if not message.camera_track_id_valid:
            return None
        geometry = self._depth_geometry.get((
            int(message.camera_source_epoch_id),
            int(message.camera_track_id),
        ))
        if (
                geometry is None
                or now_ns - geometry.stamp_ns < 0
                or now_ns - geometry.stamp_ns > self._maximum_depth_age_ns):
            return None
        return geometry

    def _candidate(self, message, memory_epoch_id, now_ns):
        geometry = self._geometry(message, now_ns)
        lifecycle = (
            'confirmed'
            if message.lifecycle_state == SemanticObject.LIFECYCLE_CONFIRMED
            else 'other')
        if message.support_state == SemanticObject.SUPPORT_CAMERA_LIDAR:
            support = 'camera_lidar'
        elif geometry is not None:
            support = 'camera_depth'
        else:
            support = 'other'
        return ObjectCandidate(
            memory_epoch_id=int(memory_epoch_id),
            global_object_id=int(message.global_object_id),
            localization_epoch_id=(
                int(message.localization_epoch_id)
                if int(message.localization_epoch_id) != 0
                else self._localization_epoch_id),
            query_id=int(message.active_query_id),
            query_version=int(message.active_query_version),
            lifecycle=lifecycle,
            support=support,
            position_frame_id=(
                self._frame_id
                if geometry is not None
                else str(message.position_frame_id)),
            position_valid=(
                geometry is not None or bool(message.position_valid)),
            x=(geometry.x if geometry is not None else float(message.position.x)),
            y=(geometry.y if geometry is not None else float(message.position.y)),
            z=(geometry.z if geometry is not None else float(message.position.z)),
            relevance=float(message.task_relevance),
            uncertainty=float(message.uncertainty),
            last_seen_ns=_stamp_ns(message.last_seen),
        )

    def _on_ranking(self, message):
        now_ns = int(self.get_clock().now().nanoseconds)
        publish_stamp = self.get_clock().now().to_msg()
        spatial_output = SemanticObjectArray()
        spatial_output.header = message.header
        spatial_output.header.stamp = publish_stamp
        spatial_output.header.frame_id = self._frame_id
        spatial_output.memory_epoch_id = message.memory_epoch_id
        spatial_output.snapshot_sequence = message.snapshot_sequence
        candidates = []
        spatial_by_key = {}
        for item in message.objects:
            candidate = self._candidate(
                item, message.memory_epoch_id, now_ns)
            candidates.append(candidate)
            spatial = copy.deepcopy(item)
            geometry = self._geometry(item, now_ns)
            if geometry is not None:
                spatial.header.frame_id = self._frame_id
                spatial.localization_epoch_id = (
                    candidate.localization_epoch_id)
                spatial.position_frame_id = self._frame_id
                spatial.position_valid = True
                spatial.position.x = geometry.x
                spatial.position.y = geometry.y
                spatial.position.z = geometry.z
                spatial.position_covariance = [
                    0.04, 0.0, 0.0,
                    0.0, 0.04, 0.0,
                    0.0, 0.0, 0.09,
                ]
                spatial.support_state = SemanticObject.SUPPORT_CAMERA_ONLY
                spatial.association_confidence = geometry.quality
            if spatial.position_valid:
                spatial_output.objects.append(spatial)
            spatial_by_key[candidate.key] = spatial
        self._spatial_objects_publisher.publish(spatial_output)
        result = self._selector.update(SelectionSnapshot(
            now_ns=now_ns,
            query_id=self._query_id,
            query_version=self._query_version,
            candidates=tuple(candidates),
        ))
        output = SemanticObjectArray()
        output.header = message.header
        output.header.stamp = publish_stamp
        output.header.frame_id = self._frame_id
        output.memory_epoch_id = message.memory_epoch_id
        output.snapshot_sequence = message.snapshot_sequence
        if result.target is not None:
            selected = spatial_by_key.get(result.target.key)
            if selected is not None:
                selected.header.stamp = publish_stamp
                selected.header.frame_id = self._frame_id
                output.objects.append(selected)
        self._selected_publisher.publish(output)
        self._publish_diagnostics(result, now_ns)

    def _publish_diagnostics(self, result, now_ns):
        output = DiagnosticArray()
        output.header.stamp.sec = int(now_ns // 1_000_000_000)
        output.header.stamp.nanosec = int(now_ns % 1_000_000_000)
        status = DiagnosticStatus()
        status.name = 'semantic_search/phase4a_selector'
        status.hardware_id = 'fixed_base_test_only'
        status.level = (
            DiagnosticStatus.OK
            if result.status == 'READY'
            else DiagnosticStatus.WARN)
        status.message = result.reason
        target = result.target
        values = {
            'test_only': 'true',
            'status': result.status,
            'reason': result.reason,
            'confirmation_count': str(result.confirmation_count),
            'xy_spread_m': '{:.3f}'.format(result.xy_spread_m),
            'spatial_support': (
                target.support if target else 'none'),
            'memory_epoch_id': str(
                target.memory_epoch_id if target else 0),
            'global_object_id': str(
                target.global_object_id if target else 0),
            'localization_epoch_id': str(
                target.localization_epoch_id if target else 0),
            'query_id': str(target.query_id if target else self._query_id),
            'query_version': str(
                target.query_version if target else self._query_version),
        }
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()]
        output.status.append(status)
        self._diagnostic_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = Phase4ASelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
