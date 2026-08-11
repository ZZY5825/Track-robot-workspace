"""ROS adapter for the conservative Phase 3A test selector."""

import copy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from track_robot_interfaces.msg import (
    SemanticLocalizationState,
    SemanticObject,
    SemanticObjectArray,
    SemanticTask,
)

from .phase4a_selector import (
    classify_spatial_support,
    FixedBaseTargetSelector,
    ObjectCandidate,
    SelectionSnapshot,
    SelectorConfig,
)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Phase4ASelectorNode(Node):
    """Select one test target without enabling production calibration."""

    def __init__(self):
        super().__init__('phase4a_target_selector')
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
        localization_topic = self.declare_parameter(
            'localization_topic',
            '/semantic_search/phase4a/localization_state').value
        self._frame_id = str(self.declare_parameter(
            'frame_id', 'base_link').value)
        self._selector = FixedBaseTargetSelector(SelectorConfig(
            minimum_relevance=float(self.declare_parameter(
                'minimum_relevance', 0.50).value),
            retained_minimum_relevance=float(self.declare_parameter(
                'retained_minimum_relevance', 0.40).value),
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

    def _candidate(self, message, memory_epoch_id, now_ns):
        lifecycle = (
            'confirmed'
            if message.lifecycle_state == SemanticObject.LIFECYCLE_CONFIRMED
            else 'other')
        if message.support_state == SemanticObject.SUPPORT_CAMERA_LIDAR:
            public_support = 'camera_lidar'
        elif message.support_state == SemanticObject.SUPPORT_CAMERA_ONLY:
            public_support = 'camera_only'
        else:
            public_support = 'other'
        support = classify_spatial_support(
            support=public_support,
            position_valid=bool(message.position_valid),
            fallback_depth_available=False,
        )
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
            position_frame_id=str(message.position_frame_id),
            position_valid=bool(message.position_valid),
            x=float(message.position.x),
            y=float(message.position.y),
            z=float(message.position.z),
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
