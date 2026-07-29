"""ROS adapter for the planning-only Phase 4 standoff planner."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from track_robot_interfaces.msg import (
    SemanticLocalizationState,
    SemanticObject,
    SemanticObjectArray,
)
from visualization_msgs.msg import Marker, MarkerArray

from .approach_planning import (
    GridMap,
    Phase4Planner,
    PlannerConfig,
    PlanningContext,
    Pose2D,
    TargetCandidate,
)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _output_header(frame_id, now_ns):
    header = Header()
    header.frame_id = str(frame_id)
    header.stamp.sec = int(now_ns // 1_000_000_000)
    header.stamp.nanosec = int(now_ns % 1_000_000_000)
    return header


def _pose(pose):
    output = Pose()
    output.position.x = float(pose.x)
    output.position.y = float(pose.y)
    output.orientation.x, output.orientation.y = 0.0, 0.0
    output.orientation.z, output.orientation.w = _quaternion(pose.yaw)[2:]
    return output


class Phase4ApproachPlannerNode(Node):
    """Publish only inspection and visualization products."""

    def __init__(self):
        super().__init__('phase4_approach_planner')
        selected_target_topic = self.declare_parameter(
            'selected_target_topic',
            '/semantic_memory/best_candidate').value
        costmap_topic = self.declare_parameter(
            'costmap_topic', '/safety/local_obstacle_grid').value
        localization_topic = self.declare_parameter(
            'localization_topic',
            '/semantic_memory/localization_state').value
        approach_candidates_topic = self.declare_parameter(
            'approach_candidates_topic',
            '/semantic_search/phase4/approach_candidates').value
        selected_goal_topic = self.declare_parameter(
            'selected_goal_topic',
            '/semantic_search/phase4/selected_goal').value
        path_topic = self.declare_parameter(
            'path_topic', '/semantic_search/phase4/planned_path').value
        markers_topic = self.declare_parameter(
            'markers_topic', '/semantic_search/phase4/markers').value
        diagnostics_topic = self.declare_parameter(
            'diagnostics_topic',
            '/semantic_search/phase4/diagnostics').value
        planning_only = bool(self.declare_parameter(
            'planning_only', True).value)
        if not planning_only:
            raise ValueError('Phase 4 runtime supports planning_only=true only')

        config = PlannerConfig(
            standoff_distance=float(self.declare_parameter(
                'standoff_distance', 0.8).value),
            candidate_count=int(self.declare_parameter(
                'candidate_count', 16).value),
            minimum_target_relevance=float(self.declare_parameter(
                'minimum_target_relevance', 0.5).value),
            minimum_target_margin=float(self.declare_parameter(
                'minimum_target_margin', 0.08).value),
            maximum_target_uncertainty=float(self.declare_parameter(
                'maximum_target_uncertainty', 0.5).value),
            maximum_target_age_sec=float(self.declare_parameter(
                'maximum_target_age_sec', 0.75).value),
            maximum_map_age_sec=float(self.declare_parameter(
                'maximum_map_age_sec', 0.5).value),
            maximum_search_expansions=int(self.declare_parameter(
                'maximum_search_expansions', 30_000).value),
            occupied_threshold=int(self.declare_parameter(
                'occupied_threshold', 50).value),
            unknown_is_obstacle=bool(self.declare_parameter(
                'unknown_is_obstacle', True).value),
            enable_path_shortcutting=bool(self.declare_parameter(
                'enable_path_shortcutting', False).value),
        )
        planning_rate = float(self.declare_parameter(
            'planning_rate', 5.0).value)
        self._planner = Phase4Planner(config)
        self._targets = None
        self._costmap = None
        self._localization = None
        self._map_orientation_valid = False
        self._last_result = None

        selected_qos = QoSProfile(depth=1)
        selected_qos.reliability = ReliabilityPolicy.RELIABLE
        selected_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._target_subscription = self.create_subscription(
            SemanticObjectArray,
            selected_target_topic,
            self._on_target,
            selected_qos)
        self._costmap_subscription = self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self._on_costmap,
            5)
        self._localization_subscription = self.create_subscription(
            SemanticLocalizationState,
            localization_topic,
            self._on_localization,
            10)
        self._candidate_publisher = self.create_publisher(
            PoseArray, approach_candidates_topic, 5)
        self._goal_publisher = self.create_publisher(
            PoseStamped, selected_goal_topic, 5)
        self._path_publisher = self.create_publisher(
            Path, path_topic, 5)
        self._marker_publisher = self.create_publisher(
            MarkerArray, markers_topic, 5)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10)
        self._timer = self.create_timer(
            1.0 / max(1.0, planning_rate), self._plan)
        self.get_logger().warn(
            'Phase 4 started in planning-only mode; no execution interface exists')

    def _on_target(self, message):
        self._targets = message

    def _on_costmap(self, message):
        orientation = message.info.origin.orientation
        self._map_orientation_valid = (
            abs(orientation.x) < 1e-6
            and abs(orientation.y) < 1e-6
            and abs(orientation.z) < 1e-6
            and abs(orientation.w - 1.0) < 1e-6)
        self._costmap = message

    def _on_localization(self, message):
        self._localization = message

    @staticmethod
    def _target(message, memory_epoch_id):
        lifecycle = (
            'confirmed'
            if message.lifecycle_state == SemanticObject.LIFECYCLE_CONFIRMED
            else 'lost')
        return TargetCandidate(
            memory_epoch_id=int(memory_epoch_id),
            global_object_id=int(message.global_object_id),
            localization_epoch_id=int(message.localization_epoch_id),
            query_id=int(message.active_query_id),
            query_version=int(message.active_query_version),
            position_frame_id=str(message.position_frame_id),
            position_valid=bool(message.position_valid),
            x=float(message.position.x),
            y=float(message.position.y),
            z=float(message.position.z),
            lifecycle_state=lifecycle,
            task_relevance=float(message.task_relevance),
            uncertainty=float(message.uncertainty),
            last_seen_ns=_stamp_ns(message.last_seen),
        )

    @staticmethod
    def _grid(message):
        return GridMap(
            frame_id=str(message.header.frame_id),
            stamp_ns=_stamp_ns(message.header.stamp),
            resolution=float(message.info.resolution),
            width=int(message.info.width),
            height=int(message.info.height),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            data=tuple(int(value) for value in message.data),
        )

    def _plan(self):
        started = time.perf_counter_ns()
        now_ns = int(self.get_clock().now().nanoseconds)
        if (
                self._targets is None
                or self._costmap is None
                or self._localization is None):
            self._publish_failure(
                'waiting_for_inputs', 0.0, now_ns)
            return
        if not self._map_orientation_valid:
            self._publish_failure(
                'unsupported_map_orientation', 0.0, now_ns)
            return
        context = PlanningContext(
            now_ns=now_ns,
            localization_epoch_id=int(
                self._localization.localization_epoch_id),
            localization_healthy=bool(self._localization.local_healthy),
            robot_x=0.0,
            robot_y=0.0,
            target_candidates=tuple(
                self._target(item, self._targets.memory_epoch_id)
                for item in self._targets.objects),
            grid=self._grid(self._costmap),
        )
        result = self._planner.plan(context)
        latency_ms = (
            time.perf_counter_ns() - started) / 1_000_000.0
        self._last_result = result
        self._publish(result, latency_ms, now_ns)

    def _header(self, now_ns):
        return _output_header(self._costmap.header.frame_id, now_ns)

    def _publish_failure(self, reason, latency_ms, now_ns):
        from .approach_planning import PlanResult
        self._publish(
            PlanResult(status='FAIL', reason=reason),
            latency_ms, now_ns)

    def _publish(self, result, latency_ms, now_ns):
        header = self._header(now_ns) if self._costmap is not None else None
        if header is not None:
            candidates = PoseArray()
            candidates.header = header
            candidates.poses = [
                _pose(item) for item in result.approach_candidates]
            self._candidate_publisher.publish(candidates)

            path = Path()
            path.header = header
            for item in result.path:
                stamped = PoseStamped()
                stamped.header = header
                stamped.pose = _pose(item)
                path.poses.append(stamped)
            self._path_publisher.publish(path)

            if result.selected_goal is not None:
                goal = PoseStamped()
                goal.header = header
                goal.pose = _pose(result.selected_goal)
                self._goal_publisher.publish(goal)
            self._publish_markers(result, header)
        self._publish_diagnostics(result, latency_ms, now_ns)

    def _publish_markers(self, result, header):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        if result.target is not None:
            target = Marker()
            target.header = header
            target.ns = 'phase4_target'
            target.id = 0
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position.x = result.target.x
            target.pose.position.y = result.target.y
            target.pose.position.z = result.target.z
            target.pose.orientation.w = 1.0
            target.scale.x = target.scale.y = target.scale.z = 0.25
            target.color.r = 0.1
            target.color.g = 0.9
            target.color.b = 0.2
            target.color.a = 0.9
            output.markers.append(target)
        for index, candidate in enumerate(result.approach_candidates):
            marker = Marker()
            marker.header = header
            marker.ns = 'phase4_approach_candidates'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = _pose(candidate)
            marker.scale.x = 0.25
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 0.2
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.7
            output.markers.append(marker)
        if result.selected_goal is not None:
            goal = Marker()
            goal.header = header
            goal.ns = 'phase4_selected_goal'
            goal.id = 0
            goal.type = Marker.ARROW
            goal.action = Marker.ADD
            goal.pose = _pose(result.selected_goal)
            goal.scale.x = 0.45
            goal.scale.y = 0.12
            goal.scale.z = 0.12
            goal.color.r = 1.0
            goal.color.g = 0.55
            goal.color.b = 0.0
            goal.color.a = 1.0
            output.markers.append(goal)
        self._marker_publisher.publish(output)

    def _publish_diagnostics(self, result, latency_ms, now_ns):
        output = DiagnosticArray()
        output.header.stamp.sec = int(now_ns // 1_000_000_000)
        output.header.stamp.nanosec = int(now_ns % 1_000_000_000)
        status = DiagnosticStatus()
        status.name = 'semantic_search/phase4_planning'
        status.hardware_id = 'planning_only'
        status.level = (
            DiagnosticStatus.OK
            if result.status == 'PASS'
            else DiagnosticStatus.WARN)
        status.message = result.reason
        target = result.target
        values = {
            'schema_version': 'phase4_planning/1.0.0',
            'planning_only': 'true',
            'status': result.status,
            'reason': result.reason,
            'latency_ms': '{:.3f}'.format(latency_ms),
            'candidate_count': str(len(result.approach_candidates)),
            'path_pose_count': str(len(result.path)),
            'raw_path_pose_count': str(result.raw_path_pose_count),
            'path_segment_count': str(max(0, len(result.path) - 1)),
            'path_length_m': '{:.3f}'.format(result.path_length_m),
            'path_shortcut_applied': str(
                result.path_shortcut_applied).lower(),
            'search_expansions': str(result.search_expansions),
            'search_budget_exhausted': str(
                result.search_budget_exhausted).lower(),
            'memory_epoch_id': str(
                target.memory_epoch_id if target else 0),
            'global_object_id': str(
                target.global_object_id if target else 0),
            'localization_epoch_id': str(
                target.localization_epoch_id if target else 0),
            'query_id': str(target.query_id if target else 0),
            'query_version': str(
                target.query_version if target else 0),
        }
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()]
        output.status.append(status)
        self._diagnostic_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = Phase4ApproachPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
