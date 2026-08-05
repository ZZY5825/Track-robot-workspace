"""Phase 5A task lifecycle and bounded observation manager."""

from dataclasses import dataclass, field
import json
import math
import threading
import time

from builtin_interfaces.msg import Duration as DurationMessage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from track_robot_interfaces.action import SearchForObject
from track_robot_interfaces.msg import (
    SearchMotionIntent,
    SemanticLocalizationState,
    SemanticObject,
    SemanticObjectArray,
    SemanticTask,
)
from visualization_msgs.msg import Marker, MarkerArray

from .active_search_evidence import (
    BoundedEvidenceBook,
    EvidenceConfig,
    EvidenceStatus,
    ObjectEvidenceKey,
    ViewEvidence,
)
from .active_search_policy import (
    BoundedHeadingPolicy,
    SearchMode,
    SearchPolicyConfig,
    SearchState,
)
from .query_portal import QueryIdAllocator, QueryRequest, parse_diagnostic


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _duration_message(seconds):
    bounded = max(0.0, float(seconds))
    whole = int(bounded)
    return DurationMessage(
        sec=whole,
        nanosec=int(round((bounded - whole) * 1e9)),
    )


def _feedback_reason(context):
    if context.terminal_status is not None and context.terminal_reason:
        return context.terminal_reason
    return context.state.value


def _rotation_transition_in_progress(state):
    return state in (
        SearchState.WAITING_FOR_AUTHORIZATION,
        SearchState.ROTATING,
        SearchState.SETTLING,
    )


@dataclass
class _SearchContext:
    goal_handle: object
    request: QueryRequest
    mode: SearchMode
    allow_rotation: bool
    deadline_monotonic: float
    started_monotonic: float
    query_start_ros_sec: float
    policy: BoundedHeadingPolicy
    evidence: BoundedEvidenceBook
    state: SearchState = SearchState.QUERY_ACCEPTED
    task_received: bool = False
    perception_fresh: bool = False
    model_ready: bool = True
    localization_epoch_id: int = 0
    memory_epoch_id: int = 0
    ranking_updates: int = 0
    empty_ranking_updates: int = 0
    candidate_ids: set = field(default_factory=set)
    best_score: float = 0.0
    best_uncertainty: float = 1.0
    horizontal_fov_deg: object = None
    selected_key: object = None
    selected_stamp_sec: float = 0.0
    searched_headings_deg: list = field(default_factory=list)
    current_relative_heading_deg: float = 0.0
    settling_started_monotonic: object = None
    latest_odom_monotonic: object = None
    latest_angular_speed_rad_s: object = None
    terminal_status: object = None
    terminal_reason: str = ''
    event: threading.Event = field(default_factory=threading.Event)


class ActiveSearchManager(Node):
    """Own one search task without owning object IDs or motion execution."""

    ACTION_NAME = '/semantic_search/search_for_object'
    INTENT_TOPIC = '/semantic_search/search_motion_intent'

    def __init__(self):
        super().__init__('active_search_manager')
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._goal_reserved = False
        self._context = None
        self._query_allocator = QueryIdAllocator()

        self._mode = SearchMode.parse(self.declare_parameter(
            'search_mode', SearchMode.PASSIVE_ONLY.value).value)
        self._execution_enabled = bool(self.declare_parameter(
            'active_search_execution_enabled', False).value)
        if not self._mode.available_in_phase5a:
            raise ValueError(
                '{} is not available in Phase 5A'.format(self._mode.value))
        if self._mode.motion_enabled and not self._execution_enabled:
            raise ValueError(
                'ROTATION_SUPERVISED requires '
                'active_search_execution_enabled=true')

        self._observation_timeout_sec = float(self.declare_parameter(
            'observation_timeout_sec', 4.5).value)
        self._settle_duration_sec = float(self.declare_parameter(
            'settle_duration_sec', 0.75).value)
        self._settle_angular_speed_rad_s = float(self.declare_parameter(
            'settle_angular_speed_rad_s', 0.03).value)
        self._default_task_timeout_sec = float(self.declare_parameter(
            'default_task_timeout_sec', 60.0).value)
        self._navigation_frame = str(self.declare_parameter(
            'navigation_frame', 'odom').value)

        heading_offsets = tuple(float(value) for value in self.declare_parameter(
            'heading_offsets_deg', [45.0, 90.0, 0.0, -45.0, -90.0]
        ).value)
        evidence_headings = tuple(float(value) for value in self.declare_parameter(
            'evidence_headings_deg', [45.0, 90.0, -45.0, -90.0]
        ).value)
        self._policy_config = SearchPolicyConfig(
            heading_offsets_deg=heading_offsets,
            evidence_headings_deg=evidence_headings,
            maximum_individual_rotation_deg=float(self.declare_parameter(
                'maximum_individual_rotation_deg', 90.0).value),
            maximum_cumulative_rotation_deg=float(self.declare_parameter(
                'maximum_cumulative_rotation_deg', 270.0).value),
            maximum_angular_speed_rad_s=float(self.declare_parameter(
                'maximum_angular_speed_rad_s', 0.30).value),
            duplicate_heading_tolerance_deg=float(self.declare_parameter(
                'duplicate_heading_tolerance_deg', 10.0).value),
            default_deadline_sec=self._default_task_timeout_sec,
        )
        self._evidence_config = EvidenceConfig(
            confirmation_snapshots=int(self.declare_parameter(
                'confirmation_snapshots', 3).value),
            duplicate_heading_tolerance_deg=(
                self._policy_config.duplicate_heading_tolerance_deg),
            evidence_ttl_sec=float(self.declare_parameter(
                'evidence_ttl_sec', 12.0).value),
            maximum_records=int(self.declare_parameter(
                'maximum_evidence_records', 40).value),
        )

        query_topic = str(self.declare_parameter(
            'query_topic', '/semantic_search/query').value)
        perception_topic = str(self.declare_parameter(
            'perception_diagnostics_topic',
            '/semantic_search/perception_diagnostics').value)
        tasks_topic = str(self.declare_parameter(
            'tasks_topic', '/semantic_memory/tasks').value)
        active_objects_topic = str(self.declare_parameter(
            'active_objects_topic', '/semantic_memory/active_objects').value)
        ranking_topic = str(self.declare_parameter(
            'ranking_topic', '/semantic_memory/diagnostic_ranking').value)
        selected_target_topic = str(self.declare_parameter(
            'selected_target_topic',
            '/semantic_search/phase4a/selected_target').value)
        camera_info_topic = str(self.declare_parameter(
            'camera_info_topic', '/zed/zed_node/left/camera_info').value)
        localization_topic = str(self.declare_parameter(
            'localization_topic',
            '/semantic_search/phase4a/localization_state').value)
        odometry_topic = str(self.declare_parameter(
            'odometry_topic', '/odom').value)
        motion_status_topic = str(self.declare_parameter(
            'motion_status_topic',
            '/semantic_search/active_search/motion_status').value)
        diagnostics_topic = str(self.declare_parameter(
            'diagnostics_topic',
            '/semantic_search/active_search/diagnostics').value)
        markers_topic = str(self.declare_parameter(
            'markers_topic',
            '/semantic_search/active_search/markers').value)

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        selected_qos = QoSProfile(depth=1)
        selected_qos.reliability = ReliabilityPolicy.RELIABLE
        selected_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._query_publisher = self.create_publisher(
            String, query_topic, reliable)
        self._intent_publisher = self.create_publisher(
            SearchMotionIntent, self.INTENT_TOPIC, reliable)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, reliable)
        self._marker_publisher = self.create_publisher(
            MarkerArray, markers_topic, reliable)

        self.create_subscription(
            String, perception_topic, self._on_perception_diagnostic,
            reliable, callback_group=self._callback_group)
        self.create_subscription(
            SemanticTask, tasks_topic, self.on_task,
            reliable, callback_group=self._callback_group)
        self.create_subscription(
            SemanticObjectArray, active_objects_topic,
            self.on_active_objects, selected_qos,
            callback_group=self._callback_group)
        self.create_subscription(
            SemanticObjectArray, ranking_topic, self.on_ranking,
            reliable, callback_group=self._callback_group)
        self.create_subscription(
            SemanticObjectArray, selected_target_topic,
            self.on_selected_target, selected_qos,
            callback_group=self._callback_group)
        self.create_subscription(
            CameraInfo, camera_info_topic, self.on_camera_info,
            qos_profile_sensor_data, callback_group=self._callback_group)
        self.create_subscription(
            SemanticLocalizationState, localization_topic,
            self.on_localization, selected_qos,
            callback_group=self._callback_group)
        self.create_subscription(
            Odometry, odometry_topic, self.on_odom,
            reliable, callback_group=self._callback_group)
        self.create_subscription(
            DiagnosticArray, motion_status_topic, self.on_motion_status,
            reliable, callback_group=self._callback_group)

        self._action_server = ActionServer(
            self,
            SearchForObject,
            self.ACTION_NAME,
            execute_callback=self.execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().warn(
            'Phase 5A search_mode={} execution_enabled={}'.format(
                self._mode.value, str(self._execution_enabled).lower()))

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()

    def _goal_callback(self, goal_request):
        if not str(goal_request.query_text).strip():
            return GoalResponse.REJECT
        if (
                goal_request.allow_rotation and
                float(goal_request.maximum_rotation_angle) <= 0.0):
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _new_context(self, goal_handle):
        request = QueryRequest.create(
            goal_handle.request.query_text,
            self._query_allocator.next_id(),
            1,
        )
        requested_timeout = _duration_seconds(goal_handle.request.timeout)
        timeout = (
            requested_timeout if requested_timeout > 0.0
            else self._default_task_timeout_sec)
        timeout = min(timeout, self._default_task_timeout_sec)
        maximum_angle_deg = math.degrees(
            float(goal_handle.request.maximum_rotation_angle))
        if not goal_handle.request.allow_rotation:
            maximum_angle_deg = 90.0
        policy_config = SearchPolicyConfig.defaults(
            maximum_rotation_angle_deg=max(1e-6, maximum_angle_deg))
        policy_config = SearchPolicyConfig(
            heading_offsets_deg=tuple(
                value for value in self._policy_config.heading_offsets_deg
                if abs(value) <= maximum_angle_deg
            ) if goal_handle.request.allow_rotation else tuple(),
            evidence_headings_deg=tuple(
                value for value in self._policy_config.evidence_headings_deg
                if abs(value) <= maximum_angle_deg
            ) if goal_handle.request.allow_rotation else tuple(),
            maximum_individual_rotation_deg=min(
                self._policy_config.maximum_individual_rotation_deg,
                policy_config.maximum_individual_rotation_deg),
            maximum_cumulative_rotation_deg=(
                self._policy_config.maximum_cumulative_rotation_deg),
            maximum_angular_speed_rad_s=(
                self._policy_config.maximum_angular_speed_rad_s),
            duplicate_heading_tolerance_deg=(
                self._policy_config.duplicate_heading_tolerance_deg),
            default_deadline_sec=timeout,
        )
        now_monotonic = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        return _SearchContext(
            goal_handle=goal_handle,
            request=request,
            mode=self._mode,
            allow_rotation=bool(goal_handle.request.allow_rotation),
            deadline_monotonic=now_monotonic + timeout,
            started_monotonic=now_monotonic,
            query_start_ros_sec=now_ros,
            policy=BoundedHeadingPolicy(policy_config),
            evidence=BoundedEvidenceBook(self._evidence_config),
            state=SearchState.PASSIVE_OBSERVATION,
        )

    def execute_goal(self, goal_handle):
        try:
            context = self._new_context(goal_handle)
        except (TypeError, ValueError) as error:
            with self._lock:
                self._goal_reserved = False
            goal_handle.abort()
            return self._result(
                SearchForObject.Result.INTERNAL_FAULT,
                0,
                None,
                'invalid_goal:{}'.format(error),
                None,
            )

        with self._lock:
            self._context = context
        self._query_publisher.publish(String(data=context.request.payload))
        self._publish_diagnostics(context)

        try:
            while context.terminal_status is None:
                if goal_handle.is_cancel_requested:
                    with self._lock:
                        self.publish_stop_intent('action_cancelled')
                        self._terminate_locked(
                            SearchForObject.Result.CANCELLED,
                            SearchState.CANCELLED,
                            'action_cancelled')
                    break
                if time.monotonic() >= context.deadline_monotonic:
                    with self._lock:
                        self.publish_stop_intent('task_timeout')
                        self._terminate_locked(
                            SearchForObject.Result.TIMEOUT,
                            SearchState.TIMEOUT,
                            'task_timeout')
                    break
                with self._lock:
                    self._advance_settling_locked()
                self._publish_feedback(context)
                context.event.wait(timeout=0.05)
                context.event.clear()

            with self._lock:
                self.publish_stop_intent('task_terminal')
            result = self._result(
                context.terminal_status,
                context.request.query_id,
                context.selected_key,
                context.terminal_reason,
                context,
            )
            if context.terminal_status == SearchForObject.Result.CANCELLED:
                goal_handle.canceled()
            elif context.terminal_status in (
                    SearchForObject.Result.CONFIRMED,
                    SearchForObject.Result.NOT_FOUND,
                    SearchForObject.Result.UNCERTAIN,
                    SearchForObject.Result.SEARCH_SPACE_EXHAUSTED):
                goal_handle.succeed()
            else:
                goal_handle.abort()
            self._publish_diagnostics(context)
            return result
        finally:
            with self._lock:
                if self._context is context:
                    self._context = None
                self._goal_reserved = False

    def _result(self, status, query_id, selected_key, reason, context):
        result = SearchForObject.Result()
        result.status = int(status)
        result.query_id = int(query_id)
        result.selected_object_valid = selected_key is not None
        result.selected_global_object_id = (
            int(selected_key.global_object_id)
            if selected_key is not None else 0)
        result.selected_observation_valid = False
        evidence = {
            'schema_version': 'phase5a_evidence_summary/1.0.0',
            'reason': str(reason),
            'status': int(status),
        }
        if selected_key is not None:
            evidence['target_reference'] = {
                'memory_epoch_id': selected_key.memory_epoch_id,
                'global_object_id': selected_key.global_object_id,
                'localization_epoch_id': selected_key.localization_epoch_id,
                'query_id': selected_key.query_id,
                'query_version': selected_key.query_version,
            }
        if context is not None:
            evidence['searched_headings_deg'] = list(
                context.searched_headings_deg)
            evidence['candidate_count'] = len(context.candidate_ids)
            evidence['best_score'] = context.best_score
            evidence['uncertainty'] = context.best_uncertainty
        result.evidence_summary = json.dumps(
            evidence, sort_keys=True, separators=(',', ':'))
        return result

    def _publish_feedback(self, context):
        feedback = SearchForObject.Feedback()
        feedback.query_id = int(context.request.query_id)
        if context.state in (
                SearchState.QUERY_ACCEPTED,
                SearchState.PASSIVE_OBSERVATION,
                SearchState.EVALUATING_EVIDENCE):
            feedback.phase = SearchForObject.Feedback.PASSIVE_OBSERVATION
        elif context.state in (
                SearchState.ACTIVE_SEARCH_REQUIRED,
                SearchState.SELECTING_VIEW,
                SearchState.WAITING_FOR_AUTHORIZATION,
                SearchState.ROTATING,
                SearchState.SETTLING,
                SearchState.OBSERVING,
                SearchState.UPDATING_MEMORY):
            feedback.phase = SearchForObject.Feedback.ROTATION_VERIFICATION
        elif context.state is SearchState.HANDOFF_TO_PHASE4:
            feedback.phase = SearchForObject.Feedback.FINALISING
        else:
            feedback.phase = SearchForObject.Feedback.TERMINAL
        feedback.elapsed = _duration_message(
            time.monotonic() - context.started_monotonic)
        feedback.searched_angle = math.radians(
            context.policy.cumulative_rotation_deg)
        feedback.candidate_count = len(context.candidate_ids)
        feedback.best_candidate_score = float(context.best_score)
        feedback.current_reason = _feedback_reason(context)
        context.goal_handle.publish_feedback(feedback)

    def _terminate_locked(self, status, state, reason):
        context = self._context
        if context is None or context.terminal_status is not None:
            return
        context.terminal_status = int(status)
        context.state = state
        context.terminal_reason = str(reason)
        context.event.set()

    def _context_matches(self, query_id, query_version):
        context = self._context
        return (
            context is not None and
            int(query_id) == context.request.query_id and
            int(query_version) == context.request.query_version)

    def _bind_evidence_locked(self, key):
        context = self._context
        if context is None:
            return False
        if not context.evidence.is_bound:
            if (
                    context.memory_epoch_id not in (0, key.memory_epoch_id) or
                    context.localization_epoch_id not in (
                        0, key.localization_epoch_id)):
                self._terminate_locked(
                    SearchForObject.Result.UNCERTAIN,
                    SearchState.UNCERTAIN,
                    'selected_target_domain_mismatch')
                return False
            context.memory_epoch_id = key.memory_epoch_id
            context.localization_epoch_id = key.localization_epoch_id
            context.evidence.bind_domain(*key.domain)
            return True
        if context.evidence.domain_changed(*key.domain):
            self._terminate_locked(
                SearchForObject.Result.UNCERTAIN,
                SearchState.UNCERTAIN,
                'semantic_or_localization_domain_changed')
            return False
        return True

    def _evaluate_locked(self):
        context = self._context
        if context is None or context.terminal_status is not None:
            return
        decision = context.evidence.evaluate(search_exhausted=False)
        if decision.status is EvidenceStatus.CONFIRMED:
            if context.selected_key != decision.selected_key:
                return
            context.state = SearchState.HANDOFF_TO_PHASE4
            context.selected_key = decision.selected_key
            self._terminate_locked(
                SearchForObject.Result.CONFIRMED,
                SearchState.CONFIRMED,
                'fresh_phase3_target_handed_to_phase4')
            return
        if context.ranking_updates < self._evidence_config.confirmation_snapshots:
            return
        context.state = SearchState.ACTIVE_SEARCH_REQUIRED
        if context.mode is SearchMode.PASSIVE_ONLY or not context.allow_rotation:
            status = (
                SearchForObject.Result.NOT_FOUND
                if not context.candidate_ids
                else SearchForObject.Result.UNCERTAIN)
            state = (
                SearchState.NOT_FOUND
                if not context.candidate_ids else SearchState.UNCERTAIN)
            reason = (
                'passive_observation_without_candidates'
                if not context.candidate_ids
                else 'passive_observation_ambiguous')
            self._terminate_locked(status, state, reason)
            return
        if context.mode is SearchMode.SEARCH_SHADOW:
            self._publish_shadow_sequence_locked()
            status = (
                SearchForObject.Result.NOT_FOUND
                if not context.candidate_ids
                else SearchForObject.Result.UNCERTAIN)
            state = (
                SearchState.NOT_FOUND
                if not context.candidate_ids else SearchState.UNCERTAIN)
            self._terminate_locked(status, state, 'search_shadow_complete')
            return
        self._publish_next_rotation_locked()

    def _new_intent(self, context, decision, rotation_permitted, reason):
        intent = SearchMotionIntent()
        now = self.get_clock().now()
        intent.header.stamp = now.to_msg()
        intent.header.frame_id = self._navigation_frame
        intent.query_id = int(context.request.query_id)
        intent.intent = SearchMotionIntent.INTENT_ROTATE_VERIFY
        intent.target_bearing = math.radians(decision.rotation_delta_deg)
        intent.maximum_rotation_angle = math.radians(
            self._policy_config.maximum_individual_rotation_deg)
        intent.maximum_angular_speed = (
            self._policy_config.maximum_angular_speed_rad_s)
        remaining = max(0.0, context.deadline_monotonic - time.monotonic())
        intent.deadline = (now + Duration(seconds=remaining)).to_msg()
        intent.rotation_permitted = bool(rotation_permitted)
        intent.forward_permitted = False
        intent.reason = str(reason)
        return intent

    def _publish_shadow_sequence_locked(self):
        context = self._context
        decisions = context.policy.complete_sequence(initial_yaw=0.0)
        for decision in decisions:
            self._intent_publisher.publish(self._new_intent(
                context, decision, False, 'search_shadow_proposal'))
            context.searched_headings_deg.append(
                decision.relative_heading_deg)
        self._publish_markers(context)

    def _publish_next_rotation_locked(self):
        context = self._context
        decision = context.policy.next_heading(
            SearchState.SELECTING_VIEW,
            0.0,
            math.radians(context.current_relative_heading_deg))
        if decision is None:
            evidence = context.evidence.evaluate(search_exhausted=True)
            if evidence.status is EvidenceStatus.NOT_FOUND:
                self._terminate_locked(
                    SearchForObject.Result.NOT_FOUND,
                    SearchState.NOT_FOUND,
                    evidence.reason)
            else:
                self._terminate_locked(
                    SearchForObject.Result.SEARCH_SPACE_EXHAUSTED,
                    SearchState.SEARCH_SPACE_EXHAUSTED,
                    evidence.reason)
            return
        context.state = SearchState.WAITING_FOR_AUTHORIZATION
        self._intent_publisher.publish(self._new_intent(
            context,
            decision,
            self._execution_enabled,
            'bounded_rotation_verification'))
        context.searched_headings_deg.append(decision.relative_heading_deg)
        self._publish_markers(context)

    def publish_stop_intent(self, reason):
        context = self._context
        if context is None:
            return
        intent = SearchMotionIntent()
        intent.header.stamp = self.get_clock().now().to_msg()
        intent.header.frame_id = self._navigation_frame
        intent.query_id = int(context.request.query_id)
        intent.intent = SearchMotionIntent.INTENT_STOP
        intent.deadline = intent.header.stamp
        intent.rotation_permitted = False
        intent.forward_permitted = False
        intent.reason = str(reason)
        self._intent_publisher.publish(intent)

    def _on_perception_diagnostic(self, message):
        diagnostic = parse_diagnostic(message.data)
        if diagnostic is None:
            return
        with self._lock:
            context = self._context
            if context is None or not diagnostic.matches(context.request):
                return
            context.perception_fresh = True
            context.model_ready = diagnostic.model_ready is not False
            if not context.model_ready:
                self._terminate_locked(
                    SearchForObject.Result.MODEL_UNAVAILABLE,
                    SearchState.MODEL_UNAVAILABLE,
                    diagnostic.reason or 'model_unavailable')
            context.event.set()

    def on_task(self, message):
        with self._lock:
            if not self._context_matches(message.query_id, message.query_version):
                return
            self._context.task_received = True
            self._context.state = SearchState.PASSIVE_OBSERVATION
            self._context.event.set()

    def on_active_objects(self, message):
        with self._lock:
            context = self._context
            if context is None or not context.task_received:
                return
            if context.memory_epoch_id and (
                    int(message.memory_epoch_id) != context.memory_epoch_id):
                self._terminate_locked(
                    SearchForObject.Result.UNCERTAIN,
                    SearchState.UNCERTAIN,
                    'memory_epoch_changed')
                return
            context.memory_epoch_id = int(message.memory_epoch_id)
            context.event.set()

    def on_ranking(self, message):
        with self._lock:
            context = self._context
            if context is None or not context.task_received:
                return
            if _rotation_transition_in_progress(context.state):
                context.event.set()
                return
            source_stamp = _stamp_seconds(message.header.stamp)
            if source_stamp and source_stamp < context.query_start_ros_sec:
                return
            candidates = [
                item for item in message.objects
                if self._context_matches(
                    item.active_query_id, item.active_query_version)
            ]
            context.ranking_updates += 1
            if not candidates:
                context.empty_ranking_updates += 1
            for item in candidates:
                context.candidate_ids.add(int(item.global_object_id))
            if candidates:
                best = max(candidates, key=lambda item: item.task_relevance)
                context.best_score = float(best.task_relevance)
                context.best_uncertainty = float(best.uncertainty)
            context.state = SearchState.EVALUATING_EVIDENCE
            self._evaluate_locked()
            context.event.set()

    def on_selected_target(self, message):
        with self._lock:
            context = self._context
            if context is None or not context.task_received:
                return
            if _rotation_transition_in_progress(context.state):
                context.event.set()
                return
            selected = [
                item for item in message.objects
                if self._context_matches(
                    item.active_query_id, item.active_query_version)
            ]
            if len(selected) != 1:
                return
            item = selected[0]
            if (
                    int(message.memory_epoch_id) < 1 or
                    int(item.global_object_id) < 1 or
                    int(item.localization_epoch_id) < 1):
                self._terminate_locked(
                    SearchForObject.Result.LOCALIZATION_UNAVAILABLE,
                    SearchState.LOCALIZATION_UNAVAILABLE,
                    'selected_target_missing_domain_reference')
                return
            key = ObjectEvidenceKey(
                memory_epoch_id=int(message.memory_epoch_id),
                global_object_id=int(item.global_object_id),
                localization_epoch_id=int(item.localization_epoch_id),
                query_id=int(item.active_query_id),
                query_version=int(item.active_query_version),
            )
            if not self._bind_evidence_locked(key):
                return
            source_stamp = _stamp_seconds(item.header.stamp)
            if not source_stamp:
                source_stamp = _stamp_seconds(message.header.stamp)
            evidence = ViewEvidence(
                key=key,
                heading_deg=context.current_relative_heading_deg,
                horizontal_fov_deg=context.horizontal_fov_deg,
                source_stamp_sec=source_stamp,
                task_relevance=float(item.task_relevance),
                uncertainty=float(item.uncertainty),
                phase3_selected=True,
            )
            if context.evidence.add(evidence, context.query_start_ros_sec):
                context.selected_key = key
                context.selected_stamp_sec = source_stamp
                context.candidate_ids.add(key.global_object_id)
                context.best_score = float(item.task_relevance)
                context.best_uncertainty = float(item.uncertainty)
                self._evaluate_locked()
            context.event.set()

    def on_camera_info(self, message):
        if int(message.width) < 1 or float(message.k[0]) <= 0.0:
            return
        horizontal_fov = math.degrees(
            2.0 * math.atan(float(message.width) / (2.0 * float(message.k[0]))))
        with self._lock:
            if self._context is not None:
                self._context.horizontal_fov_deg = horizontal_fov
                self._context.event.set()

    def on_localization(self, message):
        with self._lock:
            context = self._context
            if context is None:
                return
            if not message.local_healthy:
                self._terminate_locked(
                    SearchForObject.Result.LOCALIZATION_UNAVAILABLE,
                    SearchState.LOCALIZATION_UNAVAILABLE,
                    message.reason or 'localization_unhealthy')
                return
            epoch = int(message.localization_epoch_id)
            if context.localization_epoch_id and epoch != context.localization_epoch_id:
                self._terminate_locked(
                    SearchForObject.Result.UNCERTAIN,
                    SearchState.UNCERTAIN,
                    'localization_epoch_changed')
                return
            context.localization_epoch_id = epoch
            context.event.set()

    def on_odom(self, message):
        with self._lock:
            context = self._context
            if context is None:
                return
            context.latest_odom_monotonic = time.monotonic()
            context.latest_angular_speed_rad_s = float(
                message.twist.twist.angular.z)
            context.event.set()

    def _advance_settling_locked(self):
        context = self._context
        if (
                context is None or
                context.state is not SearchState.SETTLING or
                context.settling_started_monotonic is None or
                context.latest_odom_monotonic is None or
                context.latest_angular_speed_rad_s is None):
            return
        now = time.monotonic()
        if now - context.latest_odom_monotonic > 0.25:
            self._terminate_locked(
                SearchForObject.Result.LOCALIZATION_UNAVAILABLE,
                SearchState.LOCALIZATION_UNAVAILABLE,
                'odometry_stale_while_settling')
            return
        if now - context.settling_started_monotonic < self._settle_duration_sec:
            return
        if abs(context.latest_angular_speed_rad_s) > (
                self._settle_angular_speed_rad_s):
            return
        context.state = SearchState.OBSERVING
        context.query_start_ros_sec = (
            self.get_clock().now().nanoseconds * 1e-9)
        context.ranking_updates = 0
        context.empty_ranking_updates = 0
        context.settling_started_monotonic = None
        context.event.set()

    def on_motion_status(self, message):
        with self._lock:
            context = self._context
            if context is None or not message.status:
                return
            latest = message.status[0]
            values = {item.key: item.value for item in latest.values}
            if values.get('query_id') != str(context.request.query_id):
                return
            state = values.get('state', '')
            reason = values.get('reason', context.terminal_reason)
            if state == 'SPIN_COMPLETED':
                decision = context.policy.pending_decision
                if decision is None:
                    self._terminate_locked(
                        SearchForObject.Result.INTERNAL_FAULT,
                        SearchState.INTERNAL_FAULT,
                        'spin_completed_without_pending_heading')
                    return
                context.policy.mark_completed(decision)
                context.current_relative_heading_deg = (
                    decision.relative_heading_deg)
                context.state = SearchState.SETTLING
                context.settling_started_monotonic = time.monotonic()
                context.latest_angular_speed_rad_s = None
            elif state in ('AUTHORIZED', 'SPIN_REQUESTED', 'SPINNING'):
                context.state = SearchState.ROTATING
            elif state in (
                    'SAFETY_REJECTED', 'WATCHDOG_STOP',
                    'NAV2_UNAVAILABLE'):
                self._terminate_locked(
                    SearchForObject.Result.SAFETY_REJECTED,
                    SearchState.SAFETY_REJECTED,
                    reason or state.lower())
            elif state in ('SPIN_FAILED', 'REJECTED'):
                self._terminate_locked(
                    SearchForObject.Result.INTERNAL_FAULT,
                    SearchState.INTERNAL_FAULT,
                    reason or state.lower())
            context.event.set()

    def _publish_diagnostics(self, context):
        status = DiagnosticStatus()
        status.name = 'phase5a_active_search'
        status.hardware_id = 'track_robot'
        status.level = (
            DiagnosticStatus.OK
            if context.terminal_status is None or
            context.terminal_status == SearchForObject.Result.CONFIRMED
            else DiagnosticStatus.WARN)
        status.message = context.terminal_reason or context.state.value
        status.values = [
            KeyValue(key='query_id', value=str(context.request.query_id)),
            KeyValue(key='query_version', value=str(context.request.query_version)),
            KeyValue(key='search_mode', value=context.mode.value),
            KeyValue(key='state', value=context.state.value),
            KeyValue(
                key='searched_headings_deg',
                value=json.dumps(context.searched_headings_deg)),
            KeyValue(
                key='remaining_angle_budget_deg',
                value=str(max(
                    0.0,
                    self._policy_config.maximum_cumulative_rotation_deg
                    - context.policy.cumulative_rotation_deg))),
            KeyValue(key='candidate_count', value=str(len(context.candidate_ids))),
            KeyValue(key='best_score', value=str(context.best_score)),
            KeyValue(key='uncertainty', value=str(context.best_uncertainty)),
            KeyValue(key='terminal_reason', value=context.terminal_reason),
        ]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self._diagnostic_publisher.publish(output)

    def _publish_markers(self, context):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        for index, heading in enumerate(context.searched_headings_deg):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self._navigation_frame
            marker.ns = 'phase5a_search_headings'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.03
            marker.scale.y = 0.06
            marker.scale.z = 0.08
            marker.color.a = 0.9
            marker.color.g = 0.7
            marker.color.b = 1.0
            angle = math.radians(heading)
            marker.points = [
                Point(x=0.0, y=0.0, z=0.15),
                Point(
                    x=0.8 * math.cos(angle),
                    y=0.8 * math.sin(angle),
                    z=0.15),
            ]
            output.markers.append(marker)
        self._marker_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ActiveSearchManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
