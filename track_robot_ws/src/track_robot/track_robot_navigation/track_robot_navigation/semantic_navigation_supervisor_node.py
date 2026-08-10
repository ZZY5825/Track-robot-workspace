"""Correlate Phase 4A outputs and supervise Nav2 action dispatch."""

import math
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transforms
from tf2_ros import Buffer, TransformException, TransformListener
from track_robot_interfaces.msg import (
    SafetyState,
    SemanticLocalizationState,
    SemanticObject,
    SemanticObjectArray,
)
from track_robot_interfaces.srv import AuthorizeSemanticApproach

from .runtime_modes import RuntimeMode
from .physical_recovery import (
    PhysicalRecoveryPolicy,
    RecoveryCommand,
)
from .semantic_goal_policy import (
    GoalAction,
    SemanticGoalPolicy,
    SemanticGoalSnapshot,
)
from .static_target_mission import (
    StaticMissionSnapshot,
    StaticTargetMissionPolicy,
    static_mission_reference_failure,
)


def _values(status):
    return {item.key: item.value for item in status.values}


def _integer(values, key):
    try:
        return int(values.get(key, '0'))
    except (TypeError, ValueError):
        return 0


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _target_reference(array, target):
    return (
        int(array.memory_epoch_id),
        int(target.global_object_id),
        int(target.localization_epoch_id),
        int(target.active_query_id),
        int(target.active_query_version),
    )


def _authorization_reference_is_current(
        current_reference,
        current_sequence,
        requested_reference,
        requested_sequence):
    """Accept a live RViz snapshot of the same current semantic target.

    The selected-target snapshot advances at roughly 10 Hz. Requiring exact
    sequence equality makes a correct button click stale before the service
    callback can run. Identity remains exact, future/zero sequences are
    rejected, and the current snapshot still passes all planner and safety
    preflight checks in ``_authorize_approach``.
    """

    return (
        current_reference is not None
        and requested_reference == current_reference
        and int(requested_sequence) > 0
        and int(requested_sequence) <= int(current_sequence)
    )


def _authorization_survives_interruption(reason):
    """Keep one operator click across non-commanding transient holds."""

    return str(reason) in {
        'confirming_target_reference',
        'goal_stale',
        'odometry_stale',
        'recovery_base_status_stale',
        'recovery_cloud_stale',
        'recovery_odometry_stale',
        'recovery_safety_not_ready',
        'planner_diagnostics_stale',
        'planner_not_ready',
        'safety_motion_not_permitted',
        'safety_obstacle_blocked',
        'target_position_invalid',
        'target_reference_mismatch',
        'target_stale',
        'timestamp_regression',
        'waiting_for_correlated_inputs',
    }


def _physical_recovery_preflight_failure(
        safety, odom_age_sec, maximum_odom_age_sec=0.25):
    """Return why a Nav2 physical recovery must not be submitted.

    A fresh ``STATE_BLOCKED`` is intentionally accepted here. The recovery
    server predicts the requested footprint motion, and the existing safety
    supervisor and velocity gate still evaluate every resulting command.
    """

    if safety is None:
        return 'recovery_safety_unavailable'
    state = int(getattr(safety, 'state', SafetyState.STATE_WAITING_FOR_DATA))
    if (
            bool(getattr(safety, 'rc_override_active', False))
            or state == SafetyState.STATE_RC_OVERRIDE):
        return 'recovery_rc_override'
    if (
            bool(getattr(safety, 'emergency_stop_latched', False))
            or state == SafetyState.STATE_EMERGENCY_STOP):
        return 'recovery_emergency_stop'
    if (
            not bool(getattr(safety, 'base_status_ok', False))
            or state == SafetyState.STATE_BASE_FAULT):
        return 'recovery_base_fault'
    if not bool(getattr(safety, 'base_status_fresh', False)):
        return 'recovery_base_status_stale'
    if (
            not bool(getattr(safety, 'cloud_fresh', False))
            or state == SafetyState.STATE_SENSOR_STALE):
        return 'recovery_cloud_stale'
    if not bool(getattr(safety, 'armed', False)):
        return 'recovery_safety_not_armed'
    if (
            not math.isfinite(float(odom_age_sec))
            or float(odom_age_sec) > float(maximum_odom_age_sec)):
        return 'recovery_odometry_stale'
    if state not in (
            SafetyState.STATE_CLEAR,
            SafetyState.STATE_SLOWDOWN,
            SafetyState.STATE_BLOCKED,
            SafetyState.STATE_AVOIDING):
        return 'recovery_safety_not_ready'
    return None


def _classify_nav2_result(status, retry_count, maximum_retries):
    """Classify a terminal Nav2 result without conflating abort and success."""

    status = int(status)
    retry_count = max(0, int(retry_count))
    maximum_retries = max(0, int(maximum_retries))
    if status == GoalStatus.STATUS_SUCCEEDED:
        return 'complete'
    if (
            status == GoalStatus.STATUS_ABORTED
            and retry_count < maximum_retries):
        return 'retry'
    return 'stop'


def _same_static_target_location(
        authorized_reference,
        current_reference,
        authorized_xy,
        current_xy,
        maximum_distance_m):
    """Match a stationary target by run-local odom position, not transient ID."""

    if (
            authorized_reference is None or current_reference is None
            or authorized_xy is None or current_xy is None
            or len(authorized_xy) != 2 or len(current_xy) != 2
            or not math.isfinite(float(maximum_distance_m))
            or float(maximum_distance_m) <= 0.0):
        return False
    # Memory/localization resets and query revisions create a new identity
    # domain. Only global_object_id may change inside one stationary run.
    for index in (0, 2, 3, 4):
        if int(authorized_reference[index]) != int(current_reference[index]):
            return False
    values = tuple(authorized_xy) + tuple(current_xy)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return math.hypot(
        float(current_xy[0]) - float(authorized_xy[0]),
        float(current_xy[1]) - float(authorized_xy[1]),
    ) <= float(maximum_distance_m)


class _TransientTargetGrace:
    """Bound continuation through target-only gaps; never masks safety loss."""

    _REASONS = {
        'target_position_invalid',
        'target_reference_mismatch',
        'waiting_for_correlated_inputs',
    }

    def __init__(self, maximum_duration_sec):
        self._maximum_duration_sec = max(0.0, float(maximum_duration_sec))
        self._started_s = None

    def reset(self):
        self._started_s = None

    def should_hold(self, reason, now_s):
        if (
                str(reason) not in self._REASONS
                or self._maximum_duration_sec <= 0.0):
            self.reset()
            return False
        now_s = float(now_s)
        if self._started_s is None or now_s < self._started_s:
            self._started_s = now_s
        return now_s - self._started_s <= self._maximum_duration_sec


class SemanticNavigationSupervisorNode(Node):
    """The only bridge from correlated semantic goals to Nav2 actions."""

    def __init__(self):
        super().__init__('semantic_navigation_supervisor')
        mode = RuntimeMode.parse(self.declare_parameter(
            'runtime_mode', RuntimeMode.SEMANTIC_SHADOW.value).value)
        if mode not in (
                RuntimeMode.SEMANTIC_SHADOW,
                RuntimeMode.SEMANTIC_ACTIVE):
            raise ValueError('semantic supervisor requires a semantic mode')
        self._mode = mode
        self._semantic_execution_enabled = bool(self.declare_parameter(
            'semantic_execution_enabled', False).value)
        if (
                mode is RuntimeMode.SEMANTIC_ACTIVE
                and not self._semantic_execution_enabled):
            raise ValueError(
                'SEMANTIC_ACTIVE requires semantic_execution_enabled=true')

        selected_target_topic = self.declare_parameter(
            'selected_target_topic',
            '/semantic_search/phase4a/selected_target').value
        selected_goal_topic = self.declare_parameter(
            'selected_goal_topic',
            '/semantic_search/phase4/selected_goal').value
        planner_diagnostics_topic = self.declare_parameter(
            'planner_diagnostics_topic',
            '/semantic_search/phase4/diagnostics').value
        odometry_topic = self.declare_parameter(
            'odometry_topic', '/odom').value
        localization_state_topic = self.declare_parameter(
            'localization_state_topic',
            '/semantic_search/phase4a/localization_state').value
        safety_state_topic = self.declare_parameter(
            'safety_state_topic', '/safety/state').value
        shadow_path_topic = self.declare_parameter(
            'shadow_path_topic',
            '/semantic_navigation/shadow_path').value
        diagnostics_topic = self.declare_parameter(
            'diagnostics_topic',
            '/semantic_navigation/diagnostics').value
        authorize_service = self.declare_parameter(
            'authorize_service',
            '/semantic_navigation/authorize_approach').value
        cancel_disarm_service = self.declare_parameter(
            'cancel_disarm_service',
            '/semantic_navigation/cancel_and_disarm').value
        safety_arm_service = self.declare_parameter(
            'safety_arm_service', '/safety/arm').value
        safety_disarm_service = self.declare_parameter(
            'safety_disarm_service', '/safety/disarm').value
        self._navigation_frame = str(self.declare_parameter(
            'navigation_frame', 'odom').value)
        self._planner_id = str(self.declare_parameter(
            'planner_id', 'GridBased').value)
        self._transform_timeout_sec = float(self.declare_parameter(
            'transform_timeout_sec', 0.10).value)
        self._maximum_nav2_retries = int(self.declare_parameter(
            'maximum_nav2_retries', 2).value)
        if not 0 <= self._maximum_nav2_retries <= 5:
            raise ValueError('maximum_nav2_retries must be in [0, 5]')
        self._nav2_retry_cooldown_sec = float(self.declare_parameter(
            'nav2_retry_cooldown_sec', 2.0).value)
        if (
                not math.isfinite(self._nav2_retry_cooldown_sec)
                or not 0.1 <= self._nav2_retry_cooldown_sec <= 10.0):
            raise ValueError('nav2_retry_cooldown_sec must be in [0.1, 10.0]')
        self._preserve_authorization_after_retry_exhaustion = bool(
            self.declare_parameter(
                'preserve_authorization_after_retry_exhaustion',
                False).value)
        physical_recovery_requested = bool(self.declare_parameter(
            'physical_recovery_enabled', False).value)
        self._physical_recovery_enabled = bool(
            physical_recovery_requested
            and mode is RuntimeMode.SEMANTIC_ACTIVE
            and self._semantic_execution_enabled)
        self._recovery_spin_angle_rad = float(self.declare_parameter(
            'recovery_spin_angle_rad', 0.523599).value)
        if (
                not math.isfinite(self._recovery_spin_angle_rad)
                or not 0.0 < self._recovery_spin_angle_rad <= math.pi / 2.0):
            raise ValueError('recovery_spin_angle_rad must be in (0, pi/2]')
        recovery_spin_clockwise = bool(self.declare_parameter(
            'recovery_spin_clockwise', False).value)
        self._recovery_backup_distance_m = float(self.declare_parameter(
            'recovery_backup_distance_m', 0.25).value)
        if (
                not math.isfinite(self._recovery_backup_distance_m)
                or not 0.20 <= self._recovery_backup_distance_m <= 0.30):
            raise ValueError(
                'recovery_backup_distance_m must be in [0.20, 0.30]')
        self._recovery_backup_speed_mps = float(self.declare_parameter(
            'recovery_backup_speed_mps', 0.10).value)
        if (
                not math.isfinite(self._recovery_backup_speed_mps)
                or not 0.10 <= self._recovery_backup_speed_mps <= 0.15):
            raise ValueError(
                'recovery_backup_speed_mps must be in [0.10, 0.15]')
        recovery_cooldown_sec = float(self.declare_parameter(
            'recovery_cooldown_sec', 2.0).value)
        maximum_physical_recovery_cycles = int(self.declare_parameter(
            'maximum_physical_recovery_cycles', 2).value)
        self._physical_recovery = PhysicalRecoveryPolicy(
            enabled=self._physical_recovery_enabled,
            cooldown_sec=recovery_cooldown_sec,
            maximum_cycles=maximum_physical_recovery_cycles,
            spin_sign=-1 if recovery_spin_clockwise else 1,
        )
        supervision_rate_hz = float(self.declare_parameter(
            'supervision_rate_hz', 10.0).value)
        static_target_mode = bool(self.declare_parameter(
            'static_target_mode', False).value)
        self._static_target_mode = static_target_mode
        self._static_target_position_reacquisition_enabled = bool(
            self.declare_parameter(
                'static_target_position_reacquisition_enabled',
                False).value)
        self._static_target_reacquisition_radius_m = float(
            self.declare_parameter(
                'static_target_reacquisition_radius_m', 0.45).value)
        if (
                self._static_target_position_reacquisition_enabled
                and not self._static_target_mode):
            raise ValueError(
                'static target position reacquisition requires '
                'static_target_mode')
        if (
                not math.isfinite(self._static_target_reacquisition_radius_m)
                or not 0.0 < self._static_target_reacquisition_radius_m <= 1.0):
            raise ValueError(
                'static_target_reacquisition_radius_m must be in (0, 1.0]')
        target_dropout_grace_sec = float(self.declare_parameter(
            'target_dropout_grace_sec', 0.0).value)
        if (
                not math.isfinite(target_dropout_grace_sec)
                or not 0.0 <= target_dropout_grace_sec <= 1.0):
            raise ValueError('target_dropout_grace_sec must be in [0, 1.0]')
        self._target_grace = _TransientTargetGrace(
            target_dropout_grace_sec if static_target_mode else 0.0)

        maximum_odom_age_sec = float(self.declare_parameter(
            'maximum_odom_age_sec', 0.25).value)
        self._maximum_odom_age_sec = maximum_odom_age_sec
        self._policy = SemanticGoalPolicy(
            runtime_mode=mode.value,
            semantic_execution_enabled=self._semantic_execution_enabled,
            confirmation_snapshots=int(self.declare_parameter(
                'confirmation_snapshots', 2).value),
            maximum_target_age_sec=float(self.declare_parameter(
                'maximum_target_age_sec', 1.0).value),
            maximum_goal_age_sec=float(self.declare_parameter(
                'maximum_goal_age_sec', 0.5).value),
            maximum_diagnostics_age_sec=float(self.declare_parameter(
                'maximum_diagnostics_age_sec', 0.5).value),
            maximum_odom_age_sec=maximum_odom_age_sec,
            static_target_mode=static_target_mode,
        )
        self._mission_policy = StaticTargetMissionPolicy(
            maximum_odom_age_sec=maximum_odom_age_sec)

        self._target_array = None
        self._target = None
        self._goal = None
        self._odom = None
        self._localization_state = None
        self._safety = None
        self._planner_ok = False
        self._planner_reference = (0, 0, 0, 0, 0)
        self._target_received_s = None
        self._goal_received_s = None
        self._diagnostics_received_s = None
        self._odom_received_s = None
        self._last_reason = None
        self._active_goal_handle = None
        self._pending_goal_kind = None
        self._cancel_when_accepted = False
        self._cancel_preserves_authorization = False
        self._nav2_retry_count = 0
        self._nav2_retry_not_before_s = 0.0
        self._authorized_reference = None
        self._pending_authorization = None
        self._authorized_target_anchor_xy = None
        self._pending_target_anchor_xy = None
        self._mission_goal = None
        self._pending_mission_goal = None

        target_qos = QoSProfile(depth=1)
        target_qos.reliability = ReliabilityPolicy.RELIABLE
        target_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._target_subscription = self.create_subscription(
            SemanticObjectArray,
            selected_target_topic,
            self._on_target,
            target_qos)
        self._goal_subscription = self.create_subscription(
            PoseStamped, selected_goal_topic, self._on_goal, 5)
        self._planner_subscription = self.create_subscription(
            DiagnosticArray,
            planner_diagnostics_topic,
            self._on_planner_diagnostics,
            10)
        self._odom_subscription = self.create_subscription(
            Odometry, odometry_topic, self._on_odom, 10)
        self._localization_state_subscription = self.create_subscription(
            SemanticLocalizationState,
            localization_state_topic,
            self._on_localization_state,
            target_qos)
        self._safety_subscription = self.create_subscription(
            SafetyState, safety_state_topic, self._on_safety, 10)

        self._shadow_path_publisher = self.create_publisher(
            Path, shadow_path_topic, 5)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10)
        self._safety_arm_client = self.create_client(
            Trigger, safety_arm_service)
        self._safety_disarm_client = self.create_client(
            Trigger, safety_disarm_service)
        self._authorize_service = self.create_service(
            AuthorizeSemanticApproach,
            authorize_service,
            self._authorize_approach)
        self._cancel_disarm_service = self.create_service(
            Trigger,
            cancel_disarm_service,
            self._cancel_and_disarm)
        self._compute_path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self._navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._back_up_client = ActionClient(self, BackUp, 'back_up')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(
            1.0 / max(1.0, supervision_rate_hz), self._supervise)
        self.get_logger().warn(
            'Semantic navigation mode={} execution_enabled={} '
            'physical_recovery_enabled={}'.format(
                mode.value,
                str(self._semantic_execution_enabled).lower(),
                str(self._physical_recovery_enabled).lower()))

    def _on_target(self, message):
        previous_reference = (
            _target_reference(self._target_array, self._target)
            if self._target_array is not None and self._target is not None
            else None)
        self._target_array = message
        self._target = (
            message.objects[0] if len(message.objects) == 1 else None)
        self._target_received_s = time.monotonic()
        current_reference = (
            _target_reference(message, self._target)
            if self._target is not None else None)
        if (
                self._mission_goal is not None
                and self._authorized_reference is not None):
            mission_failure = static_mission_reference_failure(
                self._authorized_reference,
                current_reference,
            )
            if mission_failure is not None:
                self._clear_authorization(mission_failure)
                self._cancel_action(mission_failure)
                self._request_safety_disarm()
            return
        expected_reference = self._authorized_reference
        if expected_reference is None and self._pending_authorization is not None:
            expected_reference = self._pending_authorization[0]
        if expected_reference is None:
            expected_reference = previous_reference
        if (
                current_reference is not None
                and expected_reference is not None
                and current_reference != expected_reference):
            expected_anchor = (
                self._authorized_target_anchor_xy
                if self._authorized_reference is not None
                else self._pending_target_anchor_xy)
            current_anchor = self._target_anchor_in_navigation_frame(
                self._target)
            same_stationary_target = (
                self._static_target_mode
                and self._static_target_position_reacquisition_enabled
                and _same_static_target_location(
                    expected_reference,
                    current_reference,
                    expected_anchor,
                    current_anchor,
                    self._static_target_reacquisition_radius_m))
            if same_stationary_target:
                if self._authorized_reference is not None:
                    self._authorized_reference = current_reference
                elif self._pending_authorization is not None:
                    self._pending_authorization = (
                        current_reference,
                        int(message.snapshot_sequence),
                    )
                self.get_logger().warn(
                    'Re-correlated stationary semantic target by odom '
                    'anchor: object {} -> {}'.format(
                        expected_reference[1], current_reference[1]))
            else:
                self._clear_authorization('target_reference_changed')

    def _on_goal(self, message):
        self._goal = message
        self._goal_received_s = time.monotonic()

    def _on_odom(self, message):
        self._odom = message
        self._odom_received_s = time.monotonic()

    def _on_localization_state(self, message):
        self._localization_state = message

    def _on_safety(self, message):
        self._safety = message
        hard_stop = (
            message.rc_override_active
            or message.emergency_stop_latched
            or message.state in (
                SafetyState.STATE_RC_OVERRIDE,
                SafetyState.STATE_BASE_FAULT,
                SafetyState.STATE_EMERGENCY_STOP))
        if hard_stop:
            self._clear_authorization('safety_hard_stop')
            self._cancel_action('safety_hard_stop')
            self._request_safety_disarm()
        elif message.armed and self._pending_authorization is not None:
            pending_reference, _pending_sequence = (
                self._pending_authorization)
            current_reference, _current_sequence = (
                self._current_reference_and_sequence())
            if current_reference == pending_reference:
                self._lock_static_mission(
                    pending_reference,
                    self._pending_target_anchor_xy,
                    self._pending_mission_goal,
                )
                self.get_logger().warn(
                    'Operator authorized semantic approach for object '
                    '{}'.format(pending_reference[1]))
            else:
                self._clear_authorization(
                    'target_changed_while_safety_arming')
                self._request_safety_disarm()

    def _on_planner_diagnostics(self, message):
        self._diagnostics_received_s = time.monotonic()
        self._planner_ok = False
        self._planner_reference = (0, 0, 0, 0, 0)
        if not message.status:
            return
        status = message.status[0]
        values = _values(status)
        self._planner_ok = (
            values.get('status') == 'PASS'
            and values.get('reason') == 'planned')
        self._planner_reference = (
            _integer(values, 'memory_epoch_id'),
            _integer(values, 'global_object_id'),
            _integer(values, 'localization_epoch_id'),
            _integer(values, 'query_id'),
            _integer(values, 'query_version'),
        )
        # The supervision timer owns cancellation so the bounded static-target
        # grace policy is applied consistently to asynchronous callbacks.

    def _age_from_stamp(self, stamp, receive_s):
        source_s = _stamp_seconds(stamp)
        if source_s <= 0.0:
            return float('inf')
        now_s = float(self.get_clock().now().nanoseconds) / 1_000_000_000.0
        source_age = now_s - source_s
        receive_age = (
            time.monotonic() - receive_s
            if receive_s is not None else float('inf'))
        return max(source_age, receive_age)

    def _snapshot(self):
        if (
                self._target_array is None
                or self._target is None
                or self._goal is None
                or self._odom is None
                or self._diagnostics_received_s is None):
            return None
        target = self._target
        target_reference = _target_reference(self._target_array, target)
        return SemanticGoalSnapshot(
            memory_epoch_id=target_reference[0],
            global_object_id=target_reference[1],
            localization_epoch_id=target_reference[2],
            query_id=target_reference[3],
            query_version=target_reference[4],
            snapshot_sequence=int(self._target_array.snapshot_sequence),
            target_age_sec=self._age_from_stamp(
                target.last_seen, self._target_received_s),
            goal_age_sec=(
                time.monotonic() - self._goal_received_s
                if self._goal_received_s is not None else float('inf')),
            diagnostics_age_sec=(
                time.monotonic() - self._diagnostics_received_s),
            odom_age_sec=self._age_from_stamp(
                self._odom.header.stamp, self._odom_received_s),
            goal_frame_id=str(self._goal.header.frame_id),
            target_frame_id=str(target.position_frame_id),
            lifecycle_confirmed=(
                target.lifecycle_state
                == SemanticObject.LIFECYCLE_CONFIRMED),
            position_valid=bool(target.position_valid),
            references_match=(
                self._planner_ok
                and target_reference == self._planner_reference),
            operator_authorized=(
                self._authorized_reference == target_reference),
            safety_armed=bool(self._safety and self._safety.armed),
            safety_permits_motion=bool(
                self._safety
                and self._safety.state in (
                    SafetyState.STATE_CLEAR,
                    SafetyState.STATE_SLOWDOWN,
                    SafetyState.STATE_AVOIDING)),
            safety_temporarily_blocked=bool(
                self._safety
                and self._safety.armed
                and self._safety.state == SafetyState.STATE_BLOCKED),
        )

    def _clear_authorization(self, reason):
        if (
                self._authorized_reference is not None
                or self._pending_authorization is not None):
            self.get_logger().warn(
                'Semantic approach authorization cleared: {}'.format(reason))
        self._authorized_reference = None
        self._pending_authorization = None
        self._authorized_target_anchor_xy = None
        self._pending_target_anchor_xy = None
        self._mission_goal = None
        self._pending_mission_goal = None
        self._nav2_retry_count = 0
        self._nav2_retry_not_before_s = 0.0
        self._target_grace.reset()
        if hasattr(self, '_physical_recovery'):
            self._physical_recovery.reset()

    def _lock_static_mission(
            self, reference, target_anchor_xy, mission_goal):
        self._nav2_retry_count = 0
        self._nav2_retry_not_before_s = 0.0
        self._authorized_reference = reference
        self._pending_authorization = None
        self._authorized_target_anchor_xy = target_anchor_xy
        self._pending_target_anchor_xy = None
        self._mission_goal = mission_goal
        self._pending_mission_goal = None
        if hasattr(self, '_physical_recovery'):
            self._physical_recovery.reset()

    def _target_anchor_in_navigation_frame(self, target):
        if (
                target is None or not target.position_valid
                or not target.position_frame_id):
            return None
        pose = PoseStamped()
        pose.header = target.header
        pose.header.frame_id = str(target.position_frame_id)
        pose.pose.position = target.position
        pose.pose.orientation.w = 1.0
        try:
            transformed = self._tf_buffer.transform(
                pose,
                self._navigation_frame,
                timeout=Duration(seconds=self._transform_timeout_sec))
        except TransformException:
            return None
        values = (
            float(transformed.pose.position.x),
            float(transformed.pose.position.y),
        )
        return values if all(math.isfinite(value) for value in values) else None

    def _current_reference_and_sequence(self):
        if self._target_array is None or self._target is None:
            return None, 0
        return (
            _target_reference(self._target_array, self._target),
            int(self._target_array.snapshot_sequence),
        )

    def _authorize_approach(self, request, response):
        if (
                self._mode is not RuntimeMode.SEMANTIC_ACTIVE
                or not self._semantic_execution_enabled):
            response.accepted = False
            response.reason = 'semantic_active_mode_required'
            return response
        current_reference, current_sequence = (
            self._current_reference_and_sequence())
        requested_reference = (
            int(request.memory_epoch_id),
            int(request.global_object_id),
            int(request.localization_epoch_id),
            int(request.query_id),
            int(request.query_version),
        )
        if (
                self._mission_goal is not None
                and self._authorized_reference is not None):
            response.accepted = (
                requested_reference == self._authorized_reference)
            response.reason = (
                'mission_already_active'
                if response.accepted
                else 'different_mission_already_active')
            return response
        if not _authorization_reference_is_current(
                current_reference,
                current_sequence,
                requested_reference,
                int(request.snapshot_sequence)):
            response.accepted = False
            response.reason = 'stale_or_mismatched_target_reference'
            return response
        if not self._planner_ok or self._planner_reference != current_reference:
            response.accepted = False
            response.reason = 'planner_reference_not_ready'
            return response
        snapshot = self._snapshot()
        if snapshot is None:
            response.accepted = False
            response.reason = 'correlated_inputs_not_ready'
            return response
        preflight_failure = self._policy.authorization_failure(snapshot)
        if preflight_failure is not None:
            response.accepted = False
            response.reason = preflight_failure
            return response
        mission_goal = self._goal_in_navigation_frame()
        if mission_goal is None:
            response.accepted = False
            response.reason = 'goal_transform_unavailable'
            return response
        target_anchor_xy = self._target_anchor_in_navigation_frame(
            self._target)
        if (
                self._static_target_position_reacquisition_enabled
                and target_anchor_xy is None):
            response.accepted = False
            response.reason = 'target_anchor_transform_unavailable'
            return response
        if self._safety and self._safety.armed:
            self._lock_static_mission(
                current_reference,
                target_anchor_xy,
                mission_goal,
            )
            response.accepted = True
            response.reason = 'authorized'
            return response
        if not self._safety_arm_client.service_is_ready():
            response.accepted = False
            response.reason = 'safety_arm_service_unavailable'
            return response

        self._authorized_reference = None
        self._pending_authorization = (
            current_reference,
            current_sequence,
        )
        self._pending_target_anchor_xy = target_anchor_xy
        self._pending_mission_goal = mission_goal
        future = self._safety_arm_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_arm_result)
        response.accepted = True
        response.reason = 'approach_requested'
        return response

    def _on_arm_result(self, future):
        pending = self._pending_authorization
        if pending is None:
            return
        try:
            result = future.result()
        except Exception as error:
            self._clear_authorization('safety_arm_request_failed')
            self.get_logger().error(
                'Safety arm request failed: {}'.format(error))
            return
        current_reference, current_sequence = (
            self._current_reference_and_sequence())
        pending_reference, pending_sequence = pending
        if not result.success:
            self._clear_authorization('safety_arm_rejected')
            self.get_logger().warn(
                'Safety arm rejected: {}'.format(result.message))
            return
        if (
                current_reference != pending_reference
                or current_sequence < pending_sequence):
            self._clear_authorization('target_changed_while_safety_arming')
            self.get_logger().warn(
                'Safety armed after target changed; disarming fail-closed')
            self._request_safety_disarm()
            return
        # Wait for the authoritative SafetyState topic to report armed before
        # exposing authorization to the navigation policy.
        self.get_logger().info(
            'Safety arm accepted; waiting for armed SafetyState')

    def _cancel_and_disarm(self, request, response):
        del request
        self._clear_authorization('operator_cancel')
        self._cancel_action('operator_cancel')
        if not self._safety_disarm_client.service_is_ready():
            response.success = False
            response.message = (
                'Nav2 goal cancelled; safety disarm service unavailable')
            return response
        future = self._safety_disarm_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_disarm_result)
        response.success = True
        response.message = 'Nav2 cancellation requested; safety disarm pending'
        return response

    def _request_safety_disarm(self):
        if self._safety_disarm_client.service_is_ready():
            future = self._safety_disarm_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_disarm_result)

    def _on_disarm_result(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(
                'Safety disarm request failed: {}'.format(error))
            return
        if not result.success:
            self.get_logger().error(
                'Safety disarm rejected: {}'.format(result.message))

    def _supervise_static_mission(self):
        mission_snapshot = self._static_mission_snapshot()
        if mission_snapshot is None:
            return False
        decision = self._mission_policy.evaluate(mission_snapshot)
        if decision.terminate_mission:
            self._publish_diagnostics(
                decision.action,
                decision.reason,
                mission_snapshot.key,
            )
            self._clear_authorization(decision.reason)
            self._cancel_action(decision.reason)
            self._request_safety_disarm()
            return True

        if getattr(self, '_physical_recovery_enabled', False):
            active_recovery = self._pending_goal_kind in (
                RecoveryCommand.SPIN, RecoveryCommand.BACK_UP)
            recovery = (
                None
                if mission_snapshot.goal_in_flight
                else self._physical_recovery.next_command(time.monotonic()))
            recovery_command_pending = bool(
                active_recovery
                or (
                    recovery is not None
                    and recovery.command in (
                        RecoveryCommand.SPIN, RecoveryCommand.BACK_UP)))
            if recovery_command_pending:
                preflight_failure = _physical_recovery_preflight_failure(
                    self._safety,
                    mission_snapshot.odom_age_sec,
                    self._maximum_odom_age_sec,
                )
                if preflight_failure is not None:
                    self._publish_diagnostics(
                        GoalAction.HOLD,
                        preflight_failure,
                        mission_snapshot.key,
                    )
                    if preflight_failure in {
                            'recovery_rc_override',
                            'recovery_emergency_stop',
                            'recovery_base_fault',
                            'recovery_safety_not_armed'}:
                        self._clear_authorization(preflight_failure)
                        self._cancel_action(preflight_failure)
                        self._request_safety_disarm()
                    else:
                        self._cancel_action(preflight_failure)
                    return True
                if active_recovery:
                    self._publish_diagnostics(
                        GoalAction.HOLD,
                        'physical_recovery_active',
                        mission_snapshot.key,
                    )
                    return True
                self._publish_diagnostics(
                    GoalAction.HOLD,
                    recovery.reason,
                    mission_snapshot.key,
                )
                self._dispatch_recovery(recovery.command)
                return True
            if (
                    recovery is not None
                    and recovery.command is RecoveryCommand.HOLD):
                self._publish_diagnostics(
                    GoalAction.HOLD,
                    recovery.reason,
                    mission_snapshot.key,
                )
                return True

        self._publish_diagnostics(
            decision.action,
            decision.reason,
            mission_snapshot.key,
        )
        if decision.action is GoalAction.CANCEL:
            self._cancel_action(decision.reason)
        elif decision.action is GoalAction.NAVIGATE:
            self._dispatch(decision.action)
        return True

    def _static_mission_snapshot(self):
        if self._authorized_reference is None or self._mission_goal is None:
            return None
        odom_age_sec = float('inf')
        if self._odom is not None:
            odom_age_sec = self._age_from_stamp(
                self._odom.header.stamp,
                self._odom_received_s,
            )
        observed_localization_epoch_id = (
            int(self._localization_state.localization_epoch_id)
            if self._localization_state is not None
            else 0)
        safety_armed = bool(self._safety and self._safety.armed)
        safety_temporarily_blocked = bool(
            self._safety
            and self._safety.armed
            and self._safety.state == SafetyState.STATE_BLOCKED)
        safety_permits_motion = bool(
            self._safety
            and self._safety.state in (
                SafetyState.STATE_CLEAR,
                SafetyState.STATE_SLOWDOWN,
                SafetyState.STATE_AVOIDING))
        return StaticMissionSnapshot(
            memory_epoch_id=int(self._authorized_reference[0]),
            global_object_id=int(self._authorized_reference[1]),
            localization_epoch_id=int(self._authorized_reference[2]),
            observed_localization_epoch_id=observed_localization_epoch_id,
            odom_age_sec=odom_age_sec,
            safety_armed=safety_armed,
            safety_permits_motion=safety_permits_motion,
            safety_temporarily_blocked=safety_temporarily_blocked,
            goal_in_flight=bool(
                self._pending_goal_kind is not None
                or self._active_goal_handle is not None),
        )

    def _supervise(self):
        if self._supervise_static_mission():
            return
        snapshot = self._snapshot()
        if snapshot is None:
            if (
                    self._authorized_reference is not None
                    and self._target_grace.should_hold(
                        'waiting_for_correlated_inputs', time.monotonic())):
                self._publish_diagnostics(
                    GoalAction.HOLD,
                    'static_target_dropout_grace',
                    self._authorized_reference[:2])
                return
            self._target_grace.reset()
            decision = self._policy.invalidate(
                'waiting_for_correlated_inputs')
            if decision.action is GoalAction.CANCEL:
                self._cancel_action(decision.reason)
            self._publish_diagnostics(
                decision.action, decision.reason, decision.key)
            return
        input_failure = self._policy.authorization_failure(snapshot)
        if (
                self._authorized_reference is not None
                and input_failure is not None
                and self._target_grace.should_hold(
                    input_failure, time.monotonic())):
            self._publish_diagnostics(
                GoalAction.HOLD,
                'static_target_dropout_grace',
                snapshot.key)
            return
        self._target_grace.reset()
        decision = self._policy.evaluate(snapshot)
        retained_reasons = {
            'goal_accepted',
            'goal_already_dispatched',
            'safety_obstacle_blocked',
        }
        pending_reasons = {
            'operator_authorization_required',
            'safety_not_armed',
        }
        invalidates_authorization = (
            decision.reason not in retained_reasons
            and not _authorization_survives_interruption(decision.reason)
            and not (
                self._pending_authorization is not None
                and decision.reason in pending_reasons))
        if (
                invalidates_authorization
                and (
                    self._authorized_reference is not None
                    or self._pending_authorization is not None)):
            self._clear_authorization(decision.reason)
            self._request_safety_disarm()
        self._publish_diagnostics(
            decision.action, decision.reason, decision.key)
        if decision.action is GoalAction.CANCEL:
            self._cancel_action(decision.reason)
        elif decision.action in (
                GoalAction.COMPUTE_PATH, GoalAction.NAVIGATE):
            self._dispatch(decision.action)

    def _goal_in_navigation_frame(self):
        if self._mission_goal is not None:
            return self._mission_goal
        try:
            return self._tf_buffer.transform(
                self._goal,
                self._navigation_frame,
                timeout=Duration(seconds=self._transform_timeout_sec))
        except TransformException as error:
            self.get_logger().warn(
                'Semantic goal transform failed: {}'.format(error))
            self._policy.mark_dispatch_failed()
            return None

    def _dispatch(self, action):
        if (
                self._pending_goal_kind is not None
                or self._active_goal_handle is not None):
            self._policy.mark_dispatch_failed()
            return
        if (
                action is GoalAction.NAVIGATE
                and time.monotonic() < self._nav2_retry_not_before_s):
            return
        goal = self._goal_in_navigation_frame()
        if goal is None:
            return
        if action is GoalAction.COMPUTE_PATH:
            if not self._compute_path_client.server_is_ready():
                self._policy.mark_dispatch_failed()
                self.get_logger().warn(
                    'compute_path_to_pose action server is unavailable')
                return
            request = ComputePathToPose.Goal()
            request.pose = goal
            request.planner_id = self._planner_id
            future = self._compute_path_client.send_goal_async(request)
            self._pending_goal_kind = GoalAction.COMPUTE_PATH
        else:
            if not self._navigate_client.server_is_ready():
                self._policy.mark_dispatch_failed()
                self.get_logger().warn(
                    'navigate_to_pose action server is unavailable')
                return
            request = NavigateToPose.Goal()
            request.pose = goal
            request.behavior_tree = ''
            future = self._navigate_client.send_goal_async(request)
            self._pending_goal_kind = GoalAction.NAVIGATE
        future.add_done_callback(self._on_goal_response)

    def _dispatch_recovery(self, command):
        if command not in (RecoveryCommand.SPIN, RecoveryCommand.BACK_UP):
            raise ValueError('physical recovery command must be SPIN or BACK_UP')
        if (
                self._pending_goal_kind is not None
                or self._active_goal_handle is not None):
            self._policy.mark_dispatch_failed()
            return

        if command is RecoveryCommand.SPIN:
            if not self._spin_client.server_is_ready():
                self._policy.mark_dispatch_failed()
                self.get_logger().warn('spin action server is unavailable')
                return
            request = Spin.Goal()
            request.target_yaw = (
                self._physical_recovery.spin_sign
                * self._recovery_spin_angle_rad)
            client = self._spin_client
        else:
            if not self._back_up_client.server_is_ready():
                self._policy.mark_dispatch_failed()
                self.get_logger().warn('back_up action server is unavailable')
                return
            request = BackUp.Goal()
            request.target.x = -self._recovery_backup_distance_m
            request.target.y = 0.0
            request.target.z = 0.0
            request.speed = self._recovery_backup_speed_mps
            client = self._back_up_client

        future = client.send_goal_async(request)
        self._pending_goal_kind = command
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        action = self._pending_goal_kind
        try:
            handle = future.result()
        except Exception as error:  # action transport failure
            self.get_logger().error(
                'Nav2 goal dispatch failed: {}'.format(error))
            self._pending_goal_kind = None
            self._cancel_when_accepted = False
            self._policy.mark_dispatch_failed()
            if (
                    action in (RecoveryCommand.SPIN, RecoveryCommand.BACK_UP)
                    and self._authorized_reference is not None):
                self._physical_recovery.recovery_finished(
                    action, False, time.monotonic())
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the supervised goal')
            self._pending_goal_kind = None
            self._cancel_when_accepted = False
            self._policy.mark_dispatch_failed()
            if (
                    action in (RecoveryCommand.SPIN, RecoveryCommand.BACK_UP)
                    and self._authorized_reference is not None):
                self._physical_recovery.recovery_finished(
                    action, False, time.monotonic())
                return
            self._clear_authorization('nav2_goal_rejected')
            self._request_safety_disarm()
            return
        self._active_goal_handle = handle
        if self._cancel_when_accepted:
            self._cancel_when_accepted = False
            handle.cancel_goal_async()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def _on_action_result(self, future):
        action = self._pending_goal_kind
        self._pending_goal_kind = None
        self._active_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            self.get_logger().error(
                'Nav2 action result failed: {}'.format(error))
            if (
                    action in (RecoveryCommand.SPIN, RecoveryCommand.BACK_UP)
                    and self._authorized_reference is not None):
                self._physical_recovery.recovery_finished(
                    action, False, time.monotonic())
                self._policy.mark_dispatch_failed()
                return
            self._clear_authorization('nav2_action_result_failed')
            self._request_safety_disarm()
            return
        if (
                action is GoalAction.COMPUTE_PATH
                and wrapped is not None
                and wrapped.result is not None):
            self._shadow_path_publisher.publish(wrapped.result.path)
        if (
                self._cancel_preserves_authorization
                and self._authorized_reference is not None):
            self._cancel_preserves_authorization = False
            self._policy.mark_dispatch_failed()
            return
        self._cancel_preserves_authorization = False
        if action in (RecoveryCommand.SPIN, RecoveryCommand.BACK_UP):
            if self._authorized_reference is None:
                return
            status = (
                int(wrapped.status)
                if wrapped is not None
                else GoalStatus.STATUS_UNKNOWN)
            self._physical_recovery.recovery_finished(
                action,
                status == GoalStatus.STATUS_SUCCEEDED,
                time.monotonic(),
            )
            self._policy.mark_dispatch_failed()
            return
        if action is GoalAction.NAVIGATE:
            status = (
                int(wrapped.status)
                if wrapped is not None
                else GoalStatus.STATUS_UNKNOWN)
            if (
                    getattr(self, '_physical_recovery_enabled', False)
                    and status == GoalStatus.STATUS_ABORTED
                    and self._authorized_reference is not None):
                self._physical_recovery.navigation_aborted(time.monotonic())
                self._nav2_retry_count = 0
                self._nav2_retry_not_before_s = 0.0
                self._policy.mark_dispatch_failed()
                self.get_logger().warn(
                    'Nav2 aborted; preserving the frozen semantic mission '
                    'and starting bounded physical recovery')
                return
            disposition = _classify_nav2_result(
                status,
                self._nav2_retry_count,
                self._maximum_nav2_retries,
            )
            if disposition == 'retry':
                self._nav2_retry_count += 1
                self._nav2_retry_not_before_s = (
                    time.monotonic() + self._nav2_retry_cooldown_sec)
                self._policy.mark_dispatch_failed()
                self.get_logger().warn(
                    'Nav2 aborted; preserving operator authorization for '
                    'bounded retry {}/{} after {:.1f}s cooldown'.format(
                        self._nav2_retry_count,
                        self._maximum_nav2_retries,
                        self._nav2_retry_cooldown_sec))
                return
            if (
                    status == GoalStatus.STATUS_ABORTED
                    and self._preserve_authorization_after_retry_exhaustion
                    and self._authorized_reference is not None):
                self._nav2_retry_count = 0
                cooldown_sec = 2.0 * self._nav2_retry_cooldown_sec
                self._nav2_retry_not_before_s = (
                    time.monotonic() + cooldown_sec)
                self._policy.mark_dispatch_failed()
                self.get_logger().warn(
                    'Nav2 retry cycle exhausted; preserving operator '
                    'authorization and retrying after {:.1f}s cooldown'.format(
                        cooldown_sec))
                return
            reason = 'nav2_action_failed'
            if disposition == 'complete':
                reason = 'nav2_action_succeeded'
            elif status == GoalStatus.STATUS_ABORTED:
                reason = 'nav2_retry_exhausted'
            elif status == GoalStatus.STATUS_CANCELED:
                reason = 'nav2_action_canceled'
            self._nav2_retry_count = 0
            self._nav2_retry_not_before_s = 0.0
            self._clear_authorization(reason)
            self._request_safety_disarm()

    def _cancel_action(self, reason):
        self._cancel_preserves_authorization = bool(
            self._authorized_reference is not None
            and _authorization_survives_interruption(reason))
        if self._active_goal_handle is not None:
            self.get_logger().warn(
                'Cancelling supervised Nav2 goal: {}'.format(reason))
            self._active_goal_handle.cancel_goal_async()
        elif self._pending_goal_kind is not None:
            self._cancel_when_accepted = True

    def _publish_diagnostics(self, action, reason, key):
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'semantic_navigation/supervisor'
        status.hardware_id = 'nav2_supervised'
        status.level = (
            DiagnosticStatus.OK
            if reason in ('goal_accepted', 'goal_already_dispatched')
            else DiagnosticStatus.WARN)
        status.message = reason
        status.values = [
            KeyValue(key='runtime_mode', value=self._mode.value),
            KeyValue(
                key='semantic_execution_enabled',
                value=str(self._semantic_execution_enabled).lower()),
            KeyValue(key='action', value=action.value),
            KeyValue(key='reason', value=reason),
            KeyValue(key='memory_epoch_id', value=str(key[0])),
            KeyValue(key='global_object_id', value=str(key[1])),
            KeyValue(
                key='operator_authorized',
                value=str(self._authorized_reference is not None).lower()),
            KeyValue(
                key='authorization_pending',
                value=str(self._pending_authorization is not None).lower()),
            KeyValue(
                key='motion_capable',
                value=str(
                    self._mode is RuntimeMode.SEMANTIC_ACTIVE
                    and self._semantic_execution_enabled).lower()),
        ]
        output.status.append(status)
        self._diagnostic_publisher.publish(output)
        if reason != self._last_reason:
            self.get_logger().info(
                'Semantic navigation decision: {} ({})'.format(
                    action.value, reason))
            self._last_reason = reason

    def destroy_node(self):
        # Foxy ActionClient must be released before its owning node handle.
        self._compute_path_client.destroy()
        self._navigate_client.destroy()
        self._spin_client.destroy()
        self._back_up_client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigationSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
