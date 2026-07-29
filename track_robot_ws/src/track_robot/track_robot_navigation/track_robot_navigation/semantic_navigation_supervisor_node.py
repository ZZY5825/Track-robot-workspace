"""Correlate Phase 4A outputs and supervise Nav2 action dispatch."""

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transforms
from tf2_ros import Buffer, TransformException, TransformListener
from track_robot_interfaces.msg import (
    SafetyState,
    SemanticObject,
    SemanticObjectArray,
)

from .runtime_modes import RuntimeMode
from .semantic_goal_policy import (
    GoalAction,
    SemanticGoalPolicy,
    SemanticGoalSnapshot,
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
        safety_state_topic = self.declare_parameter(
            'safety_state_topic', '/safety/state').value
        shadow_path_topic = self.declare_parameter(
            'shadow_path_topic',
            '/semantic_navigation/shadow_path').value
        diagnostics_topic = self.declare_parameter(
            'diagnostics_topic',
            '/semantic_navigation/diagnostics').value
        self._navigation_frame = str(self.declare_parameter(
            'navigation_frame', 'odom').value)
        self._planner_id = str(self.declare_parameter(
            'planner_id', 'GridBased').value)
        self._transform_timeout_sec = float(self.declare_parameter(
            'transform_timeout_sec', 0.10).value)
        supervision_rate_hz = float(self.declare_parameter(
            'supervision_rate_hz', 10.0).value)

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
            maximum_odom_age_sec=float(self.declare_parameter(
                'maximum_odom_age_sec', 0.25).value),
        )

        self._target_array = None
        self._target = None
        self._goal = None
        self._odom = None
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
        self._safety_subscription = self.create_subscription(
            SafetyState, safety_state_topic, self._on_safety, 10)

        self._shadow_path_publisher = self.create_publisher(
            Path, shadow_path_topic, 5)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10)
        self._compute_path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')
        self._navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(
            1.0 / max(1.0, supervision_rate_hz), self._supervise)
        self.get_logger().warn(
            'Semantic navigation mode={} execution_enabled={}'.format(
                mode.value,
                str(self._semantic_execution_enabled).lower()))

    def _on_target(self, message):
        self._target_array = message
        self._target = (
            message.objects[0] if len(message.objects) == 1 else None)
        self._target_received_s = time.monotonic()

    def _on_goal(self, message):
        self._goal = message
        self._goal_received_s = time.monotonic()

    def _on_odom(self, message):
        self._odom = message
        self._odom_received_s = time.monotonic()

    def _on_safety(self, message):
        self._safety = message

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
        target_reference = (
            int(self._target_array.memory_epoch_id),
            int(target.global_object_id),
            int(target.localization_epoch_id),
            int(target.active_query_id),
            int(target.active_query_version),
        )
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
            safety_armed=bool(self._safety and self._safety.armed),
            safety_permits_motion=bool(
                self._safety
                and self._safety.state in (
                    SafetyState.STATE_CLEAR,
                    SafetyState.STATE_SLOWDOWN,
                    SafetyState.STATE_AVOIDING)),
        )

    def _supervise(self):
        snapshot = self._snapshot()
        if snapshot is None:
            decision = self._policy.invalidate(
                'waiting_for_correlated_inputs')
            if decision.action is GoalAction.CANCEL:
                self._cancel_action(decision.reason)
            self._publish_diagnostics(
                decision.action, decision.reason, decision.key)
            return
        decision = self._policy.evaluate(snapshot)
        self._publish_diagnostics(
            decision.action, decision.reason, decision.key)
        if decision.action is GoalAction.CANCEL:
            self._cancel_action(decision.reason)
        elif decision.action in (
                GoalAction.COMPUTE_PATH, GoalAction.NAVIGATE):
            self._dispatch(decision.action)

    def _goal_in_navigation_frame(self):
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

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:  # action transport failure
            self.get_logger().error(
                'Nav2 goal dispatch failed: {}'.format(error))
            self._pending_goal_kind = None
            self._policy.mark_dispatch_failed()
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the supervised goal')
            self._pending_goal_kind = None
            self._policy.mark_dispatch_failed()
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
            return
        if (
                action is GoalAction.COMPUTE_PATH
                and wrapped is not None
                and wrapped.result is not None):
            self._shadow_path_publisher.publish(wrapped.result.path)

    def _cancel_action(self, reason):
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
