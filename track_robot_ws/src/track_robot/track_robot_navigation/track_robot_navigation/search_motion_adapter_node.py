"""Translate authorized rotation-only search intents into Nav2 Spin goals."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from track_robot_interfaces.msg import SafetyState, SearchMotionIntent

from .search_motion_adapter import (
    MotionIntentRequest,
    MotionLimits,
    SearchMotionPolicy,
)


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SearchMotionAdapter(Node):
    """Run Nav2 Spin only after one query-bound operator authorization."""

    def __init__(self):
        super().__init__('search_motion_adapter')
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._policy = SearchMotionPolicy(MotionLimits(
            maximum_individual_rotation_rad=math.radians(float(
                self.declare_parameter(
                    'maximum_individual_rotation_deg', 90.0).value)),
            maximum_angular_speed_rad_s=float(self.declare_parameter(
                'maximum_angular_speed_rad_s', 0.30).value),
            odometry_timeout_sec=float(self.declare_parameter(
                'odometry_timeout_sec', 0.25).value),
            safety_timeout_sec=float(self.declare_parameter(
                'safety_timeout_sec', 0.25).value),
            intent_timeout_sec=float(self.declare_parameter(
                'intent_timeout_sec', 0.50).value),
        ))
        self._service_timeout_sec = float(self.declare_parameter(
            'service_timeout_sec', 1.0).value)
        watchdog_rate_hz = float(self.declare_parameter(
            'watchdog_rate_hz', 20.0).value)

        intent_topic = str(self.declare_parameter(
            'intent_topic',
            '/semantic_search/search_motion_intent').value)
        motion_status_topic = str(self.declare_parameter(
            'motion_status_topic',
            '/semantic_search/active_search/motion_status').value)
        odometry_topic = str(self.declare_parameter(
            'odometry_topic', '/odom').value)
        safety_state_topic = str(self.declare_parameter(
            'safety_state_topic', '/safety/state').value)
        spin_action = str(self.declare_parameter(
            'spin_action', '/spin').value)
        authorize_service = str(self.declare_parameter(
            'authorize_service',
            '/semantic_search/active_search/authorize_rotation').value)
        cancel_service = str(self.declare_parameter(
            'cancel_service',
            '/semantic_search/active_search/cancel').value)
        safety_arm_service = str(self.declare_parameter(
            'safety_arm_service', '/safety/arm').value)
        safety_disarm_service = str(self.declare_parameter(
            'safety_disarm_service', '/safety/disarm').value)

        self._latest_odom_monotonic = None
        self._latest_safety_monotonic = None
        self._latest_safety_healthy = False
        self._latest_safety_reason = 'safety_state_unavailable'
        self._arming = False
        self._spin_goal_handle = None
        self._pending_goal_query_id = None

        self._status_publisher = self.create_publisher(
            DiagnosticArray, motion_status_topic, 10)
        self.create_subscription(
            SearchMotionIntent,
            intent_topic,
            self._on_intent,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Odometry,
            odometry_topic,
            self._on_odom,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            SafetyState,
            safety_state_topic,
            self._on_safety,
            10,
            callback_group=self._callback_group,
        )
        self._spin_client = ActionClient(
            self,
            Spin,
            spin_action,
            callback_group=self._callback_group,
        )
        self._arm_client = self.create_client(
            Trigger, safety_arm_service,
            callback_group=self._callback_group)
        self._disarm_client = self.create_client(
            Trigger, safety_disarm_service,
            callback_group=self._callback_group)
        self.create_service(
            Trigger,
            authorize_service,
            self._authorize_rotation,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            cancel_service,
            self._cancel_search,
            callback_group=self._callback_group,
        )
        self.create_timer(
            1.0 / max(1.0, watchdog_rate_hz),
            self._watchdog,
            callback_group=self._callback_group,
        )
        self._publish_status(0, 'IDLE', 'waiting_for_search_intent')

    def destroy_node(self):
        # Foxy can deliver SIGINT while rclpy is tearing down the action
        # client's entities.  Treat that shutdown-only race as cleanup already
        # in progress instead of reporting a false runtime crash.
        try:
            self._spin_client.destroy()
        except (KeyboardInterrupt, RuntimeError):
            pass
        return super().destroy_node()

    def _deadline_monotonic(self, message):
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        remaining = _stamp_seconds(message.deadline) - now_ros
        return time.monotonic() + max(0.0, remaining)

    def _request_from_message(self, message):
        return MotionIntentRequest(
            query_id=int(message.query_id),
            signed_rotation_rad=float(message.target_bearing),
            maximum_rotation_rad=float(message.maximum_rotation_angle),
            maximum_angular_speed_rad_s=float(
                message.maximum_angular_speed),
            deadline_monotonic=self._deadline_monotonic(message),
            rotation_permitted=bool(message.rotation_permitted),
            forward_permitted=bool(message.forward_permitted),
            stop=(message.intent == SearchMotionIntent.INTENT_STOP),
        )

    def _on_intent(self, message):
        try:
            request = self._request_from_message(message)
            with self._lock:
                transition = self._policy.accept_intent(
                    request, time.monotonic())
        except (TypeError, ValueError) as error:
            self._publish_status(
                int(message.query_id), 'REJECTED', str(error), error=True)
            return
        if transition.cancel_spin:
            self._cancel_spin_and_disarm(transition.reason)
        state = 'PENDING_AUTHORIZATION'
        if transition.reason == 'shadow_intent_recorded':
            state = 'SHADOW'
        elif not transition.accepted:
            state = 'REJECTED'
        self._publish_status(
            int(message.query_id),
            state,
            transition.reason,
            error=not transition.accepted,
        )
        if transition.reason == 'authorized_intent_ready':
            self._start_pending_spin()

    def _safety_and_odom_ready_locked(self):
        now = time.monotonic()
        if (
                self._latest_odom_monotonic is None or
                now - self._latest_odom_monotonic
                > self._policy.limits.odometry_timeout_sec):
            return False, 'odometry_stale'
        if (
                self._latest_safety_monotonic is None or
                now - self._latest_safety_monotonic
                > self._policy.limits.safety_timeout_sec):
            return False, 'safety_state_stale'
        if not self._latest_safety_healthy:
            return False, self._latest_safety_reason
        return True, 'ready'

    def _authorize_rotation(self, _request, response):
        with self._lock:
            pending = self._policy.pending
            if pending is None:
                response.success = False
                response.message = 'no_pending_rotation_intent'
                return response
            ready, reason = self._safety_and_odom_ready_locked()
            if not ready:
                response.success = False
                response.message = reason
                return response
            transition = self._policy.authorize(
                pending.query_id, time.monotonic())
            if not transition.accepted:
                response.success = False
                response.message = transition.reason
                return response
            self._arming = True

        success, message = self._call_trigger(self._arm_client)
        with self._lock:
            self._arming = False
            if not success:
                self._policy.cancel('safety_arm_rejected')
                response.success = False
                response.message = message or 'safety_arm_rejected'
                self._publish_status(
                    pending.query_id,
                    'SAFETY_REJECTED',
                    response.message,
                    error=True,
                )
                return response
        self._publish_status(
            pending.query_id, 'AUTHORIZED', 'rotation_authorized')
        started = self._start_pending_spin()
        response.success = started
        response.message = (
            'rotation_authorized_and_spin_requested'
            if started else 'rotation_authorized_but_spin_unavailable')
        return response

    def _cancel_search(self, _request, response):
        with self._lock:
            query_id = (
                self._policy.authorized_query_id or
                (self._policy.pending.query_id
                 if self._policy.pending is not None else 0))
            transition = self._policy.cancel('operator_cancelled')
        self._cancel_spin_and_disarm(transition.reason)
        self._publish_status(query_id, 'CANCELLED', transition.reason)
        response.success = True
        response.message = 'active search cancelled and disarmed'
        return response

    def _start_pending_spin(self):
        with self._lock:
            pending = self._policy.pending
            if pending is None:
                return False
            transition = self._policy.begin_spin(
                pending.query_id, time.monotonic())
            if not transition.accepted:
                self._publish_status(
                    pending.query_id,
                    'REJECTED',
                    transition.reason,
                    error=True,
                )
                return False
            if not self._spin_client.wait_for_server(timeout_sec=0.5):
                self._policy.cancel('nav2_spin_unavailable')
                self._cancel_spin_and_disarm('nav2_spin_unavailable')
                self._publish_status(
                    pending.query_id,
                    'NAV2_UNAVAILABLE',
                    'nav2_spin_unavailable',
                    error=True,
                )
                return False
            goal = Spin.Goal()
            goal.target_yaw = float(pending.signed_rotation_rad)
            self._pending_goal_query_id = pending.query_id
            future = self._spin_client.send_goal_async(
                goal, feedback_callback=self._on_spin_feedback)
            future.add_done_callback(self._on_spin_goal_response)
            self._publish_status(
                pending.query_id, 'SPIN_REQUESTED', 'spin_goal_sent')
            return True

    def _on_spin_goal_response(self, future):
        query_id = self._pending_goal_query_id or 0
        try:
            goal_handle = future.result()
        except Exception as error:
            self._finish_spin_failure(
                query_id, 'spin_goal_exception:{}'.format(error))
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_spin_failure(query_id, 'spin_goal_rejected')
            return
        with self._lock:
            self._spin_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_spin_result)
        self._publish_status(query_id, 'SPINNING', 'spin_goal_accepted')

    def _on_spin_feedback(self, feedback_message):
        query_id = self._policy.active_query_id or 0
        distance = float(
            feedback_message.feedback.angular_distance_traveled)
        self._publish_status(
            query_id,
            'SPINNING',
            'spin_feedback',
            angular_distance=distance,
        )

    def _on_spin_result(self, future):
        query_id = self._policy.active_query_id or 0
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as error:
            self._finish_spin_failure(
                query_id, 'spin_result_exception:{}'.format(error))
            return
        with self._lock:
            self._spin_goal_handle = None
            self._pending_goal_query_id = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                transition = self._policy.complete_spin(query_id)
            else:
                transition = self._policy.cancel(
                    'spin_failed_status_{}'.format(status))
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(
                query_id, 'SPIN_COMPLETED', transition.reason)
        else:
            self._cancel_spin_and_disarm(transition.reason)
            self._publish_status(
                query_id,
                'SPIN_FAILED',
                transition.reason,
                error=True,
            )

    def _finish_spin_failure(self, query_id, reason):
        with self._lock:
            self._spin_goal_handle = None
            self._pending_goal_query_id = None
            transition = self._policy.cancel(reason)
        self._cancel_spin_and_disarm(transition.reason)
        self._publish_status(
            query_id, 'SPIN_FAILED', transition.reason, error=True)

    def _cancel_spin_and_disarm(self, reason):
        with self._lock:
            goal_handle = self._spin_goal_handle
            self._spin_goal_handle = None
            self._pending_goal_query_id = None
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self._call_trigger(self._disarm_client)
        self._publish_status(0, 'DISARMED', reason)

    def _call_trigger(self, client):
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return False, 'service_unavailable'
        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=self._service_timeout_sec):
            return False, 'service_timeout'
        try:
            response = future.result()
        except Exception as error:
            return False, 'service_exception:{}'.format(error)
        if response is None:
            return False, 'empty_service_response'
        return bool(response.success), str(response.message)

    def _on_odom(self, _message):
        with self._lock:
            self._latest_odom_monotonic = time.monotonic()

    def _on_safety(self, message):
        faults = {
            SafetyState.STATE_BLOCKED: 'rotation_blocked',
            SafetyState.STATE_SENSOR_STALE: 'safety_sensor_stale',
            SafetyState.STATE_RC_OVERRIDE: 'rc_override',
            SafetyState.STATE_BASE_FAULT: 'base_fault',
            SafetyState.STATE_EMERGENCY_STOP: 'emergency_stop',
        }
        reason = faults.get(int(message.state), '')
        if message.emergency_stop_latched:
            reason = 'emergency_stop'
        elif message.rc_override_active:
            reason = 'rc_override'
        elif not message.cloud_fresh:
            reason = 'obstacle_cloud_stale'
        elif not message.base_status_fresh or not message.base_status_ok:
            reason = 'base_status_unhealthy'
        with self._lock:
            if (
                    not reason and not message.armed and
                    self._policy.authorized_query_id is not None and
                    not self._arming):
                reason = 'safety_disarmed'
            self._latest_safety_monotonic = time.monotonic()
            self._latest_safety_healthy = not bool(reason)
            self._latest_safety_reason = reason or 'safety_healthy'
            if reason and (
                    self._policy.authorized_query_id is not None or
                    self._policy.active_query_id is not None):
                query_id = (
                    self._policy.active_query_id or
                    self._policy.authorized_query_id or 0)
                transition = self._policy.update_safety(False, reason)
            else:
                query_id = 0
                transition = None
        if transition is not None:
            self._cancel_spin_and_disarm(transition.reason)
            self._publish_status(
                query_id,
                'SAFETY_REJECTED',
                transition.reason,
                error=True,
            )

    def _watchdog(self):
        with self._lock:
            transition = self._policy.expire(time.monotonic())
            authorized = self._policy.authorized_query_id
            ready, reason = self._safety_and_odom_ready_locked()
        if not transition.accepted and transition.cancel_spin:
            self._cancel_spin_and_disarm(transition.reason)
            self._publish_status(
                authorized or 0,
                'WATCHDOG_STOP',
                transition.reason,
                error=True,
            )
        elif authorized is not None and not ready:
            with self._lock:
                fault = self._policy.update_safety(False, reason)
            self._cancel_spin_and_disarm(fault.reason)
            self._publish_status(
                authorized,
                'WATCHDOG_STOP',
                fault.reason,
                error=True,
            )

    def _publish_status(
            self,
            query_id,
            state,
            reason,
            error=False,
            angular_distance=None):
        status = DiagnosticStatus()
        status.name = 'phase5a_search_motion_adapter'
        status.hardware_id = 'track_robot'
        status.level = DiagnosticStatus.ERROR if error else DiagnosticStatus.OK
        status.message = str(reason)
        status.values = [
            KeyValue(key='query_id', value=str(int(query_id))),
            KeyValue(key='state', value=str(state)),
            KeyValue(key='reason', value=str(reason)),
            KeyValue(
                key='authorized_query_id',
                value=str(self._policy.authorized_query_id or 0)),
            KeyValue(key='forward_permitted', value='false'),
        ]
        if angular_distance is not None:
            status.values.append(KeyValue(
                key='angular_distance_traveled',
                value=str(float(angular_distance))))
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self._status_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SearchMotionAdapter()
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
