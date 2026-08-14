import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '226'

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from std_srvs.srv import Trigger

from track_robot_interfaces.msg import (
    AvoidanceState,
    FollowDecision,
    SafetyState,
    TargetState,
)


def decision_node(name='follow_behavior_tree_node', prefix=''):
    parameters = {
        'require_health_override': 'false',
        'require_avoidance_feedback_override': 'false',
        'require_safety_feedback_override': 'false',
        'tick_rate': 30.0,
        'confirmation_ticks': 3,
        'uncertain_hold_sec': 0.25,
        'search_entry_max_age_sec': 0.8,
        'search_timeout_sec': 0.7,
        'blocked_clear_ticks': 3,
    }
    if prefix:
        parameters.update({
            'target_topic': prefix + '/target_state',
            'avoidance_topic': prefix + '/avoidance_state',
            'safety_topic': prefix + '/safety_state',
            'decision_topic': prefix + '/decision',
            'debug_topic': prefix + '/decision_debug',
            'marker_topic': prefix + '/decision_markers',
            'reset_target_service': prefix + '/reset_target',
        })
    return launch_ros.actions.Node(
        package='track_robot_decision',
        executable='follow_behavior_tree_node',
        name=name,
        parameters=[parameters],
        output='screen',
    )


def generate_test_description():
    decision = decision_node()
    relock_decision = decision_node('relock_decision', '/test/relock')
    generation_decision = decision_node('generation_decision', '/test/generation')
    delayed_decision = decision_node('delayed_decision', '/test/delayed')
    unavailable_decision = decision_node('unavailable_decision', '/test/unavailable')
    return launch.LaunchDescription([
        decision,
        relock_decision,
        generation_decision,
        delayed_decision,
        unavailable_decision,
        launch_testing.actions.ReadyToTest(),
    ]), {'decision_process': decision}


class TestFollowDecision(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('follow_decision_test_client')
        prefixes = {
            'test_rc_clear_requires_reset_and_no_target_before_relock':
                '/test/relock',
            'test_new_rc_generation_waits_for_own_reset_response':
                '/test/generation',
            'test_delayed_reset_response_keeps_one_request_in_flight':
                '/test/delayed',
            'test_reset_retries_when_service_appears_after_rc_transition':
                '/test/unavailable',
        }
        prefix = prefixes.get(self._testMethodName, '')
        self.reset_target_calls = 0
        self.reset_target_successes = 0
        self.reset_target_failures_remaining = 0
        self.transition_during_first_reset = False
        self.reset_target_delay_sec = 0.0
        self.reset_service_name = prefix + '/reset_target' if prefix else \
            '/human_tracking/reset_target'
        self.reset_target_service = None
        if self._testMethodName != \
                'test_reset_retries_when_service_appears_after_rc_transition':
            self.reset_target_service = self.node.create_service(
                Trigger, self.reset_service_name, self.on_reset_target)
        self.target_pub = self.node.create_publisher(
            TargetState, prefix + '/target_state' if prefix else
            '/human_tracking/target_state', 10)
        self.avoidance_pub = self.node.create_publisher(
            AvoidanceState, prefix + '/avoidance_state' if prefix else
            '/follow/avoidance_state', 10)
        self.safety_pub = self.node.create_publisher(
            SafetyState, prefix + '/safety_state' if prefix else
            '/safety/state', 10)
        self.decisions = []
        self.decision_sub = self.node.create_subscription(
            FollowDecision, prefix + '/decision' if prefix else
            '/follow/decision', self.decisions.append, 10)

    def tearDown(self):
        self.node.destroy_node()

    def on_reset_target(self, _request, response):
        self.reset_target_calls += 1
        if self.transition_during_first_reset and self.reset_target_calls == 1:
            disarmed = SafetyState()
            disarmed.state = SafetyState.STATE_DISARMED
            self.safety_pub.publish(disarmed)
            time.sleep(0.15)
            rc = SafetyState()
            rc.state = SafetyState.STATE_RC_OVERRIDE
            rc.reason = 'test_new_rc_generation'
            self.safety_pub.publish(rc)
            time.sleep(0.3)
        elif self.reset_target_delay_sec:
            time.sleep(self.reset_target_delay_sec)
        if self.reset_target_failures_remaining:
            self.reset_target_failures_remaining -= 1
            response.success = False
            response.message = 'retry'
            return response
        self.reset_target_successes += 1
        response.success = True
        response.message = 'reset'
        return response

    def spin_until(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return self.decisions[-1]
        self.fail('Timed out; last decisions: {}'.format(
            [(msg.behavior, msg.reason) for msg in self.decisions[-8:]]))

    def publish_repeated(self, publisher, message, count=4):
        for _ in range(count):
            publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.04)

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @staticmethod
    def confirmed_target():
        msg = TargetState()
        msg.target_id = 7
        msg.lock_state = TargetState.LOCK_TARGET_LOCKED
        msg.source_state = TargetState.SOURCE_CAMERA_LIDAR
        msg.track_state = TargetState.TRACK_CAMERA_LIDAR_TRACKED
        msg.position_base_valid = True
        msg.position_base.x = 3.0
        msg.position_base.y = 0.2
        msg.distance = 3.0
        msg.bearing = 0.07
        msg.selected_tracklet_id = 12
        msg.association_state = TargetState.ASSOCIATION_CONFIRMED
        msg.identity_confidence = 0.9
        msg.geometry_confidence = 0.85
        msg.overall_confidence = 0.88
        msg.position_covariance[0] = 0.04
        msg.position_covariance[4] = 0.04
        msg.measurement_age = 0.05
        msg.time_since_camera_seen = 0.05
        msg.time_since_lidar_seen = 0.05
        return msg

    def test_complete_behavior_sequence(self):
        self.spin_until(
            lambda: self.decisions and self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_WAITING_FOR_TARGET)

        target = self.confirmed_target()
        self.publish_repeated(self.target_pub, target)
        confirmed = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_FOLLOW_CONFIRMED)
        self.assertTrue(confirmed.forward_permitted)
        self.assertLessEqual(confirmed.maximum_linear_speed, 0.30)

        lidar = self.confirmed_target()
        lidar.source_state = TargetState.SOURCE_LIDAR_ONLY
        lidar.track_state = TargetState.TRACK_LIDAR_ONLY_TRACKING
        lidar.camera_visible = False
        self.publish_repeated(self.target_pub, lidar)
        limited = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_FOLLOW_LIDAR_LIMITED)
        self.assertTrue(limited.forward_permitted)
        self.assertLessEqual(limited.maximum_linear_speed, 0.151)

        uncertain = self.confirmed_target()
        uncertain.track_state = TargetState.TRACK_PREDICTION_ONLY
        uncertain.association_state = TargetState.ASSOCIATION_AMBIGUOUS
        uncertain.overall_confidence = 0.3
        self.publish_repeated(self.target_pub, uncertain, count=1)
        hold = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_UNCERTAIN_HOLD)
        self.assertFalse(hold.motion_permitted)

        search = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_SEARCH_ROTATE, timeout=1.0)
        self.assertFalse(search.forward_permitted)
        self.assertTrue(search.rotation_permitted)

        lost = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_TARGET_LOST, timeout=1.5)
        self.assertFalse(lost.motion_permitted)

        no_target = TargetState()
        no_target.target_id = -1
        no_target.track_state = TargetState.TRACK_NO_TARGET
        self.publish_repeated(self.target_pub, no_target, count=1)
        self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_WAITING_FOR_TARGET)

        self.publish_repeated(self.target_pub, target)
        self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_FOLLOW_CONFIRMED)
        blocked = AvoidanceState()
        blocked.state = AvoidanceState.STATE_NO_SAFE_TRAJECTORY
        blocked.reason = 'test_blocked'
        self.publish_repeated(self.avoidance_pub, blocked, count=1)
        self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_BLOCKED_HOLD)

        rc = SafetyState()
        rc.state = SafetyState.STATE_RC_OVERRIDE
        rc.reason = 'test_rc_override'
        reset_calls_before_rc = self.reset_target_calls
        reset_successes_before_rc = self.reset_target_successes
        self.publish_repeated(self.safety_pub, rc, count=1)
        stopped = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.assertFalse(stopped.motion_permitted)
        self.spin_until(
            lambda: self.reset_target_successes == reset_successes_before_rc + 1)
        self.spin_for(0.3)
        self.assertEqual(self.reset_target_calls, reset_calls_before_rc + 1)
        self.assertEqual(self.reset_target_successes, reset_successes_before_rc + 1)

    def test_rc_reset_retries_after_failed_response(self):
        disarmed = SafetyState()
        disarmed.state = SafetyState.STATE_DISARMED
        clear = AvoidanceState()
        clear.state = AvoidanceState.STATE_DIRECT_CLEAR
        self.publish_repeated(self.safety_pub, disarmed, count=3)
        self.publish_repeated(self.avoidance_pub, clear, count=3)
        self.spin_until(
            lambda: self.decisions and self.decisions[-1].behavior !=
            FollowDecision.BEHAVIOR_RC_OVERRIDE)

        self.reset_target_failures_remaining = 1
        reset_calls_before_rc = self.reset_target_calls
        reset_successes_before_rc = self.reset_target_successes
        rc = SafetyState()
        rc.state = SafetyState.STATE_RC_OVERRIDE
        rc.reason = 'test_rc_override_retry'
        self.publish_repeated(self.safety_pub, rc, count=1)
        stopped = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.assertFalse(stopped.motion_permitted)
        self.spin_until(
            lambda: self.reset_target_successes == reset_successes_before_rc + 1)
        self.spin_for(0.3)
        self.assertEqual(self.reset_target_calls, reset_calls_before_rc + 2)
        self.assertEqual(self.reset_target_successes, reset_successes_before_rc + 1)

    def test_rc_clear_requires_reset_and_no_target_before_relock(self):
        disarmed = SafetyState()
        disarmed.state = SafetyState.STATE_DISARMED
        self.publish_repeated(self.safety_pub, disarmed, count=1)

        stale_target = self.confirmed_target()
        self.publish_repeated(self.target_pub, stale_target)
        self.spin_until(
            lambda: self.decisions and self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_FOLLOW_CONFIRMED)

        rc = SafetyState()
        rc.state = SafetyState.STATE_RC_OVERRIDE
        rc.reason = 'test_rc_relock'
        self.publish_repeated(self.safety_pub, rc, count=1)
        self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.spin_until(lambda: self.reset_target_successes == 1)

        self.publish_repeated(self.safety_pub, disarmed, count=1)
        self.publish_repeated(self.target_pub, stale_target)
        held = self.spin_until(
            lambda: self.decisions[-1].behavior !=
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.assertNotIn(held.behavior, (
            FollowDecision.BEHAVIOR_FOLLOW_CONFIRMED,
            FollowDecision.BEHAVIOR_FOLLOW_LIDAR_LIMITED,
            FollowDecision.BEHAVIOR_SEARCH_ROTATE,
        ))
        self.assertFalse(held.motion_permitted)
        self.assertFalse(held.automatic_resume_permitted)

        no_target = TargetState()
        no_target.target_id = -1
        no_target.lock_state = TargetState.LOCK_NO_TARGET
        no_target.track_state = TargetState.TRACK_NO_TARGET
        self.publish_repeated(self.target_pub, no_target, count=1)

        new_target = self.confirmed_target()
        new_target.target_id = 8
        self.publish_repeated(self.target_pub, new_target)
        resumed = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_FOLLOW_CONFIRMED)
        self.assertTrue(resumed.motion_permitted)
        self.assertTrue(resumed.automatic_resume_permitted)

    def test_new_rc_generation_waits_for_own_reset_response(self):
        disarmed = SafetyState()
        disarmed.state = SafetyState.STATE_DISARMED
        self.publish_repeated(self.safety_pub, disarmed, count=1)

        self.transition_during_first_reset = True
        rc = SafetyState()
        rc.state = SafetyState.STATE_RC_OVERRIDE
        rc.reason = 'test_first_rc_generation'
        self.publish_repeated(self.safety_pub, rc, count=1)
        stopped = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.assertFalse(stopped.motion_permitted)

        self.spin_until(lambda: self.reset_target_successes == 2, timeout=3.0)
        self.spin_for(0.3)
        self.assertEqual(self.reset_target_calls, 2)

    def test_delayed_reset_response_keeps_one_request_in_flight(self):
        disarmed = SafetyState()
        disarmed.state = SafetyState.STATE_DISARMED
        self.publish_repeated(self.safety_pub, disarmed, count=1)

        self.reset_target_delay_sec = 0.45
        rc = SafetyState()
        rc.state = SafetyState.STATE_RC_OVERRIDE
        rc.reason = 'test_delayed_reset'
        self.publish_repeated(self.safety_pub, rc, count=1)
        stopped = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.assertFalse(stopped.motion_permitted)
        self.spin_until(lambda: self.reset_target_successes == 1)
        self.assertEqual(self.reset_target_calls, 1)

        self.spin_for(0.4)
        self.assertEqual(self.reset_target_calls, 1)

    def test_reset_retries_when_service_appears_after_rc_transition(self):
        disarmed = SafetyState()
        disarmed.state = SafetyState.STATE_DISARMED
        self.publish_repeated(self.safety_pub, disarmed, count=1)

        rc = SafetyState()
        rc.state = SafetyState.STATE_RC_OVERRIDE
        rc.reason = 'test_reset_service_unavailable'
        self.publish_repeated(self.safety_pub, rc, count=1)
        stopped = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.assertFalse(stopped.motion_permitted)
        self.spin_for(0.4)
        self.assertEqual(self.reset_target_calls, 0)

        self.reset_target_service = self.node.create_service(
            Trigger, self.reset_service_name, self.on_reset_target)
        self.spin_until(lambda: self.reset_target_successes == 1, timeout=3.0)
        self.spin_for(0.3)
        self.assertEqual(self.reset_target_calls, 1)
