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


def generate_test_description():
    decision = launch_ros.actions.Node(
        package='track_robot_decision',
        executable='follow_behavior_tree_node',
        parameters=[{
            'require_health_override': 'false',
            'require_avoidance_feedback_override': 'false',
            'require_safety_feedback_override': 'false',
            'tick_rate': 30.0,
            'confirmation_ticks': 3,
            'uncertain_hold_sec': 0.25,
            'search_entry_max_age_sec': 0.8,
            'search_timeout_sec': 0.7,
            'blocked_clear_ticks': 3,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        decision,
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
        self.reset_target_calls = 0
        self.reset_target_service = self.node.create_service(
            Trigger, '/human_tracking/reset_target', self.on_reset_target)
        self.target_pub = self.node.create_publisher(
            TargetState, '/human_tracking/target_state', 10)
        self.avoidance_pub = self.node.create_publisher(
            AvoidanceState, '/follow/avoidance_state', 10)
        self.safety_pub = self.node.create_publisher(
            SafetyState, '/safety/state', 10)
        self.decisions = []
        self.decision_sub = self.node.create_subscription(
            FollowDecision, '/follow/decision', self.decisions.append, 10)

    def tearDown(self):
        self.node.destroy_node()

    def on_reset_target(self, _request, response):
        self.reset_target_calls += 1
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
        resets_before_rc = self.reset_target_calls
        self.publish_repeated(self.safety_pub, rc, count=1)
        stopped = self.spin_until(
            lambda: self.decisions[-1].behavior ==
            FollowDecision.BEHAVIOR_RC_OVERRIDE)
        self.assertFalse(stopped.motion_permitted)
        self.spin_until(
            lambda: self.reset_target_calls > resets_before_rc)
