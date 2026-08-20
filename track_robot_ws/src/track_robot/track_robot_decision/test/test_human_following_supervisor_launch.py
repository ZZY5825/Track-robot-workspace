import os
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '228'

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from bunker_msgs.msg import BunkerStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from track_robot_interfaces.msg import (
    AvoidanceState,
    CameraTarget,
    FollowDecision,
    GestureState,
    HumanFollowingSession,
    PerceptionHealth,
    SafetyState,
)


SCENARIOS = {
    'test_shadow_never_arms': ('/test/session_shadow', 'shadow'),
    'test_active_arms_once': ('/test/session_active', 'active'),
    'test_authorized_stop_disarms_and_resets':
        ('/test/session_stop', 'active'),
    'test_bunker_mode_three_revokes': ('/test/session_rc', 'active'),
    'test_can_return_requires_new_wave': ('/test/session_can', 'active'),
    'test_stop_from_other_visual_target_is_ignored':
        ('/test/session_wrong_stop', 'active'),
    'test_initial_arm_rejects_stale_base_status':
        ('/test/session_base_stale', 'active'),
    'test_bad_base_status_revokes_authorized_session':
        ('/test/session_base_fault', 'active'),
    'test_stale_base_status_revokes_authorized_session':
        ('/test/session_base_stale_after_arm', 'active'),
    'test_target_lost_revokes_authorized_session':
        ('/test/session_target_lost', 'active'),
    'test_health_stale_revokes_authorized_session':
        ('/test/session_health_stale', 'active'),
    'test_emergency_stop_revokes_authorized_session':
        ('/test/session_emergency_stop', 'active'),
    'test_required_input_stale_revokes_authorized_session':
        ('/test/session_input_stale', 'active'),
    'test_target_id_mismatch_revokes_authorized_session':
        ('/test/session_target_mismatch', 'active'),
    'test_lidar_only_cannot_initially_arm':
        ('/test/session_lidar_initial', 'active'),
    'test_short_block_retains_authorization':
        ('/test/session_short_block', 'active'),
    'test_stale_arm_success_disarms_and_cannot_authorize_new_session':
        ('/test/session_stale_arm', 'active'),
}


def supervisor_node(prefix, runtime_mode):
    remappings = [
        ('/human_tracking/gesture_state', prefix + '/gesture_state'),
        ('/human_tracking/camera_target', prefix + '/camera_target'),
        ('/follow/decision', prefix + '/decision'),
        ('/perception/health', prefix + '/health'),
        ('/follow/avoidance_state', prefix + '/avoidance_state'),
        ('/safety/state', prefix + '/safety_state'),
        ('/bunker_status', prefix + '/bunker_status'),
        ('/safety/arm', prefix + '/arm'),
        ('/safety/disarm', prefix + '/disarm'),
        ('/human_tracking/reset_target', prefix + '/reset_target'),
        ('/human_following/session_state', prefix + '/session_state'),
        ('/human_following/supervisor_debug', prefix + '/debug'),
        ('/human_following/supervisor_markers', prefix + '/markers'),
    ]
    return launch_ros.actions.Node(
        package='track_robot_decision',
        executable='human_following_supervisor_node',
        name='human_following_supervisor_' + prefix.rsplit('/', 1)[-1],
        parameters=[{
            'runtime_mode': runtime_mode,
            'motion_confirmed': runtime_mode == 'active',
            'tick_rate': 30.0,
            'gesture_timeout_sec': 0.50,
            'camera_timeout_sec': 0.35,
            'decision_timeout_sec': 0.30,
            'health_timeout_sec': 0.30,
            'avoidance_timeout_sec': 0.30,
            'safety_timeout_sec': 0.20,
            'bunker_timeout_sec': 0.20,
            'blocked_disarm_timeout_sec': 10.0,
            'uncertain_authorization_timeout_sec': 1.0,
        }],
        remappings=remappings,
        output='screen',
    )


def generate_test_description():
    nodes = [supervisor_node(*scenario) for scenario in SCENARIOS.values()]
    return launch.LaunchDescription(
        nodes + [launch_testing.actions.ReadyToTest()])


class TestHumanFollowingSupervisor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.prefix, self.runtime_mode = SCENARIOS[self._testMethodName]
        node_name = 'human_following_supervisor_test_' + \
            self.prefix.rsplit('/', 1)[-1]
        self.node = rclpy.create_node(node_name)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True)

        self.arm_calls = 0
        self.disarm_calls = 0
        self.reset_calls = 0
        self.delay_first_arm = False
        self.first_arm_entered = threading.Event()
        self.release_first_arm = threading.Event()
        self.service_callback_group = ReentrantCallbackGroup()

        self.arm_service = self.node.create_service(
            Trigger, self.prefix + '/arm', self.on_arm,
            callback_group=self.service_callback_group)
        self.disarm_service = self.node.create_service(
            Trigger, self.prefix + '/disarm', self.on_disarm,
            callback_group=self.service_callback_group)
        self.reset_service = self.node.create_service(
            Trigger, self.prefix + '/reset_target', self.on_reset,
            callback_group=self.service_callback_group)

        self.gesture_pub = self.node.create_publisher(
            GestureState, self.prefix + '/gesture_state', 10)
        self.camera_pub = self.node.create_publisher(
            CameraTarget, self.prefix + '/camera_target', 10)
        self.decision_pub = self.node.create_publisher(
            FollowDecision, self.prefix + '/decision', 10)
        self.health_pub = self.node.create_publisher(
            PerceptionHealth, self.prefix + '/health', 10)
        self.avoidance_pub = self.node.create_publisher(
            AvoidanceState, self.prefix + '/avoidance_state', 10)
        self.safety_pub = self.node.create_publisher(
            SafetyState, self.prefix + '/safety_state', 10)
        self.bunker_pub = self.node.create_publisher(
            BunkerStatus, self.prefix + '/bunker_status', 10)

        self.states = []
        self.debug_messages = []
        self.marker_messages = []
        self.state_sub = self.node.create_subscription(
            HumanFollowingSession, self.prefix + '/session_state',
            self.states.append, 10)
        self.debug_sub = self.node.create_subscription(
            String, self.prefix + '/debug', self.debug_messages.append, 10)
        self.marker_sub = self.node.create_subscription(
            MarkerArray, self.prefix + '/markers',
            self.marker_messages.append, 10)

        self.logical_target_id = 42
        self.visual_track_id = 7
        self.bunker_mode = 1
        self.safety_armed = False
        self.base_status_fresh = True
        self.base_status_ok = True
        self.decision_behavior = FollowDecision.BEHAVIOR_FOLLOW_CONFIRMED
        self.decision_target_id = None
        self.decision_source = 'camera_lidar'
        self.health_state = PerceptionHealth.HEALTHY
        self.avoidance_state = AvoidanceState.STATE_DIRECT_CLEAR
        self.safety_emergency_stop = False
        self.executor_thread.start()
        self.wait_for_graph()

    def tearDown(self):
        self.release_first_arm.set()
        self.executor.shutdown(timeout_sec=2.0)
        self.executor_thread.join(timeout=2.0)
        self.node.destroy_node()

    def on_arm(self, _request, response):
        self.arm_calls += 1
        if self.delay_first_arm and self.arm_calls == 1:
            self.first_arm_entered.set()
            self.release_first_arm.wait(timeout=3.0)
        response.success = True
        response.message = 'armed'
        return response

    def on_disarm(self, _request, response):
        self.disarm_calls += 1
        response.success = True
        response.message = 'disarmed'
        return response

    def on_reset(self, _request, response):
        self.reset_calls += 1
        response.success = True
        response.message = 'reset'
        return response

    def wait_for_graph(self):
        publishers = [
            self.gesture_pub,
            self.camera_pub,
            self.decision_pub,
            self.health_pub,
            self.avoidance_pub,
            self.safety_pub,
            self.bunker_pub,
        ]
        self.wait_until(
            lambda: all(pub.get_subscription_count() > 0 for pub in publishers)
            and self.node.count_publishers(self.prefix + '/session_state') > 0
            and self.node.count_publishers(self.prefix + '/debug') > 0
            and self.node.count_publishers(self.prefix + '/markers') > 0
            and self.supervisor_clients_ready(),
            timeout=6.0)

    def supervisor_clients_ready(self):
        supervisor_name = 'human_following_supervisor_' + \
            self.prefix.rsplit('/', 1)[-1]
        try:
            clients = dict(self.node.get_client_names_and_types_by_node(
                supervisor_name, '/'))
        except RuntimeError:
            return False
        return all(name in clients for name in (
            self.prefix + '/arm',
            self.prefix + '/disarm',
            self.prefix + '/reset_target',
        ))

    def wait_until(self, predicate, timeout=2.0, republish=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if republish:
                self.publish_inputs()
            time.sleep(0.04)
        recent = [
            (msg.state, msg.target_authorized, msg.reason)
            for msg in self.states[-8:]
        ]
        self.fail('Timed out; recent session states: {}'.format(recent))

    def publish_inputs(self, count=1, publish_health=True):
        for _ in range(count):
            now = self.node.get_clock().now().to_msg()

            camera = CameraTarget()
            camera.header.stamp = now
            camera.header.frame_id = 'zed_left_camera_optical_frame'
            camera.logical_target_id = self.logical_target_id
            camera.visual_track_id = self.visual_track_id
            camera.lock_state = CameraTarget.LOCK_TARGET_LOCKED
            camera.identity_state = CameraTarget.IDENTITY_CONFIRMED
            camera.detector_confidence = 0.92
            camera.identity_confidence = 0.90
            camera.camera_visible = True
            self.camera_pub.publish(camera)

            decision = FollowDecision()
            decision.header.stamp = now
            decision.header.frame_id = 'base_link'
            decision.behavior = self.decision_behavior
            decision.logical_target_id = self.logical_target_id \
                if self.decision_target_id is None else self.decision_target_id
            decision.motion_permitted = True
            decision.decision_confidence = 0.88
            decision.target_source = self.decision_source
            decision.target_position.x = 2.5
            decision.target_position.y = 0.2
            self.decision_pub.publish(decision)

            health = PerceptionHealth()
            health.header.stamp = now
            health.state = self.health_state
            if publish_health:
                self.health_pub.publish(health)

            avoidance = AvoidanceState()
            avoidance.header.stamp = now
            avoidance.state = self.avoidance_state
            self.avoidance_pub.publish(avoidance)

            safety = SafetyState()
            safety.header.stamp = now
            safety.state = SafetyState.STATE_CLEAR if self.safety_armed \
                else SafetyState.STATE_DISARMED
            safety.armed = self.safety_armed
            safety.base_status_fresh = self.base_status_fresh
            safety.base_status_ok = self.base_status_ok
            safety.emergency_stop_latched = self.safety_emergency_stop
            if self.safety_emergency_stop:
                safety.state = SafetyState.STATE_EMERGENCY_STOP
            self.safety_pub.publish(safety)

            bunker = BunkerStatus()
            bunker.header.stamp = now
            bunker.control_mode = self.bunker_mode
            bunker.vehicle_state = 0
            bunker.error_code = 0
            self.bunker_pub.publish(bunker)
            time.sleep(0.035)

    def publish_gesture(self, command, track_id=None):
        gesture = GestureState()
        gesture.header.stamp = self.node.get_clock().now().to_msg()
        gesture.track_id = self.visual_track_id if track_id is None else track_id
        gesture.command = command
        gesture.trigger_active = True
        gesture.confidence = 0.95
        self.gesture_pub.publish(gesture)

    def start_session(self):
        self.publish_inputs(count=4)
        self.publish_gesture('start_tracking')
        self.wait_until(lambda: self.arm_calls == 1, republish=True)
        self.wait_until(
            lambda: self.states and self.states[-1].target_authorized,
            republish=True)

    def assert_fault_revocation(self, reason):
        self.wait_until(
            lambda: self.disarm_calls >= 1 and self.reset_calls >= 1 and any(
                msg.state == HumanFollowingSession.STATE_FAULT and
                not msg.target_authorized and msg.reason == reason
                for msg in self.states[-20:]),
            republish=True)
        self.assertFalse(self.states[-1].target_authorized)

    def assert_debug_and_markers_publish(self):
        self.wait_until(lambda: self.debug_messages and self.marker_messages)
        namespaces = {
            marker.ns for marker_array in self.marker_messages[-3:]
            for marker in marker_array.markers
        }
        self.assertIn('human_following_session/status', namespaces)
        self.assertIn('human_following_session/mode', namespaces)

    def test_shadow_never_arms(self):
        self.publish_inputs(count=4)
        self.publish_gesture('start_tracking')
        self.wait_until(lambda: len(self.states) >= 3, republish=True)
        time.sleep(0.2)
        self.assertEqual(0, self.arm_calls)
        self.assertFalse(self.states[-1].motion_session_enabled)
        self.assertFalse(self.states[-1].target_authorized)
        self.assert_debug_and_markers_publish()

    def test_active_arms_once(self):
        self.start_session()
        self.publish_inputs(count=8)
        self.assertEqual(1, self.arm_calls)
        self.assertEqual(
            HumanFollowingSession.STATE_FOLLOWING, self.states[-1].state)
        self.assertTrue(self.states[-1].motion_session_enabled)
        self.assertEqual(self.logical_target_id,
                         self.states[-1].logical_target_id)
        self.assert_debug_and_markers_publish()

    def test_authorized_stop_disarms_and_resets(self):
        self.start_session()
        self.publish_gesture('stop_tracking')
        self.wait_until(
            lambda: self.disarm_calls >= 1 and self.reset_calls >= 1,
            republish=True)
        self.assertFalse(self.states[-1].target_authorized)

    def test_bunker_mode_three_revokes(self):
        self.start_session()
        self.bunker_mode = 3
        self.wait_until(
            lambda: self.states and self.states[-1].state ==
            HumanFollowingSession.STATE_RC_OVERRIDE,
            republish=True)
        self.assertFalse(self.states[-1].target_authorized)
        self.assertTrue(self.states[-1].rc_override_active)
        self.wait_until(
            lambda: self.disarm_calls >= 1 and self.reset_calls >= 1)

    def test_can_return_requires_new_wave(self):
        self.start_session()
        self.bunker_mode = 3
        self.wait_until(
            lambda: self.states and self.states[-1].state ==
            HumanFollowingSession.STATE_RC_OVERRIDE,
            republish=True)
        self.bunker_mode = 1
        self.wait_until(
            lambda: self.states and self.states[-1].state ==
            HumanFollowingSession.STATE_WAITING_FOR_GESTURE,
            republish=True)
        arm_calls_after_rc = self.arm_calls
        self.publish_inputs(count=10)
        self.assertEqual(arm_calls_after_rc, self.arm_calls)
        self.assertFalse(self.states[-1].target_authorized)

        self.publish_gesture('start_tracking')
        self.wait_until(
            lambda: self.arm_calls == arm_calls_after_rc + 1,
            republish=True)

    def test_stop_from_other_visual_target_is_ignored(self):
        self.start_session()
        disarm_calls = self.disarm_calls
        reset_calls = self.reset_calls
        self.publish_gesture('stop_tracking', track_id=99)
        self.publish_inputs(count=8)
        self.assertEqual(disarm_calls, self.disarm_calls)
        self.assertEqual(reset_calls, self.reset_calls)
        self.assertTrue(self.states[-1].target_authorized)

    def test_initial_arm_rejects_stale_base_status(self):
        self.base_status_fresh = False
        self.publish_inputs(count=4)
        self.publish_gesture('start_tracking')
        self.publish_inputs(count=8)
        self.assertEqual(0, self.arm_calls)
        self.assertFalse(self.states[-1].target_authorized)

    def test_bad_base_status_revokes_authorized_session(self):
        self.start_session()
        self.base_status_ok = False
        self.wait_until(
            lambda: self.disarm_calls >= 1 and self.reset_calls >= 1 and any(
                msg.state == HumanFollowingSession.STATE_FAULT and
                not msg.target_authorized for msg in self.states[-20:]),
            republish=True)
        self.assertFalse(self.states[-1].target_authorized)

    def test_stale_base_status_revokes_authorized_session(self):
        self.start_session()
        self.base_status_fresh = False
        self.wait_until(
            lambda: self.disarm_calls >= 1 and self.reset_calls >= 1 and any(
                msg.state == HumanFollowingSession.STATE_FAULT and
                not msg.target_authorized for msg in self.states[-20:]),
            republish=True)
        self.assertFalse(self.states[-1].target_authorized)

    def test_target_lost_revokes_authorized_session(self):
        self.start_session()
        self.decision_behavior = FollowDecision.BEHAVIOR_TARGET_LOST
        self.assert_fault_revocation('target_lost')

    def test_health_stale_revokes_authorized_session(self):
        self.start_session()
        self.health_state = PerceptionHealth.STALE
        self.assert_fault_revocation('hard_fault')

    def test_emergency_stop_revokes_authorized_session(self):
        self.start_session()
        self.safety_emergency_stop = True
        self.assert_fault_revocation('hard_fault')

    def test_required_input_stale_revokes_authorized_session(self):
        self.start_session()
        self.publish_inputs(count=12, publish_health=False)
        self.assert_fault_revocation('required_inputs_stale')

    def test_target_id_mismatch_revokes_authorized_session(self):
        self.start_session()
        self.decision_target_id = self.logical_target_id + 1
        self.assert_fault_revocation('logical_target_mismatch')

    def test_lidar_only_cannot_initially_arm(self):
        self.decision_behavior = FollowDecision.BEHAVIOR_FOLLOW_LIDAR_LIMITED
        self.decision_source = 'lidar_only'
        self.publish_inputs(count=4)
        self.publish_gesture('start_tracking')
        self.publish_inputs(count=8)
        self.assertEqual(0, self.arm_calls)
        self.assertFalse(self.states[-1].target_authorized)

    def test_short_block_retains_authorization(self):
        self.start_session()
        self.avoidance_state = AvoidanceState.STATE_NO_SAFE_TRAJECTORY
        self.publish_inputs(count=8)
        self.assertTrue(self.states[-1].target_authorized)
        self.assertEqual(
            HumanFollowingSession.STATE_BLOCKED, self.states[-1].state)
        self.assertEqual(0, self.disarm_calls)
        self.assertEqual(0, self.reset_calls)

        self.avoidance_state = AvoidanceState.STATE_DIRECT_CLEAR
        self.wait_until(
            lambda: self.states and self.states[-1].state ==
            HumanFollowingSession.STATE_FOLLOWING,
            republish=True)
        self.assertTrue(self.states[-1].target_authorized)

    def test_stale_arm_success_disarms_and_cannot_authorize_new_session(self):
        self.delay_first_arm = True
        self.publish_inputs(count=4)
        self.publish_gesture('start_tracking')
        self.wait_until(self.first_arm_entered.is_set, republish=True)

        self.bunker_mode = 3
        self.wait_until(
            lambda: self.states and self.states[-1].state ==
            HumanFollowingSession.STATE_RC_OVERRIDE,
            republish=True)

        self.bunker_mode = 1
        self.logical_target_id = 43
        self.visual_track_id = 8
        self.wait_until(
            lambda: self.states and self.states[-1].state ==
            HumanFollowingSession.STATE_WAITING_FOR_GESTURE,
            republish=True)
        self.publish_gesture('start_tracking')
        self.publish_inputs(count=3)

        disarms_before_stale_success = self.disarm_calls
        self.release_first_arm.set()
        self.wait_until(
            lambda: self.disarm_calls > disarms_before_stale_success,
            republish=True)
        self.assertFalse(self.states[-1].target_authorized)

        arm_calls_before_new_wave = self.arm_calls
        self.publish_gesture('start_tracking')
        self.wait_until(
            lambda: self.arm_calls == arm_calls_before_new_wave + 1,
            republish=True)
        self.wait_until(
            lambda: self.states and self.states[-1].target_authorized,
            republish=True)
        self.assertEqual(43, self.states[-1].logical_target_id)
