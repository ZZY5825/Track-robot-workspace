import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '227'

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy

from bunker_msgs.msg import BunkerRCState, BunkerStatus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from track_robot_interfaces.msg import SafetyState


def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package='track_robot_safety',
        executable='motion_safety_supervisor_node',
        parameters=[{
            'require_bunker_status': True,
            'require_rc_state': True,
            'require_can_control_mode': True,
            'require_planner_state': False,
            'require_odom': False,
            'command_timeout_sec': 2.0,
            'cloud_timeout_sec': 2.0,
            'base_status_timeout_sec': 2.0,
            'rc_timeout_sec': 2.0,
            'publish_rate': 50.0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        supervisor,
        launch_testing.actions.ReadyToTest(),
    ]), {'supervisor_process': supervisor}


class TestRcControlMode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('rc_control_mode_test_client')
        self.command_pub = self.node.create_publisher(
            Twist, '/follow/cmd_vel_avoiding', 10)
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, '/safety/filtered_obstacle_points', 10)
        self.status_pub = self.node.create_publisher(
            BunkerStatus, '/bunker_status', 10)
        self.rc_pub = self.node.create_publisher(
            BunkerRCState, '/bunker_rc_state', 10)
        self.states = []
        self.state_sub = self.node.create_subscription(
            SafetyState, '/safety/state', self.states.append, 10)
        self.arm_client = self.node.create_client(Trigger, '/safety/arm')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('Timed out; last safety states: {}'.format([
            (state.state, state.armed, state.reason)
            for state in self.states[-8:]
        ]))

    def publish_inputs(self, control_mode, count=5):
        status = BunkerStatus()
        status.vehicle_state = 0
        status.error_code = 0
        status.control_mode = control_mode
        cloud = PointCloud2()
        cloud.header.frame_id = 'base_link'
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 8
        for _ in range(count):
            self.command_pub.publish(Twist())
            self.cloud_pub.publish(cloud)
            self.status_pub.publish(status)
            self.rc_pub.publish(BunkerRCState())
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.02)

    def test_rc_mode_with_neutral_sticks_disarms_and_stays_disarmed_after_can_restore(self):
        self.publish_inputs(control_mode=1)
        self.assertTrue(self.arm_client.wait_for_service(timeout_sec=2.0))
        future = self.arm_client.call_async(Trigger.Request())
        self.spin_until(lambda: future.done())
        self.assertTrue(future.result().success)
        self.spin_until(lambda: self.states and self.states[-1].armed)

        self.publish_inputs(control_mode=3)
        self.spin_until(
            lambda: self.states
            and self.states[-1].state == SafetyState.STATE_RC_OVERRIDE)
        self.assertFalse(self.states[-1].armed)
        self.assertTrue(self.states[-1].rc_override_active)
        self.assertEqual(self.states[-1].safe_cmd.linear.x, 0.0)
        self.assertEqual(self.states[-1].safe_cmd.angular.z, 0.0)

        self.publish_inputs(control_mode=1)
        self.spin_until(
            lambda: self.states
            and self.states[-1].state == SafetyState.STATE_DISARMED)
        self.assertFalse(self.states[-1].armed)
