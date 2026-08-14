import os
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '229'

import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.markers  # noqa: E402
import rclpy  # noqa: E402

from geometry_msgs.msg import Twist  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


@launch_testing.markers.keep_alive
def generate_test_description():
    gate = launch_ros.actions.Node(
        package='track_robot_core',
        executable='cmd_vel_gate',
        name='cmd_vel_gate',
        parameters=[{
            'input_topic': '/test/cmd_vel_gate/input',
            'output_topic': '/cmd_vel',
            'timeout_sec': 5.0,
            'publish_rate': 50.0,
            'max_linear_x': 0.6,
            'max_angular_z': 1.0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        gate,
        launch_testing.actions.ReadyToTest(),
    ]), {'gate_process': gate}


class TestCmdVelGateShutdown(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('cmd_vel_gate_shutdown_test_client')
        self.input_pub = self.node.create_publisher(
            Twist, '/test/cmd_vel_gate/input', 10)
        self.outputs = []
        self.output_sub = self.node.create_subscription(
            Twist, '/cmd_vel', self.outputs.append, 10)
        self.shutdown_client = self.node.create_client(
            Trigger, '/cmd_vel_gate/shutdown')

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('timed out; received {} output commands'.format(
            len(self.outputs)))

    def test_nonzero_is_followed_by_zero_and_publisher_disappears(self):
        self.spin_until(
            lambda: self.input_pub.get_subscription_count() == 1)

        command = Twist()
        command.linear.x = 0.4
        command.angular.z = -0.3
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not any(
                output.linear.x != 0.0 or output.angular.z != 0.0
                for output in self.outputs):
            self.input_pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        nonzero_index = next(
            index for index, output in enumerate(self.outputs)
            if output.linear.x != 0.0 or output.angular.z != 0.0)

        self.assertTrue(
            self.shutdown_client.wait_for_service(timeout_sec=2.0))
        future = self.shutdown_client.call_async(Trigger.Request())
        self.spin_until(lambda: future.done(), timeout=2.0)
        self.assertTrue(future.result().success)

        self.spin_until(
            lambda: any(
                output.linear.x == 0.0 and output.linear.y == 0.0
                and output.linear.z == 0.0 and output.angular.x == 0.0
                and output.angular.y == 0.0 and output.angular.z == 0.0
                for output in self.outputs[nonzero_index + 1:]),
            timeout=2.0,
        )
        self.spin_until(
            lambda: self.node.count_publishers('/cmd_vel') == 0,
            timeout=5.0,
        )
