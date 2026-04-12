import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from track_robot_interfaces.msg import TeleopInput
from track_robot_teleop.teleop_logic import TeleopConfig, TeleopInputState, TeleopLogic


class TeleopBackendNode(Node):
    def __init__(self):
        super().__init__("teleop_backend_node")

        self.declare_parameter("input_topic", "/teleop/input_state")
        self.declare_parameter("output_topic", "/teleop/cmd_vel")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("max_linear", 0.6)
        self.declare_parameter("max_angular", 1.0)
        self.declare_parameter("linear_accel", 0.4)
        self.declare_parameter("angular_accel", 0.8)
        self.declare_parameter("linear_decay", 1.2)
        self.declare_parameter("angular_decay", 2.0)
        self.declare_parameter("input_timeout_sec", 0.25)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)

        config = TeleopConfig(
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            linear_accel=float(self.get_parameter("linear_accel").value),
            angular_accel=float(self.get_parameter("angular_accel").value),
            linear_decay=float(self.get_parameter("linear_decay").value),
            angular_decay=float(self.get_parameter("angular_decay").value),
        )

        self.logic = TeleopLogic(config)
        self.current_input = TeleopInputState()
        self.last_input_time = None
        self.timeout_active = False

        self.publisher_ = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            TeleopInput,
            self.input_topic,
            self.input_callback,
            10,
        )
        self.dt = 1.0 / self.publish_hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(f"Subscribing input state from: {self.input_topic}")
        self.get_logger().info(f"Publishing teleop cmd to: {self.output_topic}")
        self.get_logger().info(f"Backend publish rate: {self.publish_hz:.1f} Hz")

    def input_callback(self, msg: TeleopInput):
        self.current_input = TeleopInputState(
            forward=msg.forward,
            backward=msg.backward,
            left=msg.left,
            right=msg.right,
            stop=msg.stop,
        )
        self.last_input_time = self.get_clock().now()

    def input_timed_out(self) -> bool:
        if self.last_input_time is None:
            return True

        elapsed_sec = (self.get_clock().now() - self.last_input_time).nanoseconds / 1e9
        return elapsed_sec > self.input_timeout_sec

    def publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

    def publish_zero_twist(self):
        self.publish_twist(0.0, 0.0)

    def timer_callback(self):
        timed_out = self.input_timed_out()

        if timed_out and not self.timeout_active:
            self.get_logger().warn("Teleop backend input timed out; forcing zero Twist.")
        elif not timed_out and self.timeout_active:
            self.get_logger().info("Teleop backend input recovered.")
        self.timeout_active = timed_out

        linear_x, angular_z = self.logic.update(
            self.current_input,
            self.dt,
            timed_out=timed_out,
        )
        self.publish_twist(linear_x, angular_z)

    def destroy_node(self):
        self.logic.reset()
        self.publish_zero_twist()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBackendNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
