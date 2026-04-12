import sys

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from rclpy.node import Node

from track_robot_interfaces.msg import TeleopInput
from track_robot_teleop.gui_main_window import MainWindow
from track_robot_teleop.teleop_logic import InputStateController, TeleopConfig, TeleopLogic


class GuiInputNode(Node):
    def __init__(self):
        super().__init__("gui_input_node")

        self.declare_parameter("input_topic", "/teleop/input_state")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("stop_on_focus_loss", True)
        self.declare_parameter("max_linear", 0.6)
        self.declare_parameter("max_angular", 1.0)
        self.declare_parameter("linear_accel", 0.4)
        self.declare_parameter("angular_accel", 0.8)
        self.declare_parameter("linear_decay", 1.2)
        self.declare_parameter("angular_decay", 2.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.stop_on_focus_loss = bool(self.get_parameter("stop_on_focus_loss").value)

        self.input_controller = InputStateController()
        self.publisher_ = self.create_publisher(TeleopInput, self.input_topic, 10)

        visual_config = TeleopConfig(
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            linear_accel=float(self.get_parameter("linear_accel").value),
            angular_accel=float(self.get_parameter("angular_accel").value),
            linear_decay=float(self.get_parameter("linear_decay").value),
            angular_decay=float(self.get_parameter("angular_decay").value),
        )
        self.visual_logic = TeleopLogic(visual_config)

        self.get_logger().info(f"Publishing GUI input state to: {self.input_topic}")
        self.get_logger().info(f"Publish rate: {self.publish_hz:.1f} Hz")

    def build_message(self) -> TeleopInput:
        state = self.input_controller.snapshot()
        msg = TeleopInput()
        msg.stamp = self.get_clock().now().to_msg()
        msg.forward = state.forward
        msg.backward = state.backward
        msg.left = state.left
        msg.right = state.right
        msg.stop = state.stop
        return msg

    def publish_state(self):
        msg = self.build_message()
        self.publisher_.publish(msg)
        if msg.stop:
            self.input_controller.clear_stop()

    def request_stop(self):
        self.input_controller.request_stop()
        self.publish_state()

    def neutralize(self):
        self.input_controller.neutralize(stop=self.stop_on_focus_loss)
        self.publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = GuiInputNode()

    app = QApplication(sys.argv)
    window = MainWindow(
        input_controller=node.input_controller,
        stop_callback=node.request_stop,
        neutralize_callback=node.neutralize,
    )
    window.show()
    window.setFocus()

    timer = QTimer()
    interval_ms = max(1, int(1000.0 / node.publish_hz))
    dt = 1.0 / node.publish_hz

    def loop():
        rclpy.spin_once(node, timeout_sec=0.0)
        state = node.input_controller.snapshot()
        linear_x, angular_z = node.visual_logic.update(state, dt, timed_out=False)
        window.update_feedback(
            input_summary=state.summary(),
            forward=state.forward,
            backward=state.backward,
            left=state.left,
            right=state.right,
            stop=state.stop,
            linear_x=linear_x,
            max_linear=node.visual_logic.config.max_linear,
            angular_z=angular_z,
            max_angular=node.visual_logic.config.max_angular,
        )
        node.publish_state()

    timer.timeout.connect(loop)
    timer.start(interval_ms)

    exit_code = app.exec_()

    node.input_controller.neutralize(stop=True)
    node.publish_state()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
