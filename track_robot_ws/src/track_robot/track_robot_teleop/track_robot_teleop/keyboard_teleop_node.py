import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pynput import keyboard


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # Parameters
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_topic', '/teleop/cmd_vel')
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('linear_acc_step', 0.4)   # m/s^2
        self.declare_parameter('angular_acc_step', 0.8)  # rad/s^2
        self.declare_parameter('zero_on_release', True)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_acc_step = float(self.get_parameter('linear_acc_step').value)
        self.angular_acc_step = float(self.get_parameter('angular_acc_step').value)
        self.zero_on_release = bool(self.get_parameter('zero_on_release').value)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Internal state
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.w_pressed = False
        self.a_pressed = False
        self.s_pressed = False
        self.d_pressed = False

        self.state_lock = threading.Lock()

        # Timer
        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        self.get_logger().info('Keyboard teleop started.')
        self.get_logger().info('Use W/A/S/D to control the robot.')
        self.get_logger().info('Publishing to: %s' % self.cmd_vel_topic)

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return

        with self.state_lock:
            if k == 'w':
                self.w_pressed = True
            elif k == 'a':
                self.a_pressed = True
            elif k == 's':
                self.s_pressed = True
            elif k == 'd':
                self.d_pressed = True

    def on_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            # ESC to quit
            if key == keyboard.Key.esc:
                self.get_logger().info('ESC pressed, stopping teleop node.')
                self.publish_zero_twist()
                rclpy.shutdown()
            return

        with self.state_lock:
            if k == 'w':
                self.w_pressed = False
                if self.zero_on_release and not self.s_pressed:
                    self.linear_x = 0.0
            elif k == 'a':
                self.a_pressed = False
                if self.zero_on_release and not self.d_pressed:
                    self.angular_z = 0.0
            elif k == 's':
                self.s_pressed = False
                if self.zero_on_release and not self.w_pressed:
                    self.linear_x = 0.0
            elif k == 'd':
                self.d_pressed = False
                if self.zero_on_release and not self.a_pressed:
                    self.angular_z = 0.0

    def timer_callback(self):
        with self.state_lock:
            # Linear velocity logic
            if self.w_pressed and not self.s_pressed:
                self.linear_x += self.linear_acc_step * self.dt
                if self.linear_x > self.max_linear_speed:
                    self.linear_x = self.max_linear_speed
            elif self.s_pressed and not self.w_pressed:
                self.linear_x -= self.linear_acc_step * self.dt
                if self.linear_x < -self.max_linear_speed:
                    self.linear_x = -self.max_linear_speed
            elif not self.w_pressed and not self.s_pressed and self.zero_on_release:
                self.linear_x = 0.0
            # if both pressed -> keep unchanged

            # Angular velocity logic
            if self.a_pressed and not self.d_pressed:
                self.angular_z += self.angular_acc_step * self.dt
                if self.angular_z > self.max_angular_speed:
                    self.angular_z = self.max_angular_speed
            elif self.d_pressed and not self.a_pressed:
                self.angular_z -= self.angular_acc_step * self.dt
                if self.angular_z < -self.max_angular_speed:
                    self.angular_z = -self.max_angular_speed
            elif not self.a_pressed and not self.d_pressed and self.zero_on_release:
                self.angular_z = 0.0
            # if both pressed -> keep unchanged

            msg = Twist()
            msg.linear.x = self.linear_x
            msg.angular.z = self.angular_z

        self.cmd_pub.publish(msg)

    def publish_zero_twist(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        self.publish_zero_twist()
        try:
            self.listener.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_twist()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()