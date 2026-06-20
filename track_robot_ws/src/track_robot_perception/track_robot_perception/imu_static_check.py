#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def _mean(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / len(values)


def _stddev(values: Sequence[float], mean: float) -> float:
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


class ImuStaticCheck(Node):
    def __init__(self) -> None:
        super().__init__("imu_static_check")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("window_size", 500)
        self.declare_parameter("print_period_sec", 1.0)

        self.samples: Deque[Tuple[float, float, float, float, float, float]] = deque(
            maxlen=int(self.get_parameter("window_size").value)
        )

        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(
            float(self.get_parameter("print_period_sec").value),
            self._print_stats,
        )

    def _on_imu(self, message: Imu) -> None:
        self.samples.append(
            (
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            )
        )

    def _print_stats(self) -> None:
        if not self.samples:
            self.get_logger().warn("No IMU samples received yet")
            return

        columns = list(zip(*self.samples))
        acc_mean = tuple(_mean(columns[index]) for index in range(3))
        gyr_mean = tuple(_mean(columns[index]) for index in range(3, 6))
        acc_std = tuple(_stddev(columns[index], acc_mean[index]) for index in range(3))
        gyr_std = tuple(
            _stddev(columns[index + 3], gyr_mean[index]) for index in range(3)
        )
        acc_norm = math.sqrt(sum(value * value for value in acc_mean))

        if acc_norm > 1.0e-6:
            fast_lio_gravity = tuple(-value / acc_norm * 9.81 for value in acc_mean)
        else:
            fast_lio_gravity = (0.0, 0.0, 0.0)

        self.get_logger().info(
            "samples=%d | acc_mean[m/s^2]=(%.4f, %.4f, %.4f) "
            "| acc_norm=%.4f | acc_std=(%.4f, %.4f, %.4f) "
            "| gyro_mean[rad/s]=(%.5f, %.5f, %.5f) "
            "| gyro_std=(%.5f, %.5f, %.5f) "
            "| fast_lio_init_gravity=(%.4f, %.4f, %.4f)"
            % (
                len(self.samples),
                acc_mean[0],
                acc_mean[1],
                acc_mean[2],
                acc_norm,
                acc_std[0],
                acc_std[1],
                acc_std[2],
                gyr_mean[0],
                gyr_mean[1],
                gyr_mean[2],
                gyr_std[0],
                gyr_std[1],
                gyr_std[2],
                fast_lio_gravity[0],
                fast_lio_gravity[1],
                fast_lio_gravity[2],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuStaticCheck()
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
