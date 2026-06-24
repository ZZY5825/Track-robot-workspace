#!/usr/bin/env python3

from typing import Iterable, List

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu


def _stamp_to_seconds(stamp: TimeMsg) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _seconds_to_stamp(seconds: float) -> TimeMsg:
    stamp_ns = int(round(seconds * 1.0e9))
    msg = TimeMsg()
    msg.sec = stamp_ns // 1000000000
    msg.nanosec = stamp_ns % 1000000000
    return msg


def _mat_vec_mul(matrix: List[float], vector: Iterable[float]) -> List[float]:
    x, y, z = vector
    return [
        matrix[0] * x + matrix[1] * y + matrix[2] * z,
        matrix[3] * x + matrix[4] * y + matrix[5] * z,
        matrix[6] * x + matrix[7] * y + matrix[8] * z,
    ]


def _rotate_covariance(matrix: List[float], covariance: Iterable[float]) -> List[float]:
    cov = list(covariance)
    if len(cov) != 9 or cov[0] < 0.0 or all(abs(value) < 1.0e-18 for value in cov):
        return cov

    tmp = [0.0] * 9
    out = [0.0] * 9
    for row in range(3):
        for col in range(3):
            tmp[row * 3 + col] = sum(
                matrix[row * 3 + k] * cov[k * 3 + col]
                for k in range(3)
            )
    for row in range(3):
        for col in range(3):
            out[row * 3 + col] = sum(
                tmp[row * 3 + k] * matrix[col * 3 + k]
                for k in range(3)
            )
    return out


def _validate_rotation_matrix(matrix: List[float]) -> None:
    if len(matrix) != 9:
        raise ValueError("rotation_matrix must contain 9 row-major values")

    rows = [matrix[0:3], matrix[3:6], matrix[6:9]]
    for row in rows:
        if abs(sum(value * value for value in row) - 1.0) > 1.0e-3:
            raise ValueError("rotation_matrix rows must be unit length")
    for first, second in ((0, 1), (0, 2), (1, 2)):
        if abs(sum(rows[first][i] * rows[second][i] for i in range(3))) > 1.0e-3:
            raise ValueError("rotation_matrix rows must be orthogonal")

    determinant = (
        matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7])
        - matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6])
        + matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6])
    )
    if abs(determinant - 1.0) > 1.0e-3:
        raise ValueError("rotation_matrix must be a proper rotation with determinant +1")


class ImuLioAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_lio_adapter")
        self.declare_parameter("input_topic", "/imu/data_raw")
        self.declare_parameter("output_topic", "/imu/data_lio")
        self.declare_parameter("output_frame_id", "rslidar")
        self.declare_parameter("time_offset_sec", 0.0)
        self.declare_parameter(
            "rotation_matrix",
            [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0],
        )

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.time_offset_sec = float(self.get_parameter("time_offset_sec").value)
        self.rotation_matrix = [
            float(value) for value in self.get_parameter("rotation_matrix").value
        ]
        _validate_rotation_matrix(self.rotation_matrix)

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            Imu, self.output_topic, output_qos
        )
        self.subscription = self.create_subscription(
            Imu, self.input_topic, self._callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            "IMU LIO adapter: {} -> {}, frame={}, time_offset_sec={:.6f}".format(
                self.input_topic,
                self.output_topic,
                self.output_frame_id,
                self.time_offset_sec,
            )
        )
        if self.rotation_matrix == [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]:
            self.get_logger().warning(
                "IMU-to-LiDAR rotation is identity; verify the physical axis alignment"
            )

    def _callback(self, message: Imu) -> None:
        out = Imu()
        out.header = message.header
        out.header.frame_id = self.output_frame_id
        out.header.stamp = _seconds_to_stamp(
            _stamp_to_seconds(message.header.stamp) - self.time_offset_sec
        )

        out.orientation = message.orientation
        out.orientation_covariance = message.orientation_covariance

        acc = _mat_vec_mul(
            self.rotation_matrix,
            [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ],
        )
        out.linear_acceleration.x = acc[0]
        out.linear_acceleration.y = acc[1]
        out.linear_acceleration.z = acc[2]
        out.linear_acceleration_covariance = _rotate_covariance(
            self.rotation_matrix, message.linear_acceleration_covariance
        )

        gyro = _mat_vec_mul(
            self.rotation_matrix,
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
        )
        out.angular_velocity.x = gyro[0]
        out.angular_velocity.y = gyro[1]
        out.angular_velocity.z = gyro[2]
        out.angular_velocity_covariance = _rotate_covariance(
            self.rotation_matrix, message.angular_velocity_covariance
        )
        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuLioAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
