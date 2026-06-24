#!/usr/bin/env python3

import json
import math
import time
from collections import deque
from pathlib import Path
from statistics import mean, pstdev
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64


G = 9.80665
Vector3 = Tuple[float, float, float]


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _mean_vector(vectors: Sequence[Vector3]) -> Vector3:
    if not vectors:
        return (0.0, 0.0, 0.0)
    return tuple(mean(vector[index] for vector in vectors) for index in range(3))


def _accel_tilt(acceleration: Vector3) -> Vector3:
    ax, ay, az = acceleration
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return (roll, pitch, 0.0)


def _quaternion_rpy(quaternion) -> Vector3:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return (roll, pitch, yaw)


class ImuLioDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_lio_debug")
        self.declare_parameter("raw_imu_topic", "/imu/data_raw")
        self.declare_parameter("lio_imu_topic", "/imu/data_lio")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("output_prefix", "/imu_lio_debug")
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("stationary_window_sec", 1.0)
        self.declare_parameter("stationary_gyro_threshold_radps", 0.06)
        self.declare_parameter("stationary_accel_std_threshold_mps2", 0.20)
        self.declare_parameter("runaway_speed_threshold_mps", 0.20)
        self.declare_parameter("runaway_hold_sec", 1.0)
        self.declare_parameter("message_timeout_sec", 1.0)
        self.declare_parameter("velocity_filter_alpha", 0.25)
        self.declare_parameter("calibrate_on_start", False)
        self.declare_parameter("calibration_duration_sec", 10.0)
        self.declare_parameter(
            "calibration_output_path",
            "/tmp/imu_lio_stationary_calibration.json",
        )

        self.raw_topic = str(self.get_parameter("raw_imu_topic").value)
        self.lio_topic = str(self.get_parameter("lio_imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.status_period_sec = max(
            0.1, float(self.get_parameter("status_period_sec").value)
        )
        self.stationary_window_sec = max(
            0.2, float(self.get_parameter("stationary_window_sec").value)
        )
        self.stationary_gyro_threshold = float(
            self.get_parameter("stationary_gyro_threshold_radps").value
        )
        self.stationary_accel_std_threshold = float(
            self.get_parameter("stationary_accel_std_threshold_mps2").value
        )
        self.runaway_speed_threshold = float(
            self.get_parameter("runaway_speed_threshold_mps").value
        )
        self.runaway_hold_sec = float(
            self.get_parameter("runaway_hold_sec").value
        )
        self.message_timeout_sec = float(
            self.get_parameter("message_timeout_sec").value
        )
        self.velocity_filter_alpha = min(
            1.0, max(0.0, float(self.get_parameter("velocity_filter_alpha").value))
        )
        self.calibrate_on_start = bool(
            self.get_parameter("calibrate_on_start").value
        )
        self.calibration_duration_sec = max(
            1.0, float(self.get_parameter("calibration_duration_sec").value)
        )
        self.calibration_output_path = Path(
            str(self.get_parameter("calibration_output_path").value)
        ).expanduser()

        self.raw_samples: Deque[Tuple[float, Vector3, Vector3]] = deque()
        self.latest_raw: Optional[Tuple[float, float, Vector3, Vector3]] = None
        self.latest_lio: Optional[Tuple[float, float, Vector3, Vector3]] = None
        self.latest_odom_arrival: Optional[float] = None
        self.latest_body_rpy: Vector3 = (0.0, 0.0, 0.0)
        self.filtered_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.body_speed = 0.0
        self.previous_odom: Optional[Tuple[float, Vector3]] = None
        self.stationary = False
        self.stationary_since: Optional[float] = None
        self.runaway = False
        self.start_monotonic = time.monotonic()

        self.counts: Dict[str, int] = {"raw": 0, "lio": 0, "odom": 0}
        self.last_status_monotonic = self.start_monotonic

        self.calibration_collecting = False
        self.calibration_done = False
        self.calibration_start: Optional[float] = None
        self.calibration_samples: List[Tuple[Vector3, Vector3]] = []

        self.raw_acc_pub = self._vector_publisher("raw_acceleration")
        self.raw_gyro_pub = self._vector_publisher("raw_angular_velocity")
        self.raw_tilt_pub = self._vector_publisher("raw_accel_tilt_rpy")
        self.lio_acc_pub = self._vector_publisher("lio_acceleration")
        self.lio_gyro_pub = self._vector_publisher("lio_angular_velocity")
        self.lio_tilt_pub = self._vector_publisher("lio_accel_tilt_rpy")
        self.body_rpy_pub = self._vector_publisher("body_rpy")
        self.body_velocity_pub = self._vector_publisher("body_velocity")
        self.body_speed_pub = self.create_publisher(
            Float64, f"{self.prefix}/body_speed", 10
        )
        self.stationary_pub = self.create_publisher(
            Bool, f"{self.prefix}/stationary", 10
        )
        self.runaway_pub = self.create_publisher(
            Bool, f"{self.prefix}/runaway", 10
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, f"{self.prefix}/status", 10
        )

        self.create_subscription(
            Imu, self.raw_topic, self._on_raw_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, self.lio_topic, self._on_lio_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.create_timer(self.status_period_sec, self._publish_status)

        self.get_logger().info(
            f"Monitoring raw={self.raw_topic}, lio={self.lio_topic}, "
            f"odom={self.odom_topic}; outputs under {self.prefix}"
        )
        if self.calibrate_on_start:
            self.get_logger().warning(
                "Stationary calibration enabled; keep the assembly still until complete"
            )

    def _vector_publisher(self, suffix: str):
        return self.create_publisher(Vector3Stamped, f"{self.prefix}/{suffix}", 10)

    @staticmethod
    def _imu_vectors(message: Imu) -> Tuple[Vector3, Vector3]:
        acceleration = (
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        )
        angular_velocity = (
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        )
        return acceleration, angular_velocity

    @staticmethod
    def _publish_vector(publisher, header, vector: Vector3) -> None:
        message = Vector3Stamped()
        message.header = header
        message.vector.x, message.vector.y, message.vector.z = vector
        publisher.publish(message)

    def _on_raw_imu(self, message: Imu) -> None:
        now = time.monotonic()
        stamp = _stamp_sec(message.header.stamp)
        acceleration, angular_velocity = self._imu_vectors(message)
        self.latest_raw = (now, stamp, acceleration, angular_velocity)
        self.raw_samples.append((now, acceleration, angular_velocity))
        cutoff = now - max(self.stationary_window_sec, 2.0)
        while self.raw_samples and self.raw_samples[0][0] < cutoff:
            self.raw_samples.popleft()
        self.counts["raw"] += 1

        self._publish_vector(self.raw_acc_pub, message.header, acceleration)
        self._publish_vector(self.raw_gyro_pub, message.header, angular_velocity)
        self._publish_vector(
            self.raw_tilt_pub, message.header, _accel_tilt(acceleration)
        )

        if self.calibration_collecting:
            self.calibration_samples.append((acceleration, angular_velocity))

    def _on_lio_imu(self, message: Imu) -> None:
        now = time.monotonic()
        stamp = _stamp_sec(message.header.stamp)
        acceleration, angular_velocity = self._imu_vectors(message)
        self.latest_lio = (now, stamp, acceleration, angular_velocity)
        self.counts["lio"] += 1
        self._publish_vector(self.lio_acc_pub, message.header, acceleration)
        self._publish_vector(self.lio_gyro_pub, message.header, angular_velocity)
        self._publish_vector(
            self.lio_tilt_pub, message.header, _accel_tilt(acceleration)
        )

    def _on_odom(self, message: Odometry) -> None:
        now = time.monotonic()
        stamp = _stamp_sec(message.header.stamp)
        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        self.latest_odom_arrival = now
        self.latest_body_rpy = _quaternion_rpy(message.pose.pose.orientation)
        self.counts["odom"] += 1

        if self.previous_odom is not None:
            previous_stamp, previous_position = self.previous_odom
            dt = stamp - previous_stamp
            if 1.0e-4 < dt < 1.0:
                velocity = tuple(
                    (position[index] - previous_position[index]) / dt
                    for index in range(3)
                )
                alpha = self.velocity_filter_alpha
                self.filtered_velocity = tuple(
                    alpha * velocity[index]
                    + (1.0 - alpha) * self.filtered_velocity[index]
                    for index in range(3)
                )
                self.body_speed = _norm(self.filtered_velocity)
        self.previous_odom = (stamp, position)

        self._publish_vector(
            self.body_rpy_pub, message.header, self.latest_body_rpy
        )
        self._publish_vector(
            self.body_velocity_pub, message.header, self.filtered_velocity
        )
        speed_message = Float64()
        speed_message.data = self.body_speed
        self.body_speed_pub.publish(speed_message)

    def _stationary_stats(self, now: float) -> Optional[Dict[str, object]]:
        samples = [
            sample
            for sample in self.raw_samples
            if sample[0] >= now - self.stationary_window_sec
        ]
        if len(samples) < 20:
            return None
        accelerations = [sample[1] for sample in samples]
        gyros = [sample[2] for sample in samples]
        accel_norms = [_norm(vector) for vector in accelerations]
        gyro_norms = [_norm(vector) for vector in gyros]
        return {
            "sample_count": len(samples),
            "accel_mean": _mean_vector(accelerations),
            "gyro_mean": _mean_vector(gyros),
            "accel_norm_mean": mean(accel_norms),
            "accel_norm_std": pstdev(accel_norms),
            "gyro_norm_mean": mean(gyro_norms),
        }

    def _update_stationary_and_calibration(
        self, now: float, stats: Optional[Dict[str, object]]
    ) -> None:
        self.stationary = bool(
            stats
            and stats["gyro_norm_mean"] <= self.stationary_gyro_threshold
            and stats["accel_norm_std"] <= self.stationary_accel_std_threshold
        )
        if self.stationary:
            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None

        if not self.calibrate_on_start or self.calibration_done:
            return
        if not self.stationary:
            if self.calibration_collecting:
                self.get_logger().warning(
                    "Calibration motion detected; restarting stationary window"
                )
            self.calibration_collecting = False
            self.calibration_start = None
            self.calibration_samples.clear()
            return
        if not self.calibration_collecting:
            self.calibration_collecting = True
            self.calibration_start = now
            self.calibration_samples.clear()
            return
        if (
            self.calibration_start is not None
            and now - self.calibration_start >= self.calibration_duration_sec
        ):
            self._finish_calibration()

    def _finish_calibration(self) -> None:
        if not self.calibration_samples:
            return
        accelerations = [sample[0] for sample in self.calibration_samples]
        gyros = [sample[1] for sample in self.calibration_samples]
        accel_mean = _mean_vector(accelerations)
        gyro_mean = _mean_vector(gyros)
        accel_norms = [_norm(value) for value in accelerations]
        accel_norm_mean = mean(accel_norms)
        uniform_scale = G / accel_norm_mean if accel_norm_mean > 1.0e-6 else 1.0
        gravity_init = tuple(
            -value / _norm(accel_mean) * G for value in accel_mean
        )
        result = {
            "sample_count": len(self.calibration_samples),
            "duration_sec": self.calibration_duration_sec,
            "linear_acceleration_mean_mps2": accel_mean,
            "linear_acceleration_norm_mean_mps2": accel_norm_mean,
            "suggested_uniform_acceleration_scale": uniform_scale,
            "suggested_linear_acceleration_scale": [uniform_scale] * 3,
            "suggested_angular_velocity_bias_radps": gyro_mean,
            "suggested_point_lio_gravity_init": gravity_init,
            "warning": (
                "One stationary pose estimates gyro bias and uniform accel gain only; "
                "use imu_collect_static_sample plus imu_six_face_calibrate for per-axis calibration."
            ),
        }
        self.calibration_output_path.parent.mkdir(parents=True, exist_ok=True)
        self.calibration_output_path.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.calibration_done = True
        self.calibration_collecting = False
        self.get_logger().info(
            "Calibration complete: gyro_bias=(%.6f, %.6f, %.6f), "
            "uniform_accel_scale=%.8f, output=%s"
            % (*gyro_mean, uniform_scale, self.calibration_output_path)
        )

    @staticmethod
    def _age(now: float, sample) -> float:
        return float("inf") if sample is None else now - sample[0]

    def _publish_status(self) -> None:
        now = time.monotonic()
        stats = self._stationary_stats(now)
        self._update_stationary_and_calibration(now, stats)

        stationary_duration = (
            0.0 if self.stationary_since is None else now - self.stationary_since
        )
        odom_fresh = (
            self.latest_odom_arrival is not None
            and now - self.latest_odom_arrival <= self.message_timeout_sec
        )
        self.runaway = bool(
            self.stationary
            and stationary_duration >= self.runaway_hold_sec
            and odom_fresh
            and self.body_speed >= self.runaway_speed_threshold
        )

        stationary_message = Bool()
        stationary_message.data = self.stationary
        self.stationary_pub.publish(stationary_message)
        runaway_message = Bool()
        runaway_message.data = self.runaway
        self.runaway_pub.publish(runaway_message)

        elapsed = max(1.0e-6, now - self.last_status_monotonic)
        rates = {name: count / elapsed for name, count in self.counts.items()}
        self.counts = {"raw": 0, "lio": 0, "odom": 0}
        self.last_status_monotonic = now

        raw_age = self._age(now, self.latest_raw)
        lio_age = self._age(now, self.latest_lio)
        odom_age = (
            float("inf")
            if self.latest_odom_arrival is None
            else now - self.latest_odom_arrival
        )
        stamp_delta_ms = (
            float("nan")
            if self.latest_raw is None or self.latest_lio is None
            else (self.latest_lio[1] - self.latest_raw[1]) * 1.0e3
        )
        raw_acc = (0.0, 0.0, 0.0) if self.latest_raw is None else self.latest_raw[2]
        raw_gyro = (0.0, 0.0, 0.0) if self.latest_raw is None else self.latest_raw[3]
        lio_acc = (0.0, 0.0, 0.0) if self.latest_lio is None else self.latest_lio[2]
        lio_gyro = (0.0, 0.0, 0.0) if self.latest_lio is None else self.latest_lio[3]

        status = DiagnosticStatus()
        status.name = "imu_lio_debug/state_consistency"
        status.hardware_id = "point_lio"
        missing = []
        if raw_age > self.message_timeout_sec:
            missing.append(self.raw_topic)
        if lio_age > self.message_timeout_sec:
            missing.append(self.lio_topic)
        if odom_age > self.message_timeout_sec:
            missing.append(self.odom_topic)
        if self.runaway:
            status.level = DiagnosticStatus.ERROR
            status.message = "RUNAWAY: body moves while IMU is stationary"
        elif missing and now - self.start_monotonic > self.message_timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "Missing or stale topics: " + ", ".join(missing)
        else:
            status.level = DiagnosticStatus.OK
            status.message = "IMU and Point-LIO state are consistent"

        status.values = [
            KeyValue(key="raw_rate_hz", value=f"{rates['raw']:.2f}"),
            KeyValue(key="lio_rate_hz", value=f"{rates['lio']:.2f}"),
            KeyValue(key="odom_rate_hz", value=f"{rates['odom']:.2f}"),
            KeyValue(key="raw_age_sec", value=f"{raw_age:.3f}"),
            KeyValue(key="lio_age_sec", value=f"{lio_age:.3f}"),
            KeyValue(key="odom_age_sec", value=f"{odom_age:.3f}"),
            KeyValue(key="raw_lio_stamp_delta_ms", value=f"{stamp_delta_ms:.3f}"),
            KeyValue(key="raw_accel_norm_mps2", value=f"{_norm(raw_acc):.4f}"),
            KeyValue(key="lio_accel_norm_mps2", value=f"{_norm(lio_acc):.4f}"),
            KeyValue(key="raw_gyro_norm_radps", value=f"{_norm(raw_gyro):.5f}"),
            KeyValue(key="lio_gyro_norm_radps", value=f"{_norm(lio_gyro):.5f}"),
            KeyValue(key="stationary", value=str(self.stationary)),
            KeyValue(key="stationary_duration_sec", value=f"{stationary_duration:.2f}"),
            KeyValue(key="body_speed_mps", value=f"{self.body_speed:.4f}"),
            KeyValue(key="runaway", value=str(self.runaway)),
            KeyValue(
                key="body_rpy_deg",
                value=",".join(f"{math.degrees(value):.2f}" for value in self.latest_body_rpy),
            ),
            KeyValue(
                key="raw_accel_tilt_rp_deg",
                value=",".join(
                    f"{math.degrees(value):.2f}" for value in _accel_tilt(raw_acc)[:2]
                ),
            ),
        ]
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        diagnostic.status = [status]
        self.diagnostic_pub.publish(diagnostic)

        label = " RUNAWAY" if self.runaway else ""
        self.get_logger().info(
            "raw acc_norm=%.3f gyro_norm=%.4f tilt_rp=(%.1f, %.1f) deg | "
            "lio acc_norm=%.3f gyro_norm=%.4f | body_rpy=(%.1f, %.1f, %.1f) "
            "deg speed=%.3f m/s | stationary=%s%s"
            % (
                _norm(raw_acc),
                _norm(raw_gyro),
                math.degrees(_accel_tilt(raw_acc)[0]),
                math.degrees(_accel_tilt(raw_acc)[1]),
                _norm(lio_acc),
                _norm(lio_gyro),
                math.degrees(self.latest_body_rpy[0]),
                math.degrees(self.latest_body_rpy[1]),
                math.degrees(self.latest_body_rpy[2]),
                self.body_speed,
                self.stationary,
                label,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuLioDebugNode()
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
