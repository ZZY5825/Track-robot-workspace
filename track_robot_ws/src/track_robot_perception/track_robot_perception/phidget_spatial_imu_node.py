#!/usr/bin/env python3

import math
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Sequence, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, MagneticField

try:
    from Phidget22.Devices.Spatial import Spatial
    from Phidget22.PhidgetException import PhidgetException
except ImportError as exc:
    raise RuntimeError(
        "Phidget22 is not installed. Run: python3 -m pip install --user Phidget22"
    ) from exc


G_TO_METRES_PER_SECOND_SQUARED = 9.80665
DEGREES_TO_RADIANS = math.pi / 180.0
GAUSS_TO_TESLA = 1.0e-4


@dataclass(frozen=True)
class RawSpatialSample:
    device_time_s: float
    receipt_realtime_s: float
    receipt_monotonic_s: float
    acceleration_g: Tuple[float, float, float]
    angular_rate_deg_s: Tuple[float, float, float]
    magnetic_field_gauss: Tuple[float, float, float]


class AffineClockMapper:
    """Maps the free-running Phidget clock into Jetson CLOCK_REALTIME."""

    def __init__(
        self,
        window_sec: float,
        fit_period_sec: float,
        lower_quantile: float,
        max_skew_ppm: float,
        clock_step_reset_sec: float,
    ) -> None:
        self.window_sec = window_sec
        self.fit_period_sec = fit_period_sec
        self.lower_quantile = lower_quantile
        self.max_skew = max_skew_ppm * 1.0e-6
        self.clock_step_reset_sec = clock_step_reset_sec
        self.anchors: Deque[Tuple[float, float]] = deque()
        self.scale = 1.0
        self.offset = 0.0
        self.residual_std_sec = 0.0
        self.last_fit_device_time: Optional[float] = None
        self.last_device_time: Optional[float] = None
        self.last_host_time: Optional[float] = None
        self.reset_count = 0

    def reset(self) -> None:
        self.anchors.clear()
        self.scale = 1.0
        self.offset = 0.0
        self.residual_std_sec = 0.0
        self.last_fit_device_time = None
        self.last_device_time = None
        self.last_host_time = None
        self.reset_count += 1

    def observe(self, device_time_s: float, host_time_s: float) -> None:
        if self.last_device_time is not None and self.last_host_time is not None:
            device_delta = device_time_s - self.last_device_time
            host_delta = host_time_s - self.last_host_time
            clock_delta_error = host_delta - device_delta
            if (
                device_delta <= 0.0
                or host_delta <= 0.0
                or abs(clock_delta_error) > self.clock_step_reset_sec
            ):
                self.reset()

        self.last_device_time = device_time_s
        self.last_host_time = host_time_s
        self.anchors.append((device_time_s, host_time_s))

        cutoff = device_time_s - self.window_sec
        while self.anchors and self.anchors[0][0] < cutoff:
            self.anchors.popleft()

        if len(self.anchors) < 8:
            self.scale = 1.0
            self.offset = min(host - device for device, host in self.anchors)
            return

        if (
            self.last_fit_device_time is not None
            and device_time_s - self.last_fit_device_time < self.fit_period_sec
        ):
            return

        self._fit()
        self.last_fit_device_time = device_time_s

    def to_host_time(self, device_time_s: float) -> float:
        return self.scale * device_time_s + self.offset

    @property
    def ready(self) -> bool:
        if len(self.anchors) < 8:
            return False
        return self.anchors[-1][0] - self.anchors[0][0] >= 1.0

    @staticmethod
    def _least_squares(
        points: Sequence[Tuple[float, float]]
    ) -> Tuple[float, float]:
        mean_x = sum(point[0] for point in points) / len(points)
        mean_y = sum(point[1] for point in points) / len(points)
        variance_x = sum((point[0] - mean_x) ** 2 for point in points)
        if variance_x <= 1.0e-15:
            return 1.0, mean_y - mean_x
        covariance = sum(
            (point[0] - mean_x) * (point[1] - mean_y) for point in points
        )
        scale = covariance / variance_x
        return scale, mean_y - scale * mean_x

    def _fit(self) -> None:
        points = list(self.anchors)
        preliminary_scale, preliminary_offset = self._least_squares(points)
        residual_points = sorted(
            points,
            key=lambda point: (
                point[1] - (preliminary_scale * point[0] + preliminary_offset)
            ),
        )
        selected_count = max(
            4, min(len(points), int(math.ceil(len(points) * self.lower_quantile)))
        )
        selected = residual_points[:selected_count]
        scale, offset = self._least_squares(selected)

        if not (1.0 - self.max_skew <= scale <= 1.0 + self.max_skew):
            scale = 1.0
            offset = min(host - device for device, host in points)

        residuals = [host - (scale * device + offset) for device, host in selected]
        self.scale = scale
        self.offset = offset
        self.residual_std_sec = (
            statistics.pstdev(residuals) if len(residuals) > 1 else 0.0
        )


class PhidgetSpatialImuNode(Node):
    def __init__(self) -> None:
        super().__init__("phidget_spatial_imu")

        self.declare_parameter("serial_number", 166154)
        self.declare_parameter("frame_id", "phidget_imu_link")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("magnetic_field_topic", "/imu/mag")
        self.declare_parameter("diagnostics_topic", "/imu/time_sync_status")
        self.declare_parameter("data_interval_ms", 4)
        self.declare_parameter("publish_magnetic_field", True)
        self.declare_parameter("zero_gyro_on_start", True)
        self.declare_parameter("gyro_zero_wait_sec", 2.0)
        self.declare_parameter("timestamp_mode", "device_affine")
        self.declare_parameter("calibrated_time_offset_sec", 0.0)
        self.declare_parameter("sync_window_sec", 60.0)
        self.declare_parameter("sync_fit_period_sec", 1.0)
        self.declare_parameter("sync_lower_quantile", 0.2)
        self.declare_parameter("max_clock_skew_ppm", 2000.0)
        self.declare_parameter("clock_step_reset_sec", 0.5)
        self.declare_parameter(
            "acceleration_stddev",
            [0.002942, 0.002942, 0.004903],
        )
        self.declare_parameter("angular_velocity_stddev", [0.0, 0.0, 0.0])
        self.declare_parameter("magnetic_field_stddev", [0.0, 0.0, 0.0])

        self.serial_number = int(self.get_parameter("serial_number").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.data_interval_ms = int(self.get_parameter("data_interval_ms").value)
        self.publish_magnetic_field = bool(
            self.get_parameter("publish_magnetic_field").value
        )
        self.zero_gyro_on_start = bool(
            self.get_parameter("zero_gyro_on_start").value
        )
        self.gyro_zero_wait_sec = max(
            0.0, float(self.get_parameter("gyro_zero_wait_sec").value)
        )
        self.timestamp_mode = str(self.get_parameter("timestamp_mode").value)
        self.calibrated_time_offset_sec = float(
            self.get_parameter("calibrated_time_offset_sec").value
        )

        if self.timestamp_mode not in ("device_affine", "arrival"):
            raise ValueError("timestamp_mode must be 'device_affine' or 'arrival'")

        self.acceleration_covariance = self._diagonal_covariance(
            self.get_parameter("acceleration_stddev").value
        )
        self.angular_velocity_covariance = self._diagonal_covariance(
            self.get_parameter("angular_velocity_stddev").value
        )
        self.magnetic_field_covariance = self._diagonal_covariance(
            self.get_parameter("magnetic_field_stddev").value
        )

        self.clock_mapper = AffineClockMapper(
            window_sec=float(self.get_parameter("sync_window_sec").value),
            fit_period_sec=float(
                self.get_parameter("sync_fit_period_sec").value
            ),
            lower_quantile=float(
                self.get_parameter("sync_lower_quantile").value
            ),
            max_skew_ppm=float(
                self.get_parameter("max_clock_skew_ppm").value
            ),
            clock_step_reset_sec=float(
                self.get_parameter("clock_step_reset_sec").value
            ),
        )
        self.clock_lock = threading.Lock()

        self.imu_publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("imu_topic").value),
            qos_profile_sensor_data,
        )
        self.magnetic_field_publisher = self.create_publisher(
            MagneticField,
            str(self.get_parameter("magnetic_field_topic").value),
            qos_profile_sensor_data,
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )

        self.batch_lock = threading.Lock()
        self.current_batch: List[RawSpatialSample] = []
        self.completed_batches: Deque[List[RawSpatialSample]] = deque()
        self.batch_gap_sec = 0.001
        self.batch_flush_sec = 0.002
        self.last_published_stamp_ns = 0
        self.last_sample_monotonic_s: Optional[float] = None
        self.last_error = ""
        self.attached = False
        self.samples_since_diagnostic = 0
        self.last_diagnostic_monotonic_s = time.monotonic()
        self.measured_rate_hz = 0.0
        self.latest_transport_delay_sec = 0.0
        self.publish_not_before_monotonic_s = 0.0

        self.spatial = Spatial()
        self.spatial.setDeviceSerialNumber(self.serial_number)
        self.spatial.setOnSpatialDataHandler(self._on_spatial_data)
        self.spatial.setOnAttachHandler(self._on_attach)
        self.spatial.setOnDetachHandler(self._on_detach)
        self.spatial.setOnErrorHandler(self._on_error)

        try:
            self.spatial.open()
        except Exception:
            self.spatial.close()
            raise

        self.flush_timer = self.create_timer(0.001, self._flush_batches)
        self.diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f"Waiting for PhidgetSpatial serial {self.serial_number}"
        )

    @staticmethod
    def _diagonal_covariance(stddev_values: Sequence[float]) -> List[float]:
        if len(stddev_values) != 3:
            raise ValueError("standard deviation parameters must have 3 values")
        covariance = [0.0] * 9
        for index, value in enumerate(stddev_values):
            value = float(value)
            if value > 0.0:
                covariance[index * 3 + index] = value * value
        return covariance

    def _on_attach(self, _device: Spatial) -> None:
        try:
            minimum_interval = int(self.spatial.getMinDataInterval())
            maximum_interval = int(self.spatial.getMaxDataInterval())
            if not minimum_interval <= self.data_interval_ms <= maximum_interval:
                raise ValueError(
                    f"data_interval_ms must be between {minimum_interval} and "
                    f"{maximum_interval}"
                )
            self.spatial.setDataInterval(self.data_interval_ms)
            self.publish_not_before_monotonic_s = 0.0

            if self.zero_gyro_on_start:
                self.get_logger().info(
                    "Zeroing gyroscope; keep the robot completely stationary"
                )
                self.spatial.zeroGyro()
                self.publish_not_before_monotonic_s = (
                    time.monotonic() + self.gyro_zero_wait_sec
                )

            with self.clock_lock:
                self.clock_mapper.reset()
            with self.batch_lock:
                self.current_batch.clear()
                self.completed_batches.clear()
            self.last_error = ""
            self.attached = True
            self.get_logger().info(
                "Attached %s SKU=%s firmware=%s serial=%d at %.1f Hz"
                % (
                    self.spatial.getDeviceName(),
                    self.spatial.getDeviceSKU(),
                    self.spatial.getDeviceVersion(),
                    self.serial_number,
                    1000.0 / self.data_interval_ms,
                )
            )
        except (PhidgetException, ValueError) as exc:
            self.attached = False
            self.last_error = str(exc)
            self.get_logger().error(f"Failed to configure attached IMU: {exc}")

    def _on_detach(self, _device: Spatial) -> None:
        self.attached = False
        with self.clock_lock:
            self.clock_mapper.reset()
        with self.batch_lock:
            self.current_batch.clear()
            self.completed_batches.clear()

    def _on_error(self, _device: Spatial, code: int, description: str) -> None:
        self.last_error = f"{code}: {description}"

    def _on_spatial_data(
        self,
        _device: Spatial,
        acceleration: Sequence[float],
        angular_rate: Sequence[float],
        magnetic_field: Sequence[float],
        timestamp_ms: float,
    ) -> None:
        receipt_monotonic_s = time.monotonic_ns() * 1.0e-9
        if receipt_monotonic_s < self.publish_not_before_monotonic_s:
            return
        receipt_realtime_s = time.time_ns() * 1.0e-9
        sample = RawSpatialSample(
            device_time_s=float(timestamp_ms) * 1.0e-3,
            receipt_realtime_s=receipt_realtime_s,
            receipt_monotonic_s=receipt_monotonic_s,
            acceleration_g=tuple(float(value) for value in acceleration),
            angular_rate_deg_s=tuple(float(value) for value in angular_rate),
            magnetic_field_gauss=tuple(float(value) for value in magnetic_field),
        )

        with self.batch_lock:
            if (
                self.current_batch
                and receipt_monotonic_s
                - self.current_batch[-1].receipt_monotonic_s
                > self.batch_gap_sec
            ):
                self.completed_batches.append(self.current_batch)
                self.current_batch = []
            self.current_batch.append(sample)

    def _flush_batches(self) -> None:
        now_monotonic_s = time.monotonic_ns() * 1.0e-9
        with self.batch_lock:
            if (
                self.current_batch
                and now_monotonic_s
                - self.current_batch[-1].receipt_monotonic_s
                >= self.batch_flush_sec
            ):
                self.completed_batches.append(self.current_batch)
                self.current_batch = []
            batches = list(self.completed_batches)
            self.completed_batches.clear()

        for batch in batches:
            self._publish_batch(batch)

    def _publish_batch(self, batch: Sequence[RawSpatialSample]) -> None:
        if not batch:
            return

        newest = batch[-1]
        packet_receipt_time_s = min(
            sample.receipt_realtime_s for sample in batch
        )
        with self.clock_lock:
            self.clock_mapper.observe(
                newest.device_time_s, packet_receipt_time_s
            )
            mapped_newest_time_s = self.clock_mapper.to_host_time(
                newest.device_time_s
            )
            mapped_times = [
                self.clock_mapper.to_host_time(sample.device_time_s)
                for sample in batch
            ]
        self.latest_transport_delay_sec = max(
            0.0, packet_receipt_time_s - mapped_newest_time_s
        )

        for sample, mapped_time_s in zip(batch, mapped_times):
            if self.timestamp_mode == "arrival":
                stamp_s = sample.receipt_realtime_s
            else:
                stamp_s = mapped_time_s
            stamp_s += self.calibrated_time_offset_sec
            self._publish_sample(sample, stamp_s)

    def _publish_sample(
        self, sample: RawSpatialSample, stamp_s: float
    ) -> None:
        stamp_ns = int(round(stamp_s * 1.0e9))
        if stamp_ns <= self.last_published_stamp_ns:
            stamp_ns = self.last_published_stamp_ns + 1
        self.last_published_stamp_ns = stamp_ns
        stamp = Time(nanoseconds=stamp_ns).to_msg()

        imu_message = Imu()
        imu_message.header.stamp = stamp
        imu_message.header.frame_id = self.frame_id
        imu_message.orientation_covariance[0] = -1.0
        imu_message.linear_acceleration.x = (
            sample.acceleration_g[0] * G_TO_METRES_PER_SECOND_SQUARED
        )
        imu_message.linear_acceleration.y = (
            sample.acceleration_g[1] * G_TO_METRES_PER_SECOND_SQUARED
        )
        imu_message.linear_acceleration.z = (
            sample.acceleration_g[2] * G_TO_METRES_PER_SECOND_SQUARED
        )
        imu_message.angular_velocity.x = (
            sample.angular_rate_deg_s[0] * DEGREES_TO_RADIANS
        )
        imu_message.angular_velocity.y = (
            sample.angular_rate_deg_s[1] * DEGREES_TO_RADIANS
        )
        imu_message.angular_velocity.z = (
            sample.angular_rate_deg_s[2] * DEGREES_TO_RADIANS
        )
        imu_message.linear_acceleration_covariance = (
            self.acceleration_covariance
        )
        imu_message.angular_velocity_covariance = (
            self.angular_velocity_covariance
        )
        self.imu_publisher.publish(imu_message)

        if self.publish_magnetic_field and self._valid_magnetic_field(
            sample.magnetic_field_gauss
        ):
            magnetic_message = MagneticField()
            magnetic_message.header.stamp = stamp
            magnetic_message.header.frame_id = self.frame_id
            magnetic_message.magnetic_field.x = (
                sample.magnetic_field_gauss[0] * GAUSS_TO_TESLA
            )
            magnetic_message.magnetic_field.y = (
                sample.magnetic_field_gauss[1] * GAUSS_TO_TESLA
            )
            magnetic_message.magnetic_field.z = (
                sample.magnetic_field_gauss[2] * GAUSS_TO_TESLA
            )
            magnetic_message.magnetic_field_covariance = (
                self.magnetic_field_covariance
            )
            self.magnetic_field_publisher.publish(magnetic_message)

        self.samples_since_diagnostic += 1
        self.last_sample_monotonic_s = sample.receipt_monotonic_s

    @staticmethod
    def _valid_magnetic_field(values: Sequence[float]) -> bool:
        return all(math.isfinite(value) and abs(value) < 1.0e6 for value in values)

    def _publish_diagnostics(self) -> None:
        now_monotonic_s = time.monotonic()
        elapsed = now_monotonic_s - self.last_diagnostic_monotonic_s
        if elapsed > 0.0:
            self.measured_rate_hz = self.samples_since_diagnostic / elapsed
        self.samples_since_diagnostic = 0
        self.last_diagnostic_monotonic_s = now_monotonic_s

        status = DiagnosticStatus()
        status.name = "phidget_spatial_imu/time_sync"
        status.hardware_id = f"PhidgetSpatial-1056-{self.serial_number}"
        with self.clock_lock:
            clock_ready = self.clock_mapper.ready
            clock_scale = self.clock_mapper.scale
            clock_residual_std_sec = self.clock_mapper.residual_std_sec
            clock_anchor_count = len(self.clock_mapper.anchors)
            clock_reset_count = self.clock_mapper.reset_count

        sample_age = (
            float("inf")
            if self.last_sample_monotonic_s is None
            else now_monotonic_s - self.last_sample_monotonic_s
        )
        if not self.attached:
            status.level = DiagnosticStatus.ERROR
            status.message = "IMU detached"
        elif sample_age > 0.1:
            status.level = DiagnosticStatus.ERROR
            status.message = "No recent IMU samples"
        elif self.timestamp_mode == "device_affine" and not clock_ready:
            status.level = DiagnosticStatus.WARN
            status.message = "Clock mapping is converging"
        elif self.last_error:
            status.level = DiagnosticStatus.WARN
            status.message = self.last_error
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Publishing synchronized raw IMU data"

        status.values = [
            KeyValue(key="timestamp_mode", value=self.timestamp_mode),
            KeyValue(key="frame_id", value=self.frame_id),
            KeyValue(key="sample_rate_hz", value=f"{self.measured_rate_hz:.3f}"),
            KeyValue(
                key="clock_scale",
                value=f"{clock_scale:.12f}",
            ),
            KeyValue(
                key="clock_skew_ppm",
                value=f"{(clock_scale - 1.0) * 1.0e6:.3f}",
            ),
            KeyValue(
                key="clock_fit_residual_std_ms",
                value=f"{clock_residual_std_sec * 1.0e3:.3f}",
            ),
            KeyValue(
                key="transport_delay_lower_bound_ms",
                value=f"{self.latest_transport_delay_sec * 1.0e3:.3f}",
            ),
            KeyValue(
                key="clock_anchor_count",
                value=str(clock_anchor_count),
            ),
            KeyValue(
                key="clock_reset_count",
                value=str(clock_reset_count),
            ),
            KeyValue(
                key="calibrated_time_offset_sec",
                value=f"{self.calibrated_time_offset_sec:.9f}",
            ),
        ]

        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        diagnostic_array.status = [status]
        self.diagnostics_publisher.publish(diagnostic_array)

    def destroy_node(self) -> bool:
        try:
            self.spatial.setOnSpatialDataHandler(None)
            self.spatial.close()
        except (AttributeError, PhidgetException):
            pass
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[PhidgetSpatialImuNode] = None
    try:
        node = PhidgetSpatialImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (PhidgetException, RuntimeError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"Failed to start PhidgetSpatial IMU node: {exc}")
        raise
    finally:
        if node is not None:
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
