#!/usr/bin/env python3

import argparse
import bisect
import math
import sqlite3
import struct
from pathlib import Path
from statistics import mean, pstdev
from typing import Dict, Iterable, List, Optional, Tuple

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointField


LIDAR_TOPIC = "/rslidar_points"
RAW_IMU_TOPIC = "/imu/data_raw"
LIO_IMU_TOPIC = "/imu/data_lio"
ODOM_TOPIC = "/aft_mapped_to_init"


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def rate_summary(stamps: List[float]) -> str:
    if len(stamps) < 2:
        return "n/a"
    duration = stamps[-1] - stamps[0]
    if duration <= 0.0:
        return "n/a"
    return f"{(len(stamps) - 1) / duration:.2f} Hz over {duration:.3f}s"


def fmt_stats(values: List[float], unit: str = "") -> str:
    if not values:
        return "n/a"
    suffix = f" {unit}" if unit else ""
    return (
        f"mean={mean(values):.6f}{suffix}, std={pstdev(values):.6f}{suffix}, "
        f"min={min(values):.6f}{suffix}, max={max(values):.6f}{suffix}"
    )


def field_map(cloud) -> Dict[str, object]:
    return {field.name: field for field in cloud.fields}


def unpack_field(data: memoryview, point_offset: int, field, is_bigendian: bool):
    endian = ">" if is_bigendian else "<"
    offset = point_offset + field.offset
    if field.datatype == PointField.FLOAT64:
        return struct.unpack_from(endian + "d", data, offset)[0]
    if field.datatype == PointField.FLOAT32:
        return struct.unpack_from(endian + "f", data, offset)[0]
    if field.datatype == PointField.UINT32:
        return struct.unpack_from(endian + "I", data, offset)[0]
    if field.datatype == PointField.INT32:
        return struct.unpack_from(endian + "i", data, offset)[0]
    if field.datatype == PointField.UINT16:
        return struct.unpack_from(endian + "H", data, offset)[0]
    if field.datatype == PointField.INT16:
        return struct.unpack_from(endian + "h", data, offset)[0]
    if field.datatype == PointField.UINT8:
        return struct.unpack_from(endian + "B", data, offset)[0]
    if field.datatype == PointField.INT8:
        return struct.unpack_from(endian + "b", data, offset)[0]
    raise ValueError(f"Unsupported PointField datatype {field.datatype}")


def lidar_timestamp_stats(cloud) -> Optional[Dict[str, float]]:
    fields = field_map(cloud)
    timestamp_field = fields.get("timestamp")
    if timestamp_field is None:
        return None
    point_count = int(cloud.width) * int(cloud.height)
    if point_count <= 0:
        return None

    data = memoryview(bytes(cloud.data))
    values = []
    for index in range(point_count):
        point_offset = index * cloud.point_step
        values.append(
            float(unpack_field(data, point_offset, timestamp_field, cloud.is_bigendian))
        )
    header = stamp_sec(cloud.header.stamp)
    return {
        "header": header,
        "first": values[0],
        "last": values[-1],
        "min": min(values),
        "max": max(values),
        "span": max(values) - min(values),
        "header_minus_first": header - values[0],
        "point_count": float(point_count),
    }


def vector3_values(vector) -> Tuple[float, float, float]:
    return float(vector.x), float(vector.y), float(vector.z)


def monotonic(stamps: List[float]) -> bool:
    return all(curr > prev for prev, curr in zip(stamps, stamps[1:]))


def iter_bag_messages(bag_dir: Path, wanted_topics: Iterable[str]):
    db_files = sorted(bag_dir.glob("*.db3"))
    if not db_files:
        raise FileNotFoundError(f"No .db3 files found in {bag_dir}")
    wanted = set(wanted_topics)

    for db_file in db_files:
        conn = sqlite3.connect(str(db_file))
        try:
            topics = {
                row[0]: (row[1], row[2])
                for row in conn.execute("SELECT id, name, type FROM topics")
                if row[1] in wanted
            }
            classes = {
                topic_id: get_message(type_name)
                for topic_id, (_, type_name) in topics.items()
            }
            query = (
                "SELECT topic_id, timestamp, data FROM messages "
                "ORDER BY timestamp ASC"
            )
            for topic_id, db_timestamp, data in conn.execute(query):
                if topic_id not in topics:
                    continue
                topic, _ = topics[topic_id]
                yield topic, db_timestamp * 1.0e-9, deserialize_message(
                    bytes(data), classes[topic_id]
                )
        finally:
            conn.close()


class Analyzer:
    def __init__(self) -> None:
        self.lidar_stamps: List[float] = []
        self.lidar_header_minus_first: List[float] = []
        self.lidar_spans: List[float] = []
        self.lidar_end_times: List[float] = []
        self.lidar_point_counts: List[float] = []

        self.imu: Dict[str, Dict[str, List[float]]] = {
            RAW_IMU_TOPIC: self._new_imu_bucket(),
            LIO_IMU_TOPIC: self._new_imu_bucket(),
        }
        self.odom_stamps: List[float] = []
        self.odom_positions: List[Tuple[float, float, float]] = []

    @staticmethod
    def _new_imu_bucket() -> Dict[str, List[float]]:
        return {
            "stamps": [],
            "acc_x": [],
            "acc_y": [],
            "acc_z": [],
            "acc_norm": [],
            "gyro_x": [],
            "gyro_y": [],
            "gyro_z": [],
            "gyro_norm": [],
        }

    def add_lidar(self, cloud) -> None:
        stats = lidar_timestamp_stats(cloud)
        if stats is None:
            return
        self.lidar_stamps.append(stats["header"])
        self.lidar_header_minus_first.append(stats["header_minus_first"])
        self.lidar_spans.append(stats["span"])
        self.lidar_end_times.append(stats["max"])
        self.lidar_point_counts.append(stats["point_count"])

    def add_imu(self, topic: str, message) -> None:
        bucket = self.imu[topic]
        stamp = stamp_sec(message.header.stamp)
        bucket["stamps"].append(stamp)
        ax, ay, az = vector3_values(message.linear_acceleration)
        gx, gy, gz = vector3_values(message.angular_velocity)
        bucket["acc_x"].append(ax)
        bucket["acc_y"].append(ay)
        bucket["acc_z"].append(az)
        bucket["acc_norm"].append(math.sqrt(ax * ax + ay * ay + az * az))
        bucket["gyro_x"].append(gx)
        bucket["gyro_y"].append(gy)
        bucket["gyro_z"].append(gz)
        bucket["gyro_norm"].append(math.sqrt(gx * gx + gy * gy + gz * gz))

    def add_odom(self, message) -> None:
        self.odom_stamps.append(stamp_sec(message.header.stamp))
        pos = message.pose.pose.position
        self.odom_positions.append((float(pos.x), float(pos.y), float(pos.z)))

    def lidar_to_imu_end_deltas(self, imu_topic: str) -> List[float]:
        imu_stamps = self.imu[imu_topic]["stamps"]
        if not self.lidar_end_times or not imu_stamps:
            return []
        deltas = []
        for lidar_end in self.lidar_end_times:
            index = bisect.bisect_left(imu_stamps, lidar_end)
            candidates = []
            if index < len(imu_stamps):
                candidates.append(imu_stamps[index] - lidar_end)
            if index > 0:
                candidates.append(imu_stamps[index - 1] - lidar_end)
            if candidates:
                deltas.append(min(candidates, key=abs))
        return deltas

    def odom_result(self) -> Optional[Dict[str, float]]:
        if len(self.odom_positions) < 2:
            return None
        start = self.odom_positions[0]
        end = self.odom_positions[-1]

        def dist(a, b):
            return math.sqrt(sum((aa - bb) ** 2 for aa, bb in zip(a, b)))

        max_distance = max(dist(start, pos) for pos in self.odom_positions)
        return {
            "duration": self.odom_stamps[-1] - self.odom_stamps[0],
            "return_drift": dist(start, end),
            "max_distance": max_distance,
            "drift_ratio": dist(start, end) / max_distance if max_distance > 0 else 0.0,
            "start_x": start[0],
            "start_y": start[1],
            "start_z": start[2],
            "end_x": end[0],
            "end_y": end[1],
            "end_z": end[2],
        }


def print_imu_summary(topic: str, bucket: Dict[str, List[float]]) -> None:
    print(f"\n[{topic}]")
    print(f"count: {len(bucket['stamps'])}")
    print(f"rate: {rate_summary(bucket['stamps'])}")
    print(f"monotonic stamps: {monotonic(bucket['stamps'])}")
    print(
        "acc mean xyz: "
        f"{mean(bucket['acc_x']) if bucket['acc_x'] else float('nan'):.6f}, "
        f"{mean(bucket['acc_y']) if bucket['acc_y'] else float('nan'):.6f}, "
        f"{mean(bucket['acc_z']) if bucket['acc_z'] else float('nan'):.6f}"
    )
    print(f"acc norm: {fmt_stats(bucket['acc_norm'], 'm/s^2')}")
    print(
        "gyro mean xyz: "
        f"{mean(bucket['gyro_x']) if bucket['gyro_x'] else float('nan'):.6f}, "
        f"{mean(bucket['gyro_y']) if bucket['gyro_y'] else float('nan'):.6f}, "
        f"{mean(bucket['gyro_z']) if bucket['gyro_z'] else float('nan'):.6f}"
    )
    print(f"gyro norm: {fmt_stats(bucket['gyro_norm'], 'rad/s')}")
    if bucket["acc_norm"]:
        accel_ok = abs(mean(bucket["acc_norm"]) - 9.81) <= 0.2
        print(f"acc norm check 9.81 +/- 0.2: {'PASS' if accel_ok else 'WARN'}")
    if bucket["gyro_norm"]:
        gyro_ok = mean(bucket["gyro_norm"]) <= 0.03
        print(f"gyro static norm <= 0.03: {'PASS' if gyro_ok else 'WARN'}")


def print_report(analyzer: Analyzer) -> None:
    print("\n=== LiDAR ===")
    print(f"count: {len(analyzer.lidar_stamps)}")
    print(f"rate: {rate_summary(analyzer.lidar_stamps)}")
    print(f"point count: {fmt_stats(analyzer.lidar_point_counts)}")
    print(f"scan span: {fmt_stats(analyzer.lidar_spans, 's')}")
    print(
        "header - first point timestamp: "
        f"{fmt_stats(analyzer.lidar_header_minus_first, 's')}"
    )
    if analyzer.lidar_header_minus_first:
        header_ok = max(abs(v) for v in analyzer.lidar_header_minus_first) <= 0.002
        print(f"header/first timestamp <= 2 ms: {'PASS' if header_ok else 'WARN'}")

    for topic, bucket in analyzer.imu.items():
        print_imu_summary(topic, bucket)
        deltas = analyzer.lidar_to_imu_end_deltas(topic)
        print(
            f"nearest IMU stamp - LiDAR scan end for {topic}: "
            f"{fmt_stats(deltas, 's')}"
        )

    print("\n=== Odometry ===")
    result = analyzer.odom_result()
    if result is None:
        print("No odometry samples found.")
    else:
        print(f"duration: {result['duration']:.3f}s")
        print(
            "start xyz: "
            f"{result['start_x']:.3f}, {result['start_y']:.3f}, {result['start_z']:.3f}"
        )
        print(
            "end xyz: "
            f"{result['end_x']:.3f}, {result['end_y']:.3f}, {result['end_z']:.3f}"
        )
        print(f"max distance from start: {result['max_distance']:.3f} m")
        print(f"return drift: {result['return_drift']:.3f} m")
        print(f"drift ratio: {result['drift_ratio'] * 100.0:.2f}%")
        drift_ok = result["return_drift"] <= 0.5 or result["drift_ratio"] <= 0.05
        print(f"return drift target: {'PASS' if drift_ok else 'WARN'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to a rosbag2 directory")
    args = parser.parse_args()

    analyzer = Analyzer()
    wanted = [LIDAR_TOPIC, RAW_IMU_TOPIC, LIO_IMU_TOPIC, ODOM_TOPIC]
    for topic, _, message in iter_bag_messages(Path(args.bag).expanduser(), wanted):
        if topic == LIDAR_TOPIC:
            analyzer.add_lidar(message)
        elif topic in (RAW_IMU_TOPIC, LIO_IMU_TOPIC):
            analyzer.add_imu(topic, message)
        elif topic == ODOM_TOPIC:
            analyzer.add_odom(message)
    print_report(analyzer)


if __name__ == "__main__":
    main()
