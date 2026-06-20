#!/usr/bin/env python3

import argparse
import json
import math
import time
from pathlib import Path
from typing import List, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _stddev(values: Sequence[float], mean: float) -> float:
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


class StaticSampleCollector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("imu_collect_static_sample")
        self.samples: List[List[float]] = []
        self.create_subscription(Imu, topic, self._on_imu, qos_profile_sensor_data)

    def _on_imu(self, message: Imu) -> None:
        self.samples.append(
            [
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ]
        )


def summarize(samples: Sequence[Sequence[float]]) -> dict:
    columns = list(zip(*samples))
    acc_mean = [_mean(columns[index]) for index in range(3)]
    gyr_mean = [_mean(columns[index]) for index in range(3, 6)]
    acc_std = [_stddev(columns[index], acc_mean[index]) for index in range(3)]
    gyr_std = [_stddev(columns[index + 3], gyr_mean[index]) for index in range(3)]
    acc_norm = math.sqrt(sum(value * value for value in acc_mean))

    return {
        "sample_count": len(samples),
        "linear_acceleration_mean_mps2": acc_mean,
        "linear_acceleration_std_mps2": acc_std,
        "linear_acceleration_norm_mps2": acc_norm,
        "angular_velocity_mean_radps": gyr_mean,
        "angular_velocity_std_radps": gyr_std,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--topic", default="/imu/data_raw")
    parser.add_argument(
        "--output",
        default="/tmp/phidget_imu_static_samples.jsonl",
        help="Append one JSON record per static pose.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = StaticSampleCollector(args.topic)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not node.samples:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("No IMU samples received")

    record = summarize(node.samples)
    record["label"] = args.label
    record["duration_sec"] = args.duration
    record["topic"] = args.topic

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("a", encoding="utf-8") as output_file:
        output_file.write(json.dumps(record, sort_keys=True) + "\n")

    print(json.dumps(record, indent=2, sort_keys=True))
    print(f"appended_to: {output_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
