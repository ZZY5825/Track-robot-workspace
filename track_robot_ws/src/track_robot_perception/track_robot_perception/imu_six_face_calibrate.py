#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import Dict, List


G = 9.80665
REQUIRED_LABELS = ("x_plus", "x_minus", "y_plus", "y_minus", "z_plus", "z_minus")


def load_records(path: Path) -> Dict[str, dict]:
    records: Dict[str, dict] = {}
    with path.open("r", encoding="utf-8") as input_file:
        for line in input_file:
            if not line.strip():
                continue
            record = json.loads(line)
            records[record["label"]] = record
    return records


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        default="/tmp/phidget_imu_static_samples.jsonl",
        help="JSONL file written by imu_collect_static_sample.",
    )
    args = parser.parse_args()

    records = load_records(Path(args.input))
    missing = [label for label in REQUIRED_LABELS if label not in records]
    if missing:
        raise SystemExit(
            "Missing labels: %s\nExpected labels: %s"
            % (", ".join(missing), ", ".join(REQUIRED_LABELS))
        )

    axis_indices = {"x": 0, "y": 1, "z": 2}
    accel_bias: List[float] = [0.0, 0.0, 0.0]
    accel_scale: List[float] = [1.0, 1.0, 1.0]
    for axis, index in axis_indices.items():
        plus = records[f"{axis}_plus"]["linear_acceleration_mean_mps2"][index]
        minus = records[f"{axis}_minus"]["linear_acceleration_mean_mps2"][index]
        accel_bias[index] = 0.5 * (plus + minus)
        span = plus - minus
        if abs(span) < 1.0e-9:
            raise SystemExit(f"Cannot calibrate {axis}: plus/minus span is zero")
        accel_scale[index] = (2.0 * G) / span

    gyro_means = [
        records[label]["angular_velocity_mean_radps"] for label in REQUIRED_LABELS
    ]
    gyro_bias = [
        sum(mean[index] for mean in gyro_means) / len(gyro_means)
        for index in range(3)
    ]

    print("Suggested phidget_imu.yaml values:")
    print(
        "linear_acceleration_bias_mps2: [%.9f, %.9f, %.9f]"
        % tuple(accel_bias)
    )
    print(
        "linear_acceleration_scale: [%.9f, %.9f, %.9f]"
        % tuple(accel_scale)
    )
    print("angular_velocity_bias_radps: [%.9f, %.9f, %.9f]" % tuple(gyro_bias))
    print("angular_velocity_scale: [1.0, 1.0, 1.0]")


if __name__ == "__main__":
    main()
