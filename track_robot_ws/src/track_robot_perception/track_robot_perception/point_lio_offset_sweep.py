#!/usr/bin/env python3

import argparse


COARSE_OFFSETS = [-0.08, -0.04, -0.02, 0.0, 0.02, 0.04, 0.08]


def fine_offsets(center: float) -> list:
    return [center + step * 0.005 for step in range(-4, 5)]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Print repeatable Point-LIO IMU time-offset sweep commands."
    )
    parser.add_argument(
        "--mode",
        choices=("coarse", "fine"),
        default="coarse",
        help="coarse uses the first-pass offsets; fine sweeps +/-20 ms around --center.",
    )
    parser.add_argument("--center", type=float, default=0.0)
    parser.add_argument(
        "--config",
        default="install/track_robot_perception/share/track_robot_perception/config/point_lio_rshelios.yaml",
    )
    args = parser.parse_args()

    offsets = COARSE_OFFSETS if args.mode == "coarse" else fine_offsets(args.center)
    print("# Run one command at a time, record the return-path drift, then compare.")
    print("# Convention: corrected IMU stamp = raw IMU stamp - offset.")
    for offset in offsets:
        label = f"{offset:+.3f}".replace("+", "p").replace("-", "m").replace(".", "p")
        print()
        print(f"# offset {offset:+.3f} s")
        print(
            "ros2 launch track_robot_perception point_lio_rshelios.launch.py "
            f"config_file:={args.config} "
            f"imu_time_offset_sec:={offset:.6f}"
        )
        print(
            "ros2 bag record "
            f"-o ~/track_robot_bags/point_lio_offset_{label}_$(date +%Y%m%d_%H%M%S) "
            "/rslidar_points /imu/data_raw /imu/data_lio /imu/time_sync_status "
            "/cloud_registered /cloud_registered_body /Laser_map "
            "/aft_mapped_to_init /path /tf /tf_static"
        )


if __name__ == "__main__":
    main()
