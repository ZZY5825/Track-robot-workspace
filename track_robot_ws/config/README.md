# Machine-Local Configuration

This directory is for measured configuration that belongs to one robot or
development machine rather than to a reusable ROS package.

## Phase 2 Camera/LiDAR Extrinsic

Copy the reviewed example and then replace every value with a measured result:

```bash
cp "$(ros2 pkg prefix --share track_robot_bringup)/config/camera_extrinsic.example.yaml" \
  ~/track_robot_ws/config/camera_extrinsic.measured.yaml
```

`camera_extrinsic.measured.yaml` is ignored by default because a placeholder or
another robot's transform must not be mistaken for valid calibration.

## Local Runtime Snapshots

Generated launch parameter snapshots and temporary machine-specific YAML files
belong under `config/local/`. Examples and reusable defaults remain in the
owning ROS package under `src/<package>/config/`.

Promote a local file into a package configuration only after its provenance,
scope, and safety impact have been reviewed.
