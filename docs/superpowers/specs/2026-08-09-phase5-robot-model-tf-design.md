# Phase 5 Robot Model and TF Integration Design

## Goal

Show the calibrated Bunker Pro 2 and sensor-station model in the current Phase
1-5 RViz consoles while preserving a single-parent runtime TF tree during
supervised motion.

## Physical tree

`src/bunker_pro2/urdf/bunker_pro2.urdf` remains the only description source.
Add an empty `robot_bottom` root 0.45 m below the existing chassis top-plane
`base_link`:

```text
odom -> robot_bottom -> base_link -> sensor_station_link
                                      |- camera_link
                                      `- lidar_link
```

The fixed transform stored in URDF is `robot_bottom -> base_link`,
`xyz="0 0 0.45"`, `rpy="0 0 0"`. The existing calibrated sensor-station,
camera, and LiDAR joints remain byte-for-byte unchanged; `camera_link` and
`lidar_link` remain empty.

## Runtime TF ownership

A new pure `bunker_pro2/description.launch.py` starts only
`robot_state_publisher`. Phase 4A includes it, so Phase 4B and Phase 5A inherit
the fixed physical tree through their existing Phase 4A composition.

The third-party Bunker driver already exposes a `base_frame` launch argument.
`semantic_search_platform.launch.py` forwards a new compatible `base_frame`
argument whose default remains `base_link`. Phase 4B and Phase 5A explicitly
set it to `robot_bottom`. Therefore their `/odom` message and dynamic TF use
`robot_bottom`, while Nav2 and perception continue to use `base_link` through
the fixed 0.45 m transform. No third-party driver source or Nav2 algorithm is
changed.

When a future localization source owns `map -> odom`, it extends the same tree
without changing this contract.

## RViz

Add one Foxy-compatible `rviz_default_plugins/RobotModel` display subscribed
to transient-local `/robot_description` in the Phase 4A, Phase 4B, and Phase
5A RViz configurations. Existing fixed frames, paths, maps, point clouds, and
semantic overlay displays remain unchanged.

## Safety and compatibility

- Phase 1-3 and generic platform bringup retain the historical `base_link`
  default.
- Only Phase 4B/5A explicitly opt into `robot_bottom` dynamic odometry.
- `robot_state_publisher` is included once through Phase 4A; Phase 4B/5A do
  not start duplicate description publishers.
- The change does not publish velocity or alter the safety/velocity gate.

## Validation

Contract tests lock all four fixed transforms, the empty sensor links,
description launch ownership, Bunker `base_frame` forwarding, Phase 4B/5A
selection of `robot_bottom`, and all three RobotModel displays. Build and test
the `bunker_pro2` and `track_robot_bringup` packages, then run a no-motion
smoke test with the Bunker driver in simulated mode to verify
`odom -> robot_bottom -> base_link` and stop all launched nodes afterward.
