# Phase 0–4 Planning-Only Integration Test

This is the standard ROS Domain 20 procedure for validating the passive
language-to-approach chain. Phase 4 never commands the robot.

## Acceptance chain

The live test passes only when one query produces:

1. a healthy Phase 0 localization state with one stable localization epoch;
2. non-empty Phase 1 observations with the submitted query ID/version;
3. a stable Phase 2 `(memory_epoch_id, global_object_id)` and valid 3D position;
4. one Phase 3 target with the same IDs, query reference, confidence, and
   bounded uncertainty;
5. Phase 4 approach candidates, one selected goal, and a non-empty path in the
   costmap frame.

Any missing stage makes that stage and the complete chain fail. Never replace
missing live inputs with synthetic evidence in the live report.

## Start the live stack

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_bringup semantic_search_live.launch.py \
  stage:=phase3 \
  start_camera:=true \
  start_lidar:=true \
  start_base:=true \
  start_imu:=true \
  extrinsic_mode:=prototype \
  allow_degraded:=true
```

The `prototype` extrinsic is suitable for engineering integration only. A
measured camera-to-base transform is required before a physical acceptance
claim.

In a second terminal, start only the local obstacle-map producer:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
ros2 run track_robot_safety local_obstacle_map_node \
  --ros-args \
  --params-file ~/track_robot_ws/src/track_robot/track_robot_safety/config/local_obstacle_map.yaml
```

In a third terminal, start the planning-only Phase 4 node and RViz:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch track_robot_semantic_search semantic_search_phase4.launch.py \
  start_rviz:=true
```

RViz should contain the local costmap, semantic target marker, 16 approach
candidates, selected goal, and planned path. Missing layers are evidence of an
upstream rejection; inspect `/semantic_search/phase4/diagnostics`.

## Submit exactly one query

Use an explicit key so every phase can be correlated:

```bash
ros2 run track_robot_semantic_search semantic_search_query \
  "green bottle" \
  --query-id 2026072801 \
  --query-version 1 \
  --timeout 10 \
  --subscriber-timeout 5
```

The query command must report `ACCEPTED`. Use a new positive ID for a later
independent run.

## Collect bounded live evidence

```bash
ros2 run track_robot_semantic_search semantic_search_phase04_live_validate \
  --query "green bottle" \
  --query-id 2026072801 \
  --query-version 1 \
  --duration-sec 25 \
  --output /tmp/phase0_4_live.json
```

The report records each phase as `PASS`, `FAIL`, or `NOT EVALUATED`, plus
frames, epochs, global IDs, query references, object/path counts, planner
latency, and `cmd_vel` publisher count.

## Run the Phase 4 failure matrix

This deterministic contract is required even when the live chain cannot
produce a selected target:

```bash
ros2 run track_robot_semantic_search semantic_search_phase4_validate \
  --output /tmp/phase4_contract.json
```

The eight required cases are:

- success;
- no target;
- ambiguous target;
- target lost;
- invalid position or frame;
- blocked path;
- stale map;
- localization reset.

Each failure case passes only when planning fails for the exact expected
reason and publishes no usable path. This validates Phase 4 logic, not the
live Phase 0–3 chain.

## Live gates

- Phase 0: all samples healthy; one localization epoch and canonical frame.
- Phase 1: non-empty observations; one query ID/version; monotonic stamps.
- Phase 2: one memory epoch and global object ID for the target; valid 3D
  position; monotonic stamps; matching query reference.
- Phase 3: exactly one selected target; matching memory/global/localization and
  query IDs; confidence present and uncertainty within the configured bound.
- Phase 4: fresh costmap and target in one frame; at least one candidate and
  path pose; diagnostic reason `planned`; planner P95 below the 200 ms 5 Hz
  budget.
- Safety: zero publishers capable of executing Phase 4 motion.

The current blocked-path search is also recorded. If it exceeds 200 ms, report
it as a Phase 4 performance blocker even if the functional rejection is
correct.

## Shutdown

Press `Ctrl-C` in the Phase 4, obstacle-map, and live-stack terminals. Confirm
that no test-owned ROS, RViz, ZED, or LiDAR process remains before ending the
test.
