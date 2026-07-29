# Phase 4A Fixed-Base Local Approach Advisory

Date: 2026-07-28

## Objective

Phase 4A is the minimum workable language-to-local-approach system for a
physically stationary robot. For the query `green bottle`, it must use the
real camera and LiDAR pipeline to report:

- the target's rough position in `base_link`;
- range and left/right bearing from the robot;
- a collision-checked standoff goal;
- a human-readable approach instruction.

Phase 4A never commands the base. It does not publish `Twist`, `cmd_vel`,
navigation goals, actions, or controller inputs.

## Explicit test boundary

The operator guarantees that the chassis remains stationary for the complete
test. Phase 4A does not use IMU or odometry to infer this condition.

A test-only fixed-base session bridge publishes
`SemanticLocalizationState` with:

- memory mode `MEMORY_LOCAL_SESSION`;
- one non-zero localization epoch for the launch lifetime;
- `canonical_frame_id=base_link`;
- `local_frame_id=base_link`;
- `local_healthy=true`;
- reason `operator_asserted_fixed_base_test`.

The bridge does not publish `/odom`, TF, or motion. Its output uses a dedicated
Phase 4A topic so it cannot silently replace production localization. The
Phase 4A launch remaps semantic memory and the planner to that topic.

Stopping or restarting the bridge creates a new epoch and invalidates every
old Phase 4A object and recommendation.

## Data flow

1. ZED 2i publishes the rectified image.
2. YOLO-World processes the English query and publishes semantic observations.
3. The existing camera/LiDAR association attaches the visual observation to
   RoboSense geometry.
4. Semantic memory runs in the fixed `base_link` local session and publishes
   stable `(memory_epoch_id, global_object_id)` objects plus diagnostic ranking.
5. A Phase 3A test selector examines diagnostic ranking without enabling the
   uncalibrated production best-candidate threshold.
6. The existing Phase 4 planning-only planner consumes the selected test
   target, fixed-base session state, and local obstacle grid.
7. A Phase 4A advisor publishes a bounded text summary and diagnostics while
   RViz displays the target, candidates, selected goal, costmap, and path.

## Phase 3A test selector

The selector publishes `/semantic_search/phase4a/selected_target`. It remains
separate from `/semantic_memory/best_candidate`.

A target becomes ready only when all gates pass:

- exactly one active query ID/version;
- lifecycle is confirmed;
- support is `SUPPORT_CAMERA_LIDAR`;
- position is valid and expressed in `base_link`;
- active query reference matches the submitted query;
- task relevance is at least 0.50;
- top-to-second relevance margin is at least 0.08;
- uncertainty is at most 0.50;
- the same compound object key passes for at least three consecutive fresh
  snapshots;
- the rolling five-sample XY spread is at most 0.35 m;
- source age is at most 1.0 second.

The defaults are explicit Phase 4A engineering-test gates, not production
calibration evidence.

If a gate fails, the selector publishes an empty selected-target array and a
diagnostic reason such as `no_target`, `ambiguous_target`,
`no_camera_lidar_support`, `invalid_position`, `unstable_position`,
`query_mismatch`, or `stale_target`.

## Phase 4A advisory output

The advisor publishes:

- `/semantic_search/phase4a/advice` (`std_msgs/String`);
- `/semantic_search/phase4a/diagnostics` (`DiagnosticArray`).

A ready message is a single bounded English line suitable for a terminal:

```text
READY target="green bottle" position=front 1.60m, right 0.20m
range=1.61m bearing=-7.1deg approach=front-right goal=(0.81,-0.10)m
standoff=0.80m path=clear confidence=0.72 uncertainty=0.28
ADVISORY_ONLY
```

The actual published line contains no newline. Position, goal, path length,
compound object key, localization epoch, and query key are also present as
diagnostic key/value fields.

Failure output begins with `NOT_READY` and includes the exact rejection reason.
No stale successful advice is retained after a failure.

## Launch contract

One operator entry point starts the minimum system:

```bash
ros2 launch track_robot_bringup semantic_search_phase4a.launch.py
```

Defaults:

- `ROS_DOMAIN_ID=20` is required by the operator environment;
- camera and LiDAR start;
- Bunker and IMU do not start;
- fixed-base bridge, Phase 1 perception, Phase 2 memory, Phase 3A selector,
  local obstacle map, Phase 4 planner, advisor, live overlay, and RViz start;
- Phase 4 planning remains `planning_only=true`.

The launch rejects attempts to enable base motion from Phase 4A arguments.

## Acceptance criteria

With a green bottle roughly 1.6 m ahead of the camera and the chassis
stationary:

1. the query is acknowledged with one query ID/version;
2. Phase 1 publishes non-empty observations for that query;
3. Phase 2 keeps one compound object key for at least three consecutive
   accepted snapshots and supplies a valid `base_link` position;
4. Phase 3A publishes one selected test target with matching query, memory,
   object, and localization references;
5. Phase 4 publishes at least one candidate, selected goal, and non-empty path;
6. the advisor publishes `READY`, rough target position, bearing, standoff
   goal, and `path=clear`;
7. RViz shows target, candidates, goal, costmap, and path;
8. the ROS graph contains no Phase 4A motion publisher, action client, or
   executable navigation goal;
9. target removal changes the output to `NOT_READY` within 1.5 seconds;
10. all test-owned nodes stop when the launch exits.

If live camera/LiDAR association cannot produce a stable valid 3D object, the
test result is FAIL with evidence; the implementation must not substitute a
synthetic target.

## Out of scope

- robot motion or navigation execution;
- world/map localization;
- operation while the chassis moves;
- production best-candidate calibration;
- centimeter-level position accuracy;
- obstacle-map persistence beyond the current fixed-base session.

## Verification

Implementation follows test-first development:

- pure selector tests for confirmation, ambiguity, query/key changes,
  position spread, support, and freshness;
- advisor formatting and stale-output tests;
- launch/source contracts proving base and IMU are off and motion interfaces
  do not exist;
- deterministic fixed-base Phase 0–4A replay;
- one bounded live green-bottle run with per-stage evidence and process cleanup.
