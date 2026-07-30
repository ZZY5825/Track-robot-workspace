# Phase 2 Depth-Backed Memory and Phase 4B Operator Gate Design

Date: 2026-07-30

Status: approved in conversation

## 1. Objective

Make the existing Phase 0-4B pipeline repeatable without weakening the tested
motion-safety chain:

1. Phase 2 must be able to maintain a canonical three-dimensional semantic
   object from camera detection plus registered ZED depth. A missing LiDAR
   association must not veto that object.
2. LiDAR remains authoritative for obstacle clearing/marking and may strengthen
   semantic geometry, but a LiDAR cluster is optional semantic evidence.
3. Nav2 dynamic-obstacle cells must clear after the obstacle leaves instead of
   leaving blue/pink costmap trails.
4. The operator must explicitly authorize the exact visible target in RViz
   before semantic navigation can move the robot.
5. A normal live test must use one fixed Domain 20 entry point rather than a
   sequence of ad-hoc topic probes.

The approved supervised test threshold is `0.30`. It applies to the existing
bounded, fused `task_relevance` score, not to raw YOLO confidence.

## 2. Frozen safety constraints

- ROS domain is always `20`.
- IMU is not part of this pipeline.
- The RViz panel never publishes `cmd_vel` and never sends a Nav2 action.
- All executable velocity follows:

  ```text
  Nav2 /nav2/cmd_vel_raw
    -> motion_safety_supervisor
    -> /nav2/cmd_vel_safe
    -> cmd_vel_gate
    -> /cmd_vel
    -> Bunker
  ```

- RC override, E-stop, stale Bunker status, stale odometry, stale obstacle
  input and the existing speed/collision limits remain authoritative.
- Phase 2 `MemoryCore` remains the only owner of memory epochs, global object
  IDs and lifecycle state.
- A transient obstacle may hold a goal and permit later resumption. RC
  override, E-stop, target/query/reference changes and stale correlated inputs
  cancel authorization and navigation.

## 3. Current defect

`SemanticObservation` already has position, covariance, frame and
`EVIDENCE_STEREO_DEPTH` fields. Phase 1 leaves these fields invalid. The current
Phase 4A selector independently samples registered ZED depth and attaches it to
the diagnostic ranking after Phase 2. The Phase 2 ROS conversion deliberately
discards observation geometry, and its output marks position valid only when a
LiDAR key is attached.

The resulting split is:

```text
Phase 1 camera observation -> Phase 2 camera-only object without position
                                      |
                                      +-> canonical best_candidate abstains

Phase 1 camera observation + ZED depth -> Phase 4A test-only spatial copy
```

This makes Phase 2 appear blocked even when the detector and stereo depth are
working. It also creates two spatial representations of the same memory ID.

## 4. Considered approaches

### 4.1 Route Phase 4B directly from the Phase 4A test selector

This is the smallest patch, but it leaves canonical Phase 2 output invalid and
preserves duplicate spatial ownership. Rejected.

### 4.2 Enrich observations before Phase 2 and let MemoryCore retain geometry

A small depth-enrichment node copies each observation, samples registered ZED
depth inside its ROI, transforms the point into `base_link`, sets the existing
position/covariance/evidence fields and publishes a spatial observation array.
Phase 2 consumes this array and stores valid camera-depth position while
preserving its existing ID/lifecycle owner. LiDAR attachment is an optional
upgrade. Approved.

### 4.3 Replace Phase 2 with another tracker

This would invalidate tested lifecycle, replay and ID contracts and is not
supported by current evidence. Rejected.

## 5. Phase 2 spatial observation design

### 5.1 Topics

```text
/semantic_memory/observations
  -> semantic_depth_enricher
  -> /semantic_memory/spatial_observations
  -> semantic_memory
```

The input remains available for diagnostics and replay. The Phase 4A/4B launch
profile remaps only the memory consumer to the enriched topic. Other profiles
retain their current default.

### 5.2 Depth acceptance

The enricher reuses the tested median depth sampler:

- registered `32FC1` ZED depth;
- inner ROI fraction `0.5`;
- range `[0.3, 8.0]` metres;
- at least 20 valid samples;
- maximum depth/image age `0.5 s`;
- timestamped `base_link <- depth_frame` TF only;
- finite transformed point;
- fixed conservative covariance initially equal to the existing Phase 4A
  values;
- `EVIDENCE_CAMERA | EVIDENCE_STEREO_DEPTH`.

If any gate fails, the original observation is still published unchanged. This
preserves two-dimensional semantic memory while failing closed for navigation.

### 5.3 Memory representation

`CameraObservation` gains optional metric position, covariance, frame,
localization epoch and stereo-depth evidence. `MemoryObject` gains explicit
validity for stored position independent of `lidar_key`.

On a valid camera-depth update:

- the stable visual key updates the existing object;
- the position is robustly updated without creating another global ID;
- support remains `CAMERA_ONLY`, meaning camera-owned semantic evidence with
  metric camera depth;
- the public object has `position_valid=true`;
- velocity/extent remain invalid unless separately measured;
- later LiDAR attachment changes support to `CAMERA_LIDAR` without replacing
  the global ID.

The source remains visible through evidence flags and diagnostics; the public
message does not add a new support enum, avoiding an interface break.

### 5.4 Candidate gate

Only the supervised Phase 4A/4B test profile enables canonical selection at
`best_candidate_minimum_relevance=0.30`. Normal production defaults remain
fail-closed.

The candidate must still be:

- lifecycle `CONFIRMED`;
- correlated to the current query/version;
- fresh;
- position-valid in the current localization epoch and canonical frame;
- below the uncertainty limit;
- separated from the runner-up by the configured ambiguity margin.

The Phase 4A selector consumes canonical best candidate/spatial memory output
instead of constructing a second spatial object.

## 6. Dynamic costmap clearing

The independent safety grid is rebuilt from every accepted cloud and is not
the persistent blue/pink display. The blue/pink displays are Nav2 global and
local rolling costmaps; their inflation cells inherit the lifetime of marked
obstacle cells.

Each Nav2 obstacle layer uses two sources:

1. `/rslidar_points`, clearing only, to raytrace current free space;
2. `/safety/filtered_obstacle_points`, marking only, to mark the current
   non-ground, non-self obstacle set.

Observation persistence is explicitly zero. Expected update rates and
timestamped TF are configured so stale input makes the layer non-current and
the safety chain stops motion. It does not silently retain a navigable map.

An automated synthetic walk-past test must prove:

- a person-shaped return produces lethal and inflation costs;
- a later unobstructed raw scan plus empty filtered scan clears those costs
  within a bounded interval;
- a continuously present obstacle is never cleared;
- stale point-cloud or TF input prevents execution.

If the stock Foxy layer cannot pass this test, the fallback is a small
time-decaying obstacle layer behind a disabled-by-default feature flag. It is
not implemented unless the measured failure demonstrates the need.

## 7. Operator authorization

### 7.1 Interface

Add an exact-target authorization service to `track_robot_interfaces`.
The request carries:

- memory epoch;
- global object ID;
- localization epoch;
- query ID and query version;
- snapshot sequence.

The response contains `accepted` and a bounded reason. A separate
cancel-and-disarm Trigger service remains available.

### 7.2 Supervisor behavior

`SEMANTIC_ACTIVE` requires operator approval by default. Before accepting, the
supervisor compares the request with its current target and planner reference,
checks freshness, TF, odometry, planner PASS, safety inputs and current runtime
mode, and then requests `/safety/arm`.

Only a successful arm permits the already correlated Nav2 goal to dispatch.
Approval is one-shot for one exact target reference. It is invalidated by:

- query revision;
- memory/global/localization/reference change;
- target loss or stale input;
- planning failure;
- RC override;
- E-stop;
- explicit cancel;
- completion or abort.

A temporary obstacle block holds the approved goal. It does not disable
perception/planning and may resume only while the exact reference remains
valid.

### 7.3 RViz panel

The panel subscribes to canonical best candidate, Phase 4A selected target,
planner diagnostics, semantic-navigation diagnostics and safety state.

It displays the exact target ID/reference, relevance, position and readiness.
`Start Approach` is enabled only for one correlated candidate. The button calls
the authorization service and displays its response. `Cancel & Disarm` calls
the supervisor cancellation path. The panel contains no velocity publisher and
no Nav2 action client.

## 8. Standard test pipeline

The supported operator entry point is:

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase4b
```

It fixes Domain 20, never starts IMU, performs static and ROS-graph preflight,
launches the Phase 0-4B supervised stack and RViz, remains disarmed until
button authorization, captures bounded evidence and cleanly cancels/disarms
before stopping owned nodes.

The normal workflow is:

```text
run -> type English query -> inspect canonical best candidate
    -> click Start Approach -> click Cancel & Disarm or finish
```

Ad-hoc topic probing remains a diagnostic fallback, not the standard test.

## 9. Regression gates

Each independent change must pass its affected unit tests and package build.
Before live motion:

1. deterministic Phase 2 replay preserves memory/global IDs;
2. camera-depth observations produce canonical spatial objects without LiDAR;
3. LiDAR association upgrades support without changing the global ID;
4. threshold `0.30` is applied to fused relevance only;
5. walk-past costmap replay clears within its bound;
6. PLANNING_ONLY and SEMANTIC_SHADOW publish no motion;
7. SEMANTIC_ACTIVE without authorization publishes no motion;
8. stale or mismatched authorization is rejected;
9. panel source contains no `cmd_vel` publisher or Nav2 action client;
10. every executable command still traverses supervisor and gate.

Live testing occurs once after these gates pass. Nodes and services are stopped
after the test.
