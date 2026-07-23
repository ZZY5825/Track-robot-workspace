# Semantic Search Modular Bringup and Live-Test Design

**Date:** 2026-07-23
**Status:** Approved for implementation
**Scope:** Phase 1 and Phase 2 live hardware startup, diagnosis, query, and test
**Default ROS domain:** `20`

## 1. Problem

The semantic-search stack currently requires users to start camera, LiDAR,
platform, localization-health, Phase 1 perception, Phase 2 memory, and query
tools through separate commands. Environment variables, model paths, network
setup, TF readiness, and cleanup are also manual. A missing topic or transform
can leave a command waiting without explaining which dependency is absent.

The new bringup must provide one simple user entry point while preserving
independent ROS launch files for debugging and advanced use. It must remain
passive: no launch path in this feature may start a motion controller or publish
`/cmd_vel`.

## 2. Approved approach

Use a layered design:

1. Small ROS launch files own individual hardware and feature modules.
2. One aggregate live launch composes Phase 1 or Phase 2.
3. A `semantic_search_ctl` command supplies the beginner-facing
   `doctor/start/status/query/test/stop` workflow.
4. Existing semantic-search, LiDAR-tracking, semantic-memory, ZED, IMU, and
   Bunker nodes remain the functional implementations.

This provides a one-command path without hiding the underlying ROS interfaces.

## 3. Module and stage boundaries

```text
Hardware modules
├── camera       ZED camera
├── lidar        RS-LiDAR plus base_link -> rslidar
└── platform     Bunker state/odometry plus IMU

Feature stages
├── Phase 1      camera + CLIP semantic perception + text query
└── Phase 2      Phase 1 + lidar + platform
                 + localization health + LiDAR tracklets + 3D semantic memory
```

Rules:

- Phase 1 runtime graph starts/depends only on ZED and local CLIP; multi-stage
  bringup manifest may declare Phase2 packages.
- `hardware:=auto` reuses healthy hardware already running and starts only
  missing modules required by the selected stage.
- `hardware:=external` never starts hardware; it validates existing publishers.
- A sensors-only profile starts and checks hardware without loading CLIP.
- Direct low-level launch use exposes explicit module booleans so advanced users
  can choose exactly which drivers to start.
- No module in this design starts navigation, following, safety motion,
  velocity-gate, or other `/cmd_vel` publishing nodes.

## 4. User interface

The primary interface is:

```bash
ros2 run track_robot_bringup semantic_search_ctl <command>
```

Supported commands:

```bash
semantic_search_ctl doctor sensors
semantic_search_ctl doctor phase1
semantic_search_ctl doctor phase2

semantic_search_ctl start phase1
semantic_search_ctl start phase2

semantic_search_ctl status phase1
semantic_search_ctl status phase2

semantic_search_ctl query "blue chair"

semantic_search_ctl test phase1 "blue chair"
semantic_search_ctl test phase2 "blue chair"
semantic_search_ctl test phase1 "blue chair" --start-stack

semantic_search_ctl stop
```

The executable sets `ROS_DOMAIN_ID=20` for its ROS context and all managed child
processes. It also applies the supported DDS profile required for discovery.
The profile must preserve both shared-memory and UDP transports so image
transport performance is not degraded.

`start` remains in the foreground and forwards logs. `Ctrl+C` stops only the
processes started by that invocation. `stop`, run from another terminal, reads
owned process state and performs the same cleanup. It verifies process identity
before signalling a recorded PID.

`test` reuses a ready stack by default. With `--start-stack`, it starts the
requested stack, waits for readiness, tests it, and stops only the processes it
started.

The existing `semantic_search_query` executable remains supported.
`semantic_search_ctl query` is a consistent front end, not a second query
protocol.

## 5. TF authority and calibration

The authoritative live TF tree is:

```text
base_link
├── rslidar
└── zed_camera_link
    └── ZED optical frames
```

The ZED driver owns transforms below `zed_camera_link`. Bringup owns only the
physical robot mounting transform `base_link -> zed_camera_link`. There must be
one publisher for each static edge.

The current source contains a valid `base_link -> rslidar` publisher and rough
camera/LiDAR prototype values, but it does not publish an authoritative
`base_link -> zed_camera_link`. Prototype camera values are not calibration.

Camera extrinsic modes:

- `measured`: load a validated `base_link -> zed_camera_link` calibration.
- `none`: publish no base/camera mounting transform; valid for Phase 1.
- `prototype`: publish explicitly selected rough values and mark the system
  `DEGRADED / NOT CALIBRATED`.

Phase 2 in normal mode is `NOT READY` without a measured camera extrinsic.
Prototype operation requires both `extrinsic_mode:=prototype` and
`allow_degraded:=true`.

## 6. Components and responsibilities

`track_robot_bringup` owns composition and user control:

```text
track_robot_bringup/
├── launch/
│   ├── semantic_search_camera.launch.py
│   ├── semantic_search_platform.launch.py
│   ├── semantic_search_sensors.launch.py
│   └── semantic_search_live.launch.py
├── config/
│   ├── semantic_search_defaults.yaml
│   ├── camera_extrinsic.example.yaml
│   └── fastdds_semantic_search.xml
├── track_robot_bringup/
│   ├── control_cli.py
│   ├── readiness.py
│   ├── process_control.py
│   └── live_test.py
└── test/
```

Responsibilities:

- Camera launch: ZED adapter and camera mounting TF policy.
- Platform launch: Bunker state/odometry and IMU only.
- Sensors launch: explicit hardware-module composition.
- Live launch: Phase 1 or Phase 2 feature composition.
- Control CLI: commands, environment, and user-facing output.
- Readiness: bounded checks for files, topics, rates, freshness, TF, model
  runtime, and safety.
- Process control: owned child lifecycle, state file, signals, and cleanup.
- Live test: query submission, bounded metric capture, overlay rendering, and
  JSON reporting.

Core semantic algorithms remain in their current packages. Bringup may depend
on those packages; algorithm packages must not depend on bringup.

The existing Point-LIO and Fast-LIO launch adapters previously made
`track_robot_perception` depend on `track_robot_bringup` for the RS-LiDAR
driver/configuration. Because modular bringup also correctly depends on
perception for the Phidget IMU, that reverse edge forms a package cycle. A
small, motion-free `track_robot_sensor_bringup` package therefore owns the
shared RS-LiDAR network/driver/TF launch and configuration. Both perception
and top-level bringup depend on this lower-level package. The existing
`track_robot_bringup/rslidar_with_tf.launch.py` remains as a compatibility
wrapper.

## 7. Startup and readiness flow

```text
Resolve stage and defaults
→ set Domain 20 and DDS environment
→ validate static files and configuration
→ inspect existing hardware publishers
→ start only missing required hardware
→ wait for bounded sensor readiness
→ start requested feature nodes
→ perform bounded stage readiness checks
→ print readiness table
→ remain in foreground
```

Readiness uses four states:

- `PASS`: required condition is healthy.
- `NOT READY`: a required condition is absent; automated testing stops.
- `DEGRADED`: diagnostics may continue, but the result is not a formal test.
- `FAIL`: a process exited or runtime data is invalid.

Checks include, as applicable:

- ROS domain and package availability.
- CLIP runtime, checkpoint path, and SHA-256.
- Image, camera-info, LiDAR, IMU, and odometry publishers.
- Bounded topic frequency and message freshness.
- Frame IDs and required TF paths.
- Query subscriber and semantic-region publication.
- LiDAR tracklets, localization-health, and semantic-memory diagnostics.
- Zero `/cmd_vel` publishers throughout bringup and test.

Every wait has a stage-specific timeout. Failures print an actionable cause and
the relevant path, topic, transform, interface, or manual network command.

## 8. Live test semantics

Phase 1 automated pipeline checks:

- The query is accepted.
- Query ID and version remain consistent.
- Semantic regions publish for the test interval.
- Scores are finite and messages are structurally valid.
- Output rate and latency are reported.
- No model or image-encoding failure occurs.

Phase 2 adds:

- LiDAR tracklets publish.
- Localization-health state matches available inputs.
- Camera observations can enter the common spatial frame.
- Semantic memory creates or updates objects.
- Formal mode does not use prototype extrinsics.

Pipeline health is not semantic correctness. Without ground truth, a non-empty
ROI cannot prove that the requested object was localized correctly. Each test
therefore produces:

- candidate-region overlay;
- minimum, mean, and maximum scores;
- non-empty-frame ratio;
- Phase 2 association and memory counts;
- a machine-readable JSON report.

Default report directory:

```text
~/.ros/track_robot_semantic_search/reports/<timestamp>/
```

Without ground truth, the result is:

```text
Pipeline: PASS
Semantic result: REVIEW REQUIRED
```

Future labelled recordings or simulation truth can add an automatic semantic
PASS/FAIL without changing the control interface.

## 9. Error and cleanup policy

- Missing or invalid model: do not start Phase 1.
- Missing measured camera extrinsic: Phase 2 is `NOT READY`.
- LiDAR interface lacks non-interactive privileges: fail promptly and print the
  required manual command; never block waiting for a password.
- Existing healthy drivers: reuse them in auto mode.
- Duplicate drivers or conflicting TF publishers: do not proceed silently.
- Required child exits: mark the stage `FAIL` and stop other owned children.
- Stale process state: verify command identity; never signal an unrelated PID.
- `Ctrl+C`, `stop`, and test completion stop feature nodes before hardware.
- Externally running processes are never owned or stopped by the tool.

## 10. Acceptance criteria

1. From a sourced workspace, one command starts Phase 1 or Phase 2 using Domain
   20.
2. Phase 1 starts and tests without LiDAR, Bunker, or IMU.
3. Auto mode reuses active hardware and starts only missing dependencies.
4. External mode never starts or stops hardware.
5. Missing models, topics, TF, or live data produce an actionable result within
   15–30 seconds rather than waiting indefinitely.
6. Phase 1 live test writes metrics, an overlay, and JSON.
7. Phase 2 live test reports tracklet, localization, association, and memory
   status.
8. Results without ground truth say `REVIEW REQUIRED`.
9. All owned ROS processes stop after testing or debugging.
10. `/cmd_vel` has zero publishers in every supported mode.
11. Unit tests cover stage resolution, environment generation, readiness
    classification, process-identity safety, cleanup order, and report format.
12. Launch-contract tests run without hardware; real devices are required only
    for final live acceptance.

## 11. Out of scope

- Motion control, autonomous following, and navigation.
- Camera extrinsic calibration measurement itself.
- Changing Phase 1 model architecture or Phase 2 association algorithms.
- Automatic semantic accuracy claims without labelled or simulated ground
  truth.
- A persistent system daemon or boot-time service.
- The future RViz query panel.
