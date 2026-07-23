# Phase 2 pilot recording and evaluation guide

This guide creates the first evidence-bearing dataset for Generalized
Multi-Object 3D Semantic Memory. Do not reuse the legacy human-tracking bag as
proof of camera/LiDAR association, persistent odom/map memory, re-ID, or task
ranking: it lacks the required connected sensor, TF and annotation evidence.

## Beginner quick start: modular live checks (ROS Domain 20)

Use this section before the longer recording procedure. The control tool fixes
its managed ROS context to Domain 20; exporting the same value also puts any
manual `ros2 topic` and TF commands in this guide on the same graph:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
```

The modular bringup is passive. It does not start navigation, a follower,
motion controllers, or any `/cmd_vel` publisher, and the safety preflight
reports `FAIL` if `/cmd_vel` already has a publisher. None of the commands
below moves the robot. Motion used later to collect a recording remains a
separate, supervised operator action with the normal safety chain.

### Phase 1: camera and language-conditioned image search

First inspect requirements without starting anything:

```bash
ros2 run track_robot_bringup semantic_search_ctl doctor phase1 --hardware external
```

For the simplest live start, let bringup reuse a camera publisher when one
already exists or start the camera when no publisher exists. Readiness then
checks whether the reused or started camera is healthy. `start` stays in the
foreground so its logs are visible:

```bash
ros2 run track_robot_bringup semantic_search_ctl start phase1 --hardware auto
```

In a second Domain 20 terminal, inspect status, enter a language query, and run
a bounded live test:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run track_robot_bringup semantic_search_ctl status phase1 --hardware external
ros2 run track_robot_bringup semantic_search_ctl query "blue chair"
ros2 run track_robot_bringup semantic_search_ctl test phase1 "blue chair" --hardware external
```

`test` normally reuses an already-ready stack. To make one command start a
missing stack, wait for readiness, test, and clean up only what it owns:

```bash
ros2 run track_robot_bringup semantic_search_ctl test phase1 "blue chair" \
  --start-stack --hardware auto
```

### Phase 2: camera, LiDAR, platform state, association, and memory

Formal Phase 2 requires a measured physical transform from `base_link` to
`zed_camera_link`. Copy the installed
`camera_extrinsic.example.yaml`, replace the placeholder calibration ID and
all transform values with measured values, then keep its required frame names.
The example file itself is deliberately rejected as unmeasured.

```bash
mkdir -p ~/track_robot_ws/config
EXTRINSIC=~/track_robot_ws/config/camera_extrinsic.measured.yaml
cp "$(ros2 pkg prefix --share track_robot_bringup)/config/camera_extrinsic.example.yaml" \
  "$EXTRINSIC"
# Edit "$EXTRINSIC" with the measured transform before continuing.

ros2 run track_robot_bringup semantic_search_ctl doctor phase2 \
  --hardware external --extrinsic-mode measured --extrinsic-file "$EXTRINSIC"

ros2 run track_robot_bringup semantic_search_ctl start phase2 \
  --hardware auto --extrinsic-mode measured --extrinsic-file "$EXTRINSIC"
```

In a second terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
EXTRINSIC=~/track_robot_ws/config/camera_extrinsic.measured.yaml

ros2 run track_robot_bringup semantic_search_ctl status phase2 \
  --hardware external --extrinsic-mode measured --extrinsic-file "$EXTRINSIC"
ros2 run track_robot_bringup semantic_search_ctl query "blue chair"
ros2 run track_robot_bringup semantic_search_ctl test phase2 "blue chair" \
  --hardware external --extrinsic-mode measured --extrinsic-file "$EXTRINSIC"
```

Without a measured extrinsic, Phase 2 is `NOT READY`. Rough prototype geometry
is available only for diagnostic work and must be selected explicitly:

```bash
ros2 run track_robot_bringup semantic_search_ctl start phase2 \
  --hardware auto --extrinsic-mode prototype --allow-degraded
```

That mode is labelled `DEGRADED / NOT CALIBRATED`; it cannot count as a formal
Phase 2 result. Do not substitute the prototype values for calibration.

### Hardware ownership, results, and cleanup

- `--hardware auto` reuses required modules that already have a publisher and
  starts only modules with no publisher. Readiness then validates topic rate,
  freshness, TF, and the other stage health requirements.
- `--hardware external` never starts hardware; it only checks publishers that
  another terminal or launch system owns.
- `doctor` is a bounded, read-only preflight. `status` adds verified managed
  process state. Neither command starts nodes.
- `start` owns only the child group it creates. Press `Ctrl+C` in that terminal,
  or use the following command from another Domain 20 terminal:

```bash
ros2 run track_robot_bringup semantic_search_ctl stop
```

Readiness states are `PASS` (healthy), `NOT READY` (required input absent),
`DEGRADED` (diagnostics may continue but the run is not formal), and `FAIL`
(invalid runtime data, unsafe graph, or a process error). Stable control exit
codes are `0` for PASS/success, `2` for NOT READY (also no managed process to
stop), `3` for DEGRADED, `4` for FAIL, and `130` for an interrupted managed
run. The `query` command preserves the underlying query portal's result.

Live-test JSON and candidate overlays are written below:

```text
~/.ros/track_robot_semantic_search/reports/<timestamp>/
```

The command prints the exact report path. A healthy unlabelled run reports
`Pipeline: PASS`, but semantic correctness remains
`Semantic result: REVIEW REQUIRED`; inspect the overlay and score summary
instead of treating a non-empty region as ground truth.

## 1. Prepare a safe pilot scene

Use a 60–90 second session with manual or supervised robot motion. Include:

1. one static target seen from at least three viewpoints;
2. a second similar static object;
3. a person crossing between the robot and static target;
4. a short camera occlusion;
5. target outside camera FOV while LiDAR can still see it;
6. target outside both sensors and, if practical, re-entering;
7. LiDAR split and merge opportunities without unsafe staging;
8. one camera false-positive opportunity and one LiDAR false cluster;
9. robot translation and rotation;
10. a task-query change while memory remains running.

Do not issue unrestricted motion commands for this test. Keep the existing
safety chain and operator stop available.

## 2. Verify topic and clock identity

Source the built workspace and check every required source before recording:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 topic info /zed/zed_node/left/image_rect_color --verbose
ros2 topic info /zed/zed_node/left/camera_info --verbose
ros2 topic info /rslidar_points --verbose
ros2 topic info /imu/data --verbose
ros2 topic info /odom --verbose
ros2 topic info /semantic_memory/localization_state --verbose
ros2 topic info /semantic_memory/lidar_tracklets --verbose
ros2 topic info /semantic_memory/observations --verbose
ros2 topic info /semantic_memory/tasks --verbose
```

Replace only the camera, LiDAR, IMU and odometry topic names that differ on the
robot. Record the resolved names in the manifest provenance notes.

## 3. TF preflight at sensor source times

The selected pilot memory frame should initially be `odom`. First prove the
graph is connected at the current time:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom zed_left_camera_optical_frame
ros2 run tf2_ros tf2_echo odom velodyne
ros2 run tf2_ros tf2_echo zed_left_camera_optical_frame velodyne
```

Use the real frame IDs from `CameraInfo`, `PointCloud2`, and the typed
localization state. A latest-time transform alone is insufficient. Run the
semantic-memory node in default shadow mode for at least 30 seconds while both
sensors publish:

```bash
ros2 launch track_robot_semantic_memory semantic_memory_phase2.launch.py
ros2 topic echo /semantic_memory/diagnostics
ros2 topic hz /semantic_memory/association_debug
```

The node requests TF at each LiDAR `last_measurement_stamp`. Preflight passes
only when diagnostics show a valid localization domain, valid CameraInfo,
accepted LiDAR batches, nonzero shadow candidate pairs when targets overlap,
and no missing/stale TF or calibration rejection. Save the diagnostic and
association-debug output. If this cannot be demonstrated, record only for
debugging and mark `phase2.tf_preflight_passed` false; that bag cannot satisfy
LOCAL_SESSION/WORLD gates.

## 4. Record the pilot bag

Use a new directory and record raw evidence, normalized inputs, outputs and
diagnostics together:

```bash
mkdir -p rosbags/semantic_search/recordings
ros2 bag record -o rosbags/semantic_search/recordings/phase2_pilot_YYYYMMDD_HHMMSS \
  /zed/zed_node/left/image_rect_color \
  /zed/zed_node/left/camera_info \
  /rslidar_points \
  /imu/data \
  /odom \
  /tf \
  /tf_static \
  /semantic_memory/localization_state \
  /semantic_memory/lidar_tracklets \
  /semantic_memory/observations \
  /semantic_memory/tasks \
  /semantic_memory/active_objects \
  /semantic_memory/best_candidate \
  /semantic_memory/events \
  /semantic_memory/association_debug \
  /semantic_memory/diagnostics
```

Stop with Ctrl-C and wait for rosbag2 to close. Do not hash or copy an open
SQLite bag. Confirm duration, message counts and storage health:

```bash
ros2 bag info rosbags/semantic_search/recordings/phase2_pilot_YYYYMMDD_HHMMSS
sqlite3 rosbags/semantic_search/recordings/phase2_pilot_YYYYMMDD_HHMMSS/*.db3 \
  'PRAGMA quick_check;'
```

## 5. Create the cryptographic manifest

Only create the manifest after the real bag exists. This intentionally avoids
a placeholder SHA-256 that could later be mistaken for evidence:

```bash
ros2 run track_robot_semantic_search semantic_search_manifest create-field \
  rosbags/semantic_search/recordings/phase2_pilot_YYYYMMDD_HHMMSS \
  artifacts/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json \
  --dataset-id phase2_pilot_YYYYMMDD_HHMMSS \
  --workspace-root "$PWD" \
  --split extension \
  --site-id YOUR_SITE --session-id phase2_pilot_YYYYMMDD_HHMMSS \
  --lighting YOUR_LIGHTING --surface YOUR_SURFACE --weather YOUR_WEATHER \
  --camera-intrinsics-id VERIFIED_CAMERA_ID \
  --camera-lidar-extrinsics-id VERIFIED_CAMERA_LIDAR_ID \
  --lidar-imu-extrinsics-id VERIFIED_LIDAR_IMU_ID \
  --localization-config-id VERIFIED_ODOM_CONFIG
```

Add the `phase2` block defined in `dataset_manifest.schema.json`, initially with
only scenarios actually performed and boolean evidence flags set from recorded
facts. Then validate:

```bash
ros2 run track_robot_semantic_search semantic_search_manifest validate \
  artifacts/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json
```

## 6. Annotate without leaking runtime guesses into ground truth

Create JSONL records following `annotation.schema.json`. For each relevant
source stamp identify the physical `object_id`, camera box/mask, LiDAR source
key when known, visibility/support state, approximate 3D position and task
relevance. Use `ignore: true` plus a reason for genuinely unjudgeable regions.
Do not turn an uncertain association into a negative label merely because the
runtime rejected it.

Register the closed annotation file so its checksum becomes part of the
manifest:

```bash
ros2 run track_robot_semantic_search semantic_search_manifest add-annotations \
  artifacts/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json \
  artifacts/semantic_search/annotations/phase2_pilot_YYYYMMDD_HHMMSS.jsonl \
  --workspace-root "$PWD"
```

The association calibration exporter still requires at least 20 positive and
20 negative human labels. Keep `camera_attachment_enabled: false` until its
report says `calibrated` and permits attachment.

Task ranking uses a separate calibration split so the final-test threshold is
not chosen after inspecting final results. Create a JSONL file with at least 30
human-positive and 30 hard-negative candidates. Each line has this shape:

```json
{"dataset_id":"phase2_task_calibration_YYYYMMDD","split":"calibration","query_id":1,"candidate_id":"trial-001-object-a","task_relevant":true,"relevance_score":0.82}
```

Candidate identities must be unique and scores must be finite in `[0, 1]`.
Freeze the threshold before final evaluation:

```bash
ros2 run track_robot_semantic_search \
  semantic_search_phase2_calibrate_task_threshold \
  --samples artifacts/semantic_search/annotations/phase2_task_calibration_YYYYMMDD.jsonl \
  --dataset-id phase2_task_calibration_YYYYMMDD \
  --output artifacts/semantic_search/reports/phase2_task_threshold_YYYYMMDD.json
```

Exit code 0 means a threshold meets task recall `>=0.90`, hard-negative false
confirmation `<=0.05`, and both 30-sample minimums. Exit code 2 means evidence
is insufficient. Do not set `best_candidate_threshold_calibrated: true` or copy
the selected threshold into a production profile until the real report has
been reviewed.

## 7. Replay, profile and evaluate

Run normalized deterministic replay twice:

```bash
ros2 run track_robot_semantic_search semantic_search_phase2_replay \
  --executable install/track_robot_semantic_memory/lib/track_robot_semantic_memory/semantic_memory_replay \
  --input NORMALIZED_REPLAY.json \
  --output NORMALIZED_OUTPUT.json \
  --report artifacts/semantic_search/reports/phase2_deterministic_replay.json
```

Capture at least 30 minutes of runtime evidence. Phase 2 core timing remains
separate from complete semantic-path timing. `runtime.json` has this contract:

```json
{
  "duration_sec": 1800.0,
  "update_stamps_ns": [100000000000, 100200000000],
  "module_latency_ms": {"semantic_memory_core": [10.0, 12.0]},
  "semantic_path_latency_ms": [90.0, 110.0],
  "drops": 0,
  "bounded_growth_pass": true
}
```

The update-stamp source span must also reach 1,800 seconds; a long wall clock
with only a short sample window does not pass. `resources.json` contains raw
Jetson samples, not handwritten percentiles:

```json
{
  "cpu_percent": [35.0, 38.0],
  "gpu_percent": [42.0, 45.0],
  "resident_memory_mb": [920.0, 940.0],
  "cuda_reserved_memory_mib": [1100.0, 1120.0]
}
```

Matched prediction JSONL records used for task evaluation include `query_id`,
`task_relevant`, `task_rank`, `task_selected`, `task_relevance`, and
`task_threshold`. The evaluator recomputes selection as
`task_relevance >= task_threshold`, rejects contradictions, and requires every
threshold to match the frozen calibration report. Then build the fail-closed
report:

```bash
ros2 run track_robot_semantic_search semantic_search_phase2_evaluate \
  --manifest artifacts/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json \
  --annotations artifacts/semantic_search/annotations/phase2_pilot_YYYYMMDD_HHMMSS.jsonl \
  --predictions NORMALIZED_MATCHED_PREDICTIONS.jsonl \
  --runtime runtime.json \
  --resources resources.json \
  --task-threshold-calibration \
    artifacts/semantic_search/reports/phase2_task_threshold_YYYYMMDD.json \
  --output artifacts/semantic_search/reports/phase2_pilot_evaluation.json \
  --deterministic-replay-passed \
  --human-tracking-regression-passed \
  --software-revision YOUR_GIT_REVISION
```

Exit code 0 means every configured Phase 2 gate passed. Exit code 2 means the
report is failed or unavailable; inspect `reasons`, `missing_scenarios` and
individual metric availability rather than overriding the result.

The full Stage 2G gate also requires all twelve manifest scenarios, complete
semantic-path latency P95 at most 150 ms, core latency P95 at most 50 ms,
update rate at least 5 Hz, CUDA reserved-memory P95 at most 1,536 MiB,
deterministic replay, and an independently executed human-tracking regression.
Old human-tracking bags remain useful regression inputs but cannot provide the
missing Phase 2 topics, annotations or Jetson profile.
