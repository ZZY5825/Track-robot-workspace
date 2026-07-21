# Phase 2 pilot recording and evaluation guide

This guide creates the first evidence-bearing dataset for Generalized
Multi-Object 3D Semantic Memory. Do not reuse the legacy human-tracking bag as
proof of camera/LiDAR association, persistent odom/map memory, re-ID, or task
ranking: it lacks the required connected sensor, TF and annotation evidence.

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
mkdir -p rosbags/semantic_search/bags
ros2 bag record -o rosbags/semantic_search/bags/phase2_pilot_YYYYMMDD_HHMMSS \
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
ros2 bag info rosbags/semantic_search/bags/phase2_pilot_YYYYMMDD_HHMMSS
sqlite3 rosbags/semantic_search/bags/phase2_pilot_YYYYMMDD_HHMMSS/*.db3 \
  'PRAGMA quick_check;'
```

## 5. Create the cryptographic manifest

Only create the manifest after the real bag exists. This intentionally avoids
a placeholder SHA-256 that could later be mistaken for evidence:

```bash
ros2 run track_robot_semantic_search semantic_search_manifest create-field \
  rosbags/semantic_search/bags/phase2_pilot_YYYYMMDD_HHMMSS \
  rosbags/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json \
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
  rosbags/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json
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
  rosbags/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json \
  rosbags/semantic_search/annotations/phase2_pilot_YYYYMMDD_HHMMSS.jsonl \
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
  --samples rosbags/semantic_search/annotations/phase2_task_calibration_YYYYMMDD.jsonl \
  --dataset-id phase2_task_calibration_YYYYMMDD \
  --output rosbags/semantic_search/reports/phase2_task_threshold_YYYYMMDD.json
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
  --report rosbags/semantic_search/reports/phase2_deterministic_replay.json
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
  --manifest rosbags/semantic_search/manifests/phase2_pilot_YYYYMMDD_HHMMSS.json \
  --annotations rosbags/semantic_search/annotations/phase2_pilot_YYYYMMDD_HHMMSS.jsonl \
  --predictions NORMALIZED_MATCHED_PREDICTIONS.jsonl \
  --runtime runtime.json \
  --resources resources.json \
  --task-threshold-calibration \
    rosbags/semantic_search/reports/phase2_task_threshold_YYYYMMDD.json \
  --output rosbags/semantic_search/reports/phase2_pilot_evaluation.json \
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
