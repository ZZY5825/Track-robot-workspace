# Phase 0–3 YOLO-World test guide

This is the current operator procedure for the passive semantic-search stack
on Jetson AGX Orin. The software checkpoint is
`phase0_3_test_ready`: the Phase 0–3 contracts and deterministic replay pass,
but physical accuracy, measured camera/LiDAR calibration, long-duration
stability, re-entry thresholds, model licensing approval, and production
winner release are not yet proven.

Every managed command uses ROS Domain 20. None of these stages starts
navigation, a motion controller, or a `/cmd_vel` publisher. Readiness fails if
one is already present.

## 0. Mandatory live-test rules

These rules are test gates, not troubleshooting suggestions:

1. Use `ROS_DOMAIN_ID=20` for every ROS process and inspection command.
2. Use the official `zed_wrapper` path for ZED2i. Do not replace it with a UVC
   publisher unless the operator explicitly authorizes a degraded fallback.
3. Exactly one process owns the camera:
   - managed stack: start Phase 1–3 with `--hardware auto`; or
   - external camera: launch `zed_wrapper` first and use `--hardware external`.
   Never launch both camera owners.
4. ZED publishing is subscriber-driven and cold startup can take several
   seconds. A topic name without a message, or a short timeout before an
   active subscriber exists, is not evidence that the camera failed.
5. Do not declare the camera ready or failed until all three camera gates have
   been checked:
   - the launch log contains `Camera successfully opened`;
   - an active subscriber measures
     `/zed/zed_node/left/image_rect_color` above 5 Hz for a bounded sample;
   - `base_link -> zed_left_camera_optical_frame` is available for Phase 2/3.
6. This is a passive test. Before and during the run,
   `ros2 topic info /cmd_vel` must show zero publishers.
7. Use English queries only. Query switching no longer requires restarting
   the perception process: the cached CLIP text encoder is restored to FP32
   before updating the YOLO-World vocabulary, while image inference remains
   FP16. See `phase1-query-switch-fp16-clip-fix.md` for the failure analysis,
   regression evidence, and live verification boundary.
8. Stop RViz and every test-owned ROS process after collecting evidence, then
   verify that no test-owned node remains.

### Current Jetson execution path

On the 2026-07-27 Jetson run, `semantic_search_ctl start` produced a false
readiness failure: its first sequential readiness snapshot began while the
drivers were still warming up, consumed the total deadline, and then stopped
an otherwise healthy stack. Until that condition-based readiness defect is
fixed and regression-tested, use the aggregate launch directly for official
Phase 3 hardware tests:

```bash
ros2 launch track_robot_bringup semantic_search_live.launch.py \
  stage:=phase3 \
  start_camera:=true \
  start_lidar:=true \
  start_base:=true \
  start_imu:=true \
  extrinsic_mode:=prototype \
  allow_degraded:=true
```

This still has one launch owner and uses the official `zed_wrapper`; it is not
a UVC fallback. Keep the launch in the foreground so `Ctrl+C` stops everything
it owns.

### Official ZED camera gate

When the managed stack owns the camera, inspect its launch log for
`Camera successfully opened`, then create an active subscriber:

```bash
timeout 15 ros2 topic hz /zed/zed_node/left/image_rect_color
ros2 topic info --verbose /zed/zed_node/left/image_rect_color
timeout 10 ros2 run tf2_ros tf2_echo \
  base_link zed_left_camera_optical_frame
```

The first command is expected to report more than 5 Hz after warm-up. Keep the
sample and the transform output as report evidence. If one gate fails, report
the exact failed gate and stop; do not silently change camera implementations.

## 1. Build and preflight

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-up-to \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup \
  track_robot_semantic_search_rviz_plugins
source install/setup.bash
export ROS_DOMAIN_ID=20
```

The managed preflight checks the isolated YOLO-World and CLIP runtimes, the
YOLO-World, CLIP and DINO checkpoints, the local DINO source, CUDA, DDS, stage
topics, TF, calibration and the Phase 3 safe profile:

```bash
ros2 run track_robot_bringup semantic_search_ctl doctor phase1 \
  --hardware external
```

The expected model hashes are:

```text
yolov8s-worldv2.pt
9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792

ViT-B-32.pt
40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af

dinov3_vits16plus_pretrain_lvd1689m.pth
4057cbaaad8c16657adb09d6815f28d4164eeba30532fde23f0d17313124caea
```

## 2. Start one explicit stage

Phase 0 starts only passive evaluation and localization-health contracts:

```bash
ros2 run track_robot_bringup semantic_search_ctl start phase0 \
  --hardware external
```

Phase 1 starts or reuses only the camera, then runs YOLO-World localization
and DINO visual identity extraction:

```bash
ros2 run track_robot_bringup semantic_search_ctl start phase1 \
  --hardware auto
```

Phase 2 adds LiDAR, odometry, IMU, localization health, LiDAR tracklets and the
production-safe semantic-memory profile. A measured
`base_link -> zed_camera_link` transform is required:

```bash
EXTRINSIC=~/track_robot_ws/config/camera_extrinsic.measured.yaml
ros2 run track_robot_bringup semantic_search_ctl start phase2 \
  --hardware auto \
  --extrinsic-mode measured --extrinsic-file "$EXTRINSIC"
```

Phase 3 uses the explicit uncalibrated `phase123_test.yaml` profile. It allows
camera-owned objects and diagnostic camera/LiDAR attachment, but it does not
enable the production winner or re-identification mutation:

```bash
EXTRINSIC=~/track_robot_ws/config/camera_extrinsic.measured.yaml
ros2 run track_robot_bringup semantic_search_ctl start phase3 \
  --hardware auto \
  --extrinsic-mode measured --extrinsic-file "$EXTRINSIC"
```

If only a rough prototype transform is available, diagnostic work may be
started with `--extrinsic-mode prototype --allow-degraded`. Such a run is
explicitly `DEGRADED`, is not calibration evidence, and cannot release a
production target.

## 3. Enter an English query

Use a short visible description, for example:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run track_robot_bringup semantic_search_ctl query \
  "blue toothpaste container"
```

The accepted response contains one positive query ID and version. The current
portal deliberately accepts printable English/ASCII only. Keeping one
language avoids an extra multilingual translation path; it does not change
the YOLO-World model's core compute cost.

The 2026-07-27 Jetson run also showed that the query portal can time out while
the perception node has already activated the same query. Treat the portal
result and model result as separate gates. Model acceptance requires a
`perception_diagnostics` message with `state=active`, the submitted positive
query ID/version and `model_ready=true`, followed by a correlated
`SemanticRegionArray`. A portal timeout is still a portal failure and must be
reported; it is not proof that perception rejected the query.

## 4. Observe expected output

Open the stage-specific passive RViz view:

```bash
ros2 launch track_robot_bringup \
  semantic_search_visualization.launch.py stage:=phase3
```

For the current Jetson checkpoint this direct visualization launch is the
validated path. Require one publisher on `/semantic_search/overlay_image`;
an RViz window by itself is not a visualization pass.

The panel always displays:

```text
UNCALIBRATED - NOT A CONFIRMED TARGET
```

Check these topics:

```bash
timeout --signal=INT 10 ros2 topic echo --full-length \
  --qos-reliability reliable /semantic_search/perception_diagnostics
timeout --signal=INT 10 ros2 topic echo --full-length \
  --qos-reliability reliable /semantic_search/regions
timeout --signal=INT 10 ros2 topic echo --full-length \
  --qos-reliability reliable /semantic_memory/observations
timeout --signal=INT 10 ros2 topic echo --full-length \
  --qos-reliability reliable /semantic_memory/active_objects
timeout --signal=INT 10 ros2 topic echo --full-length \
  --qos-reliability reliable /semantic_memory/diagnostic_ranking
timeout --signal=INT 10 ros2 topic echo --full-length \
  --qos-reliability reliable /semantic_memory/best_candidate
timeout --signal=INT 10 ros2 topic echo --full-length \
  --qos-reliability reliable /semantic_memory/diagnostics
```

ROS 2 Foxy has no `ros2 topic echo --once` option. Do not copy one-shot
commands from newer ROS distributions into this test.

Expected progression:

1. Phase 1 publishes correlated regions for the accepted query and stable
   camera track IDs across adjacent frames.
2. Phase 2/3 may create a `CAMERA_ONLY` memory object before LiDAR geometry is
   available.
3. A gated LiDAR association may change the same global object to
   `CAMERA_LIDAR`; its global ID must not change.
4. Similar distractors must retain separate camera/global IDs.
5. `/semantic_memory/diagnostic_ranking` may contain ordered candidates with
   query ID/version, support mode and score.
6. An empty diagnostic ranking is a valid abstention.
7. `/semantic_memory/best_candidate` must remain empty while
   `best_candidate_threshold_calibrated` is false.

The score is diagnostic ordering evidence, not a calibrated probability or
proof that the physical object is correct.

## 5. Capture a bounded test report

With an already-running Phase 3 stack:

```bash
ros2 run track_robot_bringup semantic_search_ctl test phase3 \
  "blue toothpaste container" \
  --hardware external \
  --extrinsic-mode measured --extrinsic-file "$EXTRINSIC" \
  --duration-sec 15
```

Reports are written below
`~/.ros/track_robot_semantic_search/reports/<timestamp>/`. A healthy unlabelled
run says `Pipeline: PASS` and `Semantic result: REVIEW REQUIRED`.

Run the deterministic software-only contract replay separately:

```bash
ros2 run track_robot_semantic_search semantic_search_phase123_replay \
  --executable \
    install/track_robot_semantic_memory/lib/track_robot_semantic_memory/semantic_memory_replay \
  --input \
    src/track_robot_semantic_search/test/data/phase123_yolo_world_replay.json \
  --output /tmp/phase123_output.json \
  --report /tmp/phase123_report.json
```

This replay verifies data contracts and identity continuity; it does not
measure live model accuracy or sensor calibration.

## 6. Stop everything owned by the test

Press `Ctrl+C` in the foreground stack terminal, or run:

```bash
ros2 run track_robot_bringup semantic_search_ctl stop
```

Then verify no test-owned node remains:

```bash
ROS_DOMAIN_ID=20 ros2 node list
ROS_DOMAIN_ID=20 ros2 topic info /cmd_vel
pgrep -af 'zed|rslidar|bunker|semantic_search|semantic_memory|rviz2'
```

Do not leave camera, LiDAR, semantic-search or memory services running after
the test unless another explicitly managed workflow owns them.

## Remaining physical gates

The following remain pending and must not be inferred from this software
checkpoint:

- live target accuracy and hard-negative evaluation;
- measured camera/LiDAR extrinsic evidence;
- field camera/LiDAR association precision and recall;
- re-entry/re-identification calibration;
- 30-minute resource and stability run;
- deployment/model licensing approval;
- calibrated production `best_candidate` threshold;
- any Phase 4 motion or search-policy authorization.
