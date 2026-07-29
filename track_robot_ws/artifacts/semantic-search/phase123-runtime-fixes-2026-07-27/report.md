# Phase 1–3 runtime stability report

Date: 2026-07-27
ROS domain: 20

## Result

The three confirmed software faults are fixed:

- YOLO-World no longer leaves the incompatible Ultralytics `torch.load`
  wrapper installed, and DINOv3 loads successfully in the same process.
- The text query portal accepts a correlated `active` and
  `model_ready=true` diagnostic instead of reporting a false timeout.
- Duplicate active LiDAR tracklet IDs are excluded from visual association,
  and invalid shortlist input cannot terminate the Phase 3 semantic-memory
  callback.

## Verification evidence

- Semantic-search package suite: `743 tests, 0 failures`.
- Semantic-memory package suite: `221 tests, 0 failures, 4 opt-in ROS
  runtime tests skipped` in the default run.
- Duplicate-ID ROS runtime regression: `1 passed`; a valid observation was
  processed after the malformed batch.
- Bringup/control package suite after readiness fixes: `166 tests,
  0 failures`.
- Direct Jetson model load in one process:

  ```text
  YOLO_WORLD_READY YOLOWorld
  DINO_READY True DinoVisionTransformer
  ```

- Bounded hardware starts recognized ZED2i serial `37617639`, started
  YOLO-World, DINO-capable perception, LiDAR tracking, semantic memory, and
  Phase 3 publishers. The semantic-memory node remained alive until managed
  shutdown.
- `/cmd_vel` had zero publishers throughout the checks.

## Hardware-test qualification

The live runs used a prototype `base_link -> zed_camera_link` transform, so
cross-modal geometry remains `DEGRADED` and is not calibration evidence.

The bounded live-test controller still cannot issue a final PASS on this Foxy
installation because its short-lived `ros2 topic` subprocesses use a stale or
inconsistent ROS 2 CLI daemon graph. The daemon simultaneously retained
already-stopped LiDAR nodes and failed to discover live camera/semantic
publishers. It was stopped after the run, and the refreshed Domain 20 graph
was empty.

This is a test-observer problem, not a repeat of the Phase 3
`visual shortlist pair is invalid or duplicate` crash. Replacing the
short-lived CLI probes with one persistent `rclpy` readiness node is the
remaining infrastructure task before the managed hardware test can provide a
reliable end-to-end PASS.

## Cleanup

All ROS nodes and sensor services started by the bounded tests were stopped.
The ROS 2 CLI daemon was restarted/cleared, and the final Domain 20 graph was
empty.
