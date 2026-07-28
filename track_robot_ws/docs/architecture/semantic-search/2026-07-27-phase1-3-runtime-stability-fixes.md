# Phase 1–3 runtime stability fixes

Date: 2026-07-27

## Objective

Make the existing Phase 1–3 pipeline fail safely and remain usable on Jetson
Orin without replacing its architecture.

## Confirmed root causes

1. The isolated Ultralytics runtime leaves a process-global `torch.load`
   wrapper installed after YOLO-World initialization. On the Jetson PyTorch
   1.13 build, that wrapper forwards the unsupported `weights_only` keyword to
   Python's unpickler. DINOv3 therefore fails when it loads after YOLO-World.
2. The query portal waits only for the short-lived `query_accepted`
   diagnostic. A correlated `active` diagnostic with `model_ready=true`
   already proves that the query was accepted and frames are being processed,
   but the portal ignores it and reports a false timeout.
3. The semantic-memory ROS callback forwards duplicate active LiDAR tracklet
   IDs into a strict association function. The strict function correctly
   rejects ambiguous input, but its exception is not contained at the callback
   boundary, so one malformed batch terminates the Phase 3 node.

## Selected design

- Restore the original system `torch.load` after the YOLO-World checkpoint is
  constructed. The Ultralytics wrapper is needed only around its own model
  loading and must not leak into DINOv3.
- Accept either a correlated `query_accepted` diagnostic or a correlated
  `active` diagnostic with `model_ready=true` as portal success. Rejections
  remain authoritative.
- Before camera–LiDAR scoring, count active tracklet IDs and exclude every
  occurrence of a duplicated ID. Do not choose an arbitrary duplicate.
- Keep the association core strict. Add a ROS callback exception boundary so
  any remaining invalid association batch is rejected without killing the
  semantic-memory process.

## LiDAR and visualization boundary

`MarkerArray` remains the RViz presentation of semantic positions. It is not
the memory contract because it does not preserve stable identity, timestamps,
uncertainty, lifecycle, or association evidence. The internal structured
tracklet/object messages remain for Phase 3; a later simplification may derive
a local-map position directly from a camera ROI and depth/LiDAR evidence, then
publish that position as markers.

## Acceptance criteria

- The portal returns success for a matching active, model-ready query instead
  of timing out.
- Loading YOLO-World no longer leaves the incompatible global PyTorch loader
  installed; DINOv3 can initialize afterwards.
- A duplicate LiDAR tracklet ID cannot terminate semantic memory.
- A later valid observation is still processed after malformed association
  input.
- The official ZED2i Phase 1–3 bounded test runs in `ROS_DOMAIN_ID=20`, RViz
  receives its visualization streams, and all test nodes are stopped
  afterwards.

## Validation status

The model, portal, and Phase 3 crash criteria pass in unit, integration, and
Jetson model-load tests. Hardware launch also keeps semantic memory alive.
The final managed live-test PASS remains blocked by inconsistent Foxy ROS 2
CLI daemon discovery, not by the Phase 3 callback. The next test-infrastructure
change is one persistent `rclpy` readiness observer instead of a sequence of
short-lived `ros2 topic` subprocesses.
