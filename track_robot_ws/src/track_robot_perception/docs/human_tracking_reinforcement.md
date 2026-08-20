# Early-Phase Human Tracking Reinforcement

## Active Architecture

The camera assigns semantic identity and publishes `CameraTarget`. The generic
LiDAR manager publishes frame-generic tracklets. The selected-target tracker
associates one logical camera target with LiDAR geometry and publishes a fused
`TargetState` in `base_link`.

Live tracking runs in `track_robot_center`. Stationary rosbag replay uses
`base_link` explicitly through `human_tracking_rosbag_replay.launch.py`.

## Algorithm Changes

- Camera reacquisition combines torso HSV appearance, normalized torso pose,
  bbox motion, and bbox size. A new visual track ID requires two confirmed
  observations and a top-two score margin.
- LiDAR tracklet prediction and lifecycle use the original cloud timestamp.
  Tracklet arrays identify the coordinate frame in their header and publish
  observation covariance and quality.
- Camera targets and LiDAR clouds are retained in bounded queues and paired by
  nearest sensor timestamp. Pairing beyond 80 ms is rejected; data older than
  200 ms cannot be selected. A late camera message may complete one pending
  cloud pair, but out-of-sequence filter correction is prohibited.
- Camera-guided extraction uses 0.15 m contiguous depth modes inside the pose
  torso ROI. The selected mode must agree with the existing depth prediction.
  A local XY component is grown before computing a median 3D anchor.
- Camera-to-tracklet association retains up to three hypotheses. Initial bind,
  visible switching, and LiDAR-only relinking all require score margins and
  repeated evidence. Ambiguous measurements do not correct the target filter.
- The selected target uses a three-model IMM estimator: stationary, walking
  constant velocity, and maneuvering. The model acceleration standard
  deviations are 0.15, 0.8, and 2.5 m/s^2.
- Height is a separate robust scalar filter. It does not affect horizontal IMM
  model probabilities and rejects vertical jumps larger than 0.8 m.
- Cloud parsing, TF, crop, and voxel accumulation run in one traversal. The
  depth-one LiDAR subscription drops stale queued frames under overload.
- Timestamp regression clears temporal state at rosbag loop boundaries.

## Important Debug Outputs

```text
/human_tracking/camera_identity_debug
/human_tracking/fusion_timing_debug
/human_tracking/association_hypothesis_markers
/human_tracking/target_tracker_debug
```

The selected physical tracklet marker represents a measured generic tracklet.
The selected target marker represents the logical filtered target and remains
visible during bounded prediction. Missing physical geometry is shown as a
status label instead of silently clearing all target visualization.

## Ownership Boundary

Perception owns detection, gesture-selected identity, camera/LiDAR association,
and the trusted fused target state with confidence and uncertainty. It does not
authorize robot motion, choose a safe path, arm the base, or publish final
velocity commands. Ambiguous or unsupported evidence must remain ambiguous,
prediction-only, or lost for the decision layer to handle.

## RC Takeover Contract

The live following stack uses the same fail-closed takeover semantics as the
autonomous-approach stack:

- Bunker `control_mode == 3` is an authoritative RC takeover, even when every
  RC stick is centered. Stick movement remains a redundant early indication.
- RC takeover makes the motion safety supervisor publish zero velocity and
  disarm. Returning to CAN mode does not re-arm it.
- The decision node calls `/human_tracking/reset_target` when it first enters
  `RC_OVERRIDE`. Camera and LiDAR detections may continue, but the previous
  gesture-authorized logical target is discarded.
- After returning to CAN mode, the operator must perform a new start gesture
  and explicitly call `/safety/arm` before autonomous following can move.

This prevents a previous target and motion command from resuming after manual
control. It does not claim that the Bunker SDK stops transmitting CAN frames;
the enforced contract is that the safe/final velocity command is zero.

## Validation

Run the regression monitor during replay:

```bash
ros2 run track_robot_perception human_tracking_regression_monitor --duration 30
```

The report includes logical ID changes, physical tracklet changes, fused and
marker rates, ambiguity count, synchronization offset, and output position
continuity. Physical tracklet changes are not automatically failures, but each
change must correspond to confirmed relinking in target-tracker debug output.

Use `human_tracking_compare_runs` with reports from 0.5x, 1x, and 2x replay to
check identity, selected-tracklet sequence, synchronization, and rate invariants.
