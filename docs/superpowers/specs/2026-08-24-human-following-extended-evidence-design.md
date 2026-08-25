# Human-Following Extended Evidence Figures Design

## Goal

Produce the remaining five publication figures previously selected for the
human-following paper narrative, without using Semantic Search data or claiming
external tracking accuracy.

## Evidence strategy

Use one additional offline replay of
`human_tracking_lidar_20260706_145900` at rate `0.5`. The replay launches only
the human-tracking algorithms and records resource telemetry plus synchronized
camera-guided-anchor, selected-LiDAR, and production fused-state measurements.
It does not start physical sensors or publish mobile-base or arm commands.

Existing normalized replay JSON supports the continuity timeline. Existing
automated safety tests provide the safety-matrix evidence. The integrated URDF,
PiPER joint limits, recorded ZED `CameraInfo`, and configured LiDAR range support
the platform-envelope figure.

## Figures

### Perception continuity timeline

Render the primary logical-target episode from bag `145900` as aligned state
lanes: camera visibility, LiDAR visibility, measurement source, association
state, lock state, accepted measurements, selected run-local tracklet, and safe
release. State transitions use exact replay timestamps.

### Deployment resources

Record Jetson-wide CPU, GPU, RAM, temperature, and EMC telemetry with
`tegrastats`, plus per-node CPU/RSS samples and observed topic/debug update
rates. Compare a bounded idle baseline with the replay workload. Device-wide
numbers are not attributed exclusively to the tracking stack.

### Safety fault matrix

Run focused automated tests covering stale required inputs, target loss, RC
override, software emergency stop, base fault, stale planner/command input, and
stale obstacle-cloud input. Show the verified outcomes—session revocation,
disarm, zero safe command, target reset, or fail-closed state—and distinguish
unit, contract, and launch-test evidence. This is software verification, not a
physical fault-injection experiment.

### Sensor and manipulator envelope

Show top and side views of the Bunker footprint, ZED recorded horizontal and
vertical field of view, RS-Helios configured tracking range, PiPER kinematic
end-effector reach, and the L515 visual-model pose envelope. Sample the six
revolute joints within the authoritative URDF limits. The result is a
kinematic envelope, not a collision-free manipulation workspace. The L515 is
visual-model only, so no operational/calibrated L515 field of view is claimed.

### Estimator-input ablation

Replay a shared synchronized measurement sequence through four comparable
conditions: camera-guided anchor only, selected LiDAR tracklet only, both
measurements with a single constant-velocity Kalman filter, and the production
three-model IMM output. Share the production association/target authorization
across conditions; therefore this is an estimator-input ablation, not an
end-to-end detector/association ablation. Compare continuity, covariance,
trajectory smoothness, and mutual trajectory deviation. Do not call mutual
deviation ground-truth error or localization accuracy.

## Outputs

Each figure is written as PNG, vector PDF, and JSON provenance under
`docs/assets/paper/results/`:

- `human-following-perception-continuity-v1`
- `human-following-deployment-resources-v1`
- `human-following-safety-fault-matrix-v1`
- `robot-sensing-manipulation-envelope-v1`
- `human-following-estimator-ablation-v1`

Raw extended replay and test records live under
`docs/assets/paper/results/data/human-following-extended/`.

## Integrity requirements

- No Semantic Search evidence is consumed.
- No live sensor, mobile-base command, or arm command is started.
- All figure captions disclose evidence source and limitations.
- Pure analysis functions are developed test-first.
- The five PNGs are visually inspected; all output triplets and JSON invariants
  are verified before handoff.
