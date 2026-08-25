# Paper figure catalog

Each figure is provided as a review PNG, a vector PDF, and—where evidence is
quantitative—a JSON sidecar recording its provenance and measurements. These
assets are intentionally separate from the GitHub README figures.

## Measured or recorded evidence

| Figure | Evidence source | Intended use |
| --- | --- | --- |
| `human-following-replay-evidence` | synchronized replay of `human_tracking_lidar_20260706_145900` | compact qualitative camera–LiDAR confirmation |
| `camera-lidar-association-v1` | synchronized camera, cloud, tracklet, anchor, and fused state | detailed projection and metric association evidence |
| `association-candidate-ranking-v1` | synchronized messages plus published association hypotheses | hard-gate rejection and selected-tracklet score |
| `human-following-sequence-v1` | five states from one recorded target episode | target lock, motion, fusion continuity, and safe release |
| `target-estimation-dynamics-v1` | fused state, selected tracklet, camera anchor, and IMM debug | trajectory, uncertainty, confidence, source state, and IMM probabilities |
| `human-tracking-processing-latency-v1` | wall-clock node debug timing during replay | camera, LiDAR-tracklet, and fusion processing distributions |
| `human-following-association-statistics-v1` | four independent human-following bag replays | camera-anchor, range, projection, and published-score consistency |
| `human-following-filter-consistency-v1` | primary target episode from `human_tracking_lidar_20260706_145900` | innovation gates, measurement decisions, covariance, and IMM posterior |
| `human-following-four-bag-benchmark-v1` | four independent fresh launches at replay rate 0.5× | lock timing, association duration, source occupancy, identity continuity, and release evidence |
| `human-following-replay-repeatability-v1` | five fresh launches of `human_tracking_lidar_20260706_145900` | aligned trajectories, range dispersion, timing stability, and run-local identity |
| `human-following-association-funnel-v1` | synchronized debug updates from four bag replays | confirmed-candidate gate retention through final selected-target matching |
| `human-following-perception-continuity-v1` | primary locked-target episode from `human_tracking_lidar_20260706_145900` | sensor availability, fusion source, association state, run-local LiDAR identity, and safe release |
| `human-following-deployment-resources-v1` | one 0.5× replay plus Jetson `tegrastats` and per-process `/proc` sampling | device utilization, memory, thermals, algorithm-node CPU/RSS, and observed topic rates |
| `human-following-safety-fault-matrix-v1` | 54 passing automated safety and decision tests | explicitly asserted responses to seven injected software fault or hazard scenarios |
| `robot-sensing-manipulation-envelope-v1` | integrated Bunker + PiPER URDF, recorded ZED intrinsics, and configured LiDAR tracking range | sensing coverage and deterministic 30,000-sample kinematic workspace |
| `human-following-estimator-ablation-v1` | synchronized measurement sequence from one replay | input-masked estimator continuity, uncertainty, smoothness, and mutual deviation |
| `semantic-search-evidence-v1` | 2026-07-27 Phase 1 overlay and 2026-07-28 Phase 0–4A validation report | qualitative proposal plus end-to-end semantic-search evidence |
| `experimental-platform-configuration-v1` | integrated project URDF rendered in RViz | annotated digital platform configuration |

The processing-latency figure reports node callback duration, not
sensor-to-actuator latency. The semantic-search figure labels its two dated
validation sources separately and does not imply they are the same run.
Association and filter-consistency plots report internal estimator consistency,
not error against external trajectory ground truth. A release marked “not
observed” means the recording window ended before a `NO_TARGET` state; it is not
classified as an unsafe release failure.

The deployment-resource trace is a device-wide measurement and therefore
includes other host workloads; its three process panels isolate only the human
tracking nodes. The safety matrix reports automated software assertions, not
physical fault injection or functional-safety certification. The PiPER envelope
is a joint-limit sample and is not collision-free, while the arm-mounted L515 is
represented by its visual model only. The estimator ablation reuses production
target authorization and association and reports mutual consistency against the
production IMM, not error against external ground truth.

## Protocol figures awaiting dedicated data

| Figure | Current status | Missing evidence |
| --- | --- | --- |
| `human-following-evaluation-protocol-v1` | `DATA_COLLECTION_PENDING` | controlled occlusion, multi-person crossing, odometry, session state, and command traces |
| `point-lio-calibration-protocol-v1` | `CALIBRATION_DATA_PENDING` | measured LiDAR–IMU extrinsic, time-offset sweep bag, trajectory closure, and map output |

These two figures are methods/protocol schematics, not measured result plots.
Their JSON sidecars explicitly set `measured_results` or
`measured_mapping_result` to `false`.

## Reproduction

The scripts in `tools/visualization/` render the static figures and capture the
ROS replay figures. Replay uses recorded topics only; it does not start any
physical sensor or command the mobile base.

The eight JSON replay records supporting the five statistical figures are in
`data/human-following-statistics/`: four first-run benchmark records plus five
fresh-launch repeats of bag `145900` (the first repeat is shared by both sets).
The combined resource and estimator-input record, together with exact automated
safety-test provenance, is in `data/human-following-extended/`.
