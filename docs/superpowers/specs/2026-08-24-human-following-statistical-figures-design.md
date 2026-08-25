# Human-Following Statistical Figures Design

## Scope

Produce five paper-ready figures from the four existing human-tracking bags:

1. aggregate camera–LiDAR association statistics;
2. Kalman/IMM statistical consistency;
3. a four-bag benchmark;
4. five-run replay repeatability;
5. an episode-level association funnel.

Semantic Search, physical motion, Point-LIO, and new sensor acquisition are out
of scope. The work starts no live sensor and publishes no motion command.

## Evidence model

A single ROS capture utility records synchronized fused states, camera-guided
anchors, LiDAR tracklets, camera targets, camera intrinsics, and tracker debug
JSON. Each replay becomes a self-contained JSON record. The four-bag benchmark
uses one run per bag; repeatability uses five independent launches of
`human_tracking_lidar_20260706_145900` at the same playback rate.

Association geometry is recomputed from synchronized messages using the same
direct LiDAR-to-camera optical transform and configured gates as the runtime.
Published association scores, NIS, rejection reasons, and IMM probabilities
are copied from `/human_tracking/target_tracker_debug` rather than estimated.

## Figures

### Aggregate association statistics

Show distributions or CDFs for anchor XY distance, range difference,
projection-centre error, association score, and top-two score margin. Values
are internal consistency measurements, not ground-truth position errors.

### Kalman/IMM statistical consistency

Show NIS against the configured camera-anchor and tracklet thresholds,
accepted/rejected measurement counts, rejection reasons, covariance trace,
and the three IMM model probabilities.

### Four-bag benchmark

Compare bag duration, target lock, confirmed association duration, source-state
occupancy, selected-tracklet switches, and safe release. Bags without a lock
remain in the table as measured failures; they are not omitted.

### Replay repeatability

Align five independent runs by the first locked target state. Compare physical
XY trajectories, range, association duration, success, and selected internal
tracklet IDs. The figure must state that tracklet numbers are run-local.

### Association funnel

Start the monotonic causal funnel at confirmed candidate evaluations, followed
by the anchor and range gates, valid projection, published hypotheses, score
threshold, and selected target. Report raw candidate clusters and active-track
observations separately as upstream context totals: track persistence means
they are not one-to-one causal stages of a synchronized debug update. Use the
configured `min_association_score: 0.65`.

## Outputs and integrity

Each figure has PNG, vector PDF, and JSON provenance in
`docs/assets/paper/results/`. Raw replay summaries live under
`docs/assets/paper/results/data/human-following-statistics/`. Offline tests use
synthetic records to validate metric aggregation, monotonic funnels, alignment,
and truthful empty-run handling. No root README change or push is included.
