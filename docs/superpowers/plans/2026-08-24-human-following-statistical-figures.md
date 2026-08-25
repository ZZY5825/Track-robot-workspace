# Human-Following Statistical Figures Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate five evidence-backed statistical figures from the existing human-following bags.

**Architecture:** A ROS collector serializes one normalized record per replay. A pure offline analysis/render module consumes those records for every figure, while unit tests exercise aggregation without ROS.

**Tech Stack:** ROS 2 Foxy, Python 3, NumPy, Matplotlib, pytest, JSON.

## Global Constraints

- Do not start live sensors or publish motion.
- Do not consume Semantic Search data.
- Use playback rate `0.5` for all compared runs.
- Call geometric residuals internal consistency, never ground-truth accuracy.
- Use configured association score threshold `0.65`.
- Preserve PNG, vector PDF, and JSON provenance for every figure.

---

### Task 1: Pure statistical aggregation

**Files:**
- Create: `tools/visualization/human_following_statistics.py`
- Create: `tools/visualization/tests/test_human_following_statistics.py`

**Interfaces:**
- Consumes: normalized replay dictionaries.
- Produces: `summarize_run(record)`, `aggregate_association(records)`, `aggregate_funnel(records)`, and `align_repeatability(records)`.

- [x] Write synthetic tests proving empty bags remain visible, funnel counts are monotonic, thresholds use `0.65`, and aligned time starts at zero.
- [x] Run the focused tests and confirm the tests fail because the module is absent.
- [x] Implement the four pure functions with finite-value filtering and explicit sample counts.
- [x] Re-run with Python 3 and plug-in autoload disabled; confirm all five tests pass.

### Task 2: ROS replay collector

**Files:**
- Create: `tools/visualization/capture_human_following_statistics.py`

**Interfaces:**
- Consumes: `/human_tracking/fused_target_state`, `/human_tracking/lidar_tracklets`, `/human_tracking/camera_guided_target_points`, `/human_tracking/camera_target`, `/zed/zed_node/left/camera_info`, and `/human_tracking/target_tracker_debug`.
- Produces: one JSON record containing `states`, `associations`, `debug_samples`, `funnel_updates`, and `summary`.

- [x] Implement bounded ROS subscriptions and nearest-stamp synchronization at 120 ms for state/tracklet/anchor and 200 ms for camera data.
- [x] Recompute anchor distance, range difference, projection error, and per-stage funnel counts using the runtime transform and gates.
- [x] Copy published score, hypotheses, NIS, rejection reason, measurement acceptance, and IMM probabilities into normalized samples.
- [x] Run `python3 -m py_compile tools/visualization/capture_human_following_statistics.py`.

### Task 3: Capture benchmark and repeatability records

**Files:**
- Create: `docs/assets/paper/results/data/human-following-statistics/*.json`

**Interfaces:**
- Consumes: the four existing bags and the collector from Task 2.
- Produces: four benchmark records plus four additional independent repeats of the `145900` bag, for five total `145900` runs.

- [x] Replay each of the four bags at rate `0.5` with a fresh tracking launch and fixed `base_link -> rslidar` transform.
- [x] Replay `human_tracking_lidar_20260706_145900` four additional times with fresh nodes.
- [x] Validate every JSON parses, names its bag/run, contains topic sample counts, and records unsuccessful locks instead of failing capture.
- [x] Verify no replay, launch, or static-transform process remains.

### Task 4: Render five figures

**Files:**
- Create: `tools/visualization/render_human_following_statistical_figures.py`
- Create: `docs/assets/paper/results/human-following-association-statistics-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/human-following-filter-consistency-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/human-following-four-bag-benchmark-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/human-following-replay-repeatability-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/human-following-association-funnel-v1.{png,pdf,json}`

**Interfaces:**
- Consumes: validated replay JSON records and Task 1 aggregation functions.
- Produces: five publication-ready figure triplets.

- [x] Implement a CLI accepting `--data-dir` and `--output-dir`.
- [x] Render the association distributions and annotate exact sample counts and configured gates.
- [x] Render NIS/IMM/covariance/rejection panels with separate camera and tracklet NIS thresholds.
- [x] Render all four bags in benchmark plots and an evidence table.
- [x] Render five aligned runs with run-local tracklet ID disclosure.
- [x] Render a monotonic episode-level association funnel.
- [x] Run the renderer and visually inspect all five PNGs.

### Task 5: Verification and catalog

**Files:**
- Modify: `docs/assets/paper/results/README.md`

**Interfaces:**
- Consumes: all outputs from Tasks 1–4.
- Produces: catalog entries and evidence-backed completion checks.

- [x] Add the five figures and their evidence limitations to the catalog.
- [x] Run focused pytest, py_compile, JSON assertions, `file` on PNG/PDF outputs, and `git diff --check`.
- [x] Confirm the score threshold is `0.65`, all funnel stages are monotonic, there are four benchmark bags and five repeat runs, and no ROS process remains.
