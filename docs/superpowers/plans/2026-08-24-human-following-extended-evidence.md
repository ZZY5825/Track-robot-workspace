# Human-Following Extended Evidence Figures Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate five evidence-backed human-following paper figures covering continuity, deployment resources, software safety, platform envelopes, and estimator-input ablation.

**Architecture:** Pure Python analysis functions transform normalized replay, telemetry, test, and URDF records into plotting data. One bounded ROS replay captures the only missing resource and measurement sequence; a separate renderer creates every PNG/PDF/JSON triplet.

**Tech Stack:** ROS 2 Foxy, Python 3, NumPy, Matplotlib, pytest, JSON, tegrastats, URDF XML.

## Global Constraints

- Do not start live sensors or publish robot motion.
- Do not consume Semantic Search data.
- Replay bag `145900` at rate `0.5`.
- Label all non-ground-truth comparisons as consistency, continuity, or mutual deviation.
- Label PiPER reach as kinematic, not collision-free.
- Label the L515 as visual-model only.
- Preserve PNG, vector PDF, and JSON provenance for every figure.

---

### Task 1: Pure extended-evidence analysis

**Files:**
- Create: `tools/visualization/human_following_extended_analysis.py`
- Create: `tools/visualization/tests/test_human_following_extended_analysis.py`

**Interfaces:**
- Consumes: normalized replay records, tegrastats lines, process snapshots, URDF XML, and synchronized measurement sequences.
- Produces: `build_continuity_lanes`, `parse_tegrastats_line`, `summarize_resources`, `sample_urdf_workspace`, and `run_estimator_ablation`.

- [ ] Write failing tests for state-lane alignment, tegrastats parsing, baseline-adjusted resource summaries, bounded URDF samples, masked-measurement continuity, and finite ablation metrics.
- [ ] Run the focused test file and confirm failure because the module is absent.
- [ ] Implement the minimal pure functions with finite-value filtering and deterministic random seeds.
- [ ] Re-run the focused tests and confirm they pass.

### Task 2: Extended replay capture

**Files:**
- Create: `tools/visualization/capture_human_following_extended_evidence.py`
- Create: `docs/assets/paper/results/data/human-following-extended/human_tracking_lidar_20260706_145900-extended.json`

**Interfaces:**
- Consumes: camera-guided anchors, selected tracklets, fused target state, tracker debug, camera info, tegrastats, and process statistics.
- Produces: synchronized measurements, production states, topic rates, idle/replay device telemetry, and per-node resource samples.

- [ ] Implement bounded ROS subscriptions and 120 ms nearest-stamp synchronization.
- [ ] Capture an idle telemetry baseline, then replay bag `145900` once at rate `0.5` with a fresh tracking launch.
- [ ] Validate the JSON contains finite telemetry, both measurement sources, production IMM samples, and recorded camera intrinsics.
- [ ] Verify no replay, launch, transform, or collector process remains.

### Task 3: Safety-test evidence

**Files:**
- Create: `docs/assets/paper/results/data/human-following-extended/safety-test-evidence.json`

**Interfaces:**
- Consumes: focused safety/decision unit, contract, and launch tests.
- Produces: scenario-level test identifiers, commands, pass status, and verified output contracts.

- [ ] Run focused tests for input staleness, target loss, RC override, emergency stop, base fault, planner/command staleness, and obstacle-cloud staleness.
- [ ] Record exact test commands, passed counts, and scenario-to-contract mappings without claiming physical testing.
- [ ] Validate every rendered PASS cell names at least one passing test identifier.

### Task 4: Render five figures

**Files:**
- Create: `tools/visualization/render_human_following_extended_figures.py`
- Create: `docs/assets/paper/results/human-following-perception-continuity-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/human-following-deployment-resources-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/human-following-safety-fault-matrix-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/robot-sensing-manipulation-envelope-v1.{png,pdf,json}`
- Create: `docs/assets/paper/results/human-following-estimator-ablation-v1.{png,pdf,json}`

**Interfaces:**
- Consumes: Task 1 functions and Task 2–3 evidence records.
- Produces: five publication figure triplets.

- [ ] Implement a CLI with explicit replay, safety, URDF, and output paths.
- [ ] Render the continuity lanes with exact transition times and release marker.
- [ ] Render device-wide and per-node resource distributions with baseline disclosure.
- [ ] Render the software safety matrix with test-layer provenance.
- [ ] Render top/side sensor and kinematic envelopes with model/calibration limitations.
- [ ] Render the four-condition estimator-input ablation with no accuracy claim.
- [ ] Visually inspect all five PNGs and correct any overlap or misleading annotation.

### Task 5: Catalog and verification

**Files:**
- Modify: `docs/assets/paper/results/README.md`

**Interfaces:**
- Consumes: all generated assets and evidence sidecars.
- Produces: catalog entries and final integrity evidence.

- [ ] Add all five figures and their evidence limitations to the catalog.
- [ ] Run focused pytest, py_compile, JSON assertions, `file` on all PNG/PDF outputs, and `git diff --check`.
- [ ] Confirm one extended replay, seven safety scenarios, deterministic workspace sampling, four ablation conditions, and no residual ROS processes.
