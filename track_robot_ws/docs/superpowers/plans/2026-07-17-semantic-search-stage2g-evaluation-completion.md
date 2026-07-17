# Semantic Search Stage 2G Evaluation and Completion Plan

> **Execution rule:** implement directly through the software-tooling gate;
> never substitute synthetic evidence for the physical completion gate.

**Goal:** Deliver strict, deterministic Phase 2 completion tooling and produce
the strongest truthful Stage 2G checkpoint supported by available evidence.

**Architecture:** Preserve the existing manifest, normalized replay and
evaluation entry points. Add an independent task-threshold calibration module,
harden evaluation contracts around approved gates, and bind the final report to
frozen calibration evidence. Runtime ROS behavior remains unchanged.

## Task 1: Freeze design and baseline

- [x] Compare the master acceptance gates, Phase 2 design, Stage 2 roadmap and
  current evaluator.
- [x] Run the existing four-package Stage 2F baseline.
- [x] Write the Stage 2G design and this implementation plan.
- [x] Commit the design and plan before implementation.

## Task 2: Add task-threshold calibration with TDD

Files:

- Create `track_robot_semantic_search/task_threshold_calibration.py`.
- Create `track_robot_semantic_search/task_threshold_calibration_cli.py`.
- Create `schemas/phase2_task_threshold_calibration.schema.json`.
- Create `test/test_task_threshold_calibration.py`.
- Modify `setup.py`.

Steps:

1. [x] Add RED tests for deterministic threshold selection, minimum 30/30 evidence,
   recall failure, duplicate identities, invalid scores/splits, bounded input,
   canonical hash and byte-identical CLI output.
2. [x] Run only the new test file and capture the expected failure.
3. [x] Implement the smallest pure calibration core and CLI that pass the tests.
4. [x] Validate the generated report against every required schema field.
5. [x] Commit calibration code and tests.

## Task 3: Harden Phase 2 evaluation with TDD

Files:

- Modify `phase2_evaluation.py`.
- Modify `phase2_evaluation_cli.py`.
- Modify `schemas/annotation.schema.json`.
- Modify `schemas/phase2_evaluation_report.schema.json`.
- Modify `test/test_phase2_evaluation.py`.

Steps:

1. [x] Add RED tests showing missing predictions reduce task recall, contradictory
   selection is rejected, uncalibrated thresholds fail closed, and calibration
   threshold mismatches are rejected.
2. [x] Add RED tests for semantic-path P95 `<=150 ms`, core P95 `<=50 ms`, minimum
   1,800-second stability, CUDA reserve `<=1,536 MiB`, all scenarios and human
   regression.
3. [x] Add schema tests that validate nested metrics, gates and evidence fields,
   not just top-level keys.
4. [x] Run the focused tests and capture RED.
5. [x] Implement schema `2.0.0`, recomputed task/runtime/resource metrics and
   calibrated evidence binding.
6. [x] Regenerate the intentionally unavailable report from the checked legacy
   manifest; verify it remains unavailable and lists the new dependencies.
7. [x] Commit evaluator and contract changes.

## Task 4: Re-run deterministic replay and regressions

- [x] Build the four Phase 2 packages in fresh Stage 2G build/install roots.
- [x] Run deterministic normalized replay twice and compare canonical hashes.
- [x] Run all four-package tests and summarize exact test counts.
- [x] Run explicit opt-in DDS tests if the default suite skips them.
- [x] Run `git diff --check` and schema/JSON/YAML parsing checks.
- [x] Confirm a clean Git archive does not depend on untracked workspace data.

## Task 5: Record the truthful Stage 2G checkpoint

Files:

- Modify `rosbags/semantic_search/phase2_recording_guide.md`.
- Modify `rosbags/semantic_search/reports/README.md`.
- Create `rosbags/semantic_search/reports/phase2_stage2g_runtime_2026-07-17.json`.
- Regenerate `rosbags/semantic_search/reports/phase2_evaluation_2026-07-17.json`.
- Modify the Phase 2 roadmap status.

Steps:

1. [x] Document calibration JSONL, calibration CLI, frozen-threshold wiring and
   final evaluator commands.
2. [x] Inventory the workspace for a qualifying physical bag, annotations and
   Jetson profile. Do not create a placeholder bag manifest.
3. [x] If evidence is absent, record
   `software_complete_field_evidence_unavailable`, exact missing items and the
   operator procedure. Keep all production threshold flags false.
4. [x] Record verification commands/results and the deterministic replay hash.
5. [x] Commit documentation and evidence.
6. [x] Confirm no semantic-memory node, visualizer, rosbag player or test service
   remains running; then pause at Stage 2G.
