# Stage 2D Runtime Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect calibrated camera–LiDAR association to semantic memory without allowing delayed or ambiguous visual evidence to corrupt metric state.

**Architecture:** Keep `MemoryCore` as the sole owner of persistent object state. A pure, bounded runtime-association coordinator performs thresholding, deterministic global Hungarian assignment, stable-identity multi-frame confirmation and ambiguity protection; confirmed results are submitted to a dedicated delayed-visual supplement API that may update association/semantic metadata but never rewind position, covariance or the metric state clock.

**Tech Stack:** ROS 2 Foxy, C++17, Python 3 launch-contract tests, GoogleTest, ament/colcon, YAML, nlohmann JSON.

**Execution status (2026-07-16):** Complete. All four tasks below were
implemented and verified; the unchecked boxes are retained as the original
execution recipe rather than live backlog state.

**Post-review hardening:** Whole-frame confirmation is transactional; duplicate
stable keys are rejected before state mutation; LiDAR buffer lookup is epoch
restricted; evidence age uses the latest accepted LiDAR source-time watermark;
visual ownership replacement is unique and atomic; and the installed calibration
report binds the complete scoring and confirmation contract.

## Global Constraints

- Do not modify robot control, navigation, following, obstacle avoidance or `/cmd_vel` behavior.
- Every queue, candidate set, confirmation state, label list, debug batch and history remains explicitly bounded.
- Camera attachment requires calibrated non-shadow configuration; otherwise it fails closed.
- The stable visual key is `(producer_epoch, camera_track_id)` first, then `(upstream_producer_epoch, upstream_proposal_id)`; a one-shot `visual_candidate_id` is never a confirmation key.
- Current language relevance and task-conditioned labels never become permanent semantic identity.
- Delayed camera evidence may update visual metadata only when newer than that object's last accepted camera evidence; it never changes position, covariance, velocity or `state_stamp_ns`.
- Complete Stage 2D and stop before Stage 2E runtime integration.

---

### Task 1: Stable, bounded runtime association coordinator

**Files:**
- Create: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/runtime_association.hpp`
- Create: `src/track_robot/track_robot_semantic_memory/src/runtime_association.cpp`
- Create: `src/track_robot/track_robot_semantic_memory/test/test_runtime_association.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/association_confirmation.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/association_confirmation.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_association_confirmation.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/CMakeLists.txt`

**Interfaces:**
- Consumes: calibrated pair scores, `LidarAssociationKey`, Hungarian assignment and `AssociationConfirmation`.
- Produces: `RuntimeAssociationCoordinator::process(const RuntimeAssociationFrame &)` and one deterministic `RuntimeAssociationDecision` per visual candidate.

- [ ] Replace confirmation's one-shot candidate key with a bounded `VisualAssociationKey {kind, producer_epoch_id, local_id}` and reject invalid or non-monotonic input without partial state mutation.
- [ ] Add failing tests proving changing candidate IDs with the same stable key confirm on frame three, while missing stable keys never attach and producer epochs do not share state.
- [ ] Add a coordinator test for a calibrated 2×2 matrix, below-threshold rejection, threshold-as-virtual-second-candidate margin, row/column competition ambiguity and input-permutation determinism.
- [ ] Implement the coordinator with a maximum of 64 visual identities, 256 LiDAR identities and 1,024 pairs; an oversized, duplicate or non-finite frame is rejected as a whole.
- [ ] Run `colcon test --packages-select track_robot_semantic_memory --ctest-args -R 'test_(association_confirmation|runtime_association)' --output-on-failure` and require zero failures.

### Task 2: Delayed visual supplement in the memory core

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_core.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/ros_conversions.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_ros_conversions.cpp`

**Interfaces:**
- Consumes: a confirmed `VisualMemorySupplement` addressed by the attached LiDAR producer key.
- Produces: `MemoryCore::supplement_visual(...)`, updated camera/visual source metadata, bounded permanent labels, sensor counts/support and an association event.

- [ ] Add failing tests for confirmed attachment, idempotent duplicate rejection, older camera rejection, ambiguous/no-match rejection and missing object rejection.
- [ ] Add the delayed-evidence regression: an object at metric time 100 accepts camera source time 99 as its first visual supplement, but position, velocity, covariance and `state_stamp_ns` remain byte-for-byte unchanged.
- [ ] Add tests proving task-conditioned labels are ignored, permanent/detector/operator labels remain bounded to 16, camera counts advance once, and the public ROS object exposes the accepted camera/visual fields and `last_camera_seen`.
- [ ] Implement the supplement transaction using `multisensor_update_permissions` for lifecycle/support permission checks; do not invoke metric prediction or measurement updates on delayed camera input.
- [ ] Emit `kAssociationAttached` only on the first stable attachment or a confirmed replacement, and map it to `EVENT_ASSOCIATION_ATTACHED`.
- [ ] Run the focused memory-core and conversion tests and require zero failures.

### Task 3: Node integration and fail-closed calibration gate

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/config/semantic_memory.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/config/phase2_association_baseline.yaml`
- Create: `src/track_robot/track_robot_semantic_memory/config/phase2_camera_attachment.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py`

**Interfaces:**
- Consumes: full non-truncated score matrices grouped by one source-time LiDAR batch and the Task 1 coordinator.
- Produces: confirmed memory supplements, matched/tentative/ambiguous debug decisions and diagnostics counters independent of debug publication.

- [ ] Add launch-contract failures showing the main profile stays safe-off and the calibrated overlay sets non-shadow attachment with exact threshold `0.6303095604656801`, margin `0.058443876599150624`, three confirmation frames and bounded miss/cooldown parameters.
- [ ] Read and validate match, margin, hysteresis, confirmation, miss and cooldown parameters; remove the old unconditional Stage 2D rejection while retaining the calibrated non-shadow gate.
- [ ] Refactor observation handling so attachment does not depend on `publish_association_debug`; build the complete matrix before applying the 1,024-pair safety bound, and fail closed instead of assigning from a truncated matrix.
- [ ] Reset coordinator state on domain, memory epoch, LiDAR source epoch, visual producer epoch or source-time rollback.
- [ ] Apply only `MATCHED` decisions with stable keys to `MemoryCore::supplement_visual`, then republish the active snapshot and association event; tentative, ambiguous, missing-TF/calibration and unmatched results remain read-only.
- [ ] Publish debug records with the real decision, margin, non-negative cost and global object ID when available.
- [ ] Run launch-contract and semantic-memory package tests and require zero failures.

### Task 4: Documentation, replay evidence and process cleanup

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/README.md`
- Modify: `docs/development/plans/semantic-search/2026-07-15-semantic-search-phase2-generalized-3d-memory.md`
- Create: `artifacts/semantic_search/reports/phase2_stage2d_runtime_2026-07-16.json`

**Interfaces:**
- Consumes: verified tests and deterministic coordinator/memory-core fixtures.
- Produces: the Stage 2D completion record and exact safe enable/rollback instructions.

- [ ] Document safe-off default, calibrated overlay, stable-key requirement, delayed-visual rule, camera-only limitation and the single-switch rollback (`camera_attachment_enabled: false`).
- [ ] Generate a bounded deterministic Stage 2D evidence report recording the calibrated dependency, focused test commands, decision counts and geometry-non-mutation assertion.
- [ ] Run a fresh three-package build and all tests; require zero failures except the intentional opt-in DDS skip.
- [ ] Run `git diff --check` and verify the evidence/config invariants with a read-only script.
- [ ] Inspect processes before and after testing; terminate every ROS node, launch, bag replay or service started for Stage 2D, while preserving unrelated pre-existing processes.

## Self-review

- Spec coverage: global one-to-one assignment, stable multi-frame confirmation, ambiguity rejection, calibrated gating, all support/lifecycle permissions, delayed source-time safety, bounded state, explainability and deterministic replay are assigned above.
- Intentional deferral: appearance prototype mutation and re-identification remain Stage 2E; Stage 2D records only whether confirmed appearance evidence was accepted.
- Type consistency: stable visual keys flow from coordinator decisions into `VisualMemorySupplement`; persistent ownership remains exclusively in `MemoryCore`.
- Placeholder scan: no implementation step relies on an undefined later decision.
