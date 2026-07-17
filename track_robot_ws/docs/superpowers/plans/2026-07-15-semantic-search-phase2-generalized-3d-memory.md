# Phase 2 Generalized Multi-Object 3D Semantic Memory Implementation Plan

**Date:** 2026-07-15
**Design:** `docs/superpowers/specs/2026-07-15-generalized-multi-object-3d-semantic-memory-design.md`
**Implementation worktree:** `/home/track-robot/track_robot_ws/.worktrees/semantic-search-phase0`
**Current branch:** `feature/semantic-search-phase1`

## 1. Problem summary

Build a deterministic, bounded, multi-object 3D semantic memory that combines
Phase 1 visual candidates, persistent LiDAR tracklets, geometry, time, TF, and
localization. It must assign MemoryCore-owned global IDs, preserve them through
short sensor loss, support static and dynamic objects, perform conservative
camera-LiDAR association and visual re-identification, rank stored objects for a
language task, explain every association, run online and in rosbag replay, and
leave the current human tracking pipeline unchanged.

The implementation follows the approved option C: a separate Python/GPU Phase
1 producer, separate existing LiDAR producer, one C++ Phase 2 process containing
pure association and memory modules, and a separate visualizer.

## 2. Scope and definition of done

Phase 2 is complete only when all of the following have direct evidence:

1. Multiple camera semantic candidates are received through the bounded typed
   contract.
2. Multiple persistent LiDAR tracklets are received with source epochs.
3. Physically and semantically compatible observations are associated.
4. Persistent MemoryCore-owned global IDs are created.
5. Visual, camera-track, LiDAR-tracklet, query, localization, memory, and global
   IDs remain separate.
6. Canonical position, uncertainty, and lifecycle are maintained.
7. Objects survive short camera or LiDAR loss.
8. Static and moving outdoor objects use different motion/lifetime behavior.
9. Normalized compatible visual descriptors support basic re-identification.
10. Permanent semantics and current task relevance remain separate.
11. Remembered objects can be ranked for a typed language task.
12. All active memory objects are visualized in RViz.
13. Association decisions expose bounded per-term debug evidence.
14. The same core runs online and under deterministic normalized rosbag replay.
15. Existing human tracking regression tests and launch contracts still pass.
16. A quantitative Phase 2 report covers the approved metrics and gates.

## 3. Out of scope

- Learned multimodal association or Transformer training.
- Learned LiDAR feature extraction.
- DINOv3 fine-tuning.
- Nav2, VLA, motion-control, or `/cmd_vel` output.
- Permanent database or automatic cross-process checkpoint recovery.
- Automatic odom-to-map migration of existing objects.
- Changing the default human tracking topics or configurations.

## 4. Primary implementation approach

Create `track_robot_semantic_memory`, a new C++17 ament package. Keep all domain
logic in ROS-independent classes and keep ROS conversion in node adapters.
Extend Phase 1 to publish bounded visual observations while continuing to
publish existing `SemanticRegionArray`. Add a source epoch to the existing
LiDAR array message and use a separate semantic-memory LiDAR configuration.

Development is staged so each stage produces an independently testable result:

- 2A: contracts, typed localization, producer epochs, and package skeleton;
- 2B: LiDAR-only object memory and lifecycle;
- 2C: projection and conservative camera-LiDAR association;
- 2D: multi-sensor state update and ambiguity handling;
- 2E: bounded appearance memory and re-identification;
- 2F: typed task updates, ranking, and services;
- 2G: replay evaluation, profiling, regression, and final evidence.

## 5. Stage 2A — contracts and producer normalization

### Task 2A.1: Add bounded Phase 2 interfaces

Create:

- `src/track_robot/track_robot_interfaces/msg/VisualDescriptor.msg`
- `src/track_robot/track_robot_interfaces/msg/VisualProposal.msg`
- `src/track_robot/track_robot_interfaces/msg/VisualProposalArray.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticLabelEvidence.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticObjectHistorySample.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticLocalizationState.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticTask.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticObservation.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticObservationArray.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticLidarTrackletArray.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticObject.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticObjectArray.msg`
- `src/track_robot/track_robot_interfaces/msg/AssociationTerm.msg`
- `src/track_robot/track_robot_interfaces/msg/AssociationDebug.msg`
- `src/track_robot/track_robot_interfaces/msg/SemanticMemoryEvent.msg`
- `src/track_robot/track_robot_interfaces/srv/GetSemanticObject.srv`
- `src/track_robot/track_robot_interfaces/srv/QuerySemanticObjects.srv`
- `src/track_robot/track_robot_interfaces/srv/MarkSemanticObjectInspected.srv`
- `src/track_robot/track_robot_interfaces/srv/ResetSemanticMemory.srv`

Bounds to encode in the `.msg`/`.srv` files:

- descriptor: 1024 float values;
- observations per array: 64;
- active objects per snapshot: 256;
- labels per observation/object: 16;
- compressed mask: 65,536 bytes;
- association terms per pair: 24;
- short history records: 16;
- query result page: 64 objects;
- label/provenance/model/frame strings: 128 characters;
- query text: 512 characters;
- event/debug reason: 256 characters.

Use fixed covariance arrays and one canonical position/frame. Use structured
`SemanticObjectHistorySample` records rather than parallel arrays. Do not add
the three base/local/world position copies from `ObjectObservation3D`. Keep
existing messages and `SearchForObject.action` unchanged.

Modify:

- `src/track_robot/track_robot_interfaces/CMakeLists.txt`
- `src/track_robot/track_robot_interfaces/package.xml`
- `src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py`

Add tests that instantiate maximum-bound messages, verify enum values and ID
fields, reject build-time syntax errors, and prove old interfaces still import.

Verification:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_interfaces
source install/setup.bash
colcon test --packages-select track_robot_interfaces --event-handlers console_direct+
colcon test-result --verbose
```

### Task 2A.2: Publish typed localization state

Modify:

- `src/track_robot_semantic_search/track_robot_semantic_search/localization_health_node.py`
- `src/track_robot_semantic_search/config/semantic_search_phase0.yaml`
- `src/track_robot_semantic_search/test/test_localization_health.py`
- `src/track_robot_semantic_search/test/test_launch_contract.py`

Continue publishing the existing diagnostic array and additionally publish
`SemanticLocalizationState` on `/semantic_memory/localization_state`. Include
mode, localization epoch, frame IDs, health flags, and reason. The typed state is
the authoritative Phase 2 input; diagnostic strings are not parsed by the C++
core.

MemoryCore, not the health evaluator, owns the memory epoch. It creates a new
memory epoch whenever `(mode, localization_epoch_id, canonical_frame_id)`
changes. This enforces the approved local-to-world isolation without silently
rewriting health semantics.

### Task 2A.3: Add LiDAR source epochs without changing human defaults

Modify:

- `src/track_robot/track_robot_lidar_tracking/src/lidar_tracklet_manager_node.cpp`
- `src/track_robot/track_robot_lidar_tracking/config/lidar_tracklets.yaml`

The new bounded `SemanticLidarTrackletArray` carries `uint64 source_epoch_id`
and at most 256 existing `LidarTracklet` records. The manager accepts
`source_epoch_seed`:

- non-zero seed gives deterministic replay/tests;
- zero creates a live boot nonce;
- timestamp rollback changes the epoch before track ID reuse.

Existing `LidarTrackletArray`, `tracklet_id` values, and human topics remain
unchanged. The manager dual-publishes the new bounded array only when
`semantic_output_topic` is non-empty; it emits a diagnostic drop count if more
than the configured bound is available.

Create an isolated generic configuration and launch entry:

- `src/track_robot/track_robot_lidar_tracking/config/semantic_memory_lidar_tracklets.yaml`
- `src/track_robot/track_robot_lidar_tracking/launch/semantic_memory_lidar_tracklets.launch.py`

Publish to `/semantic_memory/lidar_tracklets`; use broader configurable extent,
height, range, and motion bounds suitable for static obstacles and moving
objects. Do not edit the human launch to use this profile.

Add C++ tests by first extracting epoch generation/rollback into a small pure
helper. Verify stable IDs inside one epoch, epoch change before ID reuse, and a
fixed-seed deterministic sequence.

### Task 2A.4: Extend Phase 1 with normalized candidates and tasks

Create:

- `src/track_robot_semantic_search/track_robot_semantic_search/visual_candidates.py`
- `src/track_robot_semantic_search/test/test_visual_candidates.py`
- `src/track_robot_semantic_search/test/test_semantic_observation_contract.py`

Modify:

- `src/track_robot_semantic_search/track_robot_semantic_search/region_scoring.py`
- `src/track_robot_semantic_search/track_robot_semantic_search/perception_core.py`
- `src/track_robot_semantic_search/track_robot_semantic_search/perception_node.py`
- `src/track_robot_semantic_search/track_robot_semantic_search/query.py`
- `src/track_robot_semantic_search/config/semantic_search_phase1.yaml`
- `src/track_robot_semantic_search/launch/semantic_search_phase1.launch.py`
- `src/track_robot_semantic_search/test/test_perception_core.py`
- `src/track_robot_semantic_search/test/test_perception_node_contract.py`
- `src/track_robot_semantic_search/test/test_phase1_launch_contract.py`

Required behavior:

1. Assign a producer epoch and monotonic visual candidate IDs.
2. Compute each region descriptor by overlap-weighted pooling of the exact
   active grid cells, then L2-normalize it.
3. Include descriptor quality, encoder/checkpoint/version, proposal source,
   detector confidence, camera stamp, and image geometry.
4. Publish `SemanticObservationArray` while preserving current
   `SemanticRegionArray` output byte-for-field.
5. Publish `SemanticTask` once per accepted query/version with a normalized text
   descriptor.
6. Subscribe to bounded `VisualProposalArray` for external detector/manual
   proposals, align by source stamp, and mark existing regions as
   language-patch proposals.
7. Do not convert query labels into permanent labels.
8. Clear producer-local scheduling/candidate state on source-time rollback and
   change producer epoch before candidate ID reuse.
9. Refactor image encoding from query scoring so external/manual proposals can
   produce descriptors with no active query; with neither proposals nor a query,
   emit no visual candidates and allow LiDAR-only operation.

Benchmark grid sizes 2, 3, and 4 with the Phase 1 benchmark tool. Do not change
the deployed value until the report records candidate localization, descriptor
quality, P95 inference, GPU memory, and effective rate.

### Task 2A.5: Create the Phase 2 package skeleton

Create:

- `src/track_robot/track_robot_semantic_memory/CMakeLists.txt`
- `src/track_robot/track_robot_semantic_memory/package.xml`
- `src/track_robot/track_robot_semantic_memory/config/semantic_memory.yaml`
- `src/track_robot/track_robot_semantic_memory/launch/semantic_memory_phase2.launch.py`
- `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/types.hpp`
- `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/id_types.hpp`
- `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_domain.hpp`
- `src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`

Dependencies: C++17, `ament_cmake`, `ament_cmake_gtest`, `rclcpp`, `tf2_ros`,
`tf2_geometry_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`,
`diagnostic_msgs`, `track_robot_interfaces`, and Eigen3.

The package skeleton must start with all functional publishers disabled until
the pure-core tests exist. A launch contract test checks topic names, parameters,
queue bounds, and that the human launch graph is not included.

## 6. Stage 2B — LiDAR-only global object memory

### Task 2B.1: Implement deterministic source-time and domain primitives

Create:

- `include/track_robot_semantic_memory/source_time_buffer.hpp`
- `include/track_robot_semantic_memory/memory_clock.hpp`
- `src/source_time_buffer.cpp`
- `src/memory_domain.cpp`
- `test/test_source_time_buffer.cpp`
- `test/test_memory_domain.cpp`

The buffer has explicit count and age bounds, deterministic ordering by source
stamp plus producer key, oldest-first eviction, rollback detection, and drop
counters. Domain tests prove that mode/frame/localization-epoch changes create a
new memory epoch and that no object crosses the boundary automatically.

### Task 2B.2: Implement MemoryCore and lifecycle

Create:

- `include/track_robot_semantic_memory/memory_core.hpp`
- `include/track_robot_semantic_memory/lifecycle_policy.hpp`
- `include/track_robot_semantic_memory/motion_classifier.hpp`
- `src/memory_core.cpp`
- `src/lifecycle_policy.cpp`
- `src/motion_classifier.cpp`
- `test/test_memory_core.cpp`
- `test/test_lifecycle_policy.cpp`
- `test/test_motion_classifier.cpp`

MemoryCore consumes internal normalized LiDAR observations and:

- maps each LiDAR source key to one MemoryCore ID;
- requires configurable repeated evidence before confirmation;
- maintains canonical position, covariance, velocity, and extent;
- keeps lifecycle, support, visibility, and motion as independent axes;
- uses separate static/dynamic process-noise and retention profiles;
- emits creation, confirmation, state-change, loss, archive, and domain events;
- never reuses a global ID inside an epoch;
- bounds object count, feature bank, and short history;
- uses deterministic capacity eviction: archived oldest first, then lost oldest;
  confirmed active objects are never silently evicted.

Tests cover empty input, multiple simultaneous tracklets, tracklet miss/recovery,
source epoch changes, ID non-reuse, static zero velocity, dynamic prediction,
stale/lost/archive/reactivate paths, capacity limits, timestamp rollback, and
domain changes.

### Task 2B.3: Wire the LiDAR-only ROS node and visualizer

Create:

- `src/track_robot/track_robot_semantic_memory/src/ros_conversions.cpp`
- `src/track_robot/track_robot_semantic_memory/src/semantic_memory_visualizer_node.cpp`
- `src/track_robot/track_robot_semantic_memory/test/test_ros_conversions.cpp`

Modify `semantic_memory_node.cpp` to subscribe to typed localization state and
semantic LiDAR tracklets, transform at `last_measurement_stamp`, update
MemoryCore, and publish reliable active snapshots, events, and diagnostics.

The visualizer subscribes only to active snapshots and publishes bounded marker
arrays. Marker IDs derive from the public object key. Deletion markers are
explicit so RViz does not retain stale objects.

Stage 2B acceptance:

- at least two synthetic LiDAR tracklets receive distinct stable global IDs;
- a static tracklet remains valid with zero velocity;
- short loss changes support/lifecycle without deleting the object;
- source/domain changes cannot attach to old IDs;
- output is deterministic for a recorded normalized tracklet sequence.

### Checkpoint B implementation status — complete 2026-07-16

Stage 2B is implemented in `track_robot_semantic_memory` and stops before any
camera attachment or association behavior:

- `MemoryCore` owns bounded public IDs, separate static/dynamic lifecycle
  profiles, prediction, covariance growth, history, source-time rollback,
  capacity policy, and source/domain isolation;
- `OBSERVATION_ONLY / base_link` is explicitly instantaneous: each batch
  resets memory into a new epoch, while persistent IDs are limited to healthy
  `LOCAL_SESSION / odom` and `WORLD / map` domains;
- ROS conversion applies the canonical transform at each tracklet's
  `last_measurement_stamp`, rotates velocity/covariance, transforms axis-aligned
  extent conservatively, and publishes bounded typed snapshots/events;
- the node rejects stale localization state, malformed/duplicate producer
  identity, invalid geometry/time, and missing TF without mutating memory;
- the separate visualizer consumes only active snapshots, resolves marker-ID
  collisions deterministically, handles memory-node sequence reset on a new
  epoch, and emits explicit `DELETE` markers;
- `test_stage2b_acceptance.cpp` directly proves two stable distinct IDs,
  static zero velocity, short-loss retention, source/domain isolation, and
  exact repeatability of a fixed normalized LiDAR sequence.

Fresh verification commands completed on 2026-07-16:

```bash
colcon build --packages-select \
  track_robot_interfaces track_robot_lidar_tracking \
  track_robot_semantic_search track_robot_semantic_memory \
  --cmake-args -DBUILD_TESTING=ON

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select \
  track_robot_interfaces track_robot_lidar_tracking \
  track_robot_semantic_search track_robot_semantic_memory \
  --return-code-on-test-failure
```

The four-package result is 412 tests, 0 errors, 0 failures, and 1 explicit
skip. The skip is the opt-in executable DDS probe: this restricted environment
denies `getifaddrs`, and the authorized out-of-sandbox request could not run
because the Codex approval token was revoked. The probe remains available as
`test_ros_runtime.py` and is run on a normal ROS host with
`RUN_ROS_RUNTIME_TESTS=1`. All deterministic Stage 2B acceptance tests ran.

The semantic-memory C++ package also builds with
`-Wall -Wextra -Wpedantic -Werror`. CMake policy/deprecation messages originate
from the Foxy toolchain and are not source-code warnings.

This checkpoint does not claim physical rosbag/field evidence or Phase 2
completion. Its earlier test count is retained as the Checkpoint B historical
record; the current aggregate grows as later stages add tests.

## 7. Stage 2C — camera-to-LiDAR semantic association

### Task 2C.1: Implement generic projection

Create:

- `include/track_robot_semantic_memory/camera_lidar_projector.hpp`
- `src/camera_lidar_projector.cpp`
- `test/test_camera_lidar_projector.cpp`

Extract only generic camera model, timestamp, TF, centroid/corner projection,
inside fraction, center distance, and projected IoU mechanisms from
`selected_human_target_tracker_node.cpp`. Do not import its single-target state,
human body ROI, dimensions, weights, or topic names.

Tests use fixed intrinsics/extrinsics and verify visible, behind-camera,
out-of-FOV, partial box, unavailable TF, and exact-source-stamp cases.

### Task 2C.2: Implement gates and term calculation

Create:

- `include/track_robot_semantic_memory/association_terms.hpp`
- `include/track_robot_semantic_memory/cross_modal_associator.hpp`
- `src/association_terms.cpp`
- `src/cross_modal_associator.cpp`
- `test/test_association_terms.cpp`
- `test/test_cross_modal_associator.cpp`

Calculate each approved hard gate and soft term separately. Invalid terms never
become zero-valued evidence; they carry a validity bit and are excluded or cause
the configured gate. Reject non-finite inputs. Descriptor comparison requires
matching encoder/checkpoint/version, declared dimension, finite values, and
verified L2 normalization.

### Task 2C.3: Add shadow-mode replay and calibrate weights

The node initially publishes `AssociationDebug` without attaching camera
observations. Add:

- `src/track_robot_semantic_search/schemas/phase2_association_samples.schema.json`
- `src/track_robot_semantic_search/scripts/export_phase2_association_samples.py`
- `src/track_robot_semantic_search/test/test_phase2_association_samples.py`

Collect positive/negative annotated pair distributions from the pilot bag.
Select gates, normalization ranges, weights, match threshold, and ambiguity
margin from the distributions. Record them in:

- `src/track_robot/track_robot_semantic_memory/config/phase2_association_baseline.yaml`
- a versioned calibration report under `rosbags/semantic_search/reports/`.

No calibrated report means camera attachment remains disabled.

### Task 2C.4: Enable global assignment and confirmation

Create:

- `include/track_robot_semantic_memory/hungarian_assignment.hpp`
- `src/hungarian_assignment.cpp`
- `test/test_hungarian_assignment.cpp`
- `test/test_association_confirmation.cpp`

Use deterministic global one-to-one assignment, top-two margin, configurable
multi-frame confirmation, previous-association hysteresis, cooldown, and an
explicit ambiguous result. Unit tests cover competition, equal-cost tie-breaks,
candidate permutation, false positives, missed evidence, and split/merge
hypotheses.

### Stage 2C implementation status — calibrated checkpoint complete 2026-07-16

The Stage 2C software path is implemented and remains fail-closed:

- generic fixed-intrinsic/extrinsic projection covers visible, behind-camera,
  out-of-FOV, partial-image, missing-TF and exact-source-stamp behavior;
- hard gates and soft terms are independent bounded results; invalid evidence
  carries `valid=false` and NaN in the ROS debug message rather than becoming
  zero evidence;
- descriptor comparison verifies encoder, checkpoint, version, declared
  dimension, finite values and actual L2 norm before cosine scoring;
- deterministic Hungarian assignment sorts public candidate IDs before solving
  and leaves costly/missing edges unmatched;
- multi-frame confirmation implements explicit ambiguity, previous-match
  hysteresis, bounded misses, cooldown and split/merge protection;
- `semantic_memory_node` consumes bounded visual observations plus CameraInfo,
  queries camera/LiDAR TF at each tracklet `last_measurement_stamp`, and emits
  at most 1,024 reliable `AssociationDebug` pair records per batch;
- Phase 1 observations now carry a bounded explicit camera `calibration_id`;
- the strict sample schema and exporter deterministically join human
  annotations, compare normalized soft terms only, require positive hard-gate
  passes, and fail closed below the 95% precision / 80% recall pilot gates;
- real `human_tracking_lidar_20260706_145752` RGB/LiDAR replay produced 22
  unique human-positive source frames and 79 negative pairs; the selected
  threshold yields 18 true positives, zero false positives and four false
  negatives (100% precision, 81.8% recall);
- the data-driven baseline uses a size-ratio maximum of 40 for partial LiDAR
  human returns and normalized weights over projected centroid, inside
  fraction, projected IoU and extent consistency.

The checked-in report sets `status=calibrated` and
`camera_attachment_allowed=true`, so Checkpoint C is passed. Runtime
`camera_attachment_enabled` deliberately remains `false`: consuming this
dependency and mutating semantic memory belongs to Stage 2D, not 2C.

## 8. Stage 2D — multi-sensor memory update

Create:

- `include/track_robot_semantic_memory/multisensor_update.hpp`
- `src/multisensor_update.cpp`
- `test/test_multisensor_update.cpp`

Add camera-LiDAR, camera-only, LiDAR-only, prediction-only, and no-support
updates. Define for every support/lifecycle combination whether position,
covariance, semantics, and appearance may update and how confidence changes.
Position uses LiDAR/geometry evidence when present; camera-only evidence may
update semantics/appearance but must not invent metric depth unless an explicit
valid stereo/geometry source is present.

Tests cover short camera loss, short LiDAR loss, both lost, out-of-FOV versus
occluded, incorrect challenger rejection, static and dynamic timeouts, and
reactivation through strict multi-frame evidence.

### Stage 2D implementation status — runtime integration complete 2026-07-16

`MultisensorUpdater` defines all 25 lifecycle/support combinations and the
runtime now consumes the calibrated Checkpoint C dependency. A bounded
coordinator applies thresholding, deterministic global Hungarian assignment,
top-two/column competition ambiguity checks and three-frame confirmation using
a stable camera-track or upstream-proposal key. One-shot candidates without a
stable key remain tentative.

Confirmed delayed camera evidence is applied through `MemoryCore`, which stays
the sole persistent-state owner. The supplement records camera/visual source
metadata, bounded non-task semantic labels, support, visibility, confidence and
per-sensor counts while preserving metric position, velocity, covariance and
`state_stamp`. Confirmation and memory supplementation commit as one frame
transaction; duplicate stable keys, cross-epoch LiDAR evidence and stale
evidence are rejected before persistent change. A stable visual key has one
memory owner, so confirmed replacement emits an atomic detach/attach pair.
Missing report, drift in any calibrated gate/weight/calibration/confirmation
parameter, incomplete or oversized matrices, missing TF/calibration, ambiguity
and missing object keys all fail closed. The report is installed into package
share and the normal profile stays shadow/safe-off; the calibrated attachment
profile is an explicit reversible opt-in. Appearance prototype mutation and
re-identification remain Stage 2E.

## 9. Stage 2E — visual memory and re-identification

Create:

- `include/track_robot_semantic_memory/appearance_memory.hpp`
- `src/appearance_memory.cpp`
- `test/test_appearance_memory.cpp`
- `test/test_reidentification.cpp`

Implement at most four prototypes per object and compatible encoder version.
Each prototype uses confidence-weighted EMA with renormalization. Add a new
prototype only for a high-quality confirmed observation sufficiently different
from current prototypes. Retain a best-quality view. Do not update from
ambiguous, prediction-only, non-finite, zero-norm, or incompatible evidence.

Re-identification combines hard spatial/domain/age gates with appearance,
geometry, and semantics, requires multi-frame confirmation, and emits a
re-identification event. It never resurrects archived objects automatically.

### Stage 2E implementation status — runtime software complete 2026-07-16

The bounded appearance bank implements at most four compatible prototypes,
quality-weighted normalized EMA, diverse-view creation and a separate
best-quality view. Updates are transactional and reject low-quality,
ambiguous, prediction-only, incompatible, non-finite, zero-norm and antipodal
zero-EMA evidence without mutation. Re-identification applies lifecycle,
domain, age and spatial hard gates, combines appearance/geometry/semantic
scores, performs deterministic global one-to-one assignment and requires three
consecutive complete frames. Only an active candidate whose compatible
appearance descriptor was accepted in the current source frame may advance
confirmation; prediction-only, stale, ambiguous, archived and invalid evidence
cannot mutate identity. Confirmation state, candidate/target matrices and
prototype banks are bounded.

The ROS runtime evaluates association, appearance learning, re-identification
and identity transfer on copies, then commits them together. A successful
transfer preserves the old global key and cumulative identity counts, moves
the replacement's current metric/source/visibility state, removes the
temporary duplicate and emits one re-identification event. An executed
synthetic DDS leave/re-entry fixture exercises this software path. Production
mutation remains disabled in every checked profile and startup requires an
exact field-calibration report before it can be enabled. The physical robot
leave/re-entry pilot and field-calibrated thresholds remain pending for Stage
2G; this checkpoint does not claim physical validation.

## 10. Stage 2F — task ranking and memory services

Create:

- `include/track_robot_semantic_memory/task_relevance_scorer.hpp`
- `src/task_relevance_scorer.cpp`
- `src/memory_services.cpp`
- `test/test_task_relevance_scorer.cpp`
- `test/test_memory_services.cpp`

On each compatible `SemanticTask`, recompute task relevance for all eligible
objects using the maximum compatible appearance-prototype similarity plus
bounded permanent semantic evidence. Task values live in a separate overlay and
clearing/changing a task does not change global IDs, prototypes, lifecycle, or
permanent labels.

Services use the public object key, bounded pages, deterministic sorting, and
explicit not-found/stale-epoch reasons. `ResetSemanticMemory` increments the
memory epoch before clearing active indexes. Inspection changes emit events.

Publish `/semantic_memory/best_candidate` only for a confirmed, active,
uninspected object above a configurable calibrated relevance threshold;
otherwise publish no valid winner rather than a low-confidence guess.

### Stage 2F implementation status — runtime software complete 2026-07-17

The task overlay recomputes scores from the maximum compatible appearance
prototype and bounded permanent semantic evidence, rejects invalid tasks and
archived/unscorable objects, and clears or replaces task state without
mutating permanent object evidence. The service core provides deterministic
bounded pagination, explicit invalid/stale-epoch/not-found outcomes,
inspection events, epoch-safe reset and fail-closed best-candidate selection.
Unit evidence covers task replacement isolation, model incompatibility,
ASCII/raw-length validation, filtering, tie-breaking, event idempotence,
bounded state and threshold calibration gating. The live node consumes typed
tasks, enriches active snapshots, exposes get/query/inspection/reset services,
and publishes a reliable transient-local zero-or-one best-candidate array.
Core/store reset is copy-committed at one new epoch. Unhealthy localization,
domain changes and pending-task source-time rollback immediately fail closed.

The synthetic DDS acceptance exercises task delivery before the first memory
snapshot, appearance-backed ranking, get/query/inspection, reset/stale keys,
localization invalidation and domain isolation. Every checked production
profile keeps `best_candidate_threshold_calibrated=false`, so no valid winner
is published without later calibration. The physical task-ranking pilot and
threshold selection remain Stage 2G work; this checkpoint does not claim Phase
2 completion.

## 11. Stage 2G — evaluation, profiling, and completion evidence

### Task 2G.1: Extend manifests and annotations

Create or extend:

- `src/track_robot_semantic_search/schemas/annotation.schema.json`
- `src/track_robot_semantic_search/schemas/dataset_manifest.schema.json`
- `src/track_robot_semantic_search/schemas/phase2_evaluation_report.schema.json`
- `src/track_robot_semantic_search/track_robot_semantic_search/phase2_evaluation.py`
- `src/track_robot_semantic_search/track_robot_semantic_search/phase2_evaluation_cli.py`
- `src/track_robot_semantic_search/test/test_phase2_evaluation.py`

Annotations identify public object keys, camera boxes/masks, LiDAR source keys,
visibility/support intervals, approximate 3D ground truth, task relevance, and
ignore regions. The evaluator reports every approved correctness and resource
metric and refuses to claim a metric when required annotations are absent.

### Task 2G.2: Add deterministic normalized replay

Create:

- `src/track_robot/track_robot_semantic_memory/src/semantic_memory_replay.cpp`
- `src/track_robot/track_robot_semantic_memory/test/test_deterministic_replay.cpp`
- `src/track_robot_semantic_search/scripts/run_phase2_replay.py`

The replay path consumes serialized normalized observation, tracklet,
localization, task, and expected-event sequences without GPU inference. Two runs
with the same config and seed must produce byte-equivalent public keys,
lifecycle transitions, and association decisions.

### Task 2G.3: Record and evaluate the pilot bag

Add:

- `rosbags/semantic_search/phase2_recording_guide.md`
- a manifest for the new pilot bag;
- a checked annotation file;
- association calibration and final evaluation reports under
  `rosbags/semantic_search/reports/`.

Before recording, run a TF preflight that proves connected camera, LiDAR,
`base_link`, and selected memory frame at source timestamps. Record IMU,
Odometry, typed localization state, and diagnostics. A bag that fails capability
validation remains observation-only and cannot satisfy local/world gates.

### Task 2G.4: Full scenario, resource, and regression gates

Run the twelve approved scenario classes. On the Jetson, record Phase 1 and
Phase 2 CPU, GPU, resident memory, P50/P95 latency, input/output rates, drops,
and a long-duration replay. Run the existing human tracking validation and all
workspace tests. Document any unavailable physical scenario as not achieved;
synthetic tests cannot substitute for required quantitative field evidence.

### Stage 2G implementation status — tooling complete, field evidence blocked 2026-07-16

The annotation and manifest contracts now optionally cover public object/source
keys, support, visibility, 3D position, task relevance, ignore regions, memory
frame and all twelve scenarios while preserving existing Phase 0/1 manifests.
The fail-closed evaluator reports every approved identity, association, re-ID,
reactivation, spatial, lifetime, task, latency, rate and resource category and
uses null/unavailable values when required evidence is absent.

The C++ normalized replay consumes localization domains, LiDAR observations,
assignment matrices, compatible task evidence and expected events. A Python
runner executes it twice and requires byte-equivalent canonical output. The
checked 2026-07-16 regression report has matching output SHA-256 values.

The workspace still contains no new connected-TF pilot bag, Phase 2 annotation
JSONL or Jetson runtime/resource profile. Therefore no fake pilot manifest was
created: its bag metadata and SHA-256 can only exist after recording closes.
The recording guide supplies the TF-at-source-time preflight, exact topic set,
manifest/annotation workflow and evaluation commands. The current evaluation
report is correctly `unavailable`, lists all twelve missing scenarios, and
claims only deterministic synthetic replay. Checkpoint F and overall Phase 2
completion remain pending physical evidence and calibrated runtime wiring.

Fresh software verification on 2026-07-16:

- `track_robot_semantic_memory` builds with
  `-Wall -Wextra -Wpedantic -Werror`;
- six buildable packages (`interfaces`, `drivers`, `decision`, LiDAR tracking,
  semantic search and semantic memory) report 508 tests, zero errors, zero
  failures and zero skips;
- the run includes the legacy decision DDS launch sequence and the opt-in live
  semantic-memory test that starts both nodes and verifies two-object snapshots,
  creation events and RViz markers;
- a full ten-package build is not available on this host because the unchanged
  control and safety packages require the absent external `bunker_msgs` SDK;
  `track_robot_perception` transitively waits for those packages. Consequently
  the full human-pipeline gate remains unclaimed despite the passing decision
  and LiDAR regressions.

### Stage 2G hardening checkpoint — software complete, field evidence unavailable 2026-07-17

The evaluator now enforces the Stage 2G completion contract rather than
accepting mere field presence. Report schema `2.0.0` binds final task evidence
to an independent frozen-threshold report, counts missing positive predictions
as false negatives, rejects score/selection contradictions, and gates task
recall, hard-negative false confirmation, complete semantic-path latency,
30-minute source-span stability and CUDA reserved memory.

The new deterministic calibration CLI requires at least 30 positive and 30
hard-negative human samples. No qualifying task-calibration dataset exists in
the workspace, so every checked production profile remains
`best_candidate_threshold_calibrated=false`.

Strict four-package compilation exposed and fixed a stale normalized-replay
event switch introduced as the memory event enum grew through Stages 2D–2F.
The normalized event-name test now covers all thirteen event values. The same
build removed four legacy unused LiDAR helpers and one unused test local; no
runtime behavior changed.

Fresh 2026-07-17 software evidence:

- four packages build with `-Wall -Wextra -Wpedantic -Werror`;
- the default four-package suite reports 599 tests, zero errors, zero failures
  and three expected opt-in DDS skips;
- explicit local DDS execution passes all three runtime tests outside the
  network-restricted sandbox;
- normalized replay produces identical output SHA-256 values;
- no semantic-memory node, visualizer or rosbag player remains after testing.

The physical completion gate remains unavailable. There is still no new
connected-TF Phase 2 pilot bag, final Phase 2 annotation/prediction pair,
independent task-threshold calibration set, 30-minute Jetson runtime/resource
profile, twelve-scenario field evidence, or separately completed human-stack
regression. No placeholder manifest or production threshold was created.

## 12. Testing strategy

### Pure unit tests

- Bounded buffers, ID allocators, epochs, and deterministic tie-breaks.
- Coordinate/domain transitions and transform validity.
- Projection geometry and hard gates.
- Association term normalization, assignment, ambiguity, and confirmation.
- Lifecycle, static/dynamic behavior, appearance banks, re-ID, and task overlay.
- Service pagination and reset semantics.

### Contract and integration tests

- ROS interface generation and maximum-bound construction.
- Phase 1 old/new dual publishing.
- Typed localization and LiDAR source epochs.
- Node QoS, topic, parameter, and launch contracts.
- Multi-topic normalized sequence through the C++ node.
- RViz marker add/delete behavior.

### Replay and field evidence

- Deterministic normalized replay for regression.
- Legacy bag only for observation-only and non-regression evidence.
- New connected-TF pilot bag for local/world, association, and re-ID gates.
- Full annotated scenario suite for Phase 2 completion.

### Standard verification commands

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_interfaces \
  track_robot_semantic_search \
  track_robot_lidar_tracking \
  track_robot_semantic_memory
source install/setup.bash
colcon test --packages-select \
  track_robot_interfaces \
  track_robot_semantic_search \
  track_robot_lidar_tracking \
  track_robot_semantic_memory \
  --event-handlers console_direct+
colcon test-result --verbose
```

Python-only tests in this host must disable incompatible globally installed
pytest plugins and include the ROS Python path when invoked outside colcon.

## 13. Repository impact

### New package

- `src/track_robot/track_robot_semantic_memory/`: Phase 2 C++ core, ROS node,
  visualizer, configuration, launch, and tests.

### Existing packages changed additively

- `track_robot_interfaces`: new bounded Phase 2 messages/services; existing
  message definitions remain unchanged.
- `track_robot_semantic_search`: normalized visual candidates, descriptors,
  typed tasks/localization, Phase 2 evaluator and replay tooling.
- `track_robot_lidar_tracking`: source epochs and isolated generic-object
  configuration; current human defaults remain intact.

### Documentation/data

- approved design and this plan;
- pilot recording guide, manifest, annotations, and versioned reports.

## 14. Compatibility and migration

1. Continue publishing existing Phase 1 `SemanticRegionArray`.
2. Do not remove or rename existing observation/tracked-object messages or the
   search action.
3. Keep old message definitions unchanged; new Phase 2 messages carry the
   redesigned semantics.
4. Use new `/semantic_memory/*` topics and separate launch/config files.
5. Do not include Phase 2 from the default human bringup until regression and
   resource gates pass.
6. Mark old interfaces deprecated only after all current users migrate in a
   later phase.

## 15. Rollback strategy

- Every stage is additive and guarded by a separate launch entry.
- Stopping Phase 2 leaves Phase 1 and human tracking operational.
- Disabling `publish_semantic_observations` returns Phase 1 to old-only output.
- Disabling `enable_camera_attachment` returns Phase 2 to LiDAR-only memory.
- Removing the semantic-memory LiDAR launch returns to the untouched human
  profile.
- Association configurations are versioned; rollback selects the previous
  calibrated file without changing code.
- Interface rollback requires rebuilding dependent packages but never removes
  existing message names.

## 16. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Query-conditioned Phase 1 misses unqueried objects | External/manual proposal adapters now; query-independent patch grouping later behind the same contract |
| Grid descriptors are too coarse | Quality field, grid-size benchmark, optional crop re-encoding behind stable interface |
| Human LiDAR filters reject generic objects | Isolated broader semantic-memory profile; no human default changes |
| LiDAR IDs repeat after restart/rollback | Source epoch seed/nonce plus rollback epoch changes |
| Odom/map coordinates are silently mixed | Memory domain key forces a new memory epoch |
| Camera/LiDAR arrival order changes replay | Source-time buffers and deterministic tie-breaks |
| Wrong match corrupts appearance memory | Hard gates, ambiguity, multi-frame confirmation, and confirmed-only feature update |
| DDS payload/backpressure on Foxy | Bounded messages, depth-one inputs, compact active snapshots, optional debug |
| Association weights overfit one bag | Shadow-mode distributions, versioned calibration, full scenario suite |
| C++ core blocks on model inference | Model remains in separate Python/GPU process |
| Long-term state grows without bound | Capacity limits, bounded summaries, archive/eviction policy, paginated queries |
| Physical bag lacks connected TF/ground truth | Preflight capability gate; report metric as unavailable rather than inferred |

## 17. Implementation order and checkpoints

1. **Checkpoint A — complete:** 2A interfaces, typed localization, producer/source epochs,
   normalized Phase 1 output, and package skeleton all build and preserve old
   tests.
2. **Checkpoint B — complete:** 2B LiDAR-only memory, lifecycle, events, and RViz pass
   deterministic synthetic replay.
3. **Checkpoint C — complete:** 2C projector, explainable scoring,
   deterministic assignment/confirmation, ROS shadow output, and calibration
   tooling pass the annotated real-bag pilot. The calibrated dependency may
   now proceed into the Stage 2D update policy; attachment remains disabled in
   the 2C runtime.
4. **Checkpoint D — Stage 2D/2E runtime software integration complete;
   physical pilot pending:** calibrated runtime attachment, sensor-support
   policy, bounded appearance memory, deterministic re-identification and
   atomic identity transfer pass unit/integration and synthetic DDS tests.
   Production re-ID mutation stays safe-off; physical robot re-entry evidence
   and field calibration remain Stage 2G work.
5. **Checkpoint E — runtime software complete, calibration/physical pilot
   pending:** 2F task changes rerank the isolated overlay without changing
   identity; typed task transport, four services, zero/one winner publication,
   epoch reset and fail-closed localization windows pass synthetic DDS. Every
   production profile keeps winner selection uncalibrated/safe-off.
6. **Checkpoint F:** 2G full replay, resource, regression, and field reports
   prove all sixteen completion requirements.

No checkpoint is described as Phase 2 complete until Checkpoint F has direct
evidence for every required metric and scenario.

## 18. Open questions

None blocking implementation. Numeric association weights, lifecycle timeouts,
3D consistency threshold, and the deployed Phase 1 grid size are intentionally
calibration outputs, not architectural unknowns. The 2C association report now
exists; later-stage gates remain unclaimed until their own evidence is added.
