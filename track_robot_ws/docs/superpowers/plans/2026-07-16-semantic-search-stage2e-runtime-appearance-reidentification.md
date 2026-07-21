# Stage 2E Runtime Appearance Memory and Re-identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect bounded appearance learning and deterministic multi-frame re-identification to the Stage 2D runtime while keeping `MemoryCore` the sole persistent identity owner.

**Architecture:** `MemoryCore` owns per-object `AppearanceMemory` banks, creates immutable re-identification evidence frames, and performs the guarded identity-transfer transaction. A pure `RuntimeReidentificationCoordinator` validates each complete bounded matrix, runs deterministic Hungarian assignment and maintains three-frame confirmation; the ROS node evaluates it on copies and commits the coordinator/core together.

**Tech Stack:** ROS 2 Foxy, C++17, rclcpp, ament/colcon, GoogleTest, pytest, nlohmann_json, YAML launch profiles.

## Global Constraints

- Stop after Stage 2E; do not implement Stage 2F service/task wiring or Stage 2G physical evaluation.
- `MemoryCore` remains the only persistent owner of global IDs and appearance banks.
- Appearance banks contain at most four compatible, finite, normalized prototypes.
- Invalid optional appearance evidence must not roll back a valid Stage 2D association attachment.
- Re-identification operates on complete frames bounded to 64 active candidates, 256 lost targets and 1,024 pairs.
- Assignment is deterministic, globally one-to-one and requires three consecutive increasing source frames.
- Archived objects, cross-domain/epoch pairs, ambiguous pairs and failed transaction guards never mutate identity.
- Production identity mutation is safe-off: `reidentification_shadow_mode=true`, `reidentification_mutation_enabled=false`, `reidentification_calibration_status=uncalibrated`.
- The Stage 2E evidence report must state `physical_reentry_pilot_executed=false` and must not claim field-calibrated thresholds.
- Every ROS node, service or rosbag process started for testing must be stopped before handoff.
- Preserve all unrelated dirty-worktree changes; stage and commit only files named by the current task.

---

### Task 1: Persist bounded appearance memory in MemoryCore

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/types.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/appearance_memory.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/appearance_memory.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_core.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_appearance_memory.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_memory_core.cpp`

**Interfaces:**
- Consumes: existing `AppearanceDescriptor`, `AppearanceObservation`, `AppearanceMemory::update()` and Stage 2D `VisualMemorySupplement`.
- Produces: `AppearanceMemory::merge_from()`, `AppearanceMemory::summary_id()`, descriptor-bearing `VisualMemorySupplement`, per-object appearance summary fields and `MemoryCore::appearance_prototypes()`.

- [x] **Step 1: Write failing appearance-bank helper tests**

Add tests that require deterministic summary IDs, empty summary behavior and bounded compatible merge:

```cpp
TEST(AppearanceMemory, SummaryIdIsDeterministicAndChangesWithContent)
{
  semantic_memory::AppearanceMemory first(config());
  semantic_memory::AppearanceMemory second(config());
  EXPECT_TRUE(first.summary_id().empty());
  first.update(observation({1.0, 0.0}, 0.9));
  second.update(observation({1.0, 0.0}, 0.9));
  EXPECT_EQ(first.summary_id(), second.summary_id());
  EXPECT_EQ(first.summary_id().rfind("appearance-v1-", 0U), 0U);
  second.update(observation({0.0, 1.0}, 0.9));
  EXPECT_NE(first.summary_id(), second.summary_id());
}

TEST(AppearanceMemory, MergeUsesNormalCompatibilityAndFourPrototypeBound)
{
  semantic_memory::AppearanceMemory target(config());
  semantic_memory::AppearanceMemory source(config());
  target.update(observation({1.0, 0.0}));
  source.update(observation({0.0, 1.0}));
  const auto merged = target.merge_from(source);
  EXPECT_EQ(merged.accepted, 1U);
  EXPECT_LE(target.prototypes().size(), 4U);
}
```

- [x] **Step 2: Run the appearance test and verify RED**

Run:

```bash
colcon build --base-paths src/track_robot/track_robot_semantic_memory src/track_robot/track_robot_interfaces --build-base build-stage2e --install-base install-stage2e --packages-select track_robot_interfaces track_robot_semantic_memory --cmake-args -DBUILD_TESTING=ON
colcon test --build-base build-stage2e --install-base install-stage2e --packages-select track_robot_semantic_memory --ctest-args -R test_appearance_memory --output-on-failure
```

Expected: compile failure because `summary_id()` and `merge_from()` do not exist.

- [x] **Step 3: Implement deterministic summary and bounded merge**

Add:

```cpp
struct AppearanceMergeResult
{
  std::size_t accepted{0U};
  std::size_t rejected{0U};
};

class AppearanceMemory
{
public:
  AppearanceMergeResult merge_from(const AppearanceMemory & source);
  [[nodiscard]] std::string summary_id() const;
};
```

`summary_id()` returns empty for no prototypes and otherwise uses the approved `appearance-v1-<16 hex>` canonical FNV-1a serialization. `merge_from()` feeds each source prototype through `update()` using `best_quality`, `confirmed=true`, `ambiguous=false`, `prediction_only=false`; the existing compatibility and four-prototype limits remain authoritative.

- [x] **Step 4: Write failing MemoryCore appearance integration tests**

Extend `visual_supplement()` test fixtures with an optional descriptor and quality, then assert:

```cpp
const auto applied = core.supplement_visual(domain, supplement);
ASSERT_TRUE(applied.accepted);
ASSERT_EQ(applied.snapshot.objects.size(), 1U);
EXPECT_EQ(applied.snapshot.objects.front().appearance_prototype_count, 1U);
EXPECT_EQ(applied.snapshot.objects.front().appearance_update_count, 1U);
EXPECT_FALSE(applied.snapshot.objects.front().appearance_summary_id.empty());
```

Add a byte-equivalent snapshot comparison proving non-finite, zero-norm, incompatible and low-quality descriptors leave the bank/counter unchanged while the otherwise valid visual supplement remains accepted and reports an appearance rejection reason.

- [x] **Step 5: Run MemoryCore test and verify RED**

Run:

```bash
colcon test --build-base build-stage2e --install-base install-stage2e --packages-select track_robot_semantic_memory --ctest-args -R test_memory_core --output-on-failure
```

Expected: compile failures for the missing descriptor, summary and result fields.

- [x] **Step 6: Implement MemoryCore-owned appearance banks**

Use these fields/signatures:

```cpp
struct VisualMemorySupplement {
  // existing fields...
  std::optional<AppearanceDescriptor> appearance_descriptor;
  double appearance_quality{0.0};
  bool prediction_only{false};
};

enum class ReidentificationState : std::uint8_t {
  kNotRequired = 0U, kPending = 1U, kConfirmed = 2U, kRejected = 3U};

struct MemoryObject {
  // existing fields...
  std::string appearance_summary_id;
  std::uint8_t appearance_prototype_count{0U};
  std::string appearance_encoder_id;
  std::string appearance_checkpoint_id;
  std::uint32_t appearance_descriptor_version{0U};
  ReidentificationState reidentification_state{ReidentificationState::kNotRequired};
};

struct VisualSupplementResult {
  bool accepted{false};
  bool appearance_accepted{false};
  std::string reason;
  std::string appearance_reason;
  MemoryUpdateResult snapshot;
};
```

Define `ReidentificationState` in `types.hpp`. Add
`std::map<GlobalObjectKey, AppearanceMemory> appearance_banks_`. Lazily create a
bank only after `AppearanceMemory::update()` accepts evidence; refresh summary
fields from the bank; erase the bank on capacity eviction and epoch reset. The
full supplement still succeeds if only optional appearance evidence is
rejected.

- [x] **Step 7: Rebuild and run Task 1 tests GREEN**

Run the build command from Step 2, then:

```bash
colcon test --build-base build-stage2e --install-base install-stage2e --packages-select track_robot_semantic_memory --ctest-args -R 'test_(appearance_memory|memory_core)' --output-on-failure
colcon test-result --test-result-base build-stage2e/track_robot_semantic_memory --verbose
```

Expected: both suites pass with zero failures/errors.

- [x] **Step 8: Commit Task 1**

```bash
git add src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/types.hpp src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/appearance_memory.hpp src/track_robot/track_robot_semantic_memory/src/appearance_memory.cpp src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_core.hpp src/track_robot/track_robot_semantic_memory/src/memory_core.cpp src/track_robot/track_robot_semantic_memory/test/test_appearance_memory.cpp src/track_robot/track_robot_semantic_memory/test/test_memory_core.cpp
git commit -m "feat: persist bounded appearance memory"
```

### Task 2: Add complete deterministic runtime re-identification assignment

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/reidentification.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/reidentification.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_reidentification.cpp`

**Interfaces:**
- Consumes: `hungarian_assignment()`, existing 0.60/0.25/0.15 re-ID score and `ReidentificationTracker` confirmation behavior.
- Produces: `RuntimeReidentificationFrame`, `RuntimeReidentificationCoordinator::process()` and deterministic per-target decisions keyed by old/new `GlobalObjectKey`.

- [x] **Step 1: Write failing 2x2 assignment and confirmation tests**

Define test fixtures with two candidates, two lost targets and all four pairs. Require global one-to-one assignment even when independent row maxima collide, invariant decisions under input permutation, and confirmation only on frames 1/2/3 for the same `(lost,candidate)` pair.

```cpp
const auto first = coordinator.process(frame(1U));
const auto second = coordinator.process(frame(2U));
const auto third = coordinator.process(frame(3U));
EXPECT_EQ(first.decisions.at(0).decision, ReidentificationDecision::kTentative);
EXPECT_EQ(second.decisions.at(0).consecutive_hits, 2U);
EXPECT_EQ(third.decisions.at(0).decision, ReidentificationDecision::kConfirmed);
EXPECT_NE(third.decisions.at(0).candidate_key,
          third.decisions.at(1).candidate_key);
```

Also add RED tests for duplicate keys, incomplete matrices, >64 candidates, >256 lost targets, >1,024 pairs, NaN scores, threshold equality, row ambiguity, column ambiguity, archived targets and frame gaps.

- [x] **Step 2: Build/run test and verify RED**

Run the Task 1 build command and:

```bash
colcon test --build-base build-stage2e --install-base install-stage2e --packages-select track_robot_semantic_memory --ctest-args -R test_reidentification --output-on-failure
```

Expected: compile failure because runtime frame/coordinator types do not exist.

- [x] **Step 3: Implement runtime frame validation and Hungarian assignment**

Add these public shapes:

```cpp
struct RuntimeReidentificationPair {
  GlobalObjectKey lost_key;
  GlobalObjectKey candidate_key;
  ProducerObjectKey expected_candidate_lidar_key;
  std::optional<VisualAssociationKey> expected_candidate_visual_key;
  LifecycleState lost_lifecycle{LifecycleState::kLost};
  bool domain_compatible{false};
  std::int64_t age_ns{0};
  double spatial_distance_m{0.0};
  double appearance_similarity{0.0};
  double geometry_similarity{0.0};
  double semantic_similarity{0.0};
};

struct RuntimeReidentificationFrame {
  std::uint64_t frame_index{0U};
  std::uint64_t memory_epoch_id{0U};
  std::vector<GlobalObjectKey> candidates;
  std::vector<GlobalObjectKey> lost_targets;
  std::vector<RuntimeReidentificationPair> pairs;
};

struct RuntimeReidentificationDecision {
  GlobalObjectKey lost_key;
  GlobalObjectKey candidate_key;
  ProducerObjectKey expected_candidate_lidar_key;
  std::optional<VisualAssociationKey> expected_candidate_visual_key;
  ReidentificationDecision decision{ReidentificationDecision::kRejectedGate};
  std::uint32_t consecutive_hits{0U};
  double combined_score{0.0};
  std::string reason;
};

class RuntimeReidentificationCoordinator {
public:
  explicit RuntimeReidentificationCoordinator(ReidentificationConfig config);
  RuntimeReidentificationResult process(const RuntimeReidentificationFrame & frame);
  void reset() noexcept;
};
```

Extend `ReidentificationConfig` with `ambiguity_margin=0.05`, `maximum_candidates=64`, `maximum_lost_targets=256`, `maximum_pairs=1024`. Validate the complete Cartesian matrix before state mutation. Use `cost=1-score` and `nextafter(1-minimum_combined_score,+inf)` unmatched cost. Treat the configured threshold as the virtual runner-up; block a match when either its row or column margin is below `ambiguity_margin`. Apply confirmation on a coordinator copy and commit only when the whole frame succeeds.

- [x] **Step 4: Run Task 2 GREEN**

Rebuild and run `test_reidentification`; expected zero failures/errors. Then run `test_hungarian_assignment` to protect the reused assignment primitive.

- [x] **Step 5: Commit Task 2**

```bash
git add src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/reidentification.hpp src/track_robot/track_robot_semantic_memory/src/reidentification.cpp src/track_robot/track_robot_semantic_memory/test/test_reidentification.cpp
git commit -m "feat: add deterministic runtime reidentification"
```

### Task 3: Build re-ID evidence and transfer identity atomically in MemoryCore

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_core.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_deterministic_replay.cpp`

**Interfaces:**
- Consumes: Task 1 banks and Task 2 runtime frame/decision types.
- Produces: `MemoryCore::make_reidentification_frame()`,
  `MemoryCore::apply_reidentification_states()` and guarded
  `MemoryCore::reidentify()`.

- [x] **Step 1: Write failing evidence-frame tests**

Create one lost object with a bank and two active replacements with compatible banks. Assert the returned frame is the complete 2x1 Cartesian matrix, sorted by global keys, uses maximum prototype cosine, bounded extent consistency and only permanent non-task semantic evidence. Add tests excluding archived/empty-bank/cross-epoch objects.

- [x] **Step 2: Write failing atomic transfer tests**

After creating an old lost object and a current replacement, call:

```cpp
const auto transferred = core.reidentify(
  domain, old_key, replacement_key,
  expected_replacement_lidar_key, expected_replacement_visual_key);
ASSERT_TRUE(transferred.accepted);
EXPECT_EQ(transferred.preserved_key, old_key);
EXPECT_EQ(transferred.snapshot.objects.size(), 1U);
EXPECT_EQ(transferred.snapshot.objects.front().key, old_key);
EXPECT_EQ(transferred.snapshot.objects.front().lidar_key,
          expected_replacement_lidar_key);
EXPECT_EQ(transferred.snapshot.events.back().type,
          MemoryEventType::kReidentified);
```

Add snapshot-equality rejection tests for stale/archived old key, inactive replacement, equal keys, changed LiDAR guard, changed visual guard and incompatible epoch/domain.

- [x] **Step 3: Build/run test and verify RED**

Run the Task 1 build, then `test_memory_core`; expected compile failures for the new APIs/event.

- [x] **Step 4: Implement deterministic frame creation**

Add:

```cpp
RuntimeReidentificationFrame make_reidentification_frame(
  const MemoryDomainKey & domain, std::uint64_t frame_index,
  const ReidentificationConfig & config) const;

struct ReidentificationStateUpdate {
  GlobalObjectKey key;
  ReidentificationState state{ReidentificationState::kNotRequired};
};

MemoryUpdateResult apply_reidentification_states(
  const MemoryDomainKey & domain,
  const std::vector<ReidentificationStateUpdate> & updates);
```

Compute spatial Euclidean distance, maximum compatible prototype cosine, extent similarity in `[0,1]`, and permanent semantic overlap. Return a complete bounded frame or throw before mutating any state.

- [x] **Step 5: Implement guarded identity transfer**

Add `MemoryEventType::kReidentified` and:

```cpp
struct ReidentificationTransferResult {
  bool accepted{false};
  std::string reason;
  GlobalObjectKey preserved_key;
  MemoryUpdateResult snapshot;
};

ReidentificationTransferResult reidentify(
  const MemoryDomainKey & domain,
  const GlobalObjectKey & old_key,
  const GlobalObjectKey & replacement_key,
  const ProducerObjectKey & expected_replacement_lidar_key,
  const std::optional<VisualAssociationKey> & expected_replacement_visual_key);
```

Validate every guard first. Preserve old key/first-seen/history/permanent labels/bank; move replacement metric/source/visual state; merge bounded compatible labels and bank; erase the replacement object/bank/index; map the replacement LiDAR key to the old key; mark confirmed for this snapshot and emit one `kReidentified` event.

- [x] **Step 6: Add deterministic replay regression**

Run the same leave/re-entry fixture twice and serialize `(object keys, lidar keys, appearance summary IDs, event types)`; assert byte-equivalent output.

- [x] **Step 7: Run Task 3 GREEN**

Rebuild, then run:

```bash
colcon test --build-base build-stage2e --install-base install-stage2e --packages-select track_robot_semantic_memory --ctest-args -R 'test_(memory_core|deterministic_replay)' --output-on-failure
```

Expected: zero failures/errors.

- [x] **Step 8: Commit Task 3**

Stage only the four Task 3 files and commit `feat: transfer reidentified objects atomically`.

### Task 4: Wire Stage 2E into ROS conversion, node and safe rollout configuration

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/CMakeLists.txt`
- Create: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/reidentification_calibration.hpp`
- Create: `src/track_robot/track_robot_semantic_memory/src/reidentification_calibration.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/ros_conversions.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/config/semantic_memory.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/config/phase2_association_baseline.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/config/phase2_camera_attachment.yaml`
- Create: `src/track_robot/track_robot_semantic_memory/test/test_reidentification_calibration.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_ros_conversions.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_ros_runtime.py`

**Interfaces:**
- Consumes: Task 1 supplement/bank summaries and Task 2/3 coordinator/frame/transfer APIs.
- Produces: public appearance/re-ID fields, `EVENT_REIDENTIFIED`, runtime copy-commit integration and fail-closed launch parameters.

- [x] **Step 1: Write failing ROS conversion tests**

Populate every appearance summary and re-ID field on `MemoryObject`; assert exact `SemanticObject` values. Convert `MemoryEventType::kReidentified`; assert `EVENT_REIDENTIFIED` and reason `object_reidentified`.

- [x] **Step 2: Write failing launch-contract tests**

Require normal YAML values:

```python
assert params['appearance_memory_enabled'] is True
assert params['reidentification_shadow_mode'] is True
assert params['reidentification_mutation_enabled'] is False
assert params['reidentification_calibration_status'] == 'uncalibrated'
```

Require the attachment overlay not to enable re-ID mutation. Source-contract assertions require descriptor copying, coordinator copy, core frame creation, confirmed decision handling, `reidentify()` call and fail-closed calibration-report validation.

Add pure JSON calibration tests for a valid in-memory
`stage2e_reidentification_v1` fixture and one-field drift in status, allowed
flag, maximum age, spatial distance, appearance threshold, combined threshold,
ambiguity margin and confirmation frames. Each drift must throw.

- [x] **Step 3: Run conversion/contract tests and verify RED**

Run the Task 1 build and:

```bash
colcon test --build-base build-stage2e --install-base install-stage2e --packages-select track_robot_semantic_memory --ctest-args -R 'test_(ros_conversions|launch_contract)' --output-on-failure
```

Expected: failures for missing fields, event mapping and parameters.

- [x] **Step 4: Implement conversion and configuration**

Map all summary/re-ID fields and event types. Declare the four safe-default
parameters plus the approved thresholds/limits. Implement
`validate_reidentification_calibration_report(const nlohmann::json &,
const ReidentificationCalibrationExpectation &)` in the new pure module and
register its source/test in CMake. If mutation is enabled, require non-shadow
mode, `status=calibrated`, a readable report,
`reidentification_allowed=true`, and exact agreement on age, distance,
appearance, score, ambiguity and confirmation values. No checked production
profile is added because no physical calibration report exists.

- [x] **Step 5: Integrate copy-commit runtime flow**

`make_visual_supplement()` converts the ROS `VisualDescriptor` and quality.
After all Stage 2D supplements succeed on `next_memory_core`, create/process
one complete re-ID frame on `next_reidentification`; update pending/rejected
states; in mutation mode transfer each confirmed one-to-one decision with its
captured LiDAR/visual guards. Fold attachment events belonging only to a
temporary replacement into the single successful `kReidentified` event.
Commit association coordinator, re-ID coordinator and core together only after
every operation succeeds. Reset re-ID confirmation on
domain/memory/LiDAR/visual epoch changes and visual source-time rollback.

- [x] **Step 6: Add bounded ROS runtime software test**

Extend the existing runtime fixture to publish deterministic synthetic leave/re-entry evidence when the test-only in-memory mutation profile is enabled. Assert one old global ID survives, one `EVENT_REIDENTIFIED` is emitted and the temporary duplicate disappears. Wrap every executor/process in `try/finally`, call `destroy_node()`/`rclpy.shutdown()`, and terminate/kill launched processes on timeout.

- [x] **Step 7: Run Task 4 GREEN**

Rebuild and run conversion, launch-contract and ROS runtime tests. The DDS runtime test may retain its documented opt-in skip; all non-skipped tests must pass.

- [x] **Step 8: Commit Task 4**

Stage only Task 4 files and commit `feat: wire stage 2e runtime reidentification`.

### Task 5: Stage 2E evidence, documentation, full regression and process cleanup

**Files:**
- Create: `rosbags/semantic_search/reports/phase2_stage2e_runtime_2026-07-16.json`
- Modify: `rosbags/semantic_search/reports/README.md`
- Modify: `docs/superpowers/plans/2026-07-15-semantic-search-phase2-generalized-3d-memory.md`
- Modify: `docs/superpowers/plans/2026-07-16-semantic-search-stage2e-runtime-appearance-reidentification.md`

**Interfaces:**
- Consumes: exact Task 1–4 commands/results.
- Produces: reproducible Stage 2E software checkpoint evidence and an explicit Stage 2E stop boundary.

- [x] **Step 1: Run formatting and source checks**

```bash
git diff --check
python3 -m pytest -q src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py
```

Expected: no whitespace errors and pytest passes.

- [x] **Step 2: Run fresh full four-package build/test**

```bash
colcon build --base-paths src/track_robot/track_robot_interfaces src/track_robot/track_robot_lidar_tracking src/track_robot/track_robot_semantic_memory src/track_robot_semantic_search --build-base build-stage2e-final --install-base install-stage2e-final --packages-select track_robot_interfaces track_robot_lidar_tracking track_robot_semantic_memory track_robot_semantic_search --cmake-args -DBUILD_TESTING=ON
env -u RUN_ROS_RUNTIME_TESTS PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --build-base build-stage2e-final --install-base install-stage2e-final --packages-select track_robot_interfaces track_robot_lidar_tracking track_robot_semantic_memory track_robot_semantic_search --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base build-stage2e-final --verbose
```

Expected: build exit 0; zero failures/errors; only the two documented opt-in
DDS test cases may skip in the default run. Both must pass in the explicit DDS
run.

- [x] **Step 3: Verify no test process remains**

```bash
ps -eo pid,ppid,stat,cmd
```

Inspect for `semantic_memory_node`, ROS test nodes, `ros2 bag play`, launch processes and Stage 2E test executors. Gracefully terminate only processes started during this plan; re-run the process listing and record zero remaining Stage 2E processes.

- [x] **Step 4: Write the evidence report**

Create valid JSON containing:

```json
{
  "stage": "2E",
  "status": "software_complete_physical_pilot_unverified",
  "date": "2026-07-16",
  "appearance_memory_runtime_integrated": true,
  "reidentification_runtime_integrated": true,
  "reidentification_mutation_default_enabled": false,
  "reidentification_thresholds_field_calibrated": false,
  "physical_reentry_pilot_executed": false,
  "ros_processes_remaining_after_test": 0,
  "verification": []
}
```

Populate `verification` with exact commands, exit codes and observed counts; do not invent a physical success rate.

- [x] **Step 5: Update roadmap/report index and self-review requirements**

Mark only the Stage 2E software/runtime checkpoint complete. Keep physical
Checkpoint D evidence pending for Stage 2G. Re-read every design section and
map it to a test/report field; scan for incomplete-work markers and
contradictory safe-default claims.

- [x] **Step 6: Request independent code review and fix findings**

Review the full Stage 2E diff against the approved design. Fix every Critical/Important finding with a new failing regression test, rerun the covering suite, then rerun full verification if production code changed.

- [x] **Step 7: Commit Stage 2E evidence/docs and stop**

Stage only the five Task 5 documentation/evidence files, commit
`docs: record stage 2e software checkpoint`, verify the final diff/status, and
pause without entering Stage 2F or Stage 2G.
