# Stage 2F Runtime Task Ranking and Memory Services Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect the existing bounded task-relevance and memory-service policies to the live semantic-memory ROS runtime without moving identity or appearance ownership out of `MemoryCore`.

**Architecture:** A pure `RuntimeTaskServiceCoordinator` synchronizes complete immutable `MemoryCore` snapshots into a bounded read model, preserves inspection state for surviving public keys, owns the replaceable task overlay, and serves deterministic full-object views. `semantic_memory_node` remains a thin transport adapter and performs core/store reset on copies before commit; production best-candidate publication remains fail-closed until threshold calibration.

**Tech Stack:** ROS 2 Foxy, C++17, rclcpp, ament/colcon, GoogleTest, pytest, bounded ROS interfaces.

## Global Constraints

- Stop after Stage 2F; do not implement Stage 2G evaluation or claim Phase 2 completion.
- `MemoryCore` remains the only owner of global IDs, metric state, permanent semantics and appearance banks.
- Runtime read-model synchronization is complete and bounded to 256 objects; query pages are bounded to 64.
- Task state and inspection state never change global IDs, prototypes, lifecycle, spatial state or permanent labels.
- Checked profiles keep `best_candidate_threshold_calibrated=false` and publish no valid winner without calibration.
- `/semantic_memory/best_candidate` is a reliable transient-local `SemanticObjectArray` containing zero or one object.
- Descriptor queries never replace the active task.
- Reset advances the authoritative memory epoch before clearing and is copy-committed with the service read model.
- Every ROS process started by tests is stopped before handoff.
- Preserve unrelated untracked build/install/calibration artifacts and stage only named Stage 2F files.

---

### Task 1: Add a transactional bounded runtime task/service coordinator

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_services.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/memory_services.cpp`
- Create: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/runtime_task_services.hpp`
- Create: `src/track_robot/track_robot_semantic_memory/src/runtime_task_services.cpp`
- Create: `src/track_robot/track_robot_semantic_memory/test/test_runtime_task_services.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/CMakeLists.txt`

**Interfaces:**
- Consumes: `MemoryUpdateResult`, `AppearancePrototype`, `TaskRelevanceOverlay`, `MemoryServiceStore` and public `GlobalObjectKey`.
- Produces: `MemoryServiceStore::synchronize()`, `RuntimeObjectView`, `RuntimeTaskServiceCoordinator::synchronize()`, `accept_task()`, `query_active()`, `query_descriptor()`, `mark_inspected()`, `best_candidate()` and `reset_to_epoch()`.

- [x] **Step 1: Write failing store synchronization tests**

Extend `test_memory_services.cpp` with tests requiring `synchronize()` to reject duplicate/wrong-epoch/>256 records transactionally, preserve inspection for a surviving same-epoch key, prune disappeared keys, and clear inspection on epoch change.

Use the API:

```cpp
store.synchronize(7U, {record(7U, 1U, 0.8), record(7U, 2U, 0.7)});
store.mark_inspected({7U, 1U}, InspectionState::kComplete);
store.synchronize(7U, {record(7U, 1U, 0.9)});
EXPECT_EQ(store.get({7U, 1U}).record->inspection,
  InspectionState::kComplete);
EXPECT_EQ(store.get({7U, 2U}).reason, ServiceReason::kNotFound);
```

- [x] **Step 2: Build and verify the synchronization test is RED**

Run:

```bash
colcon build --base-paths src/track_robot/track_robot_interfaces src/track_robot/track_robot_semantic_memory --build-base build-stage2f --install-base install-stage2f --packages-select track_robot_interfaces track_robot_semantic_memory --cmake-args -DBUILD_TESTING=ON
```

Expected: compile failure because `MemoryServiceStore::synchronize()` does not exist.

- [x] **Step 3: Implement transactional store synchronization**

Add:

```cpp
void synchronize(
  std::uint64_t memory_epoch_id,
  const std::vector<MemoryServiceRecord> & records);
```

Validate the nonzero epoch, at most 256 records, every record's epoch/value/state and unique key into a temporary map. Preserve prior inspection only when the epoch and key are unchanged; commit the epoch and map together after all validation succeeds.

- [x] **Step 4: Write failing runtime coordinator tests**

Create `test_runtime_task_services.cpp` covering:

- compatible maximum-prototype task scoring;
- exact normalized permanent-label/query match only;
- active-task replacement and descriptor-query isolation;
- deterministic query pagination/filtering;
- inspection idempotence and surviving-key preservation;
- calibrated/uncalibrated best-candidate behavior;
- epoch reset and stale-key results;
- incomplete/duplicate/oversized snapshot rejection without mutation.

Use these public shapes:

```cpp
struct RuntimeObjectView {
  MemoryObject object;
  InspectionState inspection{InspectionState::kNotInspected};
  std::optional<SemanticTaskKey> active_task;
  double task_relevance{0.0};
};

struct RuntimeObjectQueryResult {
  bool accepted{false};
  ServiceReason reason{ServiceReason::kInvalidRequest};
  std::vector<RuntimeObjectView> objects;
  std::uint64_t next_page_token{0U};
  bool has_more{false};
};

class RuntimeTaskServiceCoordinator {
public:
  RuntimeTaskServiceCoordinator(
    TaskRelevanceConfig relevance_config,
    BestCandidateConfig best_candidate_config,
    std::uint64_t initial_epoch);
  void synchronize(
    const MemoryUpdateResult & snapshot,
    const std::map<GlobalObjectKey,
      std::vector<AppearancePrototype>> & appearance);
  bool accept_task(
    const SemanticTaskEvidence & task, std::string query_text,
    std::uint64_t producer_epoch_id, std::int64_t source_stamp_ns);
  RuntimeObjectQueryResult query_active(
    const SemanticTaskKey & expected_task,
    const QueryMemoryRequest & request) const;
  RuntimeObjectQueryResult query_descriptor(
    const SemanticTaskEvidence & task, const std::string & query_text,
    const QueryMemoryRequest & request) const;
  GetRuntimeObjectResult get(const GlobalObjectKey & key) const;
  InspectionResult mark_inspected(
    const GlobalObjectKey & key, InspectionState state);
  BestRuntimeCandidateResult best_candidate() const;
  void reset_to_epoch(std::uint64_t new_epoch, std::string reason);
};
```

- [x] **Step 5: Verify coordinator tests are RED**

Run the Task 1 build and:

```bash
colcon test --build-base build-stage2f --install-base install-stage2f --packages-select track_robot_semantic_memory --ctest-args -R 'test_(memory_services|runtime_task_services)$' --output-on-failure --return-code-on-test-failure
```

Expected: missing header/type/API compile failures.

- [x] **Step 6: Implement the minimal coordinator**

Synchronize exact object keys and appearance-bank subsets, copy at most 256 objects, generate permanent semantic evidence only for exact lowercased/collapsed-whitespace label/query equality, and use the existing scorer/store for all ranking/filtering. Query-descriptor builds a temporary overlay/store and never modifies active state. Task producer change or source-time rollback clears the old overlay before accepting the new valid task.

- [x] **Step 7: Run Task 1 GREEN and commit**

Run the two targeted suites plus `test_task_relevance_scorer`. Expected: zero failures/errors. Commit:

```bash
git add src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_services.hpp src/track_robot/track_robot_semantic_memory/src/memory_services.cpp src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/runtime_task_services.hpp src/track_robot/track_robot_semantic_memory/src/runtime_task_services.cpp src/track_robot/track_robot_semantic_memory/test/test_memory_services.cpp src/track_robot/track_robot_semantic_memory/test/test_runtime_task_services.cpp src/track_robot/track_robot_semantic_memory/CMakeLists.txt
git commit -m "feat: add bounded runtime task service state"
```

### Task 2: Add authoritative epoch reset and Stage 2F ROS conversions

**Files:**
- Modify: `src/track_robot/track_robot_interfaces/msg/SemanticMemoryEvent.msg`
- Modify: `src/track_robot/track_robot_interfaces/test/test_phase2_interfaces.py`
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_core.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/ros_conversions.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/ros_conversions.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_ros_conversions.cpp`

**Interfaces:**
- Consumes: current domain, `MemoryDomainTracker::advance_epoch()`, `RuntimeObjectView` and existing semantic event conversion.
- Produces: `MemoryCore::reset()`, `semantic_object_from_runtime_view()` and `EVENT_INSPECTION_CHANGED=10`.

- [x] **Step 1: Write failing core reset tests**

Require `MemoryCore::reset(domain)` to advance epoch, clear objects/source/appearance state, reset global object IDs, emit exactly one `kMemoryReset`, reject the wrong domain transactionally, and wrap max epoch to 1.

- [x] **Step 2: Write failing conversion/interface tests**

Require a `RuntimeObjectView` to populate `inspection_state`, `active_query_id`, `active_query_version` and `task_relevance` without changing permanent object fields. Require `MemoryEventType::kInspectionChanged` to map to `EVENT_INSPECTION_CHANGED`.

- [x] **Step 3: Build/run and verify RED**

Run the Task 1 build, `test_memory_core`, `test_ros_conversions`, and the interface pytest. Expected: missing reset/view/event APIs or constant.

- [x] **Step 4: Implement reset and conversions**

Add:

```cpp
MemoryUpdateResult reset(const MemoryDomainKey & domain);

track_robot_interfaces::msg::SemanticObject semantic_object_from_runtime_view(
  const RuntimeObjectView & view,
  const MemoryDomainKey & domain);
```

Reset validates the domain before advancing/clearing. Runtime-view conversion delegates permanent fields to `semantic_object_from_memory()` and overlays only inspection/task fields. Add `kInspectionChanged` and the public ROS constant/reason mapping.

- [x] **Step 5: Run Task 2 GREEN and commit**

Run all three covering suites; expected zero failures/errors. Commit the eight named files with `feat: add epoch-safe runtime memory views`.

### Task 3: Wire live tasks, services and best-candidate publication

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/config/semantic_memory.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/config/phase2_association_baseline.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/config/phase2_camera_attachment.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_ros_runtime.py`

**Interfaces:**
- Consumes: `SemanticTask`, the four existing service types, `RuntimeTaskServiceCoordinator`, `MemoryCore::reset()` and runtime-view conversions.
- Produces: task subscription, four live service servers, enriched active snapshots and zero/one best-candidate snapshots.

- [x] **Step 1: Write failing launch/config contract tests**

Require the node source and all checked configs to expose:

```yaml
tasks_topic: /semantic_memory/tasks
task_queue_depth: 1
best_candidate_topic: /semantic_memory/best_candidate
publish_best_candidate: true
task_appearance_weight: 0.75
task_semantic_weight: 0.25
best_candidate_threshold_calibrated: false
best_candidate_minimum_relevance: 1.0
```

Also require creation of all four service types and the reliable transient-local best-candidate publisher.

Extend the opt-in `RuntimeProbe` at the same time with a task publisher,
best-candidate subscriber and four service clients. Add a Stage 2F fixture that
creates one object with appearance, publishes a matching task, gets/queries
it, marks it inspected, resets memory and verifies the old key becomes stale.
This fixture is the failing executable acceptance test for the transport code
implemented below.

- [x] **Step 2: Run contract test and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py
```

Expected: missing Stage 2F parameter/topic/service assertions fail.

After the RED contract assertion, build the current code and run the explicit
DDS suite. Expected: the new Stage 2F fixture fails transport discovery because
the node has no task/service/best-candidate endpoints yet.

- [x] **Step 3: Implement parameter and transport setup**

Declare/validate the Stage 2F parameters, create a reliable depth-one task subscription, reliable transient-local best-candidate publisher, and the four services. Reject a calibrated threshold outside `[0,1]` and invalid task weights/queue bounds at startup.

- [x] **Step 4: Implement synchronized publishing and callbacks**

After every successful core result, collect the bounded appearance banks, synchronize the coordinator, enrich active snapshots, and publish a zero/one best-candidate snapshot. Implement stable `ServiceReason` strings and exact query-mode rules. Mark-inspected publishes an event only on change. Reset copies core/coordinator, checks matching next epochs, commits them together, clears association/re-ID confirmation, then publishes empty active/best snapshots and the reset event.

- [x] **Step 5: Run focused node contracts and existing core regressions**

Run `test_launch_contract`, `test_ros_conversions`, `test_runtime_task_services`, `test_memory_core` and `test_deterministic_replay`. Expected: zero failures/errors.

- [x] **Step 6: Commit Task 3**

Stage only the six Task 3 files and commit `feat: expose stage 2f runtime task services`.

### Task 4: Validate executable DDS acceptance and cleanup

**Files:**
- No source changes expected; findings return to the owning TDD task.

**Interfaces:**
- Consumes: Stage 2E synthetic camera/LiDAR fixture, task topic, best-candidate topic and four services.
- Produces: direct execution evidence for the Stage 2F end-to-end DDS test and process cleanup.

- [x] **Step 1: Audit the Stage 2F DDS fixture against the acceptance sequence**

Confirm that the Task 3 test does all of the following:

1. launch a test-only config with camera attachment and best threshold enabled;
2. create one confirmed object and accepted appearance prototype;
3. publish a matching `SemanticTask` and observe exactly one best candidate;
4. get/query the same public key;
5. mark it inspected and observe an empty best-candidate array plus one inspection event;
6. reset using the current epoch, observe a new epoch/empty memory, and verify the old key returns `stale_epoch`;
7. clean the node and child process in the outermost `finally`.

- [x] **Step 2: Build and run the explicit DDS suite**

Run outside restricted DDS sandbox:

```bash
RUN_ROS_RUNTIME_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --build-base build-stage2f --install-base install-stage2f --packages-select track_robot_semantic_memory --ctest-args -R test_ros_runtime --output-on-failure --return-code-on-test-failure
```

Expected: Stage 2B, 2E and 2F cases all pass with zero skipped.

- [x] **Step 3: Verify deterministic waits**

Inspect the fixture to ensure every service, topic subscription and exact
snapshot/event/key transition is observable; a fixed sleep may aid spinning
but cannot be the success condition. If a gap is found, return to Task 3,
write the failing assertion, then fix and rerun.

- [x] **Step 4: Verify cleanup**

Inspect `ps -eo pid,ppid,stat,cmd` for semantic-memory nodes, test executors and
`ros2 bag play`; stop only processes started by the test and require zero
remaining.

### Task 5: Full verification, review and Stage 2F evidence

**Files:**
- Create: `artifacts/semantic_search/reports/phase2_stage2f_runtime_2026-07-17.json`
- Modify: `artifacts/semantic_search/reports/README.md`
- Modify: `src/track_robot/track_robot_semantic_memory/README.md`
- Modify: `docs/development/plans/semantic-search/2026-07-15-semantic-search-phase2-generalized-3d-memory.md`
- Modify: `docs/development/plans/semantic-search/2026-07-17-semantic-search-stage2f-runtime-task-services.md`

**Interfaces:**
- Consumes: exact build/test/review/process results.
- Produces: a reproducible Stage 2F software checkpoint that explicitly leaves calibration and physical evaluation pending.

- [x] **Step 1: Run source and configuration checks**

Run `git diff --check`, JSON/YAML parsing, direct launch-contract pytest and scan checked profiles for the uncalibrated best-candidate default.

- [x] **Step 2: Run fresh full four-package verification**

Use new `build-stage2f-final`/`install-stage2f-final` directories, build the interfaces, LiDAR tracking, semantic memory and semantic search packages with testing, then run all tests and `colcon test-result --all --verbose`. Expected: zero failures/errors; only documented opt-in DDS test cases may skip in the default run.

- [x] **Step 3: Verify a clean Git archive**

Archive the Stage 2F code HEAD into a new `/tmp` directory, perform a fresh four-package build/test there, and require the same zero-failure result. Do not depend on untracked calibration/build artifacts.

- [x] **Step 4: Request independent review and resolve findings**

Review task/store correctness and ROS/reset/cleanup behavior separately. Fix every Critical/Important finding with a RED/GREEN regression and rerun affected plus full suites.

- [x] **Step 5: Write evidence and update documentation**

The report must include exact counts and:

```json
{
  "stage": "2F",
  "status": "software_complete_threshold_and_physical_pilot_unverified",
  "best_candidate_threshold_calibrated": false,
  "physical_task_ranking_pilot_executed": false,
  "synthetic_dds_service_test_passed": true,
  "ros_processes_remaining_after_test": 0
}
```

Document the zero/one best-candidate contract, live services, safety default and Stage 2G boundary. Mark every completed checkbox only after direct evidence exists.

- [x] **Step 6: Commit evidence and stop**

Commit the five Task 5 files as `docs: record stage 2f software checkpoint`, verify tracked status is clean, confirm no ROS process remains, and pause without entering Stage 2G.

## Completion evidence — 2026-07-17

- Workspace and clean-archive four-package regressions: 580 tests, 0 errors,
  0 failures, 3 expected opt-in DDS skips.
- Explicit local-DDS suite: 3 tests, 0 errors, 0 failures, 0 skipped.
- Independent core and ROS/runtime re-reviews: approved with no remaining
  Critical or Important findings after seven Important findings were closed by
  RED/GREEN regressions.
- Production best-candidate threshold: uncalibrated and fail-closed in every
  checked profile.
- Physical task-ranking pilot: not executed; deferred to Stage 2G.
- Post-test semantic-memory/visualizer/rosbag/test process count: zero.
