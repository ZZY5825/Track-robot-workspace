# Phase 4A Fixed-Base Advisory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a one-launch, stationary-robot Phase 0–4A test that uses real ZED and LiDAR data to print the green bottle's rough `base_link` position and a collision-checked approach suggestion without any motion interface.

**Architecture:** A dedicated fixed-base session bridge supplies a test-only, stable local-session epoch without odometry or IMU. Existing YOLO-World, camera/LiDAR association, semantic memory, and planning-only A* remain in the data path; a conservative Phase 3A selector and Phase 4A text advisor are isolated adapters rather than changes to production best-candidate calibration.

**Tech Stack:** ROS 2 Foxy, Python 3.8, rclpy, existing `track_robot_interfaces`, pytest, ament/colcon, RViz2.

## Global Constraints

- ROS Domain is 20.
- The robot is physically stationary by operator assertion for the entire test.
- Do not start Bunker or IMU nodes.
- Do not subscribe to IMU or odometry in Phase 4A.
- Do not publish `/odom`, TF, `Twist`, `cmd_vel`, navigation goals, action goals, or controller input.
- Keep `/semantic_memory/best_candidate` fail-closed and uncalibrated.
- Every stale, ambiguous, inconsistent, or invalid state publishes `NOT_READY`; never retain stale `READY` output.
- Live success must use real camera and LiDAR observations; no synthetic target substitution.

---

### Task 1: Fixed-base local-session bridge

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/fixed_base_session.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/fixed_base_session_node.py`
- Create: `src/track_robot_semantic_search/test/test_fixed_base_session.py`
- Modify: `src/track_robot_semantic_search/setup.py`

**Interfaces:**
- Produces: `FixedBaseSession(epoch_id: int, frame_id: str)` and `build_state(stamp_ns: int) -> FixedBaseState`.
- ROS output: `/semantic_search/phase4a/localization_state` as `SemanticLocalizationState`, reliable and transient-local.
- Depends on no sensor, pose, TF, odometry, IMU, or motion topic.

- [ ] **Step 1: Write failing pure contract tests**

Add tests that require one stable non-zero epoch, `base_link` local-session
fields, a new epoch for a new session, and rejection of zero/empty inputs:

```python
def test_fixed_base_session_keeps_one_local_epoch():
    session = FixedBaseSession(epoch_id=71, frame_id='base_link')
    first = session.build_state(1_000_000_000)
    second = session.build_state(1_100_000_000)
    assert first.localization_epoch_id == second.localization_epoch_id == 71
    assert first.memory_mode == MEMORY_LOCAL_SESSION
    assert first.canonical_frame_id == 'base_link'
    assert first.local_frame_id == 'base_link'
    assert first.local_healthy is True
    assert first.reason == 'operator_asserted_fixed_base_test'
```

- [ ] **Step 2: Verify RED**

Run:

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_fixed_base_session.py
```

Expected: collection fails because `fixed_base_session` does not exist.

- [ ] **Step 3: Implement the pure session**

Define immutable:

```python
@dataclass(frozen=True)
class FixedBaseState:
    stamp_ns: int
    memory_mode: int
    localization_epoch_id: int
    canonical_frame_id: str
    local_frame_id: str
    base_frame_id: str
    local_healthy: bool
    reason: str

class FixedBaseSession:
    def __init__(self, epoch_id: int, frame_id: str = 'base_link'): ...
    def build_state(self, stamp_ns: int) -> FixedBaseState: ...
```

Validate `epoch_id > 0`, non-negative stamps, and a 1–128 character frame.

- [ ] **Step 4: Implement the ROS adapter**

`FixedBaseSessionNode` declares `state_topic`, `frame_id`, `publish_rate_hz`,
and optional positive `epoch_id`. When no epoch is provided, allocate a
positive 63-bit value once at construction from `time.time_ns()`. Publish:

```python
message.memory_mode = SemanticLocalizationState.MEMORY_LOCAL_SESSION
message.localization_epoch_id = state.localization_epoch_id
message.canonical_frame_id = state.canonical_frame_id
message.local_frame_id = state.local_frame_id
message.base_frame_id = state.base_frame_id
message.local_healthy = True
message.world_healthy = False
message.reason = state.reason
```

Do not create subscriptions, clients, services, or action clients.

- [ ] **Step 5: Verify GREEN and register executable**

Add:

```python
'semantic_search_phase4a_fixed_base = '
'track_robot_semantic_search.fixed_base_session_node:main',
```

Run the Task 1 tests and `python3 -m pytest -q
src/track_robot_semantic_search/test/test_phase4_planning_node_contract.py`.
Expected: all pass.

- [ ] **Step 6: Commit Task 1**

Commit only the four Task 1 files with message:
`feat: add phase4a fixed-base session bridge`.

---

### Task 2: Conservative Phase 3A selector

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/phase4a_selector.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/phase4a_selector_node.py`
- Create: `src/track_robot_semantic_search/test/test_phase4a_selector.py`
- Modify: `src/track_robot_semantic_search/setup.py`

**Interfaces:**
- Consumes: `/semantic_memory/diagnostic_ranking` as `SemanticObjectArray`.
- Produces: `/semantic_search/phase4a/selected_target` as `SemanticObjectArray`.
- Produces: `/semantic_search/phase4a/selector_diagnostics`.
- Pure API:

```python
SelectorConfig
ObjectCandidate
SelectionSnapshot
SelectionResult
FixedBaseTargetSelector.update(snapshot: SelectionSnapshot) -> SelectionResult
```

- [ ] **Step 1: Write failing selection-gate tests**

Create individual tests for:

- three consecutive snapshots required;
- one skipped/mutated compound object key resets confirmation;
- query mismatch;
- lifecycle not confirmed;
- support other than camera+LiDAR;
- invalid/non-`base_link` position;
- relevance below 0.50;
- top-to-second margin below 0.08;
- uncertainty above 0.50;
- five-sample XY spread above 0.35 m;
- age above 1.0 s;
- a valid target returns `READY`.

The valid fixture must use:

```python
ObjectCandidate(
    memory_epoch_id=11,
    global_object_id=42,
    localization_epoch_id=71,
    query_id=2026072801,
    query_version=1,
    lifecycle='confirmed',
    support='camera_lidar',
    position_frame_id='base_link',
    position_valid=True,
    x=1.60,
    y=-0.20,
    z=0.35,
    relevance=0.72,
    uncertainty=0.28,
    last_seen_ns=now_ns - 50_000_000,
)
```

- [ ] **Step 2: Verify RED**

Run the selector test file. Expected: import failure for
`phase4a_selector`.

- [ ] **Step 3: Implement the pure selector**

Use immutable dataclasses and a bounded `deque(maxlen=5)`. Sort candidates by
`(-relevance, memory_epoch_id, global_object_id)`. Evaluate fail-closed gates
before changing confirmation state. Return these exact reasons:

```text
no_target
ambiguous_target
query_mismatch
target_not_confirmed
no_camera_lidar_support
invalid_position
frame_mismatch
below_test_relevance
uncertainty_too_high
stale_target
confirming_target
unstable_position
ready
```

XY spread is the maximum Euclidean distance from any sample to the rolling
median XY.

- [ ] **Step 4: Implement the ROS selector adapter**

The node declares all thresholds, `expected_query_id`, and
`expected_query_version`. It maps `SemanticObject` constants exactly, calls the
pure selector on every snapshot, and publishes a one-object array only for
`ready`. Empty arrays preserve the input memory epoch and have a fresh header.
Use transient-local reliable QoS for the selected target.

- [ ] **Step 5: Verify GREEN and register executable**

Add:

```python
'semantic_search_phase4a_selector = '
'track_robot_semantic_search.phase4a_selector_node:main',
```

Run selector tests and the complete semantic-search test directory.

- [ ] **Step 6: Commit Task 2**

Commit Task 2 files with message:
`feat: add conservative phase4a target selector`.

---

### Task 3: Human-readable Phase 4A advisor

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/phase4a_advisor.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/phase4a_advisor_node.py`
- Create: `src/track_robot_semantic_search/test/test_phase4a_advisor.py`
- Modify: `src/track_robot_semantic_search/setup.py`

**Interfaces:**
- Consumes selected target, Phase 4 goal, path, and diagnostics.
- Produces `/semantic_search/phase4a/advice` and diagnostics.
- Pure API:

```python
AdvisoryTarget
AdvisoryGoal
AdvisoryInput
AdvisoryResult
build_advice(value: AdvisoryInput) -> AdvisoryResult
```

- [ ] **Step 1: Write failing formatting and invalidation tests**

Tests require:

```text
READY target="green bottle"
position=front 1.60m, right 0.20m
range=1.61m
bearing=-7.1deg
goal=(0.81,-0.10)m
path=clear
ADVISORY_ONLY
```

Also require `front-left`, `behind-left/right`, computed path length, compound
IDs, and `NOT_READY reason=<exact>` when planner status is not `PASS`, target
is absent, references differ, or path/goal is empty. Assert output length is
at most 512 ASCII characters and contains no newline.

- [ ] **Step 2: Verify RED**

Run advisor tests. Expected: import failure for `phase4a_advisor`.

- [ ] **Step 3: Implement pure advice generation**

Use `atan2(y, x)` for bearing and `hypot(x, y)` for range. ROS `base_link`
uses positive X forward and positive Y left. Sum consecutive path segment
lengths. `READY` is allowed only when target, selected goal, non-empty path,
and planner reason `planned` share the same memory/global/localization/query
references.

- [ ] **Step 4: Implement the ROS advisor**

Subscribe to:

```text
/semantic_search/phase4a/selected_target
/semantic_search/phase4/selected_goal
/semantic_search/phase4/planned_path
/semantic_search/phase4/diagnostics
/semantic_memory/tasks
```

On every planner diagnostic, build a new result. A failure immediately
publishes `NOT_READY`; it clears cached goal/path so an old `READY` cannot
reappear. Diagnostics include target/goal coordinates, path length, IDs,
confidence, uncertainty, planner latency, and `advisory_only=true`.

- [ ] **Step 5: Verify GREEN and register executable**

Register `semantic_search_phase4a_advisor`; run advisor tests plus Phase 4 node
contract tests.

- [ ] **Step 6: Commit Task 3**

Commit Task 3 files with message:
`feat: add phase4a local approach advice`.

---

### Task 4: One-launch Phase 4A stack and safety contracts

**Files:**
- Create: `src/track_robot/track_robot_semantic_memory/config/phase4a_test.yaml`
- Create: `src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`
- Create: `src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`
- Create: `src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`
- Modify: `src/track_robot/track_robot_bringup/CMakeLists.txt`

**Interfaces:**
- Operator entry: `ros2 launch track_robot_bringup semantic_search_phase4a.launch.py`.
- Default sensor inputs: ZED image and `/rslidar_points`.
- Default output: Phase 4A advice, diagnostics, and existing Phase 4 RViz
  planning products.

- [ ] **Step 1: Write failing launch/source contract tests**

Parse the launch/config source and assert:

- sensor bringup receives `start_base=false` and `start_imu=false`;
- camera and LiDAR are true;
- no `bunker_base`, `cmd_vel`, `Twist`, navigation action, or `/odom`;
- fixed-base state uses the dedicated topic;
- semantic memory `localization_topic` and planner `localization_topic` both
  use it;
- production `best_candidate_threshold_calibrated` is false;
- planner target is Phase 3A selected target;
- `planning_only=true`;
- local obstacle map and RViz are present.

- [ ] **Step 2: Verify RED**

Run the new launch contract test. Expected: missing launch/config files.

- [ ] **Step 3: Add Phase 4A configs**

Copy the existing Phase 1–3 test profile only where required. Set:

```yaml
localization_topic: /semantic_search/phase4a/localization_state
best_candidate_threshold_calibrated: false
publish_diagnostic_ranking: true
camera_only_memory_enabled: true
association_shadow_mode: false
camera_attachment_enabled: true
```

Configure the planner with:

```yaml
selected_target_topic: /semantic_search/phase4a/selected_target
localization_topic: /semantic_search/phase4a/localization_state
planning_only: true
```

- [ ] **Step 4: Implement the aggregate launch**

Use existing include patterns to start:

1. camera and LiDAR sensor bringup only;
2. fixed-base bridge;
3. YOLO-World perception with DINO descriptors;
4. LiDAR tracklet manager;
5. semantic memory and visualizer with Phase 4A config;
6. selector;
7. local obstacle map;
8. planning-only Phase 4 planner;
9. advisor and live overlay;
10. optional Phase 4 RViz.

Expose model/checkpoint paths and `start_rviz`; do not expose any base/IMU
enable argument.

- [ ] **Step 5: Verify GREEN**

Run new launch contract, existing bringup launch contracts, and
`colcon build --symlink-install --packages-select
track_robot_semantic_search track_robot_semantic_memory
track_robot_bringup track_robot_safety`.

- [ ] **Step 6: Commit Task 4**

Commit Task 4 files with message:
`feat: add one-command phase4a test launch`.

---

### Task 5: Deterministic and live acceptance reporting

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/phase4a_validation.py`
- Create: `src/track_robot_semantic_search/test/test_phase4a_validation.py`
- Create: `docs/guides/semantic-search/phase4a-fixed-base-test.md`
- Modify: `src/track_robot_semantic_search/setup.py`
- Modify: `docs/README.md`

**Interfaces:**
- Command:

```bash
ros2 run track_robot_semantic_search semantic_search_phase4a_validate \
  --query "green bottle" \
  --query-id <positive> \
  --query-version 1 \
  --duration-sec 25 \
  --output <path>
```

- Output schema: `phase4a_validation/1.0.0`.

- [ ] **Step 1: Write failing report-evaluation tests**

Test PASS requires:

- fixed-base localization topic healthy with one epoch;
- non-empty Phase 1 observations with one query key;
- at least three consecutive Phase 2 samples with one compound object key,
  `SUPPORT_CAMERA_LIDAR`, and valid `base_link` position;
- one Phase 3A selected target with matching references;
- at least one Phase 4 candidate, goal, and path;
- one `READY` advice containing `ADVISORY_ONLY`;
- zero motion publishers.

Missing target, ambiguous target, target loss, invalid position, blocked path,
stale data, and epoch reset must each produce the exact `NOT_READY` reason.

- [ ] **Step 2: Verify RED**

Run the validation test. Expected: missing module.

- [ ] **Step 3: Implement bounded live collection and pure evaluation**

Follow `phase04_live_validation.py` topic/QoS patterns, but report fixed-base
session evidence and Phase 4A advice. Store bounded sets and min/max/count
summaries rather than every observed epoch. Never classify absent data as
zero-valued evidence.

- [ ] **Step 4: Add operator guide and executable**

The guide contains:

```bash
export ROS_DOMAIN_ID=20
ros2 launch track_robot_bringup semantic_search_phase4a.launch.py
ros2 run track_robot_semantic_search semantic_search_query \
  "green bottle" --query-id <positive> --query-version 1
ros2 topic echo /semantic_search/phase4a/advice
```

It states the stationary assumption, no-IMU/no-Bunker boundary, expected
`READY` text, failure reasons, and shutdown procedure.

- [ ] **Step 5: Run full verification**

Run:

```bash
python3 -m pytest -q src/track_robot_semantic_search/test
colcon test --packages-select \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup \
  track_robot_safety
git --git-dir=.local-git --work-tree=. diff --check
```

Expected: zero failures; runtime-only skips must be listed explicitly.

- [ ] **Step 6: Run the real green-bottle test**

Start the aggregate launch in ROS Domain 20, submit one explicit query, run the
25-second validator, inspect RViz/advice, save evidence under
`artifacts/semantic-search/phase4a-<date>/`, then stop the aggregate launch.
Use host process inspection to prove no test-owned node remains.

- [ ] **Step 7: Commit Task 5**

Commit only Task 5 files and generated small report artifacts with message:
`test: validate phase4a fixed-base advisory`.
