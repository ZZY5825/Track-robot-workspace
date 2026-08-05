# Phase 5A Bounded Active Semantic Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fail-closed `SearchForObject` workflow that starts with passive Phase 1–3 observation, then performs explicitly authorized, bounded, in-place Nav2 Spin observations when evidence is unresolved, and hands a confirmed authoritative target reference to the unchanged Phase 4 pipeline.

**Architecture:** `ActiveSearchManager` owns the search task, deterministic heading policy, and bounded multi-view evidence. It publishes the existing `SearchMotionIntent`; a separate navigation adapter validates authorization and safety before translating rotation-only intents into Nav2 `Spin` actions whose velocity commands continue through the existing safety supervisor and cmd_vel gate. Phase 2 remains the only owner of global object IDs, Phase 3 remains the target-selection authority, and Phase 4 remains the only semantic-approach planner/executor.

**Tech Stack:** Ubuntu/Jetson runtime already used by the workspace, ROS 2 Foxy, Python 3.8, `rclpy`, ROS 2 actions, `nav2_msgs/action/Spin`, `nav2_recoveries/Spin`, `std_srvs/Trigger`, `diagnostic_msgs/DiagnosticArray`, RViz2, pytest, colcon.

## Global Constraints

- Work from the tested official `main` baseline at commit `76c8ead` in a dedicated `feature/phase5a-active-search` worktree.
- Existing Phase 0–4 public topics, messages, services, actions and default behavior remain compatible.
- Phase 2 remains the sole owner of semantic-memory global IDs and object lifecycle.
- Phase 3 learned or semantic outputs never directly command motion.
- Phase 5A allows rotation in place only; `forward_permitted` is always false.
- No Phase 5 component publishes `geometry_msgs/Twist` or final `/cmd_vel`.
- Every physical rotation uses Nav2 Spin and passes through `/nav2/cmd_vel_raw` → safety supervisor → `/nav2/cmd_vel_safe` → cmd_vel gate → `/cmd_vel`.
- Physical motion requires `ROTATION_SUPERVISED`, the launch execution flag, and explicit operator authorization for the one pending search task.
- `PASSIVE_ONLY` is the default; `SEARCH_SHADOW` and `PASSIVE_ONLY` produce no movement.
- Initial search policy is deterministic, configurable and bounded; no learned search policy and no translational Next-Best-View behavior are introduced.
- ROS domain remains fixed at `ROS_DOMAIN_ID=20` for managed commands and physical validation.
- Do not start Phase 5B until all Phase 5A MWS and Phase 0–4 regression gates pass.

## Current-State and Naming Decision

The current `static_target_mission.py` behavior freezes an already selected Phase 4B static target in `odom` after operator authorization. It preserves a supervised Nav2 approach across short perception dropout or Nav2 retry cycles; it does not select observation viewpoints or search for a target. It is therefore renamed in documentation and test language to **Phase 4B static-target mission continuity**, while runtime paths and public interfaces remain unchanged.

The corrected phase boundary is:

```text
Phase 0  contracts, localization health and replay baselines
Phase 1  language-conditioned visual observations
Phase 2  3D semantic memory and global-ID lifecycle
Phase 3  task-conditioned ranking, uncertainty and abstention
Phase 4A approach-pose and path planning without motion
Phase 4B supervised Nav2 semantic approach and static-mission continuity
Phase 5A bounded rotation search, verification and reacquisition
Phase 5B future Nav2 Next-Best-View translation
Phase 5C future multi-step coverage and search-history management
```

## Runtime Contracts

### Existing authoritative interfaces

- Query input: `/semantic_search/query` (`std_msgs/msg/String` JSON produced with `QueryRequest.payload`).
- Correlated task: `/semantic_memory/tasks` (`track_robot_interfaces/msg/SemanticTask`).
- Active objects: `/semantic_memory/active_objects` (`SemanticObjectArray`).
- Diagnostic ranking: `/semantic_memory/diagnostic_ranking` (`SemanticObjectArray`).
- Selected target: `/semantic_search/phase4a/selected_target` (`SemanticObjectArray`).
- Localization: `/semantic_search/phase4a/localization_state` (`SemanticLocalizationState`).
- Safety: `/safety/state` (`SafetyState`).
- Motion chain: `/nav2/cmd_vel_raw` → `/nav2/cmd_vel_safe` → `/cmd_vel`.

### Phase 5A interfaces

- Action server: `/semantic_search/search_for_object` (`SearchForObject`).
- Motion policy output: `/semantic_search/search_motion_intent` (`SearchMotionIntent`).
- Adapter status: `/semantic_search/active_search/motion_status` (`DiagnosticArray`).
- Search diagnostics: `/semantic_search/active_search/diagnostics` (`DiagnosticArray`).
- Search visualization: `/semantic_search/active_search/markers` (`MarkerArray`).
- Operator authorization: `/semantic_search/active_search/authorize_rotation` (`std_srvs/Trigger`).
- Explicit stop: `/semantic_search/active_search/cancel` (`std_srvs/Trigger`); action cancellation remains the primary client cancellation mechanism.

`SearchForObject.action` and `SearchMotionIntent.msg` retain their field layout. Additive result constants `LOCALIZATION_UNAVAILABLE=8`, `SEARCH_SPACE_EXHAUSTED=9` and `INTERNAL_FAULT=10` make the required terminal reasons machine-readable without changing DDS serialization; older clients still receive the same `uint8 status` field. Exact Phase 4 handoff continues to use the authoritative selected-target message because it already carries query ID/version, memory epoch, global object ID, localization epoch, timestamp, position and covariance. The action result is a client summary, not a replacement object record.

For Phase 5A, all action and intent angles are radians: `SearchForObject.maximum_rotation_angle` is the permitted magnitude around the initial heading, `SearchMotionIntent.target_bearing` is the signed relative Spin delta from the current heading, and `SearchMotionIntent.maximum_rotation_angle` is the per-intent magnitude bound. Angular speed is radians per second.

## Deterministic Baseline Policy

- Passive initial observation at the current heading.
- Active heading targets relative to the initial `odom` yaw: `+45°`, `+90°`, `0°`, `-45°`, `-90°`.
- The intermediate `0°` return is a transit heading and is not counted as a second independent initial observation.
- Maximum individual requested rotation: `90°`.
- Maximum cumulative rotation: `270°`.
- Maximum angular speed: `0.30 rad/s`.
- Maximum independent view bins: `5`, including the initial passive view.
- Duplicate-view tolerance: `10°`.
- Minimum settle duration: `0.75 s` and measured `|odom.twist.twist.angular.z| < 0.03 rad/s`.
- Per-view evidence window: three fresh Phase 3 snapshots or `4.5 s`, whichever occurs first.
- Evidence TTL: `12.0 s`.
- Default overall deadline: `60.0 s`, further bounded by the action goal timeout.
- All observations accepted for a new view must have source timestamps after the settling boundary.
- Camera horizontal FOV is recorded from `CameraInfo` when available; absence of FOV data does not expand the configured angular envelope.

## Evidence and Terminal Rules

- Evidence key: `(memory_epoch_id, global_object_id, localization_epoch_id, active_query_id, active_query_version)`.
- A Phase 3-selected target is confirmable only when its key is correlated to the active task, its source timestamp is fresh for the current observation window, and the existing Phase 3 selector has completed its stability/uncertainty gate.
- An initially visible valid target returns `CONFIRMED` without any motion intent.
- During active search, one settled view that produces the required fresh, stable Phase 3 snapshots may confirm a newly exposed target.
- Repeated snapshots inside one heading bin improve temporal stability but do not increase viewpoint coverage.
- Competing keys observed from different headings retain separate evidence; they are not silently merged and result in `UNCERTAIN` if Phase 3 cannot resolve them.
- No candidate over the complete heading sequence returns `NOT_FOUND`; candidate evidence that remains ambiguous returns `UNCERTAIN`.
- Query, memory epoch or localization epoch changes terminate the task fail-closed; Phase 5 never reassigns an object ID.
- Cancellation, timeout, safety rejection, RC takeover, E-stop, base fault, stale odometry, stale TF and Nav2 failure cancel Spin and produce one explicit terminal action result.
- A per-view timeout counts as an empty observation only when fresh perception diagnostics and fresh empty Phase 3/ranking snapshots were received. No fresh perception update terminates as `SENSOR_UNAVAILABLE`; an explicit model-not-ready diagnostic terminates as `MODEL_UNAVAILABLE`.
- Before returning `CONFIRMED`, the manager enters `HANDOFF_TO_PHASE4` and revalidates the same target key and snapshot freshness once. A changed/invalid key returns to evidence evaluation when angular/time budget remains, otherwise terminates `UNCERTAIN`.
- Confirming-view provenance is retained in bounded manager records and serialized into `evidence_summary` plus the correlated diagnostics topic. Phase 4 continues to consume the unchanged authoritative selected-target message.

## Required Test Matrix

| Case | Required evidence | Expected result |
|---|---|---|
| Target visible initially | fresh stable Phase 3 key | CONFIRMED, zero intents |
| Found after one/several headings | fresh post-settle snapshots | CONFIRMED, exact handoff key |
| Ambiguity resolved | competing ranking becomes one stable key | CONFIRMED |
| Lost static target reacquired | same Phase 2 global ID returns | CONFIRMED |
| Complete absence | fresh empty observations at all views | NOT_FOUND |
| Persistent ambiguity/contradiction | multiple unresolved keys | UNCERTAIN |
| Action timeout/cancel | deadline or cancel event | TIMEOUT/CANCELLED and zero command |
| Sensor/model unavailable | stale stream or explicit model diagnostic | SENSOR_UNAVAILABLE/MODEL_UNAVAILABLE |
| Odom/TF/localization failure | freshness/domain failure | LOCALIZATION_UNAVAILABLE |
| Safety/RC/E-stop/base fault | unhealthy `SafetyState` | SAFETY_REJECTED |
| Nav2 Spin rejection/failure | failed action result | SAFETY_REJECTED or INTERNAL_FAULT with exact reason |
| Query/memory/localization epoch change | correlation mismatch | CANCELLED or UNCERTAIN, never ID reassignment |
| Repeated identical view | heading within 10° | no coverage increment |
| Phase 4 handoff target changes | final key revalidation fails | resume evaluation or UNCERTAIN |

## File Map

### Create

- `src/track_robot_semantic_search/track_robot_semantic_search/active_search_policy.py` — pure heading budget and state-transition policy.
- `src/track_robot_semantic_search/track_robot_semantic_search/active_search_evidence.py` — bounded view/evidence records and terminal classification.
- `src/track_robot_semantic_search/track_robot_semantic_search/active_search_manager_node.py` — `SearchForObject` action server and event-driven ROS orchestration.
- `src/track_robot_semantic_search/config/semantic_search_phase5a.yaml` — search limits, freshness and evidence parameters.
- `src/track_robot_semantic_search/test/test_active_search_policy.py`.
- `src/track_robot_semantic_search/test/test_active_search_evidence.py`.
- `src/track_robot_semantic_search/test/test_active_search_manager_contract.py`.
- `src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter.py` — pure authorization/watchdog/intent policy.
- `src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter_node.py` — Nav2 Spin adapter.
- `src/track_robot/track_robot_navigation/scripts/search_motion_adapter`.
- `src/track_robot/track_robot_navigation/config/nav2_phase5a.yaml` — rotation-only controller costmap, Spin and safety-compatible Nav2 parameters.
- `src/track_robot/track_robot_navigation/config/active_search_motion.yaml`.
- `src/track_robot/track_robot_navigation/test/test_search_motion_adapter.py`.
- `src/track_robot/track_robot_navigation/test/test_phase5a_nav2_config_contract.py`.
- `src/track_robot/track_robot_navigation/test/test_phase5a_launch_contract.py`.
- `src/track_robot/track_robot_navigation/launch/phase5a_rotation.launch.py`.
- `src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`.
- `src/track_robot/track_robot_bringup/rviz/semantic_search_phase5a.rviz`.
- `src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`.
- `src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py`.
- `src/track_robot_semantic_search/test/data/phase5a_search_replay.json`.
- `src/track_robot_semantic_search/test/test_phase5a_search_replay.py`.
- `docs/guides/semantic-search/phase5a-bounded-active-search-test.md`.

### Modify

- `src/track_robot_semantic_search/setup.py` — install the active-search manager executable.
- `src/track_robot_semantic_search/package.xml` — add action, visualization and service runtime dependencies.
- `src/track_robot/track_robot_interfaces/action/SearchForObject.action` — add three backward-compatible result constants without changing fields.
- `src/track_robot/track_robot_interfaces/test/test_phase2_interfaces.py` — assert the new constants and unchanged action fields.
- `src/track_robot/track_robot_navigation/CMakeLists.txt` — install the adapter and register tests.
- `src/track_robot/track_robot_navigation/package.xml` — declare `visualization_msgs` only if the navigation adapter publishes no markers; normally markers remain in semantic-search.
- `src/track_robot/track_robot_navigation/track_robot_navigation/runtime_modes.py` — add fail-closed `ROTATION_ONLY_ACTIVE` without changing existing mode definitions.
- `src/track_robot/track_robot_navigation/test/test_runtime_modes.py`.
- `src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py` — add managed `run phase5a`, passive by default.
- `src/track_robot/track_robot_bringup/test/test_control_cli.py`.
- `src/track_robot/track_robot_navigation/track_robot_navigation/static_target_mission.py` — naming-only docstring correction.
- `src/track_robot/track_robot_navigation/test/test_launch_contract.py` — naming-only test correction.
- `docs/README.md` and `docs/guides/semantic-search/phase4b-nav2-supervised-test.md` — phase-name and effective inflation documentation correction.

---

### Task 1: Correct Phase Naming and Freeze the Phase 0–4 Baseline

**Files:**
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/static_target_mission.py:1`
- Modify: `src/track_robot/track_robot_navigation/test/test_launch_contract.py:134`
- Modify: `docs/README.md`
- Modify: `docs/guides/semantic-search/phase4b-nav2-supervised-test.md:116`

**Interfaces:**
- Consumes: current Phase 4B behavior at commit `76c8ead`.
- Produces: corrected terminology only; no runtime/interface changes.

- [ ] **Step 1: Record the clean baseline and exact test count**

Run:

```bash
git status --short --branch
git rev-parse HEAD
source /opt/ros/foxy/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test \
  src/track_robot/track_robot_semantic_memory/test \
  src/track_robot/track_robot_navigation/test \
  src/track_robot/track_robot_bringup/test \
  src/track_robot/track_robot_safety/test -q
```

Expected: clean branch, SHA `76c8ead`, and all existing tests pass. Record the exact count in the commit message body and later validation report rather than copying a historical count.

- [ ] **Step 2: Write the naming-contract assertion**

Change the test name and source checks so they assert `static-target mission continuity` belongs to Phase 4B and prohibit the old phrase `Phase 5A execution policy` in `static_target_mission.py`.

```python
def test_phase4b_freezes_an_odom_goal_before_motion_authorization():
    source = STATIC_TARGET_MISSION.read_text()
    assert 'Phase 4B static-target mission continuity' in source
    assert 'Phase 5A execution policy' not in source
```

- [ ] **Step 3: Run the focused test and verify RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test/test_launch_contract.py -q
```

Expected: FAIL because the old module docstring still says Phase 5A.

- [ ] **Step 4: Apply naming-only documentation changes**

Use this module description:

```python
"""Pure Phase 4B static-target mission continuity policy."""
```

Update `docs/README.md` to list Phase 4B supervised Nav2 approach without `/5A`. Update the operator guide to state the effective costmaps use the physical `0.88 m x 0.80 m` footprint, `inflation_radius=0.60 m`, and `cost_scaling_factor=12.0`.

- [ ] **Step 5: Verify no behavior changed**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test \
  src/track_robot/track_robot_bringup/test -q
git diff --check
```

Expected: PASS; diff contains only names and documentation.

- [ ] **Step 6: Commit independently**

```bash
git add docs/README.md docs/guides/semantic-search/phase4b-nav2-supervised-test.md \
  src/track_robot/track_robot_navigation/track_robot_navigation/static_target_mission.py \
  src/track_robot/track_robot_navigation/test/test_launch_contract.py
git commit -m "docs(semantic-search): correct Phase 4B and 5A boundaries"
```

### Task 2: Implement the Pure Bounded Heading Policy

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/active_search_policy.py`
- Create: `src/track_robot_semantic_search/test/test_active_search_policy.py`

**Interfaces:**
- Produces: `SearchState`, `SearchPolicyConfig`, `HeadingDecision`, and `BoundedHeadingPolicy.next_heading()` with no ROS dependencies.
- Consumes later: the active-search manager in Task 4.

- [ ] **Step 1: Write policy tests for order, transit headings and budgets**

Define tests using these public types:

```python
from track_robot_semantic_search.active_search_policy import (
    BoundedHeadingPolicy,
    SearchPolicyConfig,
    SearchState,
)


def test_default_policy_is_bounded_and_deterministic():
    policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())
    decisions = policy.complete_sequence(initial_yaw=0.0)
    assert [round(item.relative_heading_deg) for item in decisions] == [45, 90, 0, -45, -90]
    assert sum(abs(item.rotation_delta_deg) for item in decisions) == 270.0
    assert all(abs(item.rotation_delta_deg) <= 90.0 for item in decisions)
    assert sum(item.collect_evidence for item in decisions) == 4


def test_policy_refuses_rotation_beyond_action_limit():
    config = SearchPolicyConfig.defaults(maximum_rotation_angle_deg=60.0)
    policy = BoundedHeadingPolicy(config)
    decisions = policy.complete_sequence(initial_yaw=0.0)
    assert all(abs(item.relative_heading_deg) <= 60.0 for item in decisions)


def test_terminal_states_cannot_generate_another_heading():
    policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())
    for state in (SearchState.CONFIRMED, SearchState.CANCELLED, SearchState.TIMEOUT):
        assert policy.next_heading(state=state, initial_yaw=0.0, current_yaw=0.0) is None
```

- [ ] **Step 2: Run RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_active_search_policy.py -q
```

Expected: collection fails because `active_search_policy` does not exist.

- [ ] **Step 3: Implement the pure types and validated defaults**

Use these exact public types and implement the listed methods with finite-angle validation, yaw normalization and budget enforcement:

```python
class SearchState(Enum):
    IDLE = 'IDLE'
    QUERY_ACCEPTED = 'QUERY_ACCEPTED'
    PASSIVE_OBSERVATION = 'PASSIVE_OBSERVATION'
    EVALUATING_EVIDENCE = 'EVALUATING_EVIDENCE'
    ACTIVE_SEARCH_REQUIRED = 'ACTIVE_SEARCH_REQUIRED'
    SELECTING_VIEW = 'SELECTING_VIEW'
    WAITING_FOR_AUTHORIZATION = 'WAITING_FOR_AUTHORIZATION'
    ROTATING = 'ROTATING'
    SETTLING = 'SETTLING'
    OBSERVING = 'OBSERVING'
    UPDATING_MEMORY = 'UPDATING_MEMORY'
    TARGET_CONFIRMED = 'TARGET_CONFIRMED'
    HANDOFF_TO_PHASE4 = 'HANDOFF_TO_PHASE4'
    CONFIRMED = 'CONFIRMED'
    NOT_FOUND = 'NOT_FOUND'
    UNCERTAIN = 'UNCERTAIN'
    CANCELLED = 'CANCELLED'
    TIMEOUT = 'TIMEOUT'
    SAFETY_REJECTED = 'SAFETY_REJECTED'
    SENSOR_UNAVAILABLE = 'SENSOR_UNAVAILABLE'
    MODEL_UNAVAILABLE = 'MODEL_UNAVAILABLE'
    LOCALIZATION_UNAVAILABLE = 'LOCALIZATION_UNAVAILABLE'
    SEARCH_SPACE_EXHAUSTED = 'SEARCH_SPACE_EXHAUSTED'
    INTERNAL_FAULT = 'INTERNAL_FAULT'


class SearchMode(Enum):
    PASSIVE_ONLY = 'PASSIVE_ONLY'
    SEARCH_SHADOW = 'SEARCH_SHADOW'
    ROTATION_SUPERVISED = 'ROTATION_SUPERVISED'
    NEXT_BEST_VIEW_SHADOW = 'NEXT_BEST_VIEW_SHADOW'
    NEXT_BEST_VIEW_ACTIVE = 'NEXT_BEST_VIEW_ACTIVE'


@dataclass(frozen=True)
class SearchPolicyConfig:
    heading_offsets_deg: Tuple[float, ...]
    evidence_headings_deg: Tuple[float, ...]
    maximum_individual_rotation_deg: float
    maximum_cumulative_rotation_deg: float
    maximum_angular_speed_rad_s: float
    duplicate_heading_tolerance_deg: float
    default_deadline_sec: float

    @classmethod
    def defaults(cls, maximum_rotation_angle_deg=90.0):
        envelope = min(90.0, float(maximum_rotation_angle_deg))
        offsets = tuple(value for value in (45.0, 90.0, 0.0, -45.0, -90.0) if abs(value) <= envelope)
        evidence = tuple(value for value in (45.0, 90.0, -45.0, -90.0) if abs(value) <= envelope)
        return cls(offsets, evidence, 90.0, 270.0, 0.30, 10.0, 60.0)


@dataclass(frozen=True)
class HeadingDecision:
    target_yaw_rad: float
    relative_heading_deg: float
    rotation_delta_deg: float
    cumulative_rotation_deg: float
    collect_evidence: bool


```

`SearchMode.parse()` accepts the five named modes. `NEXT_BEST_VIEW_SHADOW` reports proposed-mode unavailable without motion in Phase 5A, while `NEXT_BEST_VIEW_ACTIVE` is rejected unless a future, separately approved Phase 5B feature gate exists. `BoundedHeadingPolicy.__init__(config: SearchPolicyConfig)` stores an immutable config, a sequence cursor and cumulative completed rotation. `next_heading(state: SearchState, initial_yaw: float, current_yaw: float) -> Optional[HeadingDecision]` returns the next bounded decision or `None` for terminal/exhausted states. `mark_completed(decision: HeadingDecision) -> None` advances the cursor exactly once and adds the absolute signed-delta magnitude. `complete_sequence(initial_yaw: float) -> Tuple[HeadingDecision, ...]` is a deterministic test helper that repeatedly selects and completes decisions from the resulting current yaw.

Implementation must normalize yaw to `[-pi, pi]`, reject non-finite values and stop before exceeding either budget.

- [ ] **Step 4: Run GREEN and policy lint**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_active_search_policy.py -q
python3 -m py_compile \
  src/track_robot_semantic_search/track_robot_semantic_search/active_search_policy.py
```

Expected: all policy tests pass.

- [ ] **Step 5: Commit independently**

```bash
git add src/track_robot_semantic_search/track_robot_semantic_search/active_search_policy.py \
  src/track_robot_semantic_search/test/test_active_search_policy.py
git commit -m "feat(semantic-search): add bounded active-search policy"
```

### Task 3: Implement Bounded Multi-View Evidence

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/active_search_evidence.py`
- Create: `src/track_robot_semantic_search/test/test_active_search_evidence.py`

**Interfaces:**
- Produces: `ObjectEvidenceKey`, `ViewEvidence`, `EvidenceDecision`, and `BoundedEvidenceBook`.
- Consumes later: normalized selected-target and ranking snapshots from Task 4.

- [ ] **Step 1: Write tests for freshness, duplicates, contradictions and epochs**

```python
def test_three_fresh_phase3_snapshots_confirm_one_key():
    book = BoundedEvidenceBook(EvidenceConfig.defaults())
    key = ObjectEvidenceKey(4, 22, 7, 100, 1)
    for stamp in (10.1, 10.4, 10.8):
        book.add(ViewEvidence(key, 45.0, 70.0, stamp, 0.61, 0.18, True), settled_after=10.0)
    assert book.evaluate(search_exhausted=False).status is EvidenceStatus.CONFIRMED


def test_near_duplicate_heading_does_not_expand_coverage():
    book = BoundedEvidenceBook(EvidenceConfig.defaults())
    key = ObjectEvidenceKey(4, 22, 7, 100, 1)
    book.add(ViewEvidence(key, 45.0, 70.0, 10.1, 0.61, 0.18, True), settled_after=10.0)
    book.add(ViewEvidence(key, 51.0, 70.0, 10.2, 0.62, 0.17, True), settled_after=10.0)
    assert book.covered_heading_count == 1


def test_competing_keys_end_uncertain_not_merged():
    book = BoundedEvidenceBook(EvidenceConfig.defaults())
    first = ObjectEvidenceKey(4, 22, 7, 100, 1)
    second = ObjectEvidenceKey(4, 31, 7, 100, 1)
    book.add(ViewEvidence(first, 45.0, 70.0, 10.1, 0.55, 0.31, False), settled_after=10.0)
    book.add(ViewEvidence(second, -45.0, 70.0, 10.2, 0.56, 0.30, False), settled_after=10.0)
    assert book.evaluate(search_exhausted=True).status is EvidenceStatus.UNCERTAIN


def test_epoch_change_invalidates_the_task():
    book = BoundedEvidenceBook(EvidenceConfig.defaults())
    book.bind_domain(memory_epoch_id=4, localization_epoch_id=7, query_id=100, query_version=1)
    assert book.domain_changed(memory_epoch_id=5, localization_epoch_id=7, query_id=100, query_version=1)
```

- [ ] **Step 2: Run RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_active_search_evidence.py -q
```

Expected: collection fails because the evidence module does not exist.

- [ ] **Step 3: Implement bounded records and explicit evaluation**

Use these exact immutable record fields. Implement `bind_domain()` to bind one task domain, `add()` to reject pre-settle/stale/domain-mismatched records, `expire()` to enforce TTL, and `evaluate()` to return CONFIRMED, NOT_FOUND or UNCERTAIN without merging keys:

```python
@dataclass(frozen=True)
class ObjectEvidenceKey:
    memory_epoch_id: int
    global_object_id: int
    localization_epoch_id: int
    query_id: int
    query_version: int


@dataclass(frozen=True)
class ViewEvidence:
    key: ObjectEvidenceKey
    heading_deg: float
    horizontal_fov_deg: Optional[float]
    source_stamp_sec: float
    task_relevance: float
    uncertainty: float
    phase3_selected: bool


@dataclass(frozen=True)
class EvidenceConfig:
    confirmation_snapshots: int
    duplicate_heading_tolerance_deg: float
    evidence_ttl_sec: float
    maximum_records: int

    @classmethod
    def defaults(cls):
        return cls(3, 10.0, 12.0, 40)


class EvidenceStatus(Enum):
    OBSERVING = 'OBSERVING'
    CONFIRMED = 'CONFIRMED'
    NOT_FOUND = 'NOT_FOUND'
    UNCERTAIN = 'UNCERTAIN'


@dataclass(frozen=True)
class EvidenceDecision:
    status: EvidenceStatus
    selected_key: Optional[ObjectEvidenceKey]
    candidate_count: int
    covered_heading_count: int
    reason: str


```

`BoundedEvidenceBook` exposes `bind_domain(memory_epoch_id: int, localization_epoch_id: int, query_id: int, query_version: int) -> None`, `domain_changed(memory_epoch_id: int, localization_epoch_id: int, query_id: int, query_version: int) -> bool`, `add(evidence: ViewEvidence, settled_after: float) -> bool`, `expire(now_sec: float) -> None`, and `evaluate(search_exhausted: bool) -> EvidenceDecision`. When valid `CameraInfo` is available, coverage records `heading ± horizontal_fov/2`; without it, the book retains only the configured heading bin and never assumes a wider view.

`add()` returns false for stale-before-settle evidence. Store at most 40 records in insertion order, expire at 12 seconds, and never combine different `ObjectEvidenceKey` values.

- [ ] **Step 4: Run GREEN**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_active_search_evidence.py -q
```

Expected: all evidence tests pass with deterministic results.

- [ ] **Step 5: Commit independently**

```bash
git add src/track_robot_semantic_search/track_robot_semantic_search/active_search_evidence.py \
  src/track_robot_semantic_search/test/test_active_search_evidence.py
git commit -m "feat(semantic-search): add bounded multi-view evidence"
```

### Task 4: Add the SearchForObject Manager in Passive and Shadow Modes

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/active_search_manager_node.py`
- Create: `src/track_robot_semantic_search/config/semantic_search_phase5a.yaml`
- Create: `src/track_robot_semantic_search/test/test_active_search_manager_contract.py`
- Modify: `src/track_robot_semantic_search/setup.py`
- Modify: `src/track_robot_semantic_search/package.xml`
- Modify: `src/track_robot/track_robot_interfaces/action/SearchForObject.action`
- Modify: `src/track_robot/track_robot_interfaces/test/test_phase2_interfaces.py`

**Interfaces:**
- Consumes: `/semantic_search/query`, perception diagnostics, `SemanticTask`, Phase 2 active objects, Phase 3 selected target/ranking, ZED `CameraInfo`, localization and adapter diagnostics.
- Produces: `/semantic_search/search_for_object`, `SearchMotionIntent`, search diagnostics and RViz markers.

- [ ] **Step 1: Write static action and no-Twist contract tests**

```python
def test_manager_uses_existing_action_and_query_contract():
    source = MANAGER_SOURCE.read_text()
    assert 'ActionServer' in source
    assert 'SearchForObject' in source
    assert "'/semantic_search/query'" in source
    assert 'QueryRequest.create' in source
    assert 'SearchMotionIntent' in source


def test_manager_cannot_publish_velocity_or_call_navigation():
    source = MANAGER_SOURCE.read_text()
    forbidden = ('geometry_msgs.msg import Twist', "'/cmd_vel'", 'NavigateToPose', 'Spin')
    assert all(token not in source for token in forbidden)
```

Add pure manager-harness tests proving an initially confirmed target emits zero intents and `SEARCH_SHADOW` emits intents with both permission flags false.

Add interface tests with exact constants:

```python
assert SearchForObject.Result.LOCALIZATION_UNAVAILABLE == 8
assert SearchForObject.Result.SEARCH_SPACE_EXHAUSTED == 9
assert SearchForObject.Result.INTERNAL_FAULT == 10
```

- [ ] **Step 2: Run RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_active_search_manager_contract.py -q
```

Expected: FAIL because the manager node does not exist.

- [ ] **Step 3: Implement the event-driven action manager**

The node class is `ActiveSearchManager(Node)` with constants `ACTION_NAME = '/semantic_search/search_for_object'` and `INTENT_TOPIC = '/semantic_search/search_motion_intent'`. Its exact callbacks are `execute_goal(goal_handle)`, `on_task(message: SemanticTask) -> None`, `on_active_objects(message: SemanticObjectArray) -> None`, `on_selected_target(message: SemanticObjectArray) -> None`, `on_ranking(message: SemanticObjectArray) -> None`, `on_camera_info(message: CameraInfo) -> None`, `on_localization(message: SemanticLocalizationState) -> None`, `on_motion_status(message: DiagnosticArray) -> None`, and `publish_stop_intent(reason: str) -> None`.

Extend the action definition with constants only:

```text
uint8 LOCALIZATION_UNAVAILABLE=8
uint8 SEARCH_SPACE_EXHAUSTED=9
uint8 INTERNAL_FAULT=10
```

`execute_goal()` allocates a positive query ID with `QueryIdAllocator`, creates version `1` through `QueryRequest.create()`, publishes `request.payload`, waits for the correlated `SemanticTask`, and then advances only in response to correlated messages/action results/deadlines. A timer may enforce deadlines but must not declare observation complete solely because time elapsed.

Configure exact defaults in YAML:

```yaml
active_search_manager:
  ros__parameters:
    search_mode: PASSIVE_ONLY
    active_search_execution_enabled: false
    heading_offsets_deg: [45.0, 90.0, 0.0, -45.0, -90.0]
    evidence_headings_deg: [45.0, 90.0, -45.0, -90.0]
    maximum_individual_rotation_deg: 90.0
    maximum_cumulative_rotation_deg: 270.0
    maximum_angular_speed_rad_s: 0.30
    settle_duration_sec: 0.75
    settle_angular_speed_rad_s: 0.03
    observation_timeout_sec: 4.5
    confirmation_snapshots: 3
    duplicate_heading_tolerance_deg: 10.0
    evidence_ttl_sec: 12.0
    maximum_evidence_records: 40
    default_task_timeout_sec: 60.0
```

- [ ] **Step 4: Verify passive and shadow no-motion behavior**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_active_search_policy.py \
  src/track_robot_semantic_search/test/test_active_search_evidence.py \
  src/track_robot_semantic_search/test/test_active_search_manager_contract.py -q
```

Expected: PASS; initial confirmation has no intent, passive unresolved returns safely, and shadow intent has `rotation_permitted=false` and `forward_permitted=false`.

- [ ] **Step 5: Build the package and inspect the action executable**

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_interfaces track_robot_semantic_search --symlink-install
source install/setup.bash
ros2 pkg executables track_robot_semantic_search | rg active_search
```

Expected: one manager executable is installed; the interface package regenerates successfully with the same result field layout and the three new constants.

- [ ] **Step 6: Commit independently**

```bash
git add src/track_robot_semantic_search/setup.py \
  src/track_robot_semantic_search/package.xml \
  src/track_robot/track_robot_interfaces/action/SearchForObject.action \
  src/track_robot/track_robot_interfaces/test/test_phase2_interfaces.py \
  src/track_robot_semantic_search/config/semantic_search_phase5a.yaml \
  src/track_robot_semantic_search/track_robot_semantic_search/active_search_manager_node.py \
  src/track_robot_semantic_search/test/test_active_search_manager_contract.py
git commit -m "feat(semantic-search): add passive active-search manager"
```

### Task 5: Implement the Rotation Authorization and Nav2 Spin Adapter

**Files:**
- Create: `src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter.py`
- Create: `src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter_node.py`
- Create: `src/track_robot/track_robot_navigation/scripts/search_motion_adapter`
- Create: `src/track_robot/track_robot_navigation/config/active_search_motion.yaml`
- Create: `src/track_robot/track_robot_navigation/test/test_search_motion_adapter.py`
- Modify: `src/track_robot/track_robot_navigation/CMakeLists.txt`

**Interfaces:**
- Consumes: `SearchMotionIntent`, SafetyState, odometry, `/safety/arm`, `/safety/disarm`, operator Trigger and Nav2 Spin result.
- Produces: Nav2 `Spin` goals and `DiagnosticArray`; no Twist publisher.

- [ ] **Step 1: Write pure authorization and watchdog tests**

```python
def test_authorization_is_bound_to_one_pending_query():
    policy = SearchMotionPolicy(MotionLimits.defaults())
    policy.accept_intent(rotation_intent(query_id=44))
    assert policy.authorize(query_id=44).accepted
    assert not policy.authorize(query_id=45).accepted


def test_forward_permission_is_always_rejected():
    policy = SearchMotionPolicy(MotionLimits.defaults())
    result = policy.accept_intent(rotation_intent(query_id=44, forward_permitted=True))
    assert not result.accepted
    assert result.reason == 'forward_motion_forbidden'


def test_safety_fault_clears_authorization_and_requests_zero_stop():
    policy = authorized_policy(query_id=44)
    transition = policy.update_safety(rc_override=True)
    assert transition.cancel_spin
    assert transition.disarm
    assert not policy.authorized
```

- [ ] **Step 2: Run RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test/test_search_motion_adapter.py -q
```

Expected: collection fails because the adapter module does not exist.

- [ ] **Step 3: Implement pure policy and ROS adapter**

Use these policy limits:

```python
@dataclass(frozen=True)
class MotionLimits:
    maximum_individual_rotation_rad: float
    maximum_angular_speed_rad_s: float
    odometry_timeout_sec: float
    safety_timeout_sec: float
    intent_timeout_sec: float

    @classmethod
    def defaults(cls):
        return cls(math.radians(90.0), 0.30, 0.25, 0.25, 0.50)
```

The ROS node must create `ActionClient(self, Spin, '/spin')`, wait for a single pending intent, expose the two Trigger services, call `/safety/arm` only after authorization, and cancel/disarm for STOP, action cancellation, expired deadline or unhealthy safety/odom. It must reject non-finite angles, angles over 90°, speed over `0.30`, `rotation_permitted=false`, `forward_permitted=true`, and query mismatches.

- [ ] **Step 4: Add static safeguards**

```python
def test_ros_adapter_has_no_twist_or_final_velocity_publisher():
    source = ADAPTER_NODE.read_text()
    assert 'ActionClient' in source and 'Spin' in source
    assert 'NavigateToPose' not in source
    assert 'geometry_msgs.msg import Twist' not in source
    assert "'/cmd_vel'" not in source
    assert 'forward_motion_forbidden' in source
```

- [ ] **Step 5: Run GREEN and package build**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test/test_search_motion_adapter.py -q
colcon build --packages-select track_robot_navigation --symlink-install
```

Expected: all policy/static tests pass and the adapter executable installs.

- [ ] **Step 6: Commit independently**

```bash
git add src/track_robot/track_robot_navigation/CMakeLists.txt \
  src/track_robot/track_robot_navigation/config/active_search_motion.yaml \
  src/track_robot/track_robot_navigation/scripts/search_motion_adapter \
  src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter.py \
  src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter_node.py \
  src/track_robot/track_robot_navigation/test/test_search_motion_adapter.py
git commit -m "feat(navigation): adapt bounded search intents to Nav2 Spin"
```

### Task 6: Add Rotation-Only Nav2 Runtime and Configuration

**Files:**
- Create: `src/track_robot/track_robot_navigation/config/nav2_phase5a.yaml`
- Create: `src/track_robot/track_robot_navigation/launch/phase5a_rotation.launch.py`
- Create: `src/track_robot/track_robot_navigation/test/test_phase5a_nav2_config_contract.py`
- Create: `src/track_robot/track_robot_navigation/test/test_phase5a_launch_contract.py`
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/runtime_modes.py`
- Modify: `src/track_robot/track_robot_navigation/test/test_runtime_modes.py`
- Modify: `src/track_robot/track_robot_navigation/CMakeLists.txt`

**Interfaces:**
- Consumes: `/rslidar_points`, `/safety/filtered_obstacle_points`, `/odom`, TF and safety chain.
- Produces: Nav2 `/spin`; raw commands remapped only to `/nav2/cmd_vel_raw`.

- [ ] **Step 1: Write fail-closed runtime-mode tests**

```python
def test_rotation_only_mode_has_no_planner_bt_or_semantic_approach_adapter():
    spec = mode_spec(RuntimeMode.ROTATION_ONLY_ACTIVE)
    assert spec.controller
    assert spec.recoveries
    assert spec.safety_chain
    assert not spec.planner
    assert not spec.bt_navigator
    assert not spec.semantic_adapter


def test_rotation_only_requires_its_separate_execution_gate():
    with pytest.raises(ValueError, match='enable_rotation_execution'):
        validate_mode_request(
            RuntimeMode.ROTATION_ONLY_ACTIVE,
            enable_semantic_execution=False,
            enable_rotation_execution=False,
        )
```

The controller server is started only to host the existing local Nav2 costmap consumed by `nav2_recoveries/Spin`; no Phase 5 node creates a FollowPath client.

- [ ] **Step 2: Write YAML safety contracts**

Assert:

```python
recoveries = params['recoveries_server']['ros__parameters']
assert recoveries['recovery_plugins'] == ['spin']
assert recoveries['spin']['plugin'] == 'nav2_recoveries/Spin'
assert recoveries['spin']['max_rotational_vel'] == 0.30
assert recoveries['spin']['min_rotational_vel'] <= 0.10
assert recoveries['spin']['rotational_acc_lim'] <= 0.50
assert recoveries['spin']['simulate_ahead_time'] >= 1.0
assert params['local_costmap']['local_costmap']['ros__parameters']['footprint'] == '[[-0.44,-0.40],[-0.44,0.40],[0.44,0.40],[0.44,-0.40]]'
```

Also assert the Phase 4B file remains unchanged with `recovery_plugins: [wait]`.

- [ ] **Step 3: Run RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test/test_runtime_modes.py \
  src/track_robot/track_robot_navigation/test/test_phase5a_nav2_config_contract.py \
  src/track_robot/track_robot_navigation/test/test_phase5a_launch_contract.py -q
```

Expected: FAIL because the new mode/config/launch do not exist.

- [ ] **Step 4: Implement the separate Phase 5A Nav2 launch**

The launch starts only:

```text
controller_server       local costmap host; no FollowPath client exists
recoveries_server       Spin plugin only
motion_safety_supervisor_node
cmd_vel_gate
search_motion_adapter
nav2 lifecycle manager
```

Remap every Nav2 `cmd_vel` to `/nav2/cmd_vel_raw`. Extend `validate_mode_request(mode, enable_semantic_execution, enable_rotation_execution=False)` so all existing two-argument callers retain their behavior. Default launch arguments are `enable_rotation_execution=false` and `autostart=true`; requesting `ROTATION_ONLY_ACTIVE` without the explicit execution flag raises `ValueError`.

- [ ] **Step 5: Run GREEN and inspect launch graph statically**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test -q
colcon build --packages-select track_robot_navigation track_robot_safety track_robot_core --symlink-install
source install/setup.bash
ros2 launch track_robot_navigation phase5a_rotation.launch.py --show-args
```

Expected: tests pass; launch defaults are non-executing; no planner server, BT navigator, semantic approach supervisor or final cmd_vel publisher appears in the Phase 5A launch source.

- [ ] **Step 6: Commit independently**

```bash
git add src/track_robot/track_robot_navigation/CMakeLists.txt \
  src/track_robot/track_robot_navigation/config/nav2_phase5a.yaml \
  src/track_robot/track_robot_navigation/launch/phase5a_rotation.launch.py \
  src/track_robot/track_robot_navigation/track_robot_navigation/runtime_modes.py \
  src/track_robot/track_robot_navigation/test/test_runtime_modes.py \
  src/track_robot/track_robot_navigation/test/test_phase5a_nav2_config_contract.py \
  src/track_robot/track_robot_navigation/test/test_phase5a_launch_contract.py
git commit -m "feat(navigation): add rotation-only Phase 5A Nav2 runtime"
```

### Task 7: Compose Bringup, CLI and RViz Diagnostics

**Files:**
- Create: `src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`
- Create: `src/track_robot/track_robot_bringup/rviz/semantic_search_phase5a.rviz`
- Create: `src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`
- Create: `src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py`
- Modify: `src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_control_cli.py`

**Interfaces:**
- Produces: one managed Phase 5A launch and CLI entry while retaining all Phase 4B commands.
- Consumes: Phase 4A perception/memory/selector in planning-only form and Phase 5A rotation runtime.

- [ ] **Step 1: Write launch/CLI default-safety tests**

```python
def test_phase5a_launch_defaults_to_passive_only():
    source = PHASE5A_LAUNCH.read_text()
    assert "default_value='PASSIVE_ONLY'" in source
    assert "default_value='false'" in source
    assert 'semantic_search_phase4a.launch.py' in source
    assert 'phase5a_rotation.launch.py' in source


def test_cli_requires_explicit_rotation_supervised_flag():
    parser = build_parser()
    passive = parser.parse_args(['run', 'phase5a'])
    assert passive.search_mode == 'PASSIVE_ONLY'
    active = parser.parse_args(['run', 'phase5a', '--rotation-supervised'])
    assert active.search_mode == 'ROTATION_SUPERVISED'
```

- [ ] **Step 2: Run RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py \
  src/track_robot/track_robot_bringup/test/test_control_cli.py -q
```

Expected: FAIL because Phase 5A is not yet a CLI/launch choice.

- [ ] **Step 3: Implement managed modes without weakening Phase 4B**

Supported CLI behavior:

```text
semantic_search_ctl run phase5a
    PASSIVE_ONLY; no rotation server execution

semantic_search_ctl run phase5a --search-shadow
    search headings and diagnostics only

semantic_search_ctl run phase5a --rotation-supervised
    starts rotation-capable servers but remains motionless until
    /semantic_search/active_search/authorize_rotation succeeds
```

Mutually exclude `--search-shadow` and `--rotation-supervised`. Preserve `semantic_search_ctl run phase4b` unchanged.

- [ ] **Step 4: Add RViz displays**

Configure the Phase 5A RViz file with fixed frame `odom`, existing camera/semantic overlay and costmap displays, plus MarkerArray `/semantic_search/active_search/markers`. Diagnostics must expose current state, initial/current heading, visited headings, remaining angle budget, candidate count, best score, uncertainty and terminal reason.

- [ ] **Step 5: Verify all passive/shadow launch contracts**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_bringup/test \
  src/track_robot/track_robot_navigation/test -q
colcon build --packages-select track_robot_bringup track_robot_navigation track_robot_semantic_search --symlink-install
```

Expected: all tests pass; passive and shadow launch sources do not enable the rotation adapter or safety arm request.

- [ ] **Step 6: Commit independently**

```bash
git add src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py \
  src/track_robot/track_robot_bringup/rviz/semantic_search_phase5a.rviz \
  src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py \
  src/track_robot/track_robot_bringup/test/test_control_cli.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py
git commit -m "feat(bringup): compose supervised Phase 5A search"
```

### Task 8: Add Deterministic Replay and Full Regression Gates

**Files:**
- Create: `src/track_robot_semantic_search/test/data/phase5a_search_replay.json`
- Create: `src/track_robot_semantic_search/test/test_phase5a_search_replay.py`

**Interfaces:**
- Consumes: pure policy/evidence events; no live motion.
- Produces: deterministic terminal results and exact handoff-reference assertions.

- [ ] **Step 1: Define replay cases with explicit expected outcomes**

The JSON fixture contains these named cases and expected terminal statuses:

```json
{
  "schema_version": "phase5a_search_replay/1.0.0",
  "cases": [
    {"name": "visible_initially", "expected": "CONFIRMED", "expected_motion_intents": 0},
    {"name": "found_after_one_rotation", "expected": "CONFIRMED", "expected_motion_intents": 1},
    {"name": "found_after_several_headings", "expected": "CONFIRMED", "expected_motion_intents": 3},
    {"name": "ambiguous_then_resolved", "expected": "CONFIRMED", "expected_motion_intents": 2},
    {"name": "lost_static_target_reacquired_same_id", "expected": "CONFIRMED", "expected_motion_intents": 2},
    {"name": "absent_complete_range", "expected": "NOT_FOUND", "expected_motion_intents": 5},
    {"name": "persistent_ambiguity", "expected": "UNCERTAIN", "expected_motion_intents": 5},
    {"name": "contradictory_views", "expected": "UNCERTAIN", "expected_motion_intents": 5},
    {"name": "repeated_identical_view", "expected": "SEARCH_SPACE_EXHAUSTED", "expected_motion_intents": 1},
    {"name": "action_timeout", "expected": "TIMEOUT", "expected_motion_intents": 1},
    {"name": "action_cancelled", "expected": "CANCELLED", "expected_motion_intents": 1},
    {"name": "sensor_unavailable", "expected": "SENSOR_UNAVAILABLE", "expected_motion_intents": 0},
    {"name": "model_unavailable", "expected": "MODEL_UNAVAILABLE", "expected_motion_intents": 0},
    {"name": "odometry_stale", "expected": "LOCALIZATION_UNAVAILABLE", "expected_motion_intents": 1},
    {"name": "tf_failure", "expected": "LOCALIZATION_UNAVAILABLE", "expected_motion_intents": 1},
    {"name": "safety_rejected", "expected": "SAFETY_REJECTED", "expected_motion_intents": 1},
    {"name": "rc_takeover", "expected": "SAFETY_REJECTED", "expected_motion_intents": 1},
    {"name": "estop", "expected": "SAFETY_REJECTED", "expected_motion_intents": 1},
    {"name": "base_fault", "expected": "SAFETY_REJECTED", "expected_motion_intents": 1},
    {"name": "nav2_spin_failed", "expected": "INTERNAL_FAULT", "expected_motion_intents": 1},
    {"name": "query_changed", "expected": "CANCELLED", "expected_motion_intents": 1},
    {"name": "memory_epoch_changed", "expected": "UNCERTAIN", "expected_motion_intents": 1},
    {"name": "localization_epoch_changed", "expected": "UNCERTAIN", "expected_motion_intents": 1},
    {"name": "handoff_key_changed", "expected": "UNCERTAIN", "expected_motion_intents": 2}
  ]
}
```

- [ ] **Step 2: Write deterministic replay assertions**

```python
def test_replay_cases_are_deterministic():
    first = run_all_cases(FIXTURE)
    second = run_all_cases(FIXTURE)
    assert first == second
    for result in first:
        assert result.terminal_status == result.expected_status
        assert result.motion_intent_count == result.expected_motion_intents
        assert result.maximum_linear_command == 0.0
```

For confirmed cases, assert the exact tuple `(memory_epoch_id, global_object_id, localization_epoch_id, query_id, query_version)` equals the selected-target fixture.

- [ ] **Step 3: Run replay and all Phase 0–4 tests**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_phase5a_search_replay.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test \
  src/track_robot/track_robot_semantic_memory/test \
  src/track_robot/track_robot_navigation/test \
  src/track_robot/track_robot_bringup/test \
  src/track_robot/track_robot_safety/test -q
```

Expected: deterministic replay passes twice and every pre-existing Phase 0–4 test remains passing.

- [ ] **Step 4: Build the complete impacted workspace**

```bash
colcon build --packages-select \
  track_robot_interfaces \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_navigation \
  track_robot_bringup \
  track_robot_safety \
  track_robot_core --symlink-install
```

Expected: successful build with no interface-layout changes.

- [ ] **Step 5: Commit independently**

```bash
git add src/track_robot_semantic_search/test/data/phase5a_search_replay.json \
  src/track_robot_semantic_search/test/test_phase5a_search_replay.py
git commit -m "test(semantic-search): add Phase 5A deterministic replay"
```

### Task 9: Document and Execute Shadow-First Runtime Validation

**Files:**
- Create: `docs/guides/semantic-search/phase5a-bounded-active-search-test.md`
- Modify: `docs/README.md`

**Interfaces:**
- Produces: repeatable operator procedure and one validation report path under `artifacts/semantic_search/phase5a/`.

- [ ] **Step 1: Write the guide before physical execution**

The guide must specify:

```text
1. Export ROS_DOMAIN_ID=20 and source Foxy/workspace.
2. Run semantic_search_ctl doctor phase3 before starting Phase 5A.
3. Start PASSIVE_ONLY and verify an initially visible target causes no rotation.
4. Start SEARCH_SHADOW and verify deterministic proposed headings with zero cmd_vel.
5. Place the target outside the initial view.
6. Send one `SearchForObject` goal, start ROTATION_SUPERVISED, then authorize the one pending task explicitly.
7. Keep RC controller ready; test RC takeover, cancel and E-stop individually.
8. Verify exact target handoff while Phase 4 approach remains separately unauthorized.
9. Stop the managed stack and verify no ROS nodes/services remain owned by the test.
```

- [ ] **Step 2: Run PASSIVE_ONLY and SEARCH_SHADOW**

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 run track_robot_bringup semantic_search_ctl run phase5a
ros2 run track_robot_bringup semantic_search_ctl run phase5a --search-shadow
```

Expected: no `/spin` goal in passive mode; shadow diagnostics contain the configured heading sequence; `/nav2/cmd_vel_raw`, `/nav2/cmd_vel_safe` and `/cmd_vel` remain zero/absent.

- [ ] **Step 3: Run support-stand rotation validation**

Start supervised mode only after the robot is securely supported:

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a --rotation-supervised
ros2 action send_goal /semantic_search/search_for_object \
  track_robot_interfaces/action/SearchForObject \
  "{query_text: 'green bottle', timeout: {sec: 60, nanosec: 0}, allow_rotation: true, maximum_rotation_angle: 1.5708, client_request_id: 'phase5a-support-stand-1'}" --feedback
ros2 service call /semantic_search/active_search/authorize_rotation std_srvs/srv/Trigger '{}'
```

Expected: rotation starts only after the service succeeds; `linear.x` stays zero; angular speed never exceeds `0.30 rad/s`; each observation begins after the settle gate.

- [ ] **Step 4: Run floor safety cases**

Measure and record:

- heading error at every stop, acceptance target `<=5°`;
- total cumulative rotation, acceptance target `<=270°`;
- total task duration, acceptance target `<=60 s`;
- cancel, RC takeover and E-stop stop latency, acceptance target `<=0.30 s`;
- maximum absolute raw/safe/final `linear.x`, acceptance target `<=0.001 m/s`;
- exact terminal reason for stale odom, stale TF and safety rejection;
- target-discovery view and exact authoritative handoff key.

- [ ] **Step 5: Update the guide with measured evidence only**

Add the exact commit SHA, command, configuration, target setup, results and artifact filenames. Mark cases not physically executed as `NOT EVALUATED`; do not estimate them.

- [ ] **Step 6: Run final regression and commit documentation**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test \
  src/track_robot/track_robot_semantic_memory/test \
  src/track_robot/track_robot_navigation/test \
  src/track_robot/track_robot_bringup/test \
  src/track_robot/track_robot_safety/test -q
git diff --check
git add docs/README.md docs/guides/semantic-search/phase5a-bounded-active-search-test.md
git commit -m "docs(semantic-search): add Phase 5A supervised test flow"
```

Expected: all regression tests pass and the guide separates designed, unit-tested, replay-tested, shadow-tested, physically tested and not-evaluated evidence.

## Definition of Done

Phase 5A MWS is complete only when all of the following are true:

1. A `SearchForObject` query first runs passive Phase 1–3 observation.
2. A valid initially visible target is confirmed with zero rotation intents.
3. An unresolved query produces the configured bounded deterministic headings.
4. Physical rotation cannot start without explicit operator authorization.
5. Every evidence window starts only after heading completion and the settle gate.
6. Fresh Phase 1–3 outputs after each view update the bounded evidence book.
7. A target discovered from a new view preserves the exact Phase 2/3 target identity and is handed to Phase 4 without directly starting approach.
8. Exhaustion returns NOT_FOUND when no candidates exist and UNCERTAIN when evidence remains ambiguous.
9. Timeout, cancel, stale odometry/TF, RC takeover, E-stop, base fault and safety rejection cancel motion and terminate explicitly.
10. No Phase 5 component imports/publishes Twist or writes final `/cmd_vel`.
11. Raw, safe and final linear velocity stay at zero during Phase 5A.
12. All Phase 0–4 unit, integration and deterministic replay tests remain passing.
13. RViz and diagnostics display state, headings, remaining budget, candidate count, score, uncertainty and terminal reason.
14. Physical evidence reports actual measurements and marks unexecuted cases `NOT EVALUATED`.

## Rollback Strategy

- Every task is a logically independent commit; revert only the failing task commit.
- Existing Phase 4B launch/config remain present and unchanged in behavior throughout Phase 5A work.
- The new Phase 5A launch is additive and passive by default; removing it restores the pre-Phase-5 runtime without interface migration.
- Only compile-time `SearchForObject` result constants are added; the message field layout and DDS serialization remain unchanged. If a later result needs additional fields, first add a separate diagnostic/result topic or request explicit approval before changing the action layout.
- Any unexplained Phase 0–4 regression, nonzero Phase 5A linear command, direct cmd_vel publisher, weaker safety behavior or object-ID ownership change rejects the current change immediately.

## Explicitly Out of Scope

- Translational search and Nav2 Next-Best-View execution.
- Global exploration or map-wide autonomous search.
- Moving-target pursuit.
- Learned search policy or DINOv3-based view selection.
- Phase 2 global-ID or lifecycle changes.
- Automatic Phase 4 approach authorization after target confirmation.
- Changes to the existing Phase 4 planner, controller or public approach interfaces.

## Known Risks and Validation Gates

- Foxy Nav2 Spin has not yet been physically characterized on this tracked Bunker; support-stand validation is mandatory before floor testing.
- The controller server is needed to host the local Nav2 costmap consumed by Spin, but Phase 5A creates no FollowPath or NavigateToPose client. Static and live graph checks must confirm this boundary.
- Phase 1 inference can have multi-second gaps, so the `4.5 s` view window must be measured during shadow testing and increased only through a separate evidence-backed config commit.
- Prototype Camera–LiDAR extrinsics limit absolute-position claims; Phase 5A acceptance validates identity/reference consistency, not formal calibration accuracy.
- DINOv3 remains short-term appearance support and does not grant Phase 5 authority to merge or reassign global IDs.
- A failed Spin action never triggers a custom rotation fallback; it terminates fail-closed.

## Execution Preflight After Approval

Implementation begins only after this plan is approved. From the official main worktree, create the isolated feature worktree and verify that the tested functional baseline is an ancestor:

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration
git status --short --branch
git merge-base --is-ancestor 76c8ead HEAD
git worktree add ../phase5a-active-search -b feature/phase5a-active-search main
cd ../phase5a-active-search/track_robot_ws
git status --short --branch
```

Expected: the main worktree contains no uncommitted implementation changes, the ancestry check exits zero, and the new feature worktree is clean on `feature/phase5a-active-search`. At execution time, use the `superpowers:using-git-worktrees` skill before creating this worktree and `superpowers:executing-plans` to execute the checked tasks inline with regression gates.
