# Phase 4 Path Shortcutting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove unnecessary Phase 4 grid turns while preserving collision
checking, fail-closed behavior, planning-only operation, and existing public
interfaces.

**Architecture:** Keep the bounded multi-goal grid search. When enabled, reject
diagonal search transitions that squeeze through blocked corners, then run a
greedy farthest-visible shortcut pass over the reconstructed path. Visibility
uses conservative supercover traversal under the existing occupancy and
unknown-cell policy.

**Tech Stack:** Python 3.8, ROS 2 Foxy, pytest, nav_msgs/Path, OccupancyGrid.

**Execution note:** Performance measurement rejected always-on diagonal
side-cell checks because they raised open-map P95 latency by about 62 percent.
The implemented equivalent performs the corner-safe search only when the
legacy path fails final supercover validation, using the remaining expansion
budget. The accepted open-map P95 increase was 1.38 percent.

## Global Constraints

- Do not change Phase 0-3 target selection, IDs, frames, timestamps, or
  lifecycle ownership.
- Keep `planning_only=true`; add no motion publisher, action client, or Nav2
  execution interface.
- Preserve all existing failure reasons and fail-closed behavior.
- Add one rollback parameter, `enable_path_shortcutting`.
- Reject the change if deterministic or live regression fails, or if planner
  P95 latency increases by more than five percent on the same input.

---

### Task 1: Collision-checked shortcut core

**Files:**

- Modify:
  `src/track_robot_semantic_search/track_robot_semantic_search/approach_planning.py`
- Test:
  `src/track_robot_semantic_search/test/test_approach_planning.py`

**Interfaces:**

- Consumes: `_SearchResult.cells`, `GridMap.is_traversable()`,
  `PlannerConfig.occupied_threshold`, and
  `PlannerConfig.unknown_is_obstacle`.
- Produces: `PlannerConfig.enable_path_shortcutting: bool`,
  `Phase4Planner._supercover_cells(start, end)`,
  `Phase4Planner._line_of_sight(grid, start, end)`, and
  `Phase4Planner._shortcut_path(grid, cells)`.

- [ ] **Step 1: Add failing open-map and rollback tests**

Add tests equivalent to:

```python
def live_size_grid():
    width = 240
    height = 240
    return GridMap(
        frame_id='base_link',
        stamp_ns=NOW_NS - 50_000_000,
        resolution=0.05,
        width=width,
        height=height,
        origin_x=-6.0,
        origin_y=-6.0,
        data=tuple([0] * (width * height)),
    )


def test_open_map_shortcut_reduces_grid_staircase_to_one_segment():
    planner = Phase4Planner(PlannerConfig(
        enable_path_shortcutting=True,
        minimum_target_relevance=0.25,
        maximum_target_age_sec=2.5,
        maximum_map_age_sec=2.5,
    ))
    result = planner.plan(context(
        candidates=(target(x=3.17, y=-1.64),),
        grid=live_size_grid(),
    ))
    assert result.status == 'PASS'
    assert result.raw_path_pose_count > 2
    assert len(result.path) == 2
    assert result.path_shortcut_applied is True


def test_disabling_shortcut_preserves_raw_grid_path():
    result = Phase4Planner(PlannerConfig(
        enable_path_shortcutting=False,
    )).plan(context(candidates=(target(x=2.3, y=0.7),)))
    assert len(result.path) > 2
    assert result.raw_path_pose_count == len(result.path)
    assert result.path_shortcut_applied is False
```

- [ ] **Step 2: Add failing obstacle and diagonal-corner tests**

Construct a wall that requires a detour and assert the shortened path has more
than one segment, fewer poses than the raw path, and only traversable
supercover cells. Also assert:

```python
grid = replace_cells(make_grid(), [(41, 40), (40, 41)])
planner = Phase4Planner(PlannerConfig(enable_path_shortcutting=True))
assert planner._line_of_sight(grid, (40, 40), (41, 41)) is False
```

- [ ] **Step 3: Run the new tests and verify RED**

Run:

```bash
PYTHONPATH="$PWD/src/track_robot_semantic_search" \
  /usr/bin/python3 -m pytest -p no:launch_testing -q \
  src/track_robot_semantic_search/test/test_approach_planning.py
```

Expected: FAIL because `enable_path_shortcutting` and shortcut result metrics do
not exist.

- [ ] **Step 4: Implement the minimal shortcut algorithm**

Add to `PlannerConfig`:

```python
enable_path_shortcutting: bool = False
```

Add result evidence:

```python
raw_path_pose_count: int = 0
path_length_m: float = 0.0
path_shortcut_applied: bool = False
```

Implement a center-to-center supercover traversal. On an exact grid-corner
crossing, include both orthogonal side cells before the diagonal cell. Implement
line of sight as:

```python
def _line_of_sight(self, grid, start, end):
    return all(
        self._traversable(grid, cell)
        for cell in self._supercover_cells(start, end)
    )
```

Implement farthest-visible shortcutting:

```python
def _shortcut_path(self, grid, cells):
    if len(cells) <= 2:
        return tuple(cells)
    output = [cells[0]]
    anchor = 0
    while anchor < len(cells) - 1:
        selected = None
        for candidate in range(len(cells) - 1, anchor, -1):
            if self._line_of_sight(
                    grid, cells[anchor], cells[candidate]):
                selected = candidate
                break
        if selected is None:
            return ()
        output.append(cells[selected])
        anchor = selected
    return tuple(output)
```

When shortcutting is enabled, require diagonal graph transitions to have
traversable orthogonal side cells. This prevents the raw search from selecting
a diagonal move that the supercover safety check must reject. When disabled,
retain the original neighbor behavior for exact rollback.

If shortcutting unexpectedly returns an empty path, return `blocked_path`.
Otherwise publish the shortened cells and record raw count, output length in
metres, and whether the cell sequence changed.

- [ ] **Step 5: Run core tests and verify GREEN**

Run the command from Step 3. Expected: all tests pass.

- [ ] **Step 6: Commit the independently tested core**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/approach_planning.py \
  src/track_robot_semantic_search/test/test_approach_planning.py
git commit -m "feat: shortcut collision-free Phase 4 paths"
```

---

### Task 2: ROS parameter, configuration, and diagnostics

**Files:**

- Modify:
  `src/track_robot_semantic_search/track_robot_semantic_search/approach_planner_node.py`
- Modify: `src/track_robot_semantic_search/config/semantic_search_phase4.yaml`
- Modify: `src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`
- Test:
  `src/track_robot_semantic_search/test/test_phase4_planning_node_contract.py`
- Test:
  `src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`

**Interfaces:**

- Consumes: Task 1 `PlannerConfig.enable_path_shortcutting` and PlanResult path
  evidence.
- Produces: ROS parameter `enable_path_shortcutting` and additive diagnostic
  keys `raw_path_pose_count`, `path_segment_count`, `path_length_m`, and
  `path_shortcut_applied`.

- [ ] **Step 1: Write failing node/config contract tests**

Assert the node declares and passes `enable_path_shortcutting`, both YAML files
set it to `true`, and planning diagnostics contain:

```text
raw_path_pose_count
path_segment_count
path_length_m
path_shortcut_applied
```

- [ ] **Step 2: Run contract tests and verify RED**

Run:

```bash
PYTHONPATH="$PWD/src/track_robot_semantic_search" \
  /usr/bin/python3 -m pytest -p no:launch_testing -q \
  src/track_robot_semantic_search/test/test_phase4_planning_node_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py
```

Expected: FAIL because parameter and diagnostic keys are absent.

- [ ] **Step 3: Wire the parameter and diagnostics**

Pass the parameter into `PlannerConfig`:

```python
enable_path_shortcutting=bool(self.declare_parameter(
    'enable_path_shortcutting', False).value),
```

Publish diagnostics:

```python
'raw_path_pose_count': str(result.raw_path_pose_count),
'path_segment_count': str(max(0, len(result.path) - 1)),
'path_length_m': '{:.3f}'.format(result.path_length_m),
'path_shortcut_applied': str(
    result.path_shortcut_applied).lower(),
```

Set `enable_path_shortcutting: true` in the Phase 4 and Phase 4A YAML files.

- [ ] **Step 4: Run contract tests and verify GREEN**

Run the command from Step 2. Expected: all tests pass.

- [ ] **Step 5: Commit parameter and diagnostic wiring**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/approach_planner_node.py \
  src/track_robot_semantic_search/config/semantic_search_phase4.yaml \
  src/track_robot_semantic_search/config/semantic_search_phase4a.yaml \
  src/track_robot_semantic_search/test/test_phase4_planning_node_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py
git commit -m "feat: enable Phase 4 path shortcut diagnostics"
```

---

### Task 3: Regression and performance gate

**Files:**

- Inspect without modifying unless Task 3 Step 1 exposes a missing regression
  assertion:
  `src/track_robot_semantic_search/test/test_phase04_live_validation.py`
- Record results in the final implementation handoff; do not add generated
  build or log directories to Git.

**Interfaces:**

- Consumes: unchanged Phase 0-4A ROS interfaces and Task 2 diagnostics.
- Produces: evidence that path quality improves without safety, behavior, or
  latency regression.

- [ ] **Step 1: Run focused package tests**

```bash
PYTHONPATH="$PWD/src/track_robot_semantic_search" \
  /usr/bin/python3 -m pytest -p no:launch_testing -q \
  src/track_robot_semantic_search/test/test_approach_planning.py \
  src/track_robot_semantic_search/test/test_phase4_planning_node_contract.py \
  src/track_robot_semantic_search/test/test_phase4a_live_validation.py \
  src/track_robot_semantic_search/test/test_phase04_live_validation.py \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py
```

Expected: all tests pass.

- [ ] **Step 2: Run deterministic Phase 4 validation**

Run:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run track_robot_semantic_search semantic_search_phase4_validate \
  --output /tmp/phase4_path_shortcut_contract.json
```

Verify the JSON reports success, no target, ambiguity, lost target, invalid
position, blocked path, stale map, localization reset, and search-budget cases
without changing their expected status or reason.

- [ ] **Step 3: Build affected ROS packages**

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install \
  --packages-select track_robot_semantic_search track_robot_bringup
```

Expected: both packages build successfully.

- [ ] **Step 4: Quantify the reproduced path**

Run the deterministic empty-grid case at target `(3.17, -1.64)`. Expected:

```text
raw_path_pose_count=49
published_path_pose_count=2
path_segment_count=1
path_shortcut_applied=true
```

- [ ] **Step 5: Run bounded Phase 0-4A live regression when sensors are active**

Use `ROS_DOMAIN_ID=20`, planning-only mode, and the existing standardized
Phase 4A launch/validator. Confirm the same global ID, frames, timestamps,
failure behavior, zero motion publishers, collision-free path, and no planner
P95 latency increase above five percent. Stop all test-owned nodes afterward.

- [ ] **Step 6: Commit only any necessary additive validation assertion**

If no validator source change is necessary, do not create an empty commit.
