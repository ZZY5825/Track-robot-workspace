# ZED-Only Semantic 3D Position Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Phase 1–5 semantic target's canonical 3D position come only from timestamp-correlated ZED registered depth, while retaining LiDAR exclusively for obstacle mapping and motion safety.

**Architecture:** `semantic_depth_enricher` will own camera-depth correlation through a bounded source-time buffer and publish exactly one enriched-or-2D-only observation for each input observation. `semantic_memory` remains the only global-ID/lifecycle owner; the Phase 4A selector consumes its canonical position without computing a second depth estimate. The Phase 1–5 profile will stop launching semantic LiDAR tracklets and disable camera-to-LiDAR attachment, without changing sensor LiDAR, local obstacle map, costmaps, or Nav2.

**Tech Stack:** ROS 2 Foxy, Python 3/rclpy, C++17/rclcpp, ZED `32FC1` registered depth, tf2 source-time transforms, NumPy, pytest, colcon/ament.

## Global Constraints

- Implement only the approved 2026-08-11 ZED-only semantic-3D design; do not tune or refactor Nav2.
- Preserve all public topics, ROS messages, semantic global IDs, query IDs, memory epochs, and default behavior outside the Phase 1–5 semantic-search profile.
- Keep `/rslidar_points`, `local_obstacle_map_node`, Nav2 costmaps, and the motion safety chain active and unchanged.
- Do not alter the human-tracking Camera–LiDAR pipeline or generic semantic-memory defaults.
- Never publish `(0, 0, 0)` as a valid target position.
- Missing/stale/invalid depth or unavailable exact-time TF must produce the original 2D-only observation exactly once.
- Use test-driven development and one logically independent commit per task.
- Run from `/home/track-robot/track_robot_ws/.worktrees/main-integration`; ROS workspace commands run from its `track_robot_ws/` child.

---

## File Map

- Create `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/depth_frame_buffer.py`: bounded, source-time-only ZED depth storage and deterministic nearest-frame selection.
- Create `track_robot_ws/src/track_robot_semantic_search/test/test_depth_frame_buffer.py`: buffer bounds, tie-breaking, maximum delta, and timestamp rollback tests.
- Modify `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/phase4a_depth.py`: expose typed depth rejection reasons and valid sample counts.
- Modify `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/spatial_observation.py`: return a structured enrichment result without changing ROS interfaces.
- Modify `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/spatial_observation_node.py`: use the buffer, exact-time TF, and bounded diagnostics counters.
- Modify `track_robot_ws/src/track_robot_semantic_search/test/test_phase4a_depth.py` and `test/test_spatial_observation.py`: prove valid geometry and fail-closed 2D fallback.
- Modify `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/phase4a_selector_node.py`: remove the duplicate ZED depth owner.
- Modify `track_robot_ws/src/track_robot_semantic_search/test/test_phase4a_selector.py`: contract-test canonical-memory-only geometry.
- Modify `track_robot_ws/src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`: configure the single enricher and remove selector depth parameters.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`: stop launching semantic LiDAR tracklets and disable semantic attachment only in this profile.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`: prove ZED-only semantic geometry while keeping navigation LiDAR.
- Modify `track_robot_ws/src/track_robot/track_robot_semantic_memory/config/phase4a_test.yaml`: activate camera-only memory and disable semantic LiDAR mutation.
- Modify `track_robot_ws/src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`: permit the static target profile without degraded LiDAR attachment while keeping its bounded lifecycle checks.
- Modify `track_robot_ws/src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py`: protect generic defaults and the Phase 4A camera-only profile.
- Modify `track_robot_ws/docs/guides/semantic-search/phase4b-nav2-supervised-test.md`: record the new ownership, diagnostics, and live validation commands.
- Create `track_robot_ws/docs/reports/semantic-search/2026-08-11-zed-depth-only-live-validation.md`: record live evidence; do not pre-fill results.

---

### Task 1: Add a bounded source-time ZED depth buffer

**Files:**
- Create: `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/depth_frame_buffer.py`
- Create: `track_robot_ws/src/track_robot_semantic_search/test/test_depth_frame_buffer.py`

**Interfaces:**
- Produces: `DepthFrame(stamp_ns: int, frame_id: str, image: numpy.ndarray)`.
- Produces: `DepthMatch(frame: DepthFrame, delta_ns: int)`.
- Produces: `DepthFrameBuffer(max_frames: int, max_age_ns: int)`, `push(frame)`, `nearest(stamp_ns, maximum_delta_ns)`, `clear()`, and `size`.
- Timestamp rollback means a newly pushed positive timestamp older than the latest buffered timestamp clears the entire previous epoch before inserting the new frame.
- Equal-distance ties choose the earlier depth timestamp.

- [ ] **Step 1: Write the failing buffer tests**

```python
import numpy as np

from track_robot_semantic_search.depth_frame_buffer import (
    DepthFrame,
    DepthFrameBuffer,
)


def frame(stamp_ns):
    return DepthFrame(stamp_ns, 'zed_left_camera_optical_frame',
                      np.full((2, 2), float(stamp_ns)))


def test_nearest_uses_source_time_and_earlier_tie_break():
    buffer = DepthFrameBuffer(max_frames=4, max_age_ns=1_000)
    buffer.push(frame(100))
    buffer.push(frame(200))
    match = buffer.nearest(150, maximum_delta_ns=50)
    assert match.frame.stamp_ns == 100
    assert match.delta_ns == 50


def test_nearest_rejects_frame_outside_maximum_delta():
    buffer = DepthFrameBuffer(max_frames=4, max_age_ns=1_000)
    buffer.push(frame(100))
    assert buffer.nearest(201, maximum_delta_ns=100) is None


def test_push_enforces_count_and_age_bounds():
    buffer = DepthFrameBuffer(max_frames=2, max_age_ns=150)
    for stamp_ns in (100, 200, 300):
        buffer.push(frame(stamp_ns))
    assert buffer.size == 2
    assert buffer.nearest(100, maximum_delta_ns=1_000).frame.stamp_ns == 200


def test_timestamp_rollback_clears_previous_camera_epoch():
    buffer = DepthFrameBuffer(max_frames=4, max_age_ns=1_000)
    buffer.push(frame(10_000))
    buffer.push(frame(100))
    assert buffer.size == 1
    assert buffer.nearest(100, maximum_delta_ns=0).frame.stamp_ns == 100
```

- [ ] **Step 2: Run the tests and verify the module is missing**

Run:

```bash
cd track_robot_ws
source /opt/ros/foxy/setup.bash
python3 -m pytest src/track_robot_semantic_search/test/test_depth_frame_buffer.py -q
```

Expected: collection fails with `ModuleNotFoundError: No module named 'track_robot_semantic_search.depth_frame_buffer'`.

- [ ] **Step 3: Implement the minimal bounded buffer**

```python
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class DepthFrame:
    stamp_ns: int
    frame_id: str
    image: object


@dataclass(frozen=True)
class DepthMatch:
    frame: DepthFrame
    delta_ns: int


class DepthFrameBuffer:
    def __init__(self, max_frames, max_age_ns):
        if int(max_frames) <= 0 or int(max_age_ns) <= 0:
            raise ValueError('depth buffer bounds must be positive')
        self._frames = deque()
        self._max_frames = int(max_frames)
        self._max_age_ns = int(max_age_ns)

    @property
    def size(self):
        return len(self._frames)

    def clear(self):
        self._frames.clear()

    def push(self, frame):
        if int(frame.stamp_ns) <= 0 or not str(frame.frame_id):
            raise ValueError('depth frame stamp and frame_id are required')
        if self._frames and frame.stamp_ns < self._frames[-1].stamp_ns:
            self.clear()
        self._frames.append(frame)
        newest = frame.stamp_ns
        while self._frames and (
                len(self._frames) > self._max_frames
                or newest - self._frames[0].stamp_ns > self._max_age_ns):
            self._frames.popleft()

    def nearest(self, stamp_ns, maximum_delta_ns):
        stamp_ns = int(stamp_ns)
        maximum_delta_ns = int(maximum_delta_ns)
        if stamp_ns <= 0 or maximum_delta_ns < 0 or not self._frames:
            return None
        selected = min(
            self._frames,
            key=lambda frame: (abs(frame.stamp_ns - stamp_ns), frame.stamp_ns),
        )
        delta_ns = abs(selected.stamp_ns - stamp_ns)
        if delta_ns > maximum_delta_ns:
            return None
        return DepthMatch(selected, delta_ns)
```

- [ ] **Step 4: Run the focused tests**

Run: `python3 -m pytest src/track_robot_semantic_search/test/test_depth_frame_buffer.py -q`

Expected: `4 passed`.

- [ ] **Step 5: Commit the independent buffer**

```bash
git add track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/depth_frame_buffer.py track_robot_ws/src/track_robot_semantic_search/test/test_depth_frame_buffer.py
git commit -m "feat(semantic-search): buffer ZED depth by source time"
```

---

### Task 2: Correlate each observation with ZED depth and TF at one timestamp

**Files:**
- Modify: `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/phase4a_depth.py`
- Modify: `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/spatial_observation.py`
- Modify: `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/spatial_observation_node.py`
- Modify: `track_robot_ws/src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`
- Modify: `track_robot_ws/src/track_robot_semantic_search/test/test_phase4a_depth.py`
- Modify: `track_robot_ws/src/track_robot_semantic_search/test/test_spatial_observation.py`
- Create: `track_robot_ws/src/track_robot_semantic_search/test/test_spatial_observation_node_contract.py`

**Interfaces:**
- `DepthEstimationError(reason: str, valid_samples: int = 0)` uses only `insufficient_depth_samples` or `depth_out_of_range`.
- Extend `DepthEstimate` with `valid_samples: int` and `total_samples: int`.
- Replace the tuple result with `SpatializationResult(observation, accepted, reason, valid_depth_samples, depth_quality)`.
- `semantic_depth_enricher` publishes `diagnostic_msgs/DiagnosticArray` on `/semantic_search/spatial_observation_diagnostics`.
- Fixed counter keys are `matched_depth`, `no_matching_depth`, `depth_delta_exceeded`, `insufficient_depth_samples`, `depth_out_of_range`, `tf_unavailable`, and `invalid_transformed_position`.

- [ ] **Step 1: Add failing estimator and spatialization reason tests**

Add these assertions, and update the existing tuple-unpacking assertions to use the structured result fields shown below:

```python
from track_robot_semantic_search.phase4a_depth import DepthEstimationError

with pytest.raises(DepthEstimationError) as caught:
    estimate_depth_point(np.full((4, 4), np.nan), roi=(0, 0, 4, 4),
                         intrinsics=intrinsics, minimum_samples=4)
assert caught.value.reason == 'insufficient_depth_samples'

source = observation()
result = spatialize_observation(
    source,
    depth=np.full((4, 4), 2.0, dtype=np.float32),
    intrinsics=CameraIntrinsics(fx=100.0, fy=100.0, cx=1.5, cy=1.5),
    translation=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    localization_epoch_id=7,
    depth_stamp_ns=2_500_000_123,
    config=SpatialObservationConfig(minimum_samples=4, inner_fraction=1.0),
)
assert result.accepted is True
assert result.reason == 'matched_depth'
assert result.valid_depth_samples == 16
assert result.observation.position_valid is True

result = spatialize_observation(
    source,
    depth=np.full((4, 4), np.nan, dtype=np.float32),
    intrinsics=CameraIntrinsics(fx=100.0, fy=100.0, cx=1.5, cy=1.5),
    translation=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    localization_epoch_id=7,
    depth_stamp_ns=2_500_000_123,
    config=SpatialObservationConfig(minimum_samples=4, inner_fraction=1.0),
)
assert result.accepted is False
assert result.reason == 'insufficient_depth_samples'
assert result.observation.position_valid is False
```

Replace the existing `pytest.raises(ValueError, match='insufficient valid depth')`
assertion in `test_estimate_depth_point_rejects_sparse_or_out_of_range_depth`
with `pytest.raises(DepthEstimationError)` and assert
`caught.value.reason == 'depth_out_of_range'` for its single `20.0 m` sample.

- [ ] **Step 2: Add a failing node source contract test**

```python
from pathlib import Path


def test_enricher_matches_observation_stamp_and_uses_exact_depth_tf():
    source = (Path(__file__).resolve().parents[1]
              / 'track_robot_semantic_search'
              / 'spatial_observation_node.py').read_text()
    assert 'DepthFrameBuffer' in source
    assert 'nearest(source_stamp_ns' in source
    assert 'Time(nanoseconds=match.frame.stamp_ns)' in source
    assert 'Time()' not in source
    assert 'DiagnosticArray' in source
```

- [ ] **Step 3: Run the focused tests and verify they fail**

Run:

```bash
cd track_robot_ws
source /opt/ros/foxy/setup.bash
python3 -m pytest \
  src/track_robot_semantic_search/test/test_phase4a_depth.py \
  src/track_robot_semantic_search/test/test_spatial_observation.py \
  src/track_robot_semantic_search/test/test_spatial_observation_node_contract.py -q
```

Expected: failures for missing `DepthEstimationError`, structured result fields, and buffer usage.

- [ ] **Step 4: Add typed depth failures and sample counts**

Implement in `phase4a_depth.py`:

```python
class DepthEstimationError(ValueError):
    def __init__(self, reason, valid_samples=0):
        super().__init__(reason)
        self.reason = str(reason)
        self.valid_samples = int(valid_samples)


@dataclass(frozen=True)
class DepthEstimate:
    x: float
    y: float
    z: float
    depth_m: float
    quality: float
    valid_samples: int
    total_samples: int
```

After extracting the centre sample, distinguish the failures and return counts:

```python
finite_values = sample[np.isfinite(sample)]
if finite_values.size == 0:
    raise DepthEstimationError('insufficient_depth_samples', 0)
in_range = finite_values[
    (finite_values >= minimum_depth_m)
    & (finite_values <= maximum_depth_m)
]
if in_range.size == 0:
    raise DepthEstimationError('depth_out_of_range', 0)
if in_range.size < minimum_samples:
    raise DepthEstimationError('insufficient_depth_samples', in_range.size)
depth_m = float(np.median(in_range))
```

Set `quality=float(in_range.size) / float(sample.size)`, `valid_samples=int(in_range.size)`, and `total_samples=int(sample.size)` in the returned `DepthEstimate`.

- [ ] **Step 5: Return a structured, fail-closed spatialization result**

Implement in `spatial_observation.py`:

```python
@dataclass(frozen=True)
class SpatializationResult:
    observation: object
    accepted: bool
    reason: str
    valid_depth_samples: int
    depth_quality: float
```

Catch `DepthEstimationError` separately and return its reason/sample count. Return `invalid_transformed_position` for invalid transformed coordinates, and on success return:

```python
return SpatializationResult(
    observation=output,
    accepted=True,
    reason='matched_depth',
    valid_depth_samples=estimate.valid_samples,
    depth_quality=estimate.quality,
)
```

For all rejected results, leave the deep-copied observation untouched with `position_valid` unchanged; do not synthesize zero geometry.

- [ ] **Step 6: Replace the latest-depth fields with the bounded buffer**

In `spatial_observation_node.py`:

```python
self._depth_buffer = DepthFrameBuffer(
    max_frames=int(self.declare_parameter('depth_buffer_frames', 16).value),
    max_age_ns=int(float(self.declare_parameter(
        'depth_buffer_max_age_sec', 2.0).value) * 1_000_000_000),
)
```

`_on_depth` converts the image, validates it, and calls:

```python
self._depth_buffer.push(DepthFrame(
    stamp_ns=_stamp_ns(message.header.stamp),
    frame_id=str(message.header.frame_id),
    image=depth,
))
```

For every observation, compute its camera source stamp first, call
`match = self._depth_buffer.nearest(source_stamp_ns, self._maximum_depth_delta_ns)`, and perform:

```python
transform = self._tf_buffer.lookup_transform(
    self._config.frame_id,
    match.frame.frame_id,
    Time(nanoseconds=match.frame.stamp_ns),
    timeout=Duration(seconds=self._tf_timeout_sec),
)
```

Call `spatialize_observation` with `match.frame.image` and `match.frame.stamp_ns`. Append exactly `result.observation` once. If there is no match, append one deep copy and record `no_matching_depth` when the buffer is empty, otherwise `depth_delta_exceeded`.

- [ ] **Step 7: Add bounded diagnostics without changing observation flow**

Use a fixed-key counter dictionary and publish one `DiagnosticArray` after every observation array. Each status must include:

```python
values = [
    KeyValue(key='latest_reason', value=latest_reason),
    KeyValue(key='depth_delta_ms', value='{:.3f}'.format(depth_delta_ns / 1e6)),
    KeyValue(key='valid_depth_samples', value=str(valid_depth_samples)),
    KeyValue(key='depth_quality', value='{:.6f}'.format(depth_quality)),
] + [KeyValue(key=key, value=str(self._counters[key]))
     for key in self._COUNTER_KEYS]
```

Set level `OK` only when at least one observation was enriched; otherwise use `WARN`. TF failure records `tf_unavailable`; an invalid transformed position records `invalid_transformed_position`. Counter keys never grow dynamically.

- [ ] **Step 8: Configure the source-time bounds**

Under `semantic_depth_enricher.ros__parameters` in `semantic_search_phase4a.yaml`, set:

```yaml
diagnostics_topic: /semantic_search/spatial_observation_diagnostics
depth_buffer_frames: 16
depth_buffer_max_age_sec: 2.0
maximum_depth_delta_sec: 0.20
```

Keep `minimum_depth_samples: 20`, range `0.3`–`8.0`, `depth_inner_fraction: 0.5`, and `tf_timeout_sec: 0.05` unchanged so this change isolates source ownership and time correlation.

- [ ] **Step 9: Run focused regression**

Run the three pytest files from Step 3 plus `test_depth_frame_buffer.py`.

Expected: all tests pass; no valid result contains non-finite coordinates or valid zero geometry.

- [ ] **Step 10: Commit exact-time ZED enrichment**

```bash
git add track_robot_ws/src/track_robot_semantic_search
git commit -m "fix(semantic-search): correlate ZED depth by camera stamp"
```

---

### Task 3: Remove selector-side duplicate depth computation

**Files:**
- Modify: `track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/phase4a_selector_node.py`
- Modify: `track_robot_ws/src/track_robot_semantic_search/test/test_phase4a_selector.py`
- Modify: `track_robot_ws/src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`

**Interfaces:**
- Consumes only canonical `SemanticObject.position*`, `localization_epoch_id`, and support state from `/semantic_memory/diagnostic_ranking`.
- Produces the same `/semantic_search/phase4a/spatial_objects`, `/semantic_search/phase4a/selected_target`, and diagnostics topics.
- Does not subscribe to observations, depth, camera info, or TF.

- [ ] **Step 1: Replace the old fallback contract with failing ownership assertions**

In `test_phase4a_selector.py`, read `phase4a_selector_node.py` and assert:

```python
assert 'CvBridge' not in source
assert 'TransformListener' not in source
assert 'SemanticObservationArray' not in source
assert 'estimate_depth_point' not in source
assert '_depth_geometry' not in source
assert 'fallback_depth_available=False' in source
```

- [ ] **Step 2: Run the selector tests and verify failure**

Run: `python3 -m pytest src/track_robot_semantic_search/test/test_phase4a_selector.py -q`

Expected: ownership assertions fail against the existing duplicate depth path.

- [ ] **Step 3: Delete only the duplicate geometry path**

Remove `_DepthGeometry`, `CvBridge`, tf2, camera/depth imports and state, the four observation/depth/camera-info/TF subscriptions, `_on_camera_info`, `_on_depth`, `_on_observations`, and `_geometry`.

Build candidates directly from canonical memory fields:

```python
support = classify_spatial_support(
    support=public_support,
    position_valid=bool(message.position_valid),
    fallback_depth_available=False,
)
return ObjectCandidate(
    memory_epoch_id=int(memory_epoch_id),
    global_object_id=int(message.global_object_id),
    localization_epoch_id=(
        int(message.localization_epoch_id)
        if int(message.localization_epoch_id) != 0
        else self._localization_epoch_id),
    query_id=int(message.active_query_id),
    query_version=int(message.active_query_version),
    lifecycle=lifecycle,
    support=support,
    position_frame_id=str(message.position_frame_id),
    position_valid=bool(message.position_valid),
    x=float(message.position.x),
    y=float(message.position.y),
    z=float(message.position.z),
    relevance=float(message.task_relevance),
    uncertainty=float(message.uncertainty),
    last_seen_ns=_stamp_ns(message.last_seen),
)
```

In `_on_ranking`, deep-copy each memory object, append it to `spatial_output.objects` only when `position_valid`, and use that same copy for the selected target. Do not alter IDs, position, covariance, support, association confidence, or timestamps.

- [ ] **Step 4: Remove selector-only depth parameters**

Delete `observations_topic`, `depth_topic`, `camera_info_topic`, `maximum_depth_age_sec`, `minimum_depth_samples`, `minimum_depth_m`, `maximum_depth_m`, and `depth_inner_fraction` only from `phase4a_target_selector` in `semantic_search_phase4a.yaml`. Keep the enricher parameters.

- [ ] **Step 5: Run selector and Phase 4A config tests**

Run:

```bash
python3 -m pytest \
  src/track_robot_semantic_search/test/test_phase4a_selector.py \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py -q
```

Expected: selector tests pass; the existing launch-contract test fails only where it still expects selector depth parameters, establishing the next contract update.

- [ ] **Step 6: Commit the single-owner selector**

```bash
git add track_robot_ws/src/track_robot_semantic_search/track_robot_semantic_search/phase4a_selector_node.py track_robot_ws/src/track_robot_semantic_search/test/test_phase4a_selector.py track_robot_ws/src/track_robot_semantic_search/config/semantic_search_phase4a.yaml
git commit -m "refactor(semantic-search): keep one semantic 3D owner"
```

---

### Task 4: Isolate semantic LiDAR while preserving navigation LiDAR

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_semantic_memory/config/phase4a_test.yaml`
- Modify: `track_robot_ws/src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`
- Modify: `track_robot_ws/src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py`

**Interfaces:**
- Phase 1–5 semantic memory consumes `/semantic_memory/spatial_observations` and retains global-ID/lifecycle ownership.
- It does not consume semantic LiDAR tracklets or attach LiDAR geometry in this profile.
- Sensor launch still uses `start_lidar: true`; obstacle map and Nav2 still consume `/rslidar_points`.

- [ ] **Step 1: Write the failing Phase 4A profile contract**

Update `test_phase4a_launch_contract.py` to assert:

```python
assert 'semantic_memory_lidar_tracklets.launch.py' not in source
assert "'start_lidar': 'true'" in source
assert "executable='local_obstacle_map_node'" in source
assert "'enable_test_camera_attachment': 'false'" in source

assert memory['camera_only_memory_enabled'] is True
assert memory['camera_attachment_enabled'] is False
assert memory['enable_test_camera_attachment'] is False
assert memory['allow_degraded_calibration'] is False
assert memory['observations_topic'] == '/semantic_memory/spatial_observations'

selector = search['phase4a_target_selector']['ros__parameters']
assert 'depth_topic' not in selector
enricher = search['semantic_depth_enricher']['ros__parameters']
assert enricher['depth_topic'] == '/zed/zed_node/depth/depth_registered'
assert enricher['maximum_depth_delta_sec'] == 0.20
```

- [ ] **Step 2: Add the failing static-profile guard contract**

In semantic-memory `test_launch_contract.py`, assert the C++ source includes a camera-only requirement and conditional degraded-attachment requirement:

```python
assert 'static_target_profile requires camera_only_memory_enabled' in source
assert 'camera_attachment_enabled_ &&' in source
```

- [ ] **Step 3: Run both contract suites and verify failure**

Run:

```bash
cd track_robot_ws
source /opt/ros/foxy/setup.bash
python3 -m pytest \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py -q
```

Expected: failures identify the still-active semantic LiDAR include/attachment and old static-profile guard.

- [ ] **Step 4: Disable semantic attachment only in the Phase 4A profile**

In `phase4a_test.yaml`, set:

```yaml
camera_only_memory_enabled: true
association_shadow_mode: false
camera_attachment_enabled: false
enable_test_camera_attachment: false
allow_degraded_calibration: false
```

Do not edit `semantic_memory.yaml`, `phase123_test.yaml`, association algorithms, human tracking, or message definitions.

- [ ] **Step 5: Remove the semantic tracklet include but retain obstacle LiDAR**

Delete only the launch action whose package is `track_robot_lidar_tracking` and whose launch file is `semantic_memory_lidar_tracklets.launch.py` from `semantic_search_phase4a.launch.py`. Keep `semantic_search_sensors.launch.py` with `start_lidar: true`, `local_obstacle_map_node`, all costmap/safety nodes, and `/rslidar_points` unchanged. Pass `enable_test_camera_attachment: false` and `allow_degraded_calibration: false` to `semantic_memory_phase2.launch.py`.

- [ ] **Step 6: Decouple static-target lifecycle from degraded LiDAR attachment**

Replace only the first static-profile authorization check in `semantic_memory_node.cpp` with:

```cpp
if (!camera_only_memory_enabled_) {
  throw std::invalid_argument(
          "static_target_profile requires camera_only_memory_enabled");
}
if (camera_attachment_enabled_ &&
  (!enable_test_camera_attachment_ || !allow_degraded_calibration_))
{
  throw std::invalid_argument(
          "static target LiDAR attachment requires the explicit degraded test profile");
}
```

Keep the existing four-second maximum budget and stale/lost ordering checks verbatim.

- [ ] **Step 7: Run profile and semantic-memory regressions**

Run:

```bash
python3 -m pytest \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py -q
colcon test --packages-select track_robot_semantic_memory \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all affected tests pass, generic defaults still prove `camera_attachment_enabled: false`, and no test enables semantic LiDAR mutation for Phase 4A.

- [ ] **Step 8: Commit the profile isolation**

```bash
git add track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py track_robot_ws/src/track_robot/track_robot_semantic_memory
git commit -m "fix(semantic-memory): use ZED-only geometry in semantic search"
```

---

### Task 5: Build, regress, and run the stationary green-bottle gate

**Files:**
- Modify: `track_robot_ws/docs/guides/semantic-search/phase4b-nav2-supervised-test.md`
- Create: `track_robot_ws/docs/reports/semantic-search/2026-08-11-zed-depth-only-live-validation.md`

**Interfaces:**
- Documents `/semantic_search/spatial_observation_diagnostics`, `/semantic_memory/spatial_observations`, `/semantic_memory/diagnostic_ranking`, and `/semantic_search/phase4a/selected_target`.
- Live validation is observation/planning only; do not authorize base motion and do not change planner parameters.

- [ ] **Step 1: Run all affected Python tests**

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
python3 -m pytest \
  src/track_robot_semantic_search/test \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py -q
```

Expected: zero failures.

- [ ] **Step 2: Build the three affected packages**

```bash
colcon build --symlink-install --packages-select \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup
```

Expected: all three packages finish successfully.

- [ ] **Step 3: Run package test suites and inspect every result**

```bash
source install/setup.bash
colcon test --packages-select \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests and zero test errors. Do not proceed to live validation otherwise.

- [ ] **Step 4: Update the guide before live testing**

Add a section that explicitly states:

```text
Semantic 3D owner: ZED depth_registered -> semantic_depth_enricher -> semantic_memory.
LiDAR role: obstacle grid, Nav2 costmaps, and motion safety only.
Semantic LiDAR tracklets/attachment: disabled in this profile.
Depth diagnostic: /semantic_search/spatial_observation_diagnostics.
```

Include commands for `ros2 topic echo` on the diagnostics and selected-target topics, and state that Nav2 planner tuning is deferred until this gate passes.

- [ ] **Step 5: Launch in no-motion observation mode**

Use the guide's canonical Phase 1–5 environment and launch command, but do not call `start_approach`, `start_finding`, or publish any velocity. Confirm:

```bash
ros2 topic hz /zed/zed_node/depth/depth_registered
ros2 topic hz /semantic_memory/spatial_observations
ros2 topic echo /semantic_search/spatial_observation_diagnostics
ros2 topic echo /semantic_search/phase4a/selected_target
ros2 topic hz /rslidar_points
```

Expected: ZED depth and semantic observations update; LiDAR remains present for obstacles; diagnostics expose a finite delta and fixed counters.

- [ ] **Step 6: Record the 30–60 second green-bottle evidence**

Place one stationary green bottle at a measured fixed distance. Record start/end time, raw ZED depth rate, spatial observation rate, selected global ID, position samples, diagnostic counters, and any dropouts in `2026-08-11-zed-depth-only-live-validation.md`.

Acceptance gates:

- no `position_valid=true` sample has non-finite coordinates or distance `0 m`;
- no unexplained multi-metre one-frame jump occurs without a corresponding selected ZED depth change;
- every 3D rejection has a diagnostic reason;
- one camera track keeps one semantic global ID through short depth-only dropouts;
- `/rslidar_points` and the obstacle map remain live despite no semantic LiDAR tracklet node;
- no executable motion is published during this validation.

- [ ] **Step 7: Stop all nodes started by this test**

Interrupt the canonical launch process and verify its children, RViz, camera, LiDAR, and semantic nodes terminate. Do not stop unrelated user processes.

- [ ] **Step 8: Record measured results and rollback point**

The report must contain commit SHA, exact commands, ROS domain, configuration paths, duration, PASS/FAIL for every acceptance gate, raw counter values, failure reasons, and rollback commits from Tasks 1–4. Write `NOT MEASURED` for unavailable evidence; do not estimate values.

- [ ] **Step 9: Commit documentation and measured evidence**

```bash
git add track_robot_ws/docs/guides/semantic-search/phase4b-nav2-supervised-test.md track_robot_ws/docs/reports/semantic-search/2026-08-11-zed-depth-only-live-validation.md
git commit -m "docs: validate ZED-only semantic 3D position"
```

Only after this live gate passes may the separate Nav2 planner investigation resume.
