# Semantic Search RViz Live Console Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a passive RViz console that submits semantic queries and shows exact-stamp 2D candidates, bounded 3D semantic objects, pipeline state, and calibrated best-candidate state.

**Architecture:** The existing Python semantic-search package owns an exact-stamp image overlay node. The existing C++ semantic-memory visualizer gains labels and a fail-closed winner highlight. A new isolated C++/Qt RViz plugin package owns query/session UI behavior. Bringup owns saved RViz configurations and a foreground visualization launch/CLI command.

**Tech Stack:** ROS 2 Foxy, Python 3.8, rclpy, OpenCV/cv_bridge, C++17, rclcpp, Qt 5 Widgets, RViz 2, pluginlib, nlohmann JSON, pytest, GoogleTest.

## Global Constraints

- ROS Domain is fixed to `20` by `semantic_search_ctl`.
- Visualization is passive and must never publish an action goal, `SearchMotionIntent`, `Twist`, or `/cmd_vel`.
- Images and regions are combined only when their source timestamps match exactly.
- DDS and in-process caches remain bounded.
- Best-candidate visualization consumes only `/semantic_memory/best_candidate`; it does not derive an uncalibrated winner.
- Existing Phase 1, Phase 2, and human-tracking behavior remains unchanged when visualization is not launched.
- No dependency downloader or model download is introduced.

---

### Task 1: Exact-stamp live image overlay

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/live_overlay.py`
- Create: `src/track_robot_semantic_search/test/test_live_overlay.py`
- Modify: `src/track_robot_semantic_search/setup.py`
- Modify: `src/track_robot_semantic_search/package.xml`

**Interfaces:**
- Consumes: `sensor_msgs/msg/Image` and `track_robot_interfaces/msg/SemanticRegionArray`.
- Produces: `render_overlay(image, regions, query_id, query_version) -> numpy.ndarray`, `ExactStampBuffer`, the `semantic_search_live_overlay` console script, and `/semantic_search/overlay_image`.

- [ ] **Step 1: Write failing pure rendering and correlation tests**

```python
def test_render_overlay_copies_image_and_draws_all_valid_candidates():
    source = numpy.zeros((120, 160, 3), dtype=numpy.uint8)
    rendered = render_overlay(
        source,
        [OverlayRegion(10, 20, 30, 40, 0.8),
         OverlayRegion(80, 30, 20, 25, 0.4)],
        query_id=7,
        query_version=2,
    )
    assert numpy.array_equal(source, numpy.zeros_like(source))
    assert rendered.shape == source.shape
    assert numpy.count_nonzero(rendered) > 0

def test_exact_stamp_buffer_pairs_only_equal_source_stamps():
    buffer = ExactStampBuffer(capacity=2)
    assert buffer.add_image((1, 2), object()) is None
    assert buffer.add_regions((1, 3), object()) is None
    pair = buffer.add_regions((1, 2), object())
    assert pair is not None
```

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_live_overlay.py
```

Expected: collection fails because `track_robot_semantic_search.live_overlay`
does not exist.

- [ ] **Step 3: Implement bounded pure overlay behavior**

Implement:

```python
from collections import OrderedDict
from dataclasses import dataclass

import cv2
import math

@dataclass(frozen=True)
class OverlayRegion:
    x: int
    y: int
    width: int
    height: int
    score: float

class ExactStampBuffer:
    def __init__(self, capacity=8):
        if capacity < 1:
            raise ValueError('capacity must be positive')
        self.capacity = capacity
        self.images = OrderedDict()
        self.regions = OrderedDict()

    def add_image(self, stamp, image):
        pending = self.regions.pop(stamp, None)
        if pending is not None:
            return image, pending
        self.images[stamp] = image
        self.images.move_to_end(stamp)
        while len(self.images) > self.capacity:
            self.images.popitem(last=False)
        return None

    def add_regions(self, stamp, regions):
        image = self.images.pop(stamp, None)
        if image is not None:
            return image, regions
        self.regions[stamp] = regions
        self.regions.move_to_end(stamp)
        while len(self.regions) > self.capacity:
            self.regions.popitem(last=False)
        return None

def render_overlay(image, regions, query_id, query_version):
    output = image.copy()
    ordered = sorted(
        (item for item in regions
         if item.width > 0 and item.height > 0 and math.isfinite(item.score)),
        key=lambda item: item.score,
        reverse=True,
    )
    for rank, region in enumerate(ordered, start=1):
        height, width = output.shape[:2]
        start = (
            max(0, min(width - 1, region.x)),
            max(0, min(height - 1, region.y)),
        )
        end = (
            max(0, min(width - 1, region.x + region.width)),
            max(0, min(height - 1, region.y + region.height)),
        )
        colour = (255, 255, 0) if rank == 1 else (0, 191, 255)
        cv2.rectangle(output, start, end, colour, 2)
        cv2.putText(
            output, '#{} score={:.3f}'.format(rank, region.score),
            (start[0], max(16, start[1] - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1,
        )
    header = (
        'query={}/{} CANDIDATES - NOT GROUND TRUTH'.format(
            query_id, query_version)
        if ordered else
        'query={}/{} NO CANDIDATES - NOT GROUND TRUTH'.format(
            query_id, query_version)
    )
    cv2.putText(
        output, header, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 255), 1,
    )
    return output
```

The buffer uses two `OrderedDict` instances capped at `capacity`. Rendering
clips partially visible boxes, rejects non-finite scores and empty boxes, sorts
by descending fused score, draws the best candidate cyan and others amber, and
adds `CANDIDATES - NOT GROUND TRUTH`.

- [ ] **Step 4: Verify GREEN**

Run the Task 1 pytest command and expect all tests to pass.

- [ ] **Step 5: Add the ROS adapter and package entry point**

`SemanticSearchLiveOverlay` subscribes with sensor-data QoS for images and
reliable depth ten for regions. It publishes a `bgr8` image only after an exact
stamp pair. Conversion errors use throttled warnings. Add:

```python
'semantic_search_live_overlay = '
'track_robot_semantic_search.live_overlay:main',
```

and declare `python3-opencv` as an execution dependency.

- [ ] **Step 6: Add source-contract tests and rerun package tests**

Assert the public topic defaults, bounded capacity, exact-stamp comparison, and
console entry point. Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_live_overlay.py \
  src/track_robot_semantic_search/test/test_launch_contract.py
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/track_robot_semantic_search
git commit -m "feat: add exact-stamp semantic overlay"
```

---

### Task 2: Labelled 3D objects and calibrated winner highlight

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/visualization.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/visualization.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/semantic_memory_visualizer_node.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_visualization.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py`

**Interfaces:**
- Consumes: `/semantic_memory/active_objects` and the fail-closed `/semantic_memory/best_candidate` `SemanticObjectArray`.
- Produces: cube, text, delete, and winner-halo markers on `/semantic_memory/markers`.

- [ ] **Step 1: Write failing marker tests**

Add tests proving:

```cpp
registry.set_best_candidate(GlobalObjectKey{10U, 1U});
const auto output = registry.update(input);
EXPECT_TRUE(has_namespace(output, "semantic_memory_objects"));
EXPECT_TRUE(has_namespace(output, "semantic_memory_labels"));
EXPECT_TRUE(has_namespace(output, "semantic_memory_best_candidate"));
EXPECT_TRUE(has_text(output, "object 1"));
EXPECT_TRUE(has_text(output, "BEST CANDIDATE"));
```

Also assert that clearing the best key and removing an object emit explicit
deletes in all previously published namespaces.

- [ ] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon test --packages-select track_robot_semantic_memory \
  --ctest-args -R test_visualization --event-handlers console_direct+
```

Expected: compile failure because `set_best_candidate` and labelled markers do
not exist.

- [ ] **Step 3: Implement marker namespaces and labels**

Add:

```cpp
void MarkerRegistry::set_best_candidate(
  std::optional<GlobalObjectKey> candidate);
```

Use the same stable numeric marker ID in distinct namespaces. Every active
object emits a cube and `TEXT_VIEW_FACING` label. The label includes ID,
lifecycle, support, motion, and finite task relevance. A selected object emits
a magenta translucent cube halo and `BEST CANDIDATE` text. Registry state
tracks the previously emitted winner so clearing or changing it emits deletes.

- [ ] **Step 4: Subscribe to the winner topic**

The visualizer node declares:

```cpp
best_candidate_topic = "/semantic_memory/best_candidate"
```

The callback accepts only an empty array or one valid object from the same
memory epoch. It calls `set_best_candidate`; the next bounded active snapshot
publishes the updated highlight.

- [ ] **Step 5: Verify GREEN and regression**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_interfaces track_robot_semantic_memory
source install/setup.bash
colcon test --packages-select track_robot_semantic_memory \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failures.

- [ ] **Step 6: Commit**

```bash
git add src/track_robot/track_robot_semantic_memory
git commit -m "feat: enrich semantic memory RViz markers"
```

---

### Task 3: Passive RViz query and status plugin

**Files:**
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/CMakeLists.txt`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/package.xml`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/plugin_description.xml`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/query_session.hpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/semantic_search_panel.hpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/query_session.cpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/semantic_search_panel.cpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_query_session.cpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`

**Interfaces:**
- Consumes: perception diagnostics, region arrays, active objects, and
  best-candidate arrays.
- Produces: canonical query JSON on `/semantic_search/query` and the RViz class
  `track_robot_semantic_search_rviz_plugins/SemanticSearchPanel`.

- [ ] **Step 1: Write failing query-session tests**

Cover:

```cpp
QuerySession session;
const auto first = session.new_query("  blue   chair  ", 100U);
EXPECT_EQ(first.query_id, 100U);
EXPECT_EQ(first.query_version, 1U);
EXPECT_EQ(first.normalized_text, "blue chair");

const auto revised = session.revise("dark blue chair");
EXPECT_EQ(revised.query_id, first.query_id);
EXPECT_EQ(revised.query_version, 2U);
EXPECT_THROW(session.new_query("", 101U), std::invalid_argument);
EXPECT_THROW(session.new_query(std::string(513U, 'x'), 102U),
  std::invalid_argument);
```

Assert JSON has exactly `query_id`, `query_text`, and `query_version`; later
timestamp seeds are strictly increasing even when the supplied seed repeats or
moves backwards.

- [ ] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_semantic_search_rviz_plugins
```

Expected: package or source files are missing.

- [ ] **Step 3: Implement the pure session model and package**

Use Qt `QString::normalized(QString::NormalizationForm_KC).simplified()` and
nlohmann JSON. `new_query` accepts a microsecond seed and makes the process ID
strictly increasing. `revise` fails before a new query and on uint64 overflow.

Configure `CMAKE_AUTOMOC`, C++17, `Qt5::Widgets`, `rviz_common`, `pluginlib`,
`rclcpp`, `std_msgs`, `track_robot_interfaces`, and `nlohmann_json`.

- [ ] **Step 4: Verify the session model passes**

Build and run the package gtest. Expected: PASS.

- [ ] **Step 5: Write the failing plugin contract test**

Assert:

- plugin XML exports `SemanticSearchPanel`;
- source contains only the approved query publisher;
- subscriptions use the four approved status topics;
- no `cmd_vel`, `Twist`, action client, reset, or inspection API appears;
- package exports the plugin description through `rviz_common`.

- [ ] **Step 6: Implement the Qt panel**

Build the fixed layout with a safety banner, `QLineEdit`, New/Revise buttons,
and labels for query, acknowledgement/model state, regions, active objects, and
best candidate. `onInitialize()` uses RViz's raw node, creates one query
publisher and four subscriptions, and queues callback-driven UI changes through
the `QMetaObject::invokeMethod` functor overload with
`Qt::QueuedConnection`.

Buttons publish immediately and never wait for diagnostics. Matching
diagnostics update acceptance/model state; mismatched diagnostics cannot
acknowledge the current query. Save and load topic names through RViz config.

- [ ] **Step 7: Build and test the plugin**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_interfaces track_robot_semantic_search_rviz_plugins
source install/setup.bash
colcon test --packages-select \
  track_robot_semantic_search_rviz_plugins --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failures and pluginlib export installed.

- [ ] **Step 8: Commit**

```bash
git add src/track_robot/track_robot_semantic_search_rviz_plugins
git commit -m "feat: add passive semantic search RViz panel"
```

---

### Task 4: Saved RViz views and foreground visualization command

**Files:**
- Create: `src/track_robot/track_robot_bringup/rviz/semantic_search_phase1.rviz`
- Create: `src/track_robot/track_robot_bringup/rviz/semantic_search_phase2.rviz`
- Create: `src/track_robot/track_robot_bringup/launch/semantic_search_visualization.launch.py`
- Modify: `src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_control_cli.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_launch_contract.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_quick_start_doc.py`
- Modify: `src/track_robot/track_robot_bringup/package.xml`

**Interfaces:**
- Consumes: installed RViz plugin and live overlay executable.
- Produces: `semantic_search_ctl visualize phase1|phase2`.

- [ ] **Step 1: Write failing parser and command tests**

Add:

```python
args = build_parser().parse_args(['visualize', 'phase2'])
assert args.command == 'visualize'
assert args.stage == 'phase2'
```

Inject `execvpe` and assert the exact command:

```python
[
    'ros2', 'launch', 'track_robot_bringup',
    'semantic_search_visualization.launch.py', 'stage:=phase2',
]
```

Assert the environment has Domain 20 and no process manager or hardware launch
is invoked.

- [ ] **Step 2: Verify RED**

Run the bringup control tests and expect argparse to reject `visualize`.

- [ ] **Step 3: Implement CLI and foreground launch**

Add a bounded `visualize` subparser with stages `phase1` and `phase2`. Reuse the
managed Domain 20 environment and `execvpe` the launch command.

The visualization launch starts `semantic_search_live_overlay` and RViz with
the selected installed config. Register `OnProcessExit` for RViz that emits
`Shutdown`, ensuring the overlay exits when the window closes.

- [ ] **Step 4: Add saved RViz configurations**

Phase 1 config includes Image on `/semantic_search/overlay_image` and the custom
panel with camera optical fixed frame. Phase 2 adds TF, PointCloud2 on
`/rslidar_points`, MarkerArray on `/semantic_memory/markers`, the image, and the
same panel with `odom` fixed frame.

- [ ] **Step 5: Verify GREEN**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_control_cli.py \
  src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/track_robot/track_robot_bringup
git commit -m "feat: add modular semantic search visualization launch"
```

---

### Task 5: Documentation, build, and no-hardware verification

**Files:**
- Modify: `docs/guides/semantic-search/phase2-recording-and-evaluation.md`
- Modify: `src/track_robot_semantic_search/README.md`
- Modify: `src/track_robot/track_robot_semantic_memory/README.md`
- Modify: root `README.md` if its quick-start section links feature guides.

**Interfaces:**
- Documents the exact Phase 1/2 terminal split, panel legend, output topics,
  safety boundary, and shutdown behavior.

- [ ] **Step 1: Update operator documentation**

Document:

```bash
export ROS_DOMAIN_ID=20
ros2 run track_robot_bringup semantic_search_ctl start phase1 --hardware auto
ros2 run track_robot_bringup semantic_search_ctl visualize phase1
```

and the corresponding measured-extrinsic Phase 2 flow. Explain that boxes and
scores are candidates, not ground truth, and closing RViz stops visualization
only.

- [ ] **Step 2: Run formatting and static checks**

Run:

```bash
git diff --check
python3 -m compileall \
  src/track_robot_semantic_search/track_robot_semantic_search \
  src/track_robot/track_robot_bringup/track_robot_bringup
```

Expected: no errors.

- [ ] **Step 3: Build all affected packages**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_interfaces \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_semantic_search_rviz_plugins \
  track_robot_bringup
```

Expected: all five packages finish successfully.

- [ ] **Step 4: Run affected test suites**

Run:

```bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOG_DIR=/tmp/track_robot_ros_logs \
  colcon test --packages-select \
  track_robot_interfaces \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_semantic_search_rviz_plugins \
  track_robot_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero errors and zero failures.

- [ ] **Step 5: Run no-hardware launch inspection**

Verify installed assets and plugin registration:

```bash
source install/setup.bash
ros2 pkg executables track_robot_semantic_search_rviz_plugins
ros2 pkg prefix --share track_robot_bringup
rviz2 --help
```

Run RViz only when a graphical display is available. Do not start sensors,
motion nodes, or a semantic stack for this verification.

- [ ] **Step 6: Check process cleanup**

After any runtime smoke check:

```bash
ps -eo pid,ppid,stat,cmd
```

Confirm no visualization-owned RViz or overlay process remains.

- [ ] **Step 7: Commit**

```bash
git add README.md docs src
git commit -m "docs: add semantic search visual testing workflow"
```

- [ ] **Step 8: Review branch state**

Run:

```bash
git status --short --branch
git log --oneline main..HEAD
```

Expected: clean feature branch with the design, implementation, tests, and
documentation commits only.
