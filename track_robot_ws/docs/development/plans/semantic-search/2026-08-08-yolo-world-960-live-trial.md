# YOLO-World 960 Live Trial Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make 960 the shared production YOLO-World input size for the standard Phase 1–5A launch chain.

**Architecture:** Keep the existing single source of truth in `semantic_search_yolo_world.yaml`. Add one configuration contract assertion, then change only `input_size`; every current launch consumer already loads this YAML.

**Tech Stack:** ROS 2 Foxy, ament Python, YAML, pytest, colcon.

## Global Constraints

- Change only YOLO-World `input_size` from 640 to 960.
- Keep DINOv3 input at 224.
- Preserve confidence floor, IoU, max detections, checkpoints, topics and public interfaces.
- Preserve Phase 2–5 and Nav2 behavior.
- Keep the offline benchmark 640 baseline unchanged.

---

### Task 1: Change the Shared Production Input Size

**Files:**
- Modify: `src/track_robot_semantic_search/test/test_yolo_world_node_contract.py`
- Modify: `src/track_robot_semantic_search/config/semantic_search_yolo_world.yaml`

**Interfaces:**
- Consumes: the existing YAML key `semantic_search_yolo_world_perception.ros__parameters.input_size`.
- Produces: a shared integer default of `960` for the existing YOLO-World node; no new interface.

- [ ] **Step 1: Write the failing configuration test**

Add YAML loading and this assertion to `test_yolo_world_node_contract.py`:

```python
import yaml

CONFIG = PACKAGE_ROOT / 'config' / 'semantic_search_yolo_world.yaml'


def test_yolo_world_production_config_uses_measured_960_input():
    config = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    parameters = config[
        'semantic_search_yolo_world_perception']['ros__parameters']

    assert parameters['input_size'] == 960
    assert parameters['dino_input_size'] == 224
```

- [ ] **Step 2: Verify the test fails for the current 640 default**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_yolo_world_node_contract.py \
  -k measured_960
```

Expected: one failure showing `640 == 960` is false.

- [ ] **Step 3: Change the single production setting**

In `semantic_search_yolo_world.yaml`, change only:

```yaml
    input_size: 960
```

- [ ] **Step 4: Run targeted contract tests**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_yolo_world_node_contract.py \
  src/track_robot_semantic_search/test/test_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
```

Expected: all selected tests pass.

- [ ] **Step 5: Build and run full relevant regression**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_semantic_search track_robot_bringup
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot_semantic_search/test \
  src/track_robot/track_robot_bringup/test
```

Expected: build succeeds; all semantic-search and bringup tests pass with no regression.

- [ ] **Step 6: Commit the independent behavior change**

```bash
git add \
  src/track_robot_semantic_search/test/test_yolo_world_node_contract.py \
  src/track_robot_semantic_search/config/semantic_search_yolo_world.yaml
git commit -m "perf(semantic-search): use YOLO 960 production input"
```

