# Phase 1 Multiscale Window Robustness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Phase 1 grid-only semantic crop path with a bounded six-window, single-batch CLIP strategy that handles target scale and grid-boundary changes while preserving ROS and Phase 2 contracts.

**Architecture:** Keep the first four model inputs and their 2x2 tensor exactly compatible with the current implementation, then append a globally letterboxed whole image and an independently letterboxed centered source crop. Score the extra windows in the same CLIP space, use the global score only as a fallback, and deterministically suppress strong local duplicates.

**Tech Stack:** ROS 2 Foxy, Python 3.8, NumPy, OpenCV, PyTorch 1.13 NVIDIA, OpenAI CLIP/OpenCLIP adapters, pytest, colcon.

## Global Constraints

- No Python, PyTorch, CUDA, TensorRT, ROS 2, or JetPack upgrade.
- Keep `/semantic_search/regions`, `/semantic_memory/observations`, `/semantic_memory/tasks`, and diagnostics message contracts unchanged.
- Keep one selected visual backbone and one bounded image batch per processed frame.
- Keep `threshold=0.25`, `target_rate_hz=5.0`, and `grid_size=2` in the checked comparison profile.
- Complete-path latency P95 must remain at most 150 ms before `multiscale_v1` is accepted as the official profile.
- The workspace has no conventional `.git` directory; commit steps are replaced by explicit file and verification checkpoints.

---

### Task 1: Define bounded multiscale window contracts

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/multiscale_windows.py`
- Create: `src/track_robot_semantic_search/test/test_multiscale_windows.py`

**Interfaces:**
- Produces: `WindowEncoding(kind: str, roi: Tuple[int, int, int, int], embedding: np.ndarray)`.
- Produces: `validate_window_strategy(strategy: str, grid_size: int, center_window_scale: float) -> str`.
- Produces: `center_window_roi(width: int, height: int, scale: float) -> Tuple[int, int, int, int]`.
- Produces: `letterbox_to_square(image_rgb: np.ndarray) -> np.ndarray`.

- [ ] **Step 1: Write failing layout and validation tests**

```python
def test_center_window_is_sixty_percent_and_centered():
    assert center_window_roi(1280, 720, 0.60) == (256, 144, 768, 432)


def test_center_window_is_deterministic_for_odd_dimensions():
    x, y, width, height = center_window_roi(1279, 719, 0.60)
    assert (x, y, width, height) == (256, 144, 767, 431)
    assert x + width <= 1279
    assert y + height <= 719


@pytest.mark.parametrize('strategy', ['grid_only', 'multiscale_v1'])
def test_supported_strategy_is_returned(strategy):
    assert validate_window_strategy(strategy, 2, 0.60) == strategy


def test_multiscale_requires_two_by_two_grid():
    with pytest.raises(ValueError, match='grid_size=2'):
        validate_window_strategy('multiscale_v1', 4, 0.60)


def test_letterbox_preserves_every_source_pixel():
    source = np.arange(2 * 4 * 3, dtype=np.uint8).reshape(2, 4, 3)
    output = letterbox_to_square(source)
    assert output.shape == (4, 4, 3)
    np.testing.assert_array_equal(output[1:3], source)
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
python3 -m pytest -q src/track_robot_semantic_search/test/test_multiscale_windows.py
```

Expected: collection/import failure because `multiscale_windows` does not exist.

- [ ] **Step 3: Implement the pure contracts**

```python
from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class WindowEncoding:
    kind: str
    roi: Tuple[int, int, int, int]
    embedding: np.ndarray


def validate_window_strategy(strategy, grid_size, center_window_scale):
    selected = str(strategy).strip().lower()
    if selected not in ('grid_only', 'multiscale_v1'):
        raise ValueError('window_strategy must be grid_only or multiscale_v1')
    if not isinstance(grid_size, int) or isinstance(grid_size, bool) or grid_size <= 0:
        raise ValueError('grid_size must be a positive integer')
    scale = float(center_window_scale)
    if not math.isfinite(scale) or not 0.25 <= scale <= 1.0:
        raise ValueError('center_window_scale must be finite and in [0.25, 1.0]')
    if selected == 'multiscale_v1' and grid_size != 2:
        raise ValueError('multiscale_v1 requires grid_size=2')
    return selected


def center_window_roi(width, height, scale):
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    value = float(scale)
    if not math.isfinite(value) or not 0.25 <= value <= 1.0:
        raise ValueError('center window scale must be finite and in [0.25, 1.0]')
    crop_width = max(1, int(round(width * value)))
    crop_height = max(1, int(round(height * value)))
    return ((width - crop_width) // 2, (height - crop_height) // 2,
            crop_width, crop_height)


def letterbox_to_square(image_rgb):
    source = np.asarray(image_rgb)
    if source.ndim != 3 or source.shape[2] != 3:
        raise ValueError('image must be shaped H,W,3')
    height, width = source.shape[:2]
    side = max(height, width)
    output = np.zeros((side, side, 3), dtype=source.dtype)
    left = (side - width) // 2
    top = (side - height) // 2
    output[top:top + height, left:left + width] = source
    return output
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the command from Step 2. Expected: all tests pass.

- [ ] **Step 5: Record checkpoint**

Record that the two Task 1 files exist and the focused test output is green; no Git commit is possible in this workspace.

---

### Task 2: Extend both CLIP adapters with one six-crop batch

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/model_adapters.py`
- Modify: `src/track_robot_semantic_search/test/test_model_adapters.py`

**Interfaces:**
- Consumes: Task 1 `WindowEncoding`, `center_window_roi`, `letterbox_to_square`, and validation.
- Produces: `ImageGridEncoding.extra_windows`, an immutable tuple of
  `WindowEncoding` values.
- Extends: `OpenAIClipAdapter`, `OpenClipAdapter`, and `create_aligned_encoder` with `window_strategy` and `center_window_scale`.

- [ ] **Step 1: Add failing adapter tests**

Add fake-model assertions that `grid_only` still calls `encode_image()` with a batch of four for `grid_size=2`, while `multiscale_v1` calls it once with a batch of six and returns exactly two extras in order `global`, `center`. Assert that the first four returned vectors still reshape to `(2, 2, C)`.

```python
assert fake_model.encode_image_call_count == 1
assert fake_model.last_batch.shape[0] == 6
assert [item.kind for item in result.extra_windows] == ['global', 'center']
assert result.extra_windows[0].roi == (0, 0, 1280, 720)
assert result.extra_windows[1].roi == (256, 144, 768, 432)
```

- [ ] **Step 2: Run adapter tests and verify RED**

```bash
python3 -m pytest -q src/track_robot_semantic_search/test/test_model_adapters.py
```

Expected: constructor/signature or missing `extra_windows` failure.

- [ ] **Step 3: Implement batch construction and strict result mapping**

Add `extra_windows=()` to `ImageGridEncoding`. Preserve the current global square and row-major grid crop code. For `multiscale_v1`, append `preprocess(Image.fromarray(canvas))`, then crop the center ROI from the unpadded RGB source, letterbox it, and append its preprocess result. Call `model.encode_image(batch)` once. Require exactly `grid_size ** 2 + 2` vectors, reshape only the first four, and construct normalized `WindowEncoding` records for the final vectors.

Pass the two new settings through both adapters and `create_aligned_encoder`; validation happens before runtime imports or model inference.

- [ ] **Step 4: Run adapter and Phase 1 core tests**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_model_adapters.py \
  src/track_robot_semantic_search/test/test_perception_core.py
```

Expected: all tests pass and legacy grid-only assertions remain unchanged.

- [ ] **Step 5: Record checkpoint**

Record the modified adapter/test files and exact passing counts.

---

### Task 3: Score extra windows and suppress deterministic duplicates

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/region_scoring.py`
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/perception_core.py`
- Modify: `src/track_robot_semantic_search/test/test_region_scoring.py`
- Modify: `src/track_robot_semantic_search/test/test_perception_core.py`

**Interfaces:**
- Consumes: `ImageGridEncoding.extra_windows`.
- Produces: `score_multiscale_regions(image_embeddings, text_embedding,
  valid_patch_mask, geometry, extra_windows, threshold, threshold_mode,
  quantile, min_area, max_regions, duplicate_iou_threshold,
  duplicate_containment_threshold) -> List[RegionCandidate]`.
- Preserves: existing `score_regions()` behavior when no extra windows are supplied.

- [ ] **Step 1: Add failing behavior tests**

Use two-dimensional unit vectors so cosine scores are exact and avoid model
mocks. Cover these concrete inputs and outputs:

- grid and center vectors `[0, 1]`, global vector `[1, 0]`, text `[1, 0]`,
  threshold `0.5` -> one whole-image fallback region;
- top-left grid vector `[1, 0]` plus passing global vector -> only the top-left
  local region, never the whole-image region;
- all grid vectors `[0, 1]`, center vector `[1, 0]`, text `[1, 0]` -> exactly
  the configured center ROI;
- two manually constructed regions with IoU `>=0.50` -> keep only the region
  with the higher mean score;
- two manually constructed regions with containment `>=0.80` -> keep only the
  region with the higher mean score;
- exact score ties -> keep the region that sorts first by `y`, then `x`, then
  height and width;
- grid scores `[0.1, 0.2, 0.3, 0.4]` and extra scores `[0.8, 0.9]` at quantile
  `0.5` -> derive the cutoff from all six values;
- three non-overlapping passing locals with `max_regions=2` -> return the first
  two under deterministic score ordering.

- [ ] **Step 2: Run scoring tests and verify RED**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_region_scoring.py \
  src/track_robot_semantic_search/test/test_perception_core.py
```

Expected: missing multiscale scoring API or behavior assertion failures.

- [ ] **Step 3: Implement shared cutoff and candidate fusion**

Refactor cosine-map calculation into a private helper used by both public score functions. Calculate one absolute or quantile cutoff over valid grid scores and every finite extra score. Convert a passing center encoding to a local `RegionCandidate`; hold a passing global encoding separately. Sort local candidates by the existing deterministic key and suppress a candidate when either overlap criterion is met against a kept candidate. Return locals when non-empty, otherwise return the global fallback, always truncated to `max_regions`.

Extend `PassivePerceptionCore.__init__` with finite `[0, 1]` duplicate thresholds and call the multiscale scorer. The proposal pooling path must continue to consume only the legacy grid tensor.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all tests pass.

- [ ] **Step 5: Record checkpoint**

Record the scoring/core changes and exact focused test counts.

---

### Task 4: Wire bounded ROS parameters and preserve launch contracts

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/perception_node.py`
- Modify: `src/track_robot_semantic_search/config/semantic_search_phase1.yaml`
- Modify: `src/track_robot_semantic_search/test/test_perception_node_contract.py`
- Modify: `src/track_robot_semantic_search/test/test_phase1_launch_contract.py`

**Interfaces:**
- Consumes: new adapter/core constructor parameters.
- Produces checked defaults: `multiscale_v1`, `0.60`, `0.50`, and `0.80`.

- [ ] **Step 1: Add failing ROS contract tests**

```python
assert parameters['window_strategy'] == 'multiscale_v1'
assert parameters['center_window_scale'] == 0.60
assert parameters['duplicate_iou_threshold'] == 0.50
assert parameters['duplicate_containment_threshold'] == 0.80
```

The node contract must assert every setting is declared and passed to the correct adapter or core constructor.

- [ ] **Step 2: Run contract tests and verify RED**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_perception_node_contract.py \
  src/track_robot_semantic_search/test/test_phase1_launch_contract.py
```

- [ ] **Step 3: Wire parameters and checked profile**

Declare the four parameters before model construction, pass strategy/scale to `create_aligned_encoder`, pass duplicate thresholds to `PassivePerceptionCore`, and add the exact values to `semantic_search_phase1.yaml`. Do not add new ROS topics or messages.

- [ ] **Step 4: Run contract and package tests**

```bash
python3 -m pytest -q src/track_robot_semantic_search/test
```

Expected: the full Python package suite passes.

- [ ] **Step 5: Record checkpoint**

Record the four modified files and package test count.

---

### Task 5: Add comparable profiling controls and user documentation

**Files:**
- Modify: `src/track_robot_semantic_search/scripts/run_phase1_replay_probe.py`
- Modify: `src/track_robot_semantic_search/README.md`
- Modify: `src/track_robot_semantic_search/test/test_phase1_baselines.py`

**Interfaces:**
- Adds CLI options: `--window-strategy {grid_only,multiscale_v1}` and `--center-window-scale 0.60`.
- Adds report provenance fields with the same names.

- [ ] **Step 1: Add failing CLI/report contract tests**

Assert the CLIP replay probe passes the two settings to
`create_aligned_encoder`, includes them in JSON output, and the README explains
scale/boundary behavior, the six-window contract, and global fallback
semantics. `benchmark_phase1_model.py` remains unchanged because it profiles
the separate DINO runtime and does not construct a CLIP encoder.

- [ ] **Step 2: Run baseline tests and verify RED**

```bash
python3 -m pytest -q src/track_robot_semantic_search/test/test_phase1_baselines.py
```

- [ ] **Step 3: Implement CLI plumbing and documentation**

Add strict argparse choices, finite scale validation through the adapter contract, report fields, and comparable example commands. Document that `4x4` is not a substitute for multiscale context and that a whole-frame region is a fallback presence result rather than an object-accurate box.

- [ ] **Step 4: Run baseline and full package tests**

```bash
python3 -m pytest -q src/track_robot_semantic_search/test
```

- [ ] **Step 5: Record checkpoint**

Record script/docs/test changes and passing counts.

---

### Task 6: Build, load the real model, and gate measured performance

**Files:**
- Generated only: `build/`, `install/`, `log/`, and a new JSON report under `artifacts/semantic_search/reports/` when the comparable run is available.

**Interfaces:**
- Consumes all previous tasks.
- Produces build/test/model-load evidence and measured `grid_only` versus `multiscale_v1` results.

- [ ] **Step 1: Run complete package verification**

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select track_robot_semantic_search
colcon test --packages-select track_robot_semantic_search --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build exit 0 and zero test failures/errors.

- [ ] **Step 2: Load and warm the installed checkpoint**

Run a bounded Phase 1 launch with the installed runtime and checkpoint, first with `device:=cpu` for deterministic availability and then `device:=auto` only when the Jetson GPU is free. Expected log:

```text
Passive semantic perception ready: openai_clip:ViT-B/32 on cpu
```

or `on cuda`. Stop every node started for this verification.

- [ ] **Step 3: Run controlled strategy comparison**

Run the same frame manifest, model checkpoint, query set, threshold, and device once with `grid_only` and once with `multiscale_v1`. Save immutable JSON reports containing strategy, center scale, sample identities, latency samples, empty share, candidate counts, and scores.

- [ ] **Step 4: Apply gates without silently relaxing them**

Accept `multiscale_v1` as the checked default only when controlled evidence shows improved scale/boundary coverage, output rate at least 5 Hz, and complete-path latency P95 at most 150 ms. If physical annotated frames are unavailable, report semantic improvement as unavailable and hand off the exact live test; do not claim the accuracy gate from synthetic unit tests.

- [ ] **Step 5: Confirm cleanup**

```bash
ps -eo pid,ppid,stat,cmd
```

Confirm no model, replay, ROS node, or profiling process started by this plan remains running.
