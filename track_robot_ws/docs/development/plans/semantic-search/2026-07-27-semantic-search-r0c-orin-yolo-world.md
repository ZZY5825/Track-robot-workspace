# Semantic Search R0C Orin YOLO-World Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and physically smoke-test an isolated, zero-training
YOLOv8s-World-v2 candidate on the Jetson AGX Orin that emits frozen R0A
prediction artifacts.

**Architecture:** A pure Orin preflight and lazy YOLO-World adapter live beside
the existing grounding tools. The adapter prepends explicit isolated runtime
roots, reuses the local CLIP checkpoint for query encoding, and implements the
existing backend protocol. A dedicated CLI provides probe, smoke, and complete
R0A prediction commands without importing ROS.

**Tech Stack:** Python 3.8, NVIDIA PyTorch 1.13, CUDA 11.4,
`ultralytics==8.2.103`, OpenAI CLIP, Pillow/OpenCV, pytest, R0A JSON contracts.

## Global Constraints

- Do not upgrade system/user Ultralytics `8.0.239`.
- Do not upgrade NVIDIA Torch, torchvision, JetPack, CUDA, TensorRT, ROS, or
  system Python.
- Install isolated Python files only under `models/r0c_runtime/python`.
- Use only explicit local YOLO-World and CLIP checkpoint paths at inference.
- Do not allow model or Python dependency auto-download during probe,
  smoke, or predict.
- Do not train or fine-tune any model.
- Use PyTorch FP16 at input size 640 for the first candidate.
- Preserve the frozen R0A dataset and prediction schemas.
- Do not start ROS nodes, sensor drivers, or robot motion.
- Preserve unrelated dirty-worktree changes and do not commit or publish.

---

### Task 1: Add Orin runtime and checkpoint preflight

**Files:**
- Create: `tools/semantic_search_grounding/orin_environment.py`
- Create: `tools/semantic_search_grounding/test/test_orin_environment.py`

**Interfaces:**
- Produces: `OrinEnvironmentProbe(report: Mapping[str, object])`.
- Produces:
  `probe_orin_environment(runtime_path, clip_runtime_path,
  world_checkpoint, clip_checkpoint, platform_probe=None,
  dependency_probe=None) -> OrinEnvironmentProbe`.

- [x] **Step 1: Write failing tests**

Test wrong architecture, wrong L4T, missing isolated runtime, global
Ultralytics shadowing, missing `YOLOWorld`, CUDA unavailable, checkpoint
symlinks, valid dual-checkpoint SHA-256, and absence of credential data.

- [x] **Step 2: Verify RED**

```bash
PYTHONPATH=src/track_robot_semantic_search:. \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tools/semantic_search_grounding/test/test_orin_environment.py
```

Expected: collection fails because `orin_environment.py` does not exist.

- [x] **Step 3: Implement the preflight**

Require `aarch64`, L4T release 35, isolated Ultralytics `8.2.103`,
`YOLOWorld`, NVIDIA Torch 1.13 with CUDA device containing `Orin`, importable
local CLIP, and two regular non-symlink checkpoints. Record separate SHA-256
values and a composite SHA-256 calculated from
`world_sha256 + ":" + clip_sha256`.

- [x] **Step 4: Verify GREEN**

Run the Step 2 command. Expected: all preflight tests pass.

### Task 2: Generalize release evidence for the Orin candidate

**Files:**
- Modify: `tools/semantic_search_grounding/teacher_runner.py`
- Modify: `tools/semantic_search_grounding/test/test_teacher_runner.py`

**Interfaces:**
- Extends:
  `build_prediction_document(..., platform_compatible=False)`.
- Preserves the R0B default `platform_compatible=false`.

- [x] **Step 1: Write a failing R0C evidence test**

Call the runner with `platform_compatible=True` and assert the emitted R0A
document records true while the existing default test remains false.

- [x] **Step 2: Verify RED**

Run the focused runner test. Expected: unexpected keyword argument failure.

- [x] **Step 3: Add the bounded boolean argument**

Reject non-boolean values and pass the exact flag to `release_evidence`.

- [x] **Step 4: Verify GREEN**

Run all runner tests. Expected: old R0B and new R0C evidence tests pass.

### Task 3: Add the isolated YOLO-World backend

**Files:**
- Create: `tools/semantic_search_grounding/ultralytics_yolo_world.py`
- Create:
  `tools/semantic_search_grounding/test/test_ultralytics_yolo_world.py`

**Interfaces:**
- Produces:
  `normalize_yolo_world_result(result, query, width, height,
  max_detections) -> Tuple[TeacherDetection, ...]`.
- Produces:
  `UltralyticsYoloWorld.from_local_model(runtime_path, clip_runtime_path,
  world_checkpoint, clip_checkpoint, confidence_floor=0.05,
  iou_threshold=0.70, input_size=640, max_detections=256,
  device=0, half=True, dependencies=None)`.
- Implements `predict`, `synchronize`, and
  `incremental_cuda_reserved_mib`.

- [x] **Step 1: Write failing adapter tests**

Use fake dependencies to prove local-only checkpoint loading, exact
single-query `set_classes`, no repeated text encoding for the same query,
local CLIP checkpoint substitution, exact FP16 prediction arguments, result
clamping/sorting/truncation, invalid-result rejection, synchronization, and
peak CUDA memory.

- [x] **Step 2: Verify RED**

Run the adapter test. Expected: missing-module collection failure.

- [x] **Step 3: Implement lazy isolated loading**

Prepend and validate the two runtime roots before importing `ultralytics`,
`clip`, and `torch`. Temporarily replace `clip.load` only while calling
`set_classes`, and force that replacement to load the explicit local CLIP
checkpoint. Restore `clip.load` in `finally`.

- [x] **Step 4: Implement result normalization**

Accept exactly one image result. Convert tensor-like `xyxy`, `conf`, and `cls`
to lists; map every class ID through `result.names`; require the resulting
label to equal the active query; reject non-finite boxes or scores; clamp to
image bounds; discard collapsed boxes; sort deterministically; retain at most
256.

- [x] **Step 5: Verify GREEN**

Run all adapter tests without loading real CUDA or model packages.

### Task 4: Add R0C probe, smoke, and prediction CLI

**Files:**
- Create: `tools/semantic_search_grounding/orin_cli.py`
- Create: `tools/semantic_search_grounding/test/test_orin_cli.py`

**Interfaces:**
- Produces:
  `python3 -m tools.semantic_search_grounding.orin_cli probe ...`.
- Produces:
  `python3 -m tools.semantic_search_grounding.orin_cli smoke ...`.
- Produces:
  `python3 -m tools.semantic_search_grounding.orin_cli predict ...`.

- [x] **Step 1: Write failing CLI tests**

Prove no eager heavy imports; unavailable probe exits 2; predict preflight
failure avoids backend loading; smoke emits bounded JSON; backend failures
preserve existing outputs; successful predict emits an R0A-loadable artifact
with `platform_compatible=true`; licence approval remains explicit.

- [x] **Step 2: Verify RED**

Run the CLI test. Expected: missing-module collection failure.

- [x] **Step 3: Implement shared atomic output**

Reuse the R0B atomic JSON writer without changing R0B command behavior.
All CLI errors are a single line of at most 507 characters including prefix
and newline.

- [x] **Step 4: Implement commands**

Probe writes the environment report. Smoke loads one image and query, measures
the complete synchronized path, and writes model identity, dimensions,
latency, peak memory, and detections. Predict loads the strict R0A dataset,
uses the shared runner with `platform_compatible=True`, validates the complete
case set with the frozen prediction loader, and atomically replaces output.

- [x] **Step 5: Verify GREEN**

Run all CLI tests. Expected: all pass without real model dependencies.

### Task 5: Install, download, physically verify, and document

**Files:**
- Create: `docs/guides/semantic-search/r0c-orin-yolo-world.md`
- Modify: `docs/README.md`
- Modify:
  `docs/architecture/semantic-search/2026-07-25-semantic-search-phase1r-3r-visual-grounding-recovery-design.md`
- Modify:
  `docs/architecture/semantic-search/2026-07-27-semantic-search-r0b-zero-shot-teacher-design.md`
- Create:
  `artifacts/semantic_search/reports/r0c_orin_inventory_2026-07-27.json`
- Create:
  `artifacts/semantic_search/reports/r0c_orin_smoke_2026-07-27.json`

**Interfaces:**
- Installs ignored runtime files under `models/r0c_runtime/python`.
- Downloads ignored checkpoint `models/r0c/yolov8s-worldv2.pt`.
- Uses existing `models/phase1/ViT-B-32.pt`.

- [x] **Step 1: Install without dependencies**

Use pip `--target models/r0c_runtime/python --no-deps` for exact
`ultralytics==8.2.103` and a Python-3.8-compatible `ultralytics-thop`.
Verify global `ultralytics==8.0.239` remains unchanged.

- [x] **Step 2: Download the explicit model**

Download `yolov8s-worldv2.pt` from the official Ultralytics assets release to
`models/r0c/`, compute SHA-256, and never invoke model construction with a
remote identifier.

- [x] **Step 3: Run the actual Orin probe**

Run outside the restricted device sandbox when required. Expected:
`host_eligible=true`, `runtime_ready=true`, CUDA device `Orin`, isolated
Ultralytics `8.2.103`, and both checkpoint checksums present.

- [x] **Step 4: Run a physical smoke inference**

Use `/tmp/phase1_blue_target_probe.png` when present, otherwise the recorded
ZED image under `dataset/zed2i/rgb/`, with query `blue cylindrical container`.
Expected: exit 0 and a structurally valid smoke JSON. Detection count may be
zero; quality is not inferred from the smoke.

- [x] **Step 5: Run regression**

Run all grounding-tool tests, the 220 R0A tests, the full 710 semantic-search
package tests with ROS sourced, Python compilation, CLI help, and a read-only
`ROS_DOMAIN_ID=20 ros2 node list`.

- [x] **Step 6: Document and record**

Document setup, probe, smoke, prediction, evaluation, rollback, and the
distinction between execution readiness and quality. Mark R0B-2 cancelled,
R0B-1 optional, and R0C active. Record exact command evidence and leave all ROS
nodes stopped.

## Completion record

- Isolated runtime: Ultralytics `8.2.103` and ultralytics-thop `2.0.18`;
  global Ultralytics remains `8.0.239`.
- World checkpoint SHA-256:
  `9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792`.
- CLIP checkpoint SHA-256:
  `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af`.
- Actual Orin probe: `host_eligible=true`, `runtime_ready=true`.
- Physical FP16 smoke: exit 0, 460 MiB peak incremental CUDA reserve.
- Same-process latency diagnostic: first query 6867.698 ms; cached second
  frame 52.223 ms.
- Both `blue cylindrical container` and `blue bottle` returned zero
  detections at the fixed 0.05 proposal floor. Execution is available;
  semantic quality is not yet available.
- Grounding tools: 68 passed.
- Frozen R0A contracts: 220 passed.
- Full semantic-search package: 710 passed.
- Python compilation and CLI help: passed.
- `ROS_DOMAIN_ID=20 ros2 node list`: empty after all offline checks.
- Formal R0A prediction/evaluation remains blocked by the absence of a real
  human-reviewed grounding dataset.
