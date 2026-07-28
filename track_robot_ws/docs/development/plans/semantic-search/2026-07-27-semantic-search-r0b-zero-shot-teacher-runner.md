# Semantic Search R0B Zero-Shot Teacher Runner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a desktop-only, zero-training Grounding DINO teacher runner that consumes the frozen R0A dataset contract and emits a validated R0A prediction artifact without installing model dependencies on the Jetson.

**Architecture:** Keep the model worker under `tools/` and outside ROS runtime packages. A pure runner core depends only on R0A contracts, while a lazy Hugging Face adapter owns Torch/Transformers/Pillow imports and local checkpoint inference. A strict CLI performs desktop preflight, validates the complete artifact, and publishes it atomically.

**Tech Stack:** Python 3.8-compatible pure orchestration and tests on Jetson; separate desktop Python 3.10/3.11 virtual environment; Hugging Face Transformers Grounding DINO; PyTorch CUDA; Pillow; JSON; pytest.

**Implementation status:** R0B-1 completed on 2026-07-27. Actual desktop
inference remains the R0B-2 checkpoint.

## Global Constraints

- Do not install, download, import, or execute Grounding DINO on the current aarch64 Jetson.
- Do not upgrade JetPack 5.0.2, Ubuntu 20.04, ROS 2 Foxy, or system Python 3.8.
- Do not train or fine-tune any model in R0B-1.
- Formal prediction requires an x86_64 NVIDIA desktop with CUDA available to PyTorch.
- Model loading is local-only and never triggers an implicit network download.
- Consume the existing R0A dataset loader and emit the existing R0A prediction schema unchanged.
- Process every dataset case and publish no partial case set.
- R0B desktop teacher evidence sets `platform_compatible` to false.
- No ROS node, topic, service, action, hardware driver, or motion process is started.
- Preserve all unrelated dirty-worktree changes; do not commit or publish this slice.

---

## File structure

- `tools/semantic_search_grounding/environment.py`: desktop and runtime preflight.
- `tools/semantic_search_grounding/contracts.py`: bounded backend-facing detection and metadata values.
- `tools/semantic_search_grounding/teacher_runner.py`: model-independent dataset-to-prediction orchestration.
- `tools/semantic_search_grounding/huggingface_grounding_dino.py`: lazy local Hugging Face adapter.
- `tools/semantic_search_grounding/cli.py`: `probe` and `predict` commands.
- `tools/semantic_search_grounding/README.md`: desktop setup and execution boundary.
- `tools/semantic_search_grounding/test/`: focused pure tests requiring no model dependency.
- `docs/guides/semantic-search/r0b-zero-shot-teacher.md`: operator guide and next desktop checkpoint.

### Task 1: Add deterministic desktop preflight

**Files:**
- Create: `tools/__init__.py`
- Create: `tools/semantic_search_grounding/__init__.py`
- Create: `tools/semantic_search_grounding/environment.py`
- Create: `tools/semantic_search_grounding/test/test_environment.py`

**Interfaces:**
- Produces: `EnvironmentProbe(report: Mapping[str, object])`.
- Produces: `probe_environment(model_dir=None, checkpoint_file=None, command_runner=None, module_probe=None) -> EnvironmentProbe`.
- Produces: `EnvironmentProbe.host_eligible` and `EnvironmentProbe.runtime_ready`.

- [x] **Step 1: Write failing preflight tests**

Prove that an aarch64 Jetson is not host eligible, an x86_64 host without
`nvidia-smi` is not eligible, an RTX host without runtime modules is eligible
but not ready, and a complete desktop runtime is ready. Also prove that reports
contain no environment variables or credentials.

- [x] **Step 2: Run tests and verify RED**

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m pytest -q \
  tools/semantic_search_grounding/test/test_environment.py
```

Expected: import failure because `environment.py` does not exist.

- [x] **Step 3: Implement the pure probe**

Use injected platform, command, and module probes in tests. The production
probe may call `nvidia-smi --query-gpu=name,memory.total,driver_version
--format=csv,noheader,nounits`, but it must bound stdout and never invoke a
shell. Report architecture, OS, Python, GPU rows,
Torch/Transformers/Pillow/OpenCV availability and versions, CUDA availability,
model/checkpoint status, and explicit reasons.

- [x] **Step 4: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all preflight tests pass.

---

### Task 2: Build the model-independent teacher runner

**Files:**
- Create: `tools/semantic_search_grounding/contracts.py`
- Create: `tools/semantic_search_grounding/teacher_runner.py`
- Create: `tools/semantic_search_grounding/test/test_teacher_runner.py`

**Interfaces:**
- Produces: frozen `TeacherDetection(x1, y1, x2, y2, score, label)`.
- Produces: frozen `TeacherIdentity(candidate_id, implementation, code_revision, checkpoint_id, checkpoint_sha256, licence, platform, input_size)`.
- Consumes a backend with `predict(image_path, normalized_query)`, `synchronize()`, and `incremental_cuda_reserved_mib()`.
- Produces: `build_prediction_document(dataset, backend, identity, licence_approved, clock_ns=None) -> Mapping[str, object]`.

- [x] **Step 1: Write failing orchestration tests**

Prove exact case-set output, normalized-query forwarding, deterministic
score/geometry/label ordering, 256-result truncation, non-negative complete
path time, strict metadata, desktop teacher release evidence, and rejection of
non-finite/malformed detections.

- [x] **Step 2: Run tests and verify RED**

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m pytest -q \
  tools/semantic_search_grounding/test/test_teacher_runner.py
```

Expected: import failure because the runner does not exist.

- [x] **Step 3: Implement the minimal runner**

Call `backend.synchronize()` before starting the clock and after prediction.
Convert XYXY to XYWH only after validation. Reject duplicate or missing case
outputs by construction and emit all R0A prediction fields. Do not import a
model library.

- [x] **Step 4: Validate with the frozen R0A loader**

The test writes the returned mapping to a temporary JSON file and calls
`load_grounding_predictions()`. Expected: the emitted artifact loads and every
case ID is present.

- [x] **Step 5: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all runner tests pass.

---

### Task 3: Add the local Hugging Face Grounding DINO adapter

**Files:**
- Create: `tools/semantic_search_grounding/huggingface_grounding_dino.py`
- Create: `tools/semantic_search_grounding/test/test_huggingface_grounding_dino.py`

**Interfaces:**
- Produces: `normalize_detection_result(result, width, height, max_detections) -> Tuple[TeacherDetection, ...]`.
- Produces: `HuggingFaceGroundingDino.from_local_model(...)`.
- Implements the Task 2 backend methods.

- [x] **Step 1: Write failing adapter tests**

Use fake Torch, processor, model, and image dependencies. Prove
`local_files_only=True`, exact one-query forwarding, target size forwarding,
threshold forwarding, original-image coordinate conversion, bounds clamping,
deterministic truncation, CUDA synchronization, and memory accounting. Prove
that missing real dependencies produce one bounded `RuntimeError` rather than
an import-time failure.

- [x] **Step 2: Run tests and verify RED**

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m pytest -q \
  tools/semantic_search_grounding/test/test_huggingface_grounding_dino.py
```

Expected: import failure because the adapter does not exist.

- [x] **Step 3: Implement lazy dependency loading**

Import Torch, Pillow, and Transformers only inside
`load_huggingface_dependencies()`. Load `AutoProcessor` and
`AutoModelForZeroShotObjectDetection` from a local directory with
`local_files_only=True`. Use `post_process_grounded_object_detection()` with
the configured score and text floors.

- [x] **Step 4: Implement strict result normalization**

Accept only equal-length boxes, scores, and labels. Require finite numeric
values and a non-empty label. Clamp XYXY coordinates to image dimensions,
discard collapsed boxes, sort deterministically, and truncate to the declared
maximum.

- [x] **Step 5: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all adapter tests pass without Torch,
Transformers, or Pillow installed.

---

### Task 4: Add atomic CLI commands

**Files:**
- Create: `tools/semantic_search_grounding/cli.py`
- Create: `tools/semantic_search_grounding/test/test_cli.py`
- Create: `tools/semantic_search_grounding/README.md`

**Interfaces:**
- Produces: `python3 -m tools.semantic_search_grounding.cli probe [--model-dir ... --checkpoint-file ... --output ...]`.
- Produces: `python3 -m tools.semantic_search_grounding.cli predict --dataset ... --model-dir ... --checkpoint-file ... --model-revision ... --candidate-id ... --output ...`.

- [x] **Step 1: Write failing CLI tests**

Prove that `probe` writes bounded JSON and exits 2 when not ready; `predict`
refuses the Jetson before importing model dependencies; invalid inputs produce
one bounded stderr line and no output; backend failure preserves an existing
final output; and a fake complete desktop run writes an R0A-loadable document
without a leftover temporary file.

- [x] **Step 2: Run tests and verify RED**

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m pytest -q \
  tools/semantic_search_grounding/test/test_cli.py
```

Expected: import failure because the CLI does not exist.

- [x] **Step 3: Implement `probe`**

Print the report when no output path is supplied. Otherwise write it
atomically. Return 0 only for `runtime_ready`; return 2 for a valid but
unavailable report.

- [x] **Step 4: Implement `predict`**

Require a ready desktop preflight, verified lowercase checkpoint SHA-256,
proposal thresholds in `[0, 1]`, maximum detections in `[1, 256]`, explicit
model revision, licence, and candidate ID. Build the adapter and runner output,
write to a unique temporary file, validate with
`load_grounding_predictions()`, and use `os.replace()` only after success.

- [x] **Step 5: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all CLI tests pass.

---

### Task 5: Document and verify R0B-1

**Files:**
- Create: `docs/guides/semantic-search/r0b-zero-shot-teacher.md`
- Modify: `docs/README.md`
- Modify: `tools/semantic_search_grounding/README.md`
- Create: `artifacts/semantic_search/reports/r0b_current_host_inventory_2026-07-27.json`

- [x] **Step 1: Document the desktop transfer and preflight**

Explain that the current host is the Jetson and must not receive the desktop
runtime. Document the separate venv boundary, explicit model snapshot
download as a later operator action, local-only runtime, zero-training policy,
probe command, prediction command, and R0A evaluation command.

- [x] **Step 2: Run all R0B-1 tests**

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m pytest -q \
  tools/semantic_search_grounding/test
```

Expected: all tests pass without model dependencies.

- [x] **Step 3: Run R0A and package regression**

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_query.py \
  src/track_robot_semantic_search/test/test_grounding_dataset.py \
  src/track_robot_semantic_search/test/test_grounding_predictions.py \
  src/track_robot_semantic_search/test/test_grounding_evaluation.py \
  src/track_robot_semantic_search/test/test_grounding_selection.py \
  src/track_robot_semantic_search/test/test_grounding_cli.py
```

Expected: all R0A tests pass.

- [x] **Step 4: Run the actual current-host probe**

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m \
  tools.semantic_search_grounding.cli probe \
  --output artifacts/semantic_search/reports/r0b_current_host_inventory_2026-07-27.json
```

Expected: exit 2 with a valid report identifying aarch64/Jetson as not
desktop-eligible. No model dependency is imported.

- [x] **Step 5: Verify no ROS process was introduced**

Use `ROS_DOMAIN_ID=20 ros2 node list` only as a read-only check. R0B-1 starts no
node and must not stop unrelated user processes.

- [x] **Step 6: Record the checkpoint**

Record files, RED/GREEN evidence, regression results, current-host probe, model
download status, and the exact first desktop command. R0B-2 remains blocked
only on access to the RTX desktop environment.

## Completion record

- R0B-1 focused tests: 35 passed.
- Frozen R0A grounding-contract regression: 220 passed.
- Full `track_robot_semantic_search` package regression with Foxy and the
  installed workspace sourced: 710 passed.
- Python compilation: passed for all five new implementation modules.
- Current-host probe: valid report, expected exit 2, `aarch64`,
  `host_eligible=false`, `runtime_ready=false`.
- `ROS_DOMAIN_ID=20 ros2 node list`: empty after the offline checks.
- Model download and actual inference: not run on the Jetson.
- First desktop command: run `probe` exactly as documented in
  `docs/guides/semantic-search/r0b-zero-shot-teacher.md`.
