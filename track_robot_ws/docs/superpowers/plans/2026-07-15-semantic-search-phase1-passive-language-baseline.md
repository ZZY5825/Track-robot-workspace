# Semantic Search Phase 1 Passive Language Baseline Implementation Plan

> **For agentic workers:** Execute inline, task-by-task, with a review gate after each independently testable task. Do not dispatch subagents unless the user explicitly changes the current no-delegation preference.

**Goal:** Build and verify the passive Phase 1 language-camera baseline, deterministic text-model selection, aspect-preserving DINO runtime, bounded region scoring, and baseline 1-to-3 reports without modifying the robot motion or existing human-tracking path.

**Architecture:** Pure-Python contracts implement image geometry, query caching, model selection, score maps, proposals, and reports. Optional model adapters are loaded only when explicitly configured; ROS owns one selected visual tower, publishes compact typed regions, and fails closed when artifacts are absent. DINOv3 is corrected and benchmarkable but is never directly compared with an unrelated text space.

**Tech Stack:** Python 3.8, NumPy 1.24, OpenCV, PyTorch 1.13 NVIDIA, ROS 2 Foxy/rclpy, pytest, JSON Schema, optional isolated OpenCLIP-compatible adapter.

## Global Constraints

- Keep JetPack 5.0.2/L4T R35.1, Ubuntu 20.04, ROS 2 Foxy, Python 3.8, PyTorch 1.13 NVIDIA, CUDA 11.4, and TensorRT 8.4 unchanged.
- Do not install or upgrade ML dependencies in the working ROS environment.
- Preserve all existing human-tracking launch files, YAML defaults, controllers, safety nodes, and velocity topics.
- Phase 1 is opt-in, passive, and publishes no motion intent or command.
- No raw image, DINO, or text feature tensor is transported through DDS.
- Only one image backbone runs per frame in a selected benchmark/runtime configuration.
- Use test doubles only in unit tests; reports must identify missing real artifacts as unavailable.
- Build from Phase 0 checkpoint `8927e5ca2ac015ec3ac199901df3a637546fe4cc` on branch `feature/semantic-search-phase1`.

---

### Task 1: Aspect-preserving DINO image geometry

**Files:**
- Modify: `src/track_robot_perception/track_robot_perception/dinov3_runtime.py`
- Modify: `src/track_robot_perception/track_robot_perception/zed_dinov3_feature_node.py`
- Modify: `src/track_robot_perception/scripts/test_dinov3_on_image.py`
- Create: `src/track_robot_perception/test/test_dinov3_runtime.py`
- Modify: `src/track_robot_perception/setup.py`

**Interfaces:**
- Produces: `PreprocessTransform`, `preprocess_bgr_aspect_preserving(image_bgr, input_size, patch_size=16)`, `patch_grid(patch_tokens, grid_height, grid_width)`, `map_model_roi_to_source(...)`.
- Preserves: existing `preprocess_bgr(image_bgr, input_size)` for older callers.

- [ ] Write failing tests for a 1280x720 image mapping to a 512x512 canvas with a 512x288 valid image, top/bottom padding, a 32x32 patch mask, round-trip ROI clipping, bad dimensions, and square-image behaviour.
- [ ] Run `python3 -m pytest -q src/track_robot_perception/test/test_dinov3_runtime.py`; expect failures for missing symbols.
- [ ] Add an immutable transform record, centre padding, valid-token mask, inverse mapping, and rectangular-grid validation without changing model loading.
- [ ] Switch the DINO node and offline script to the aspect-preserving API; include scale/padding/resized/grid metadata in debug JSON and retain no feature DDS publisher.
- [ ] Add the new test directory to package test discovery and run the targeted tests; expect all pass.
- [ ] Commit as `feat: preserve aspect ratio in DINO runtime`.

### Task 2: Query contracts, caching, and model selection

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/query.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/model_selection.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/model_selection_cli.py`
- Create: `src/track_robot_semantic_search/schemas/model_benchmark.schema.json`
- Create: `src/track_robot_semantic_search/test/test_query.py`
- Create: `src/track_robot_semantic_search/test/test_model_selection.py`
- Modify: `src/track_robot_semantic_search/setup.py`

**Interfaces:**
- Produces: `normalize_query(text) -> str`, `QueryEncoding`, `CachedTextEncoder.encode(text, query_version)`, `select_candidate(report, gates) -> SelectionResult`.
- Selection order: passed gates, descending phrase-region recall, ascending P95 latency, lexical candidate ID.

- [ ] Write failing tests for Unicode NFKC normalisation, whitespace collapse, empty rejection, identical-query cache hits, version invalidation, encoder-ID separation, non-finite metric rejection, deterministic ties, and no-passing-candidate output.
- [ ] Run the two targeted test files; expect missing-module failures.
- [ ] Implement immutable query/model records and a one-entry cache that calls an injected encoder only on cache miss.
- [ ] Implement strict benchmark parsing and deterministic selection with explicit failed-gate reasons; do not import model libraries.
- [ ] Add the model benchmark schema and CLI entry point `semantic_search_select_text_model`; make non-selection exit non-zero after atomically writing a report.
- [ ] Run targeted tests and validate sample pass/unavailable reports against the schema.
- [ ] Commit as `feat: add cached query and model selection contracts`.

### Task 3: Open-vocabulary score maps and bounded proposals

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/region_scoring.py`
- Create: `src/track_robot_semantic_search/test/test_region_scoring.py`

**Interfaces:**
- Consumes: same-dimensional dense image embeddings `[H,W,C]`, text embedding `[C]`, valid-token mask `[H,W]`, and preprocessing transform metadata.
- Produces: `RegionCandidate` records sorted by descending score and `score_regions(...) -> List[RegionCandidate]`.

- [ ] Write failing tests for cosine score correctness, padding exclusion, non-finite rejection, 4-connected components, minimum area, source-coordinate inverse mapping, deterministic score ties, and `max_regions` truncation.
- [ ] Run `python3 -m pytest -q src/track_robot_semantic_search/test/test_region_scoring.py`; expect import failure.
- [ ] Implement L2-normalised dense scoring, absolute/quantile threshold modes, connected components using OpenCV, component statistics, bounded sorting, and empty-evidence behaviour.
- [ ] Run the targeted tests; expect all pass.
- [ ] Commit as `feat: add bounded open vocabulary region scoring`.

### Task 4: Optional aligned image-text adapter and passive worker

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/model_adapters.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/perception_core.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/perception_node.py`
- Create: `src/track_robot_semantic_search/test/test_model_adapters.py`
- Create: `src/track_robot_semantic_search/test/test_perception_core.py`
- Modify: `src/track_robot_semantic_search/setup.py`
- Modify: `src/track_robot_semantic_search/package.xml`

**Interfaces:**
- Produces: `AlignedImageTextEncoder` protocol, fail-closed `OpenClipAdapter`, `PassivePerceptionCore.process(image, query, version)`, and console script `semantic_search_perception`.
- Publishes: `track_robot_interfaces/SemanticRegionArray` and compact `std_msgs/String` diagnostics only.

- [ ] Write failing adapter tests proving missing optional imports and checkpoints are explicit errors, embedding dimension mismatch fails, and a test-only injected encoder can drive the core without being registrable in production.
- [ ] Write failing core tests for text-cache reuse, query-version invalidation, empty candidates, metadata propagation, observation IDs, latest-frame scheduling, timestamp rollback, and inference faults.
- [ ] Implement the adapter protocol and optional OpenCLIP adapter without an installer or implicit download path.
- [ ] Implement the pure perception core, keeping tensors in process and returning compact region records.
- [ ] Implement the ROS adapter with queue depth one, source-timestamp 5 Hz scheduler, not-ready/fault diagnostics, and no motion publisher.
- [ ] Run targeted tests; expect all pass.
- [ ] Commit as `feat: add passive language perception worker`.

### Task 5: Phase 1 opt-in launch and configuration

**Files:**
- Create: `src/track_robot_semantic_search/config/semantic_search_phase1.yaml`
- Create: `src/track_robot_semantic_search/launch/semantic_search_phase1.launch.py`
- Create: `src/track_robot_semantic_search/test/test_phase1_launch_contract.py`
- Modify: `src/track_robot_semantic_search/setup.py`
- Modify: `src/track_robot_semantic_search/README.md`

**Interfaces:**
- Launch arguments select image topic, target rate, model adapter, external runtime path, checkpoint path, and diagnostics; defaults do not start motion or alter the human stack.

- [ ] Write an AST/YAML contract test asserting one semantic perception node, opt-in activation, `/semantic_search` topics, 5 Hz default, bounded regions, and absence of `cmd_vel`, controller, planner, safety modification, action server, or motion bridge.
- [ ] Run the targeted launch test; expect missing-file failure.
- [ ] Add configuration and launch files, then package them through existing glob rules.
- [ ] Document external model artifact placement, checksums, fail-closed startup, query transport, and the fact that DINO and aligned baseline are mutually exclusive per-frame configurations.
- [ ] Run launch contract and package lint tests; expect all pass.
- [ ] Commit as `feat: add opt-in phase one semantic launch`.

### Task 6: Baseline 1-to-3 evaluation reports

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/phase1_baselines.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/phase1_baseline_cli.py`
- Modify: `src/track_robot_semantic_search/schemas/evaluation_report.schema.json`
- Create: `src/track_robot_semantic_search/test/test_phase1_baselines.py`
- Modify: `src/track_robot_semantic_search/setup.py`

**Interfaces:**
- Produces: reports with `baseline_id`, `status`, `inputs`, accuracy availability, timing/resource metrics, model/licence evidence, and gate reasons.
- CLI: `semantic_search_phase1_baselines --manifest ... --input ... --output-dir ...`.

- [ ] Write failing tests for detector-only, geometry-only, and language-camera aggregation; missing annotations as `not_evaluated`; missing model as `unavailable`; schema validity; deterministic ordering; and atomic output.
- [ ] Run the targeted test; expect import failure.
- [ ] Implement common report construction and three baseline adapters over recorded JSONL observations without adding ROS dependencies.
- [ ] Extend the schema backward-compatibly so existing Phase 0 reports still validate.
- [ ] Add the CLI and produce three reports from deterministic fixtures.
- [ ] Run Phase 0 and Phase 1 evaluator tests; expect all pass.
- [ ] Commit as `feat: report phase one semantic baselines`.

### Task 7: Real-model benchmark and Jetson evidence

**Files:**
- Create: `src/track_robot_semantic_search/scripts/benchmark_phase1_model.py`
- Create: `rosbags/semantic_search/reports/phase1_model_selection_2026-07-15.json`
- Create: `rosbags/semantic_search/reports/phase1_baseline_1_2026-07-15.json`
- Create: `rosbags/semantic_search/reports/phase1_baseline_2_2026-07-15.json`
- Create: `rosbags/semantic_search/reports/phase1_baseline_3_2026-07-15.json`
- Modify: `src/track_robot_semantic_search/setup.py`

**Interfaces:**
- Benchmark records warm-up, mean/P50/P95/max latency, CUDA allocation/reserve, environment, checkpoint checksum, licence metadata, and measured availability.

- [ ] Write parser/unit tests using an injected deterministic adapter and clock; verify no network or installer call occurs.
- [ ] Implement the benchmark runner and package script.
- [ ] Run DINO aspect-preserving benchmark using the existing local DINO checkpoint and record measured evidence.
- [ ] Evaluate locally available SigLIP 2/OpenCLIP candidates. If external artifacts are absent, write a schema-valid `unavailable` selection report naming each missing artifact and do not claim 5 Hz or accuracy.
- [ ] Generate baseline 1-to-3 reports from the frozen Phase 0 manifest and available recorded streams, preserving `not_evaluated` where semantic labels are absent.
- [ ] Validate all reports and commit as `test: record phase one baseline evidence`.

### Task 8: Regression, self-review, and Phase 1 checkpoint

**Files:**
- Create: `docs/superpowers/plans/2026-07-15-semantic-search-phase1-checkpoint.md`
- Modify: only files identified by review or verification failures.

**Interfaces:**
- Produces: an evidence-backed checkpoint separating implemented capability, measured results, unavailable external artifacts, and deferred Phase 2 work.

- [ ] Run focused Python tests for perception and semantic-search packages.
- [ ] Build `track_robot_interfaces`, `track_robot_perception`, and `track_robot_semantic_search` with `colcon build --packages-select ...`.
- [ ] Run the complete package tests and inspect `colcon test-result --all --verbose`.
- [ ] Run static searches proving no Phase 1 publisher or launch references `cmd_vel`, `SearchMotionIntent`, controller, planner, or motion bridge and no message carries feature tensors.
- [ ] Compare the Phase 0 strict replay report at 0.5/1.0/2.0 rates to ensure no regression in the frozen path.
- [ ] Self-review every Phase 1 diff for compatibility, fail-closed behaviour, report honesty, and unchanged human defaults; fix important findings and rerun affected checks.
- [ ] Write the checkpoint with exact commits, commands, counts, measured gates, and any model-artifact limitation.
- [ ] Commit as `docs: freeze semantic search phase one checkpoint` and stop before Phase 2.

## Plan self-review

- Spec coverage: every Phase 1 delivery and gate maps to Tasks 1 through 8.
- Scope: 3D fusion, temporal memory, trainable DINO-text projection, and motion remain excluded.
- Type consistency: query cache feeds the aligned adapter; preprocessing metadata feeds region inverse mapping; compact candidates feed ROS messages and baseline reports.
- Evidence integrity: synthetic encoders are limited to unit tests; missing real artifacts become unavailable/not-evaluated rather than passing results.
- Rollback: disabling the Phase 1 launch restores the Phase 0/human system without configuration changes.
