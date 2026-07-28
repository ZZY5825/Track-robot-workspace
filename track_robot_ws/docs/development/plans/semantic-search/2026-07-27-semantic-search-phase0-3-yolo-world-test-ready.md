# Semantic Search Phase 0–3 YOLO-World Test-Ready Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the pretrained Orin YOLO-World path testable from an English ROS
query through visual grounding, optional LiDAR geometry, camera-owned semantic
memory, and explicitly uncalibrated diagnostic ranking.

**Architecture:** Reuse the existing R0A/R0C model and evidence contracts,
existing ROS semantic messages, and the existing C++ memory/association core.
Add an installable YOLO-World runtime, bounded camera tracking and DINO crop
identity evidence, then generalize memory so camera-only objects remain valid
and LiDAR supplements rather than owns semantic identity. Production mutation
and best-candidate outputs remain fail-closed.

**Tech Stack:** ROS 2 Foxy, Python 3.8, rclpy, OpenCV, NVIDIA PyTorch 1.13,
isolated Ultralytics 8.2.103, YOLOv8s-World-v2, local OpenAI CLIP, local
DINOv3 ViT-S+/16, C++17, rclcpp, pytest, GTest, colcon.

## Global Constraints

- Target remains Jetson AGX Orin, JetPack 5.0.2/L4T R35.1, Ubuntu 20.04,
  Python 3.8, CUDA 11.4, TensorRT 8.4.1.
- All managed ROS checks use `ROS_DOMAIN_ID=20`.
- Do not upgrade ROS, Python, JetPack, CUDA, TensorRT, NVIDIA Torch,
  torchvision, or global Ultralytics 8.0.239.
- R0C uses isolated Ultralytics 8.2.103 and explicit local regular checkpoint
  files; no runtime download or installer is allowed.
- Accept one normalized printable-ASCII English object query.
- YOLO-World owns language-conditioned localization. DINO owns
  visual-to-visual identity only. Never compare CLIP text and DINO descriptors.
- No navigation, base controller, motion action, `Twist`, or `/cmd_vel`
  publisher may be introduced.
- Default production camera attachment, re-identification mutation, and
  calibrated best-candidate output remain disabled.
- Existing user and agent changes in the dirty workspace must be preserved.
  Record diff/test checkpoints; do not create partial commits until repository
  history is reconciled.

---

## File structure

- `track_robot_semantic_search/yolo_world_backend.py`: installable isolated
  YOLO-World runtime and generic detections/text descriptors.
- `tools/semantic_search_grounding/ultralytics_yolo_world.py`: R0C compatibility
  wrapper over the installed backend.
- `track_robot_semantic_search/camera_tracking.py`: ROS-independent bounded
  camera candidate tracker.
- `track_robot_semantic_search/dino_crop_descriptors.py`: top-K crop batching
  and DINO identity descriptors.
- `track_robot_semantic_search/yolo_world_perception_node.py`: query/image
  scheduling and ROS publication.
- `track_robot_semantic_memory/memory_core.*`: optional LiDAR identity and
  camera-owned object lifecycle.
- `track_robot_semantic_memory/runtime_association.*`: visual-first candidate
  shortlist.
- `track_robot_semantic_memory/task_relevance_scorer.*`: separated grounding
  and DINO evidence spaces.
- `semantic_memory_node.cpp`: camera-only ingestion, optional geometry,
  diagnostic ranking and safe output policy.
- bringup/config/RViz files: explicit Phase 0–3 stages and operator evidence.

---

### Task 1: Share the isolated YOLO-World backend with the ROS package

**Files:**
- Create:
  `src/track_robot_semantic_search/track_robot_semantic_search/yolo_world_backend.py`
- Create:
  `src/track_robot_semantic_search/test/test_yolo_world_backend.py`
- Modify:
  `tools/semantic_search_grounding/ultralytics_yolo_world.py`
- Modify:
  `tools/semantic_search_grounding/test/test_ultralytics_yolo_world.py`

**Interfaces:**
- Produces:
  `GroundedDetection(x1, y1, x2, y2, score, label)`.
- Produces:
  `YoloWorldBackend.from_local_model(...)`.
- Produces:
  `predict(image_path_or_bgr, normalized_query) -> Tuple[GroundedDetection, ...]`.
- Produces:
  `active_text_descriptor() -> numpy.ndarray` with shape `(512,)`, finite,
  unit-normalized values from `model.model.txt_feats`.
- Preserves the R0C `UltralyticsYoloWorld` public API by converting generic
  detections to `TeacherDetection`.

- [x] **Step 1: Write failing shared-backend tests**

Add tests proving local-path validation, exact isolated origin/version,
CUDA initialization before peak-memory reset, local CLIP interception,
query-change caching, finite box normalization, deterministic truncation, and
unit-normalized extraction of the active `1x1x512` text feature.

```python
def test_active_text_descriptor_is_flat_unit_vector(fake_backend):
    fake_backend.model.model.txt_feats = FakeTensor(
        [[[3.0, 4.0] + [0.0] * 510]])
    descriptor = fake_backend.active_text_descriptor()
    assert descriptor.shape == (512,)
    assert np.linalg.norm(descriptor) == pytest.approx(1.0)
```

- [x] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search:. \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_yolo_world_backend.py
```

Expected: collection fails because `yolo_world_backend` does not exist.

- [x] **Step 3: Implement the shared backend**

Move the dependency isolation, Torch compatibility shims, local CLIP wrapper,
inference call, result validation and CUDA accounting into the installed
module. Use this immutable result:

```python
@dataclass(frozen=True)
class GroundedDetection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    label: str
```

`active_text_descriptor()` must copy the cached model text feature to CPU
float32, flatten exactly one class, reject non-finite/wrong-sized input, and
normalize it without returning a Torch tensor.

- [x] **Step 4: Convert the R0C adapter to a compatibility wrapper**

The tools wrapper delegates model loading/prediction/memory to the shared
backend and maps every `GroundedDetection` to the existing
`TeacherDetection`. It must retain all R0C CLI behavior and artifact schemas.

- [x] **Step 5: Verify GREEN and regression**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search:. \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_yolo_world_backend.py \
  tools/semantic_search_grounding/test
```

Expected: all tests pass; no network access and no ROS node.

---

### Task 2: Add deterministic camera candidate tracking

**Files:**
- Create:
  `src/track_robot_semantic_search/track_robot_semantic_search/camera_tracking.py`
- Create:
  `src/track_robot_semantic_search/test/test_camera_tracking.py`

**Interfaces:**
- Consumes: validated `GroundedDetection` values and optional DINO descriptors.
- Produces:
  `CameraCandidate(candidate_id, camera_track_id, detection, descriptor,
  descriptor_quality)`.
- Produces:
  `CameraTrackManager.update(stamp_ns, query_key, detections)`.
- Produces:
  `CameraTrackManager.reset(producer_epoch_id)`.

- [x] **Step 1: Write failing tracker tests**

Cover stable ID on high IoU, deterministic handling of reordered detections,
new ID for ambiguity, bounded missed-frame lifetime, query-change reset,
timestamp rollback rejection, incompatible descriptor identity, and ID
overflow rejection.

```python
def test_ambiguous_match_creates_new_track():
    manager = CameraTrackManager(config())
    first = manager.update(1, (10, 1), [det(0, 0, 20, 20)])
    second = manager.update(
        2, (10, 1), [det(0, 0, 20, 20), det(1, 0, 21, 20)])
    assert second.candidates[0].camera_track_id != (
        second.candidates[1].camera_track_id)
```

- [x] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_camera_tracking.py
```

Expected: collection fails because `camera_tracking` does not exist.

- [x] **Step 3: Implement bounded tracking**

Use at most 64 detections and tracks. Build accepted pairs from IoU, normalized
centre distance and optional compatible descriptor cosine. Sort pairs by
descending score and stable IDs, reject a match whose best/second-best margin
is below the configured ambiguity margin, then greedily assign one-to-one.
Never mutate state until the entire frame validates.

- [x] **Step 4: Verify GREEN**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_camera_tracking.py
```

Expected: all tracker tests pass.

---

### Task 3: Add bounded DINOv3 crop identity descriptors

**Files:**
- Create:
  `src/track_robot_semantic_search/track_robot_semantic_search/dino_crop_descriptors.py`
- Create:
  `src/track_robot_semantic_search/test/test_dino_crop_descriptors.py`
- Modify:
  `src/track_robot_perception/track_robot_perception/dinov3_runtime.py`
- Modify:
  `src/track_robot_perception/test/test_dinov3_runtime.py`

**Interfaces:**
- Consumes: BGR image and at most three original-image boxes.
- Produces:
  `CropDescriptor(values, quality, encoder_id, checkpoint_id, version)`.
- Produces:
  `DinoCropDescriptorBackend.encode(image_bgr, boxes)`.

- [x] **Step 1: Write failing crop tests**

Prove context-margin clipping, aspect padding, exact top-three bound,
deterministic input order, batch shape, unit normalization, finite validation,
empty degraded output when disabled, and no CLIP/text comparison.

- [x] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search:src/track_robot_perception \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_dino_crop_descriptors.py \
  src/track_robot_perception/test/test_dinov3_runtime.py
```

Expected: collection fails because `dino_crop_descriptors` does not exist.

- [x] **Step 3: Add reusable batched feature extraction**

Extend `dinov3_runtime.py` with:

```python
def extract_cls_batch(model, input_tensor, backend):
    cls_tokens, _, details = extract_features(model, input_tensor, backend)
    if cls_tokens.ndim != 2 or cls_tokens.shape[0] != input_tensor.shape[0]:
        raise RuntimeError('DINO batch output shape is invalid')
    return cls_tokens, details
```

The crop backend loads only the explicit local Python-3.8-compatible repo and
checkpoint, batches at most three crops, returns CPU float32 unit vectors, and
records exact model identity. It does not publish heatmaps or raw tokens.

- [x] **Step 4: Verify GREEN**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search:src/track_robot_perception \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_dino_crop_descriptors.py \
  src/track_robot_perception/test
```

Expected: all tests pass.

---

### Task 4: Implement the Phase 1R YOLO-World ROS worker

**Files:**
- Create:
  `src/track_robot_semantic_search/track_robot_semantic_search/yolo_world_perception_node.py`
- Create:
  `src/track_robot_semantic_search/test/test_yolo_world_perception_core.py`
- Create:
  `src/track_robot_semantic_search/test/test_yolo_world_node_contract.py`
- Create:
  `src/track_robot_semantic_search/config/semantic_search_yolo_world.yaml`
- Create:
  `src/track_robot_semantic_search/launch/semantic_search_yolo_world.launch.py`
- Modify: `src/track_robot_semantic_search/setup.py`

**Interfaces:**
- Subscribes: existing image and `/semantic_search/query`.
- Publishes: `/semantic_search/regions`,
  `/semantic_memory/observations`, `/semantic_memory/tasks`,
  `/semantic_search/perception_diagnostics`.
- Uses `PROPOSAL_OPEN_VOCABULARY`.
- The task descriptor is the active YOLO CLIP text vector; appearance
  descriptors are DINO-only.

- [x] **Step 1: Write failing pure-core and source-contract tests**

Prove no inference without a query, cached vocabulary, latest-image scheduling,
producer-epoch reset, exact query/source correlation, maximum 64 observations,
camera-only geometry fields, task-conditioned label provenance, DINO identity
fields, and absence of motion publishers.

- [x] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search:src/track_robot_perception \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_yolo_world_perception_core.py \
  src/track_robot_semantic_search/test/test_yolo_world_node_contract.py
```

Expected: collection fails because the worker module and entry point do not
exist.

- [x] **Step 3: Implement the ROS-independent frame coordinator**

The coordinator accepts image, query key and source time, calls the backend,
encodes top-three DINO crops, updates tracks, and returns immutable output.
Model faults change state to `fault` and clear pending frames. Empty model
results produce valid empty arrays.

- [x] **Step 4: Implement ROS conversion and diagnostics**

Publish bounded messages with original source headers. Diagnostics JSON has
schema `phase1r_runtime/1.0.0`, model readiness, producer epoch, active query,
frame/detection/track counts, model and DINO latency, degraded reasons, dropped
frames and rollback count.

- [x] **Step 5: Add launch/config/entry point**

Use explicit local defaults under `models/r0c*` and `models/dinov3*`. No
implicit download and no CLIP fallback. The launch remains default-off when
invoked directly.

- [x] **Step 6: Verify GREEN**

Run:

```bash
PYTHONPATH=src/track_robot_semantic_search:src/track_robot_perception:. \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q src/track_robot_semantic_search/test
python3 -m py_compile \
  src/track_robot_semantic_search/track_robot_semantic_search/yolo_world_backend.py \
  src/track_robot_semantic_search/track_robot_semantic_search/camera_tracking.py \
  src/track_robot_semantic_search/track_robot_semantic_search/dino_crop_descriptors.py \
  src/track_robot_semantic_search/track_robot_semantic_search/yolo_world_perception_node.py
```

Expected: all semantic-search tests pass and compilation exits zero.

---

### Task 5: Generalize semantic memory for camera-only object ownership

**Files:**
- Modify:
  `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_core.hpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/memory_core.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/ros_conversions.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/test/test_memory_core.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/test/test_ros_conversions.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/test/test_ros_runtime.py`

**Interfaces:**
- Changes `MemoryObject::lidar_key` to
  `std::optional<ProducerObjectKey>`.
- Adds `MemoryCore::update_camera(const MemoryDomainKey &,
  const CameraObservation &)`.
- Adds `CameraObservation` with visual key, query key, source time, semantic
  confidence, optional descriptor and camera ROI provenance.
- Preserves existing LiDAR-only `update()` behavior.

- [x] **Step 1: Write failing C++ camera-only tests**

Add tests proving camera observation creates one object with no LiDAR ID,
repeated camera track preserves global ID, different camera track creates a new
ID, rollback advances/rejects safely, camera-only lifecycle becomes stale/lost,
and ROS conversion emits `SUPPORT_CAMERA_ONLY`, invalid position and
`lidar_tracklet_id_valid=false`.

- [x] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_interfaces track_robot_semantic_memory
```

Expected: the semantic-memory test target fails to compile because
`CameraObservation` and `update_camera` do not exist.

- [x] **Step 3: Implement optional LiDAR identity**

Guard every source-index and conversion access with `has_value()`. Add a
camera-index keyed by `VisualAssociationKey`. Camera-only history carries no
fabricated position sample. Existing LiDAR object creation and deterministic
replay output remain byte-compatible.

- [x] **Step 4: Ingest valid unmatched camera observations**

In `on_observations`, after strict message validation, update camera-only
memory even when no LiDAR batch is available. If association is shadow-only,
the camera object remains unmodified by LiDAR. Publish the resulting active
snapshot and events.

- [x] **Step 5: Verify GREEN**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_interfaces track_robot_semantic_memory
source install/setup.bash
colcon test --packages-select track_robot_semantic_memory
colcon test-result --all --verbose
```

Expected: zero semantic-memory test failures and unchanged legacy deterministic
replay output.

---

### Task 6: Add visual-first optional LiDAR geometry and safe test profile

**Files:**
- Modify:
  `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/runtime_association.hpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/runtime_association.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/test/test_runtime_association.cpp`
- Create:
  `src/track_robot/track_robot_semantic_memory/config/phase123_test.yaml`
- Modify:
  `src/track_robot/track_robot_semantic_memory/launch/semantic_memory_phase2.launch.py`

**Interfaces:**
- Produces `shortlist_visual_pairs(...)` with at most
  `maximum_lidar_candidates_per_visual` accepted projected candidates.
- Adds explicit launch flags `enable_test_camera_attachment` and
  `allow_degraded_calibration`.

- [x] **Step 1: Write failing shortlist and launch-policy tests**

Prove out-of-FOV/gate-rejected tracklets are removed before scoring, nearest
source-time batch is used, shortlist order is deterministic, ambiguity yields
no attachment, and attachment cannot enable without the explicit test flag and
accepted calibration status.

- [x] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_interfaces track_robot_semantic_memory
```

Expected: compile or launch-contract failure for the missing shortlist/test
profile API.

- [x] **Step 3: Implement visual-first shortlist**

Project each active tracklet, retain only those with visible intersection and
hard-gate compatibility, sort by accepted status, total score and stable LiDAR
key, then truncate. The diagnostic all-pairs path remains available only in
shadow mode.

- [x] **Step 4: Attach geometry without replacing camera identity**

When confirmation accepts one pair, supplement the camera-owned record. If a
LiDAR-only record already represents the tracklet, merge physical state into
the camera-owned global ID and archive/remove the duplicate atomically. Loss of
LiDAR changes support back to camera-only and never deletes semantic identity.

- [x] **Step 5: Verify GREEN**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_interfaces track_robot_semantic_memory
source install/setup.bash
colcon test --packages-select track_robot_semantic_memory
ros2 run track_robot_semantic_memory semantic_memory_replay \
  src/track_robot_semantic_search/test/data/phase2_normalized_replay.json \
  /tmp/phase2_replay_a.json
ros2 run track_robot_semantic_memory semantic_memory_replay \
  src/track_robot_semantic_search/test/data/phase2_normalized_replay.json \
  /tmp/phase2_replay_b.json
cmp /tmp/phase2_replay_a.json /tmp/phase2_replay_b.json
```

Expected: zero test failures and byte-identical replay files.

---

### Task 7: Separate grounding relevance from DINO identity and publish diagnostics

**Files:**
- Modify:
  `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/task_relevance_scorer.hpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/task_relevance_scorer.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/runtime_task_services.hpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/runtime_task_services.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/src/semantic_memory_node.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/test/test_task_relevance_scorer.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/test/test_runtime_task_services.cpp`
- Modify:
  `src/track_robot/track_robot_semantic_memory/test/test_ros_runtime.py`

**Interfaces:**
- Adds `TaskConditionedGroundingEvidence(query_id, query_version,
  producer_epoch_id, source_stamp_ns, confidence, stability)`.
- `ObjectTaskEvidence` carries optional active grounding evidence separately
  from DINO prototypes and permanent semantics.
- Publishes `/semantic_memory/diagnostic_ranking` as bounded
  `SemanticObjectArray`.

- [x] **Step 1: Write failing relevance-space tests**

Prove matching query-version grounding confidence is eligible without
descriptor cosine, stale/wrong-version evidence is ineligible, DINO and CLIP
identity mismatch never rejects valid grounding evidence, permanent labels
remain separate, diagnostic ranking is deterministic, and production
best-candidate remains empty while uncalibrated.

- [x] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_interfaces track_robot_semantic_memory
```

Expected: compile failure for missing
`TaskConditionedGroundingEvidence`.

- [x] **Step 3: Implement separated scoring**

Compute relevance from active grounding confidence, temporal stability,
permanent semantic evidence where valid, and support quality. DINO appearance
is used only for continuity/re-ID, not query similarity. Store query evidence
with producer epoch and expire it on query/version change or bounded age.

- [x] **Step 4: Publish diagnostic ranking**

Publish active eligible objects in descending score with deterministic ID
tie-break. Add bounded JSON diagnostics keyed by memory epoch/global ID with
component scores, evidence mode, `calibration_state=UNCALIBRATED`, and
rejection reason. Do not call or bypass calibrated `best_candidate()`.

- [x] **Step 5: Verify GREEN**

Run:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
colcon test --packages-select track_robot_semantic_memory
colcon test-result --all --verbose
```

Expected: zero failures. The DDS runtime test must run explicitly when its
existing opt-in environment flag is set, and every spawned node is stopped.

---

### Task 8: Add explicit Phase 0–3 bringup, readiness and RViz evidence

**Files:**
- Modify:
  `src/track_robot/track_robot_bringup/track_robot_bringup/control_config.py`
- Modify:
  `src/track_robot/track_robot_bringup/track_robot_bringup/readiness.py`
- Modify:
  `src/track_robot/track_robot_bringup/track_robot_bringup/live_test.py`
- Modify:
  `src/track_robot/track_robot_bringup/launch/semantic_search_live.launch.py`
- Modify:
  `src/track_robot/track_robot_bringup/launch/semantic_search_visualization.launch.py`
- Create:
  `src/track_robot/track_robot_bringup/rviz/semantic_search_phase3.rviz`
- Modify:
  `src/track_robot/track_robot_semantic_search_rviz_plugins/src/semantic_search_panel.cpp`
- Modify:
  `src/track_robot/track_robot_bringup/test/test_control_config.py`
- Modify:
  `src/track_robot/track_robot_bringup/test/test_launch_contract.py`
- Modify:
  `src/track_robot/track_robot_bringup/test/test_readiness.py`
- Modify:
  `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`

**Interfaces:**
- Adds managed `phase0`, `phase1`, `phase2`, `phase3`.
- Adds model/checkpoint/CUDA, topic, TF/calibration, localization and safe-profile
  readiness reasons.
- Phase 3 RViz consumes diagnostic ranking and always renders
  `UNCALIBRATED - NOT A CONFIRMED TARGET`.

- [x] **Step 1: Write failing stage/readiness/RViz tests**

Prove exact stage composition, Domain 20, no motion components, correct model
paths, Phase 1 camera-only hardware, Phase 2/3 optional externally owned
hardware, production-safe defaults, phase3 visualization topic and warning.

- [x] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/track_robot/track_robot_bringup \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_control_config.py \
  src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_readiness.py \
  src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py
```

Expected: failures because phase0/phase3 and diagnostic ranking are unsupported.

- [x] **Step 3: Implement stages and readiness**

Phase 0 starts only passive evaluation contracts. Phase 1 starts camera and
YOLO worker. Phase 2 adds LiDAR/localization/memory. Phase 3 selects the
explicit test profile and diagnostic ranking without enabling production
best-candidate or motion.

- [x] **Step 4: Implement RViz diagnostic rendering**

Render diagnostic candidates with a colour distinct from production best
candidate, show query ID/version, support mode and score, and permanently
display the uncalibrated warning. Empty output is a valid abstention.

- [x] **Step 5: Verify GREEN**

Run:

```bash
PYTHONPATH=src/track_robot/track_robot_bringup \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test \
  src/track_robot/track_robot_semantic_search_rviz_plugins/test
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_bringup track_robot_semantic_search_rviz_plugins
```

Expected: all focused tests pass and both packages build.

---

### Task 9: End-to-end replay, documentation and completion verification

**Files:**
- Create:
  `src/track_robot_semantic_search/test/data/phase123_yolo_world_replay.json`
- Create:
  `src/track_robot_semantic_search/test/test_phase123_yolo_world_replay.py`
- Modify:
  `docs/guides/semantic-search/phase2-recording-and-evaluation.md`
- Create:
  `docs/guides/semantic-search/phase0-3-yolo-world-test.md`
- Modify: `README.md`
- Modify: `docs/README.md`
- Update this plan’s checkboxes and completion record.

**Interfaces:**
- Produces one deterministic software-only Phase 1→2→3 fixture and one
  operator live-test procedure.
- Produces no formal accuracy claim.

- [x] **Step 1: Write failing end-to-end replay test**

The fixture must prove:

1. an English query produces a correlated YOLO candidate;
2. the candidate retains one camera track ID across frames;
3. a no-LiDAR frame creates a camera-only memory object;
4. a later gated LiDAR match supplements the same global object;
5. a distractor does not replace its identity;
6. diagnostic ranking contains the object;
7. production best candidate remains empty while uncalibrated.

- [x] **Step 2: Verify RED**

Run the replay test and expect the not-yet-wired stage failure.

- [x] **Step 3: Implement the deterministic fixture and runner**

Use fixed timestamps, boxes, DINO descriptors, LiDAR tracklets, transforms and
query versions. Serialize canonical results and require two runs to be
byte-identical.

- [x] **Step 4: Write the operator guide**

Document one-command stage startup, readiness, English query input, expected
topics/RViz output, camera-only and camera+LiDAR expectations, evidence capture,
timeouts, known calibration limits and shutdown. Use ROS domain 20 throughout.

- [x] **Step 5: Run final verification**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-up-to \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup \
  track_robot_semantic_search_rviz_plugins
source install/setup.bash
export ROS_DOMAIN_ID=20
colcon test --packages-select \
  track_robot_interfaces \
  track_robot_perception \
  track_robot_semantic_search \
  track_robot_lidar_tracking \
  track_robot_semantic_memory \
  track_robot_bringup \
  track_robot_semantic_search_rviz_plugins
colcon test-result --all --verbose
```

Also run the 68 R0C tests, 220 R0A tests, complete semantic-search pytest
suite, Python compilation, model checksum verification, actual Orin probe,
offline smoke, `git diff --check`, process inventory, and a read-only
`ROS_DOMAIN_ID=20 ros2 node list`.

Expected: zero test failures, R0C runtime ready, no model checksum change, no
global dependency change, and no remaining ROS node/service started by tests.

- [x] **Step 6: Record the honest checkpoint**

Mark software as `phase0_3_test_ready`. Keep model accuracy, measured
extrinsic, field association, re-entry calibration, 30-minute stability,
licence approval and production best-candidate release status explicitly
pending physical evidence.

## Completion record

- Checkpoint: `phase0_3_test_ready`; this is software and deterministic
  contract readiness, not a physical accuracy release.
- Affected build: 22 packages finished successfully.
- Selected Phase 0–3 ROS packages: 1156 tests, 0 errors, 0 failures and
  3 explicitly disabled runtime tests.
- Semantic-search package: 736 passed.
- R0C grounding tools: 68 passed.
- Frozen R0A contracts: 220 passed.
- Deterministic Phase 1→2→3 replay: two byte-identical outputs with SHA-256
  `521bc97f3c8957e496dbfb8c7c53578d789c2402ad48e4e18d059e06ed32d76d`;
  camera-only to camera+LiDAR identity continuity passed and production best
  candidate remained empty.
- Actual Orin probe:
  `artifacts/semantic_search/reports/phase0_3_orin_inventory_2026-07-27.json`
  reports `runtime_ready=true`, CUDA device `Orin`, isolated Ultralytics
  `8.2.103` and unchanged model hashes.
- Offline smoke:
  `artifacts/semantic_search/reports/phase0_3_orin_smoke_2026-07-27.json`
  exited zero with 460 MiB incremental CUDA reserve and 6950.676494 ms cold
  complete-path latency. The fixed 0.05-floor query
  `blue toothpaste container` returned no detection, so model quality remains
  unproven.
- Python compilation, installed CLI help, JSON validation and
  `git diff --check` passed. Global Ultralytics remains `8.0.239`.
- The final read-only ROS Domain 20 graph was empty. No test-owned ROS node,
  RViz process or sensor launch remained running.
- Still pending physical evidence: labelled live target/hard-negative
  accuracy, measured camera/LiDAR extrinsic, field association precision and
  recall, re-entry calibration, 30-minute stability, licence approval and a
  calibrated production best-candidate threshold.
