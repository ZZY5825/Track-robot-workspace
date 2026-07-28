# Semantic Search Phase 0–3 YOLO-World Test-Ready Design

## Status

- Date: 2026-07-27
- Direction approved in conversation: test-ready Phase 0–3 completion
- Written review: pending
- Deployment target: Jetson AGX Orin 32 GB, JetPack 5.0.2, L4T R35.1,
  Ubuntu 20.04, ROS 2 Foxy, Python 3.8, CUDA 11.4, TensorRT 8.4.1
- ROS domain for all managed live checks: 20

## 1. Outcome and completion boundary

This work makes the passive language-conditioned path testable from one
English object query through:

1. YOLO-World visual grounding;
2. stable camera candidate identity and bounded appearance evidence;
3. optional camera-led LiDAR geometry;
4. camera-only or camera-plus-LiDAR semantic memory;
5. an explicitly uncalibrated diagnostic ranking for operator review.

“Test-ready” means the software path, contracts, launch composition,
diagnostics, replay fixtures, and safe defaults are complete enough to collect
honest physical evidence. It does not mean the model has passed the held-out
accuracy gates, the camera/LiDAR extrinsic is measured, or production
best-candidate and re-identification mutation are enabled.

Formal release completion remains evidence-driven. It requires the physical
dataset, field calibration, scenario coverage, resource profile, licence
approval, and measured sensor configuration already declared by the parent
designs.

## 2. Current-state audit

### 2.1 Phase 0

The contracts, schemas, replay tools, evaluator, passive launch boundary, and
baseline report already exist. The checked Phase 0 report passes all five
declared gates. Phase 0 is frozen; this work changes it only when a backward
compatible contract extension is unavoidable.

### 2.2 Phase 1

The existing OpenAI CLIP worker correctly transports queries and publishes
bounded observations, but it is a coarse scene/window scorer rather than a
production object localizer. The 2026-07-25 physical target produced no formal
region at the 0.25 threshold.

The Orin YOLO-World R0C runner now loads and executes a local
`yolov8s-worldv2.pt` checkpoint, but it is offline-only. It does not subscribe
to a ROS image, publish the Phase 1 contracts, maintain camera track identity,
or feed semantic memory.

### 2.3 Phase 2

The existing C++ package already provides LiDAR tracklets, projection,
explainable camera/LiDAR association, source-time buffers, calibration binding,
appearance memory, deterministic assignment, and shadow-mode diagnostics.
Production configuration intentionally keeps association in shadow mode and
camera attachment disabled. The existing evaluator reports field evidence as
unavailable.

### 2.4 Phase 3

The existing semantic-memory core already provides lifecycle, epochs, local
session mode, object services, appearance prototypes, re-identification
mechanics, task overlays, and zero-or-one best-candidate publication.
However, its current live creation path is LiDAR-owned, re-identification
mutation is safe-off, and the best-candidate threshold is uncalibrated.
The current task scorer also assumes compatible task and appearance
descriptors; DINO visual identity descriptors must never be compared directly
with CLIP text embeddings.

## 3. Selected architecture

```text
English query
  -> existing query portal and immutable query ID/version
  -> YOLO-World ROS perception worker
       -> bounded boxes and grounding scores
       -> deterministic camera-track association
       -> optional top-K DINOv3 crop descriptors
  -> SemanticRegionArray + SemanticObservationArray + SemanticTask
  -> semantic memory
       -> create/update camera-only object records
       -> optionally attach source-time-compatible LiDAR geometry
       -> preserve camera semantic identity
       -> maintain appearance prototypes and lifecycle
  -> production best_candidate stays fail-closed
  -> diagnostic ranking is published with an explicit UNCALIBRATED state
  -> RViz/operator report
```

The existing CLIP worker remains selectable as `clip_baseline`. It is never
used as an implicit fallback when YOLO-World is unavailable.

## 4. Phase 1R: YOLO-World ROS perception

### 4.1 Runtime isolation

The shared YOLO-World backend moves behind an installable
`track_robot_semantic_search` module. Both the offline R0C CLI and the ROS
worker consume that module. The adapter:

- prepends `models/r0c_runtime/python` only inside its process;
- requires isolated Ultralytics `8.2.103`;
- uses the local `models/r0c/yolov8s-worldv2.pt`;
- uses the local OpenAI CLIP runtime and `ViT-B-32.pt`;
- never invokes an automatic model or dependency download;
- preserves the system NVIDIA Torch and global Ultralytics installation.

Model loading, vocabulary update, warm-up, and CUDA failures leave the worker
alive but unavailable and produce bounded diagnostics.

### 4.2 Image and query scheduling

The worker reuses the existing latest-image and source-timestamp scheduling
policy:

- depth-one image and query inputs;
- default target rate 5 Hz;
- no processing without an accepted English query;
- query text is printable ASCII with normalized whitespace;
- vocabulary encoding occurs only when query ID/version changes;
- source-time rollback advances the visual producer epoch and clears tracks;
- duplicate source timestamps are not processed twice.

The initial test profile uses FP16, input size 640, confidence proposal floor
0.05, NMS IoU 0.70, and at most 64 ROS observations per frame.

### 4.3 Visual candidate identity

Detections are validated and clamped in original image coordinates. A pure
bounded tracker assigns `camera_track_id` using:

- source timestamp;
- IoU;
- normalized centre displacement;
- optional compatible DINO appearance cosine;
- deterministic score and identifier tie-breaks.

Ambiguous association creates a new camera track rather than switching
identity. Tracks have a bounded missed-frame lifetime and are reset on producer
epoch, query identity, or timestamp rollback.

### 4.4 Appearance descriptors

YOLO-World owns language-conditioned localization. The existing local DINOv3
ViT-S+/16 runtime owns visual identity only.

For at most the top three candidates:

- expand the crop by a bounded context margin;
- preserve aspect ratio;
- batch crops where the runtime supports it;
- extract a unit-normalized descriptor;
- publish exact encoder, checkpoint, version, dimension, and quality.

DINO descriptors are used only for visual-to-visual association and
re-identification. They are never compared with CLIP text embeddings.
If DINO is unavailable, boxes still publish with empty appearance evidence and
an explicit degraded diagnostic; no fake descriptor is generated.

### 4.5 ROS outputs

The worker publishes existing bounded contracts:

- `/semantic_search/regions`;
- `/semantic_memory/observations`;
- `/semantic_memory/tasks`;
- `/semantic_search/perception_diagnostics`.

Every observation uses `PROPOSAL_OPEN_VOCABULARY`, carries query ID/version,
YOLO-World confidence, a stable visual candidate ID, optional camera track ID,
and task-conditioned semantic label evidence. It starts camera-only with
invalid 3D position. No feature grid or image-sized tensor crosses DDS.

`SemanticTask.task_descriptor` is the unit-normalized CLIP text vector used by
the active YOLO-World vocabulary, with the exact CLIP encoder and checkpoint
identity. It remains valid for contract compatibility, but the Phase 3R
task-conditioned score does not compare it to DINO descriptors.

## 5. Phase 2R: camera-led optional LiDAR geometry

### 5.1 Visual-first association

The current all-visual/all-tracklet diagnostic matrix remains available for
comparison. The test-ready path is visual-first:

1. choose the nearest source-time LiDAR batch;
2. reject stale or wrong-epoch evidence;
3. transform tracklet geometry at measurement time;
4. project into the camera;
5. retain only tracklets intersecting the visual ROI or mask;
6. apply field-of-view, depth, size, ambiguity, and calibration gates;
7. keep a small deterministic shortlist per visual candidate;
8. attach geometry only after the configured confirmation sequence.

This reduces irrelevant LiDAR candidates without allowing LiDAR to choose
semantic identity.

### 5.2 Camera-only degradation

An accepted visual observation creates or updates a camera-only semantic
object even if no LiDAR evidence is available. Its position remains invalid,
support state is `CAMERA_ONLY`, and memory mode is observation-only unless a
valid localization domain and 3D measurement exist.

When compatible LiDAR geometry is later attached, the same camera-owned object
identity is preserved and support becomes `CAMERA_LIDAR`. If geometry is lost,
semantic identity and camera tracking continue; no position is fabricated.

### 5.3 Calibration policy

Measured extrinsics are required for release evidence. Prototype extrinsics
are permitted only with `allow_degraded:=true`, are marked in diagnostics and
reports, and cannot produce a formal Phase 2 pass.

The default production profile remains shadow-only. A separate
`phase123_test` profile may enable mutation only when:

- an explicit `enable_test_camera_attachment:=true` flag is supplied;
- calibration status is `calibrated` or explicitly `degraded_prototype`;
- all source-time and transform gates pass.

## 6. Phase 3R: memory and diagnostic ranking

### 6.1 Camera-owned memory records

The memory core is generalized so a record may be keyed initially by a camera
track without a LiDAR tracklet. Camera-only objects have:

- a global object ID and producer epoch;
- camera track and visual candidate provenance;
- invalid 3D position;
- bounded semantic labels and DINO appearance prototypes;
- visible/stale/lost lifecycle driven by camera observations;
- no invented LiDAR source key.

Later LiDAR attachment supplements the record rather than replacing its
semantic identity. Epoch changes and ambiguous matches never merge objects
silently.

### 6.2 Task relevance

Phase 3R separates two score spaces:

- query-conditioned grounding evidence: YOLO-World confidence tied to the
  exact query ID/version;
- visual identity evidence: DINO-to-DINO appearance continuity.

The task overlay uses the current query’s grounding evidence directly and may
use temporal stability and support quality. It does not compute
CLIP-text-to-DINO cosine. Query-conditioned evidence expires with its query
version and producer epoch and is never promoted to a permanent class label.

### 6.3 Safe test output

Two outputs have different authority:

- `/semantic_memory/best_candidate`: production zero-or-one output; remains
  empty until a frozen calibration report is bound;
- `/semantic_memory/diagnostic_ranking`: a bounded
  `track_robot_interfaces/SemanticObjectArray` in descending diagnostic score
  order. Its object records carry query ID/version, task relevance and support
  state. The existing bounded `/semantic_memory/diagnostics` JSON carries
  `calibration_state=UNCALIBRATED`, component scores and rejection reasons
  keyed by memory epoch/global object ID.

RViz may display the diagnostic top candidate with a permanent
`UNCALIBRATED - NOT A CONFIRMED TARGET` warning. It must not use the same
colour or label as a calibrated production candidate.

Re-identification mutation remains off by default. A test flag may exercise
the already implemented confirmation and identity-transfer path, but its
result is diagnostic until field calibration passes.

## 7. Bringup and operator workflow

Managed stages become:

- `phase0`: contracts, replay and evaluator only;
- `phase1`: camera, query portal, YOLO-World and optional DINO crop evidence;
- `phase2`: Phase 1 plus LiDAR, localization, visual-first geometry and memory;
- `phase3`: Phase 2 plus test-mode diagnostic ranking and optional diagnostic
  re-entry exercise.

All stages use ROS domain 20. They remain passive and start no navigation,
controller, base-motion command, or `/cmd_vel` publisher.

Readiness checks verify model paths and checksums, CUDA, camera topic type and
rate, LiDAR topic type and rate when required, TF/calibration state,
localization state, and safe profile binding. A failed prerequisite prevents
feature activation and produces an actionable reason.

The control CLI starts and stops only processes it owns. Every automated live
check has a timeout and shutdown verification.

## 8. Evidence and testing

### 8.1 Automated tests

Implementation follows test-first development and includes:

- isolated runtime and local-checkpoint tests;
- query/cache/source-time scheduler tests;
- box normalization and deterministic camera tracking tests;
- DINO crop/descriptor bounds and incompatible-space tests;
- ROS output contract and launch safety tests;
- camera-only memory creation and lifecycle tests;
- visual-first LiDAR shortlist and attachment tests;
- query-version evidence expiry tests;
- diagnostic-versus-production candidate authority tests;
- deterministic Phase 1→2→3 replay;
- affected package build and regression.

### 8.2 Physical test-ready gate

Before asking the operator to judge accuracy, the software must prove:

- all affected packages build on the current Jetson environment;
- R0C probe reports ready with exact checkpoint checksums;
- a recorded image produces valid ROS detections or an explicit empty result;
- Phase 1 outputs correlate with query ID/version and source timestamp;
- Phase 2 preserves camera-only evidence and only attaches gated geometry;
- Phase 3 creates stable camera-owned records and emits diagnostic ranking;
- production best candidate remains empty without calibration;
- RViz labels all diagnostic candidates as uncalibrated;
- no ROS nodes or services remain after the test.

### 8.3 Evidence still requiring field work

The following cannot be completed honestly before physical testing:

- held-out grounding recall, false acceptance, and IoU;
- measured camera/LiDAR extrinsic;
- wrong-depth attachment distribution and 3D accuracy;
- re-entry and identity-switch calibration;
- independent positive/hard-negative best-candidate threshold calibration;
- twelve physical Phase 2/3 scenario classes;
- 30-minute Jetson resource and stability profile;
- AGPL-3.0 deployment approval.

## 9. Failure handling

- Missing model/runtime: worker unavailable; no fallback.
- Empty or invalid query: active query unchanged.
- YOLO or DINO fault: bounded diagnostic; no stale result reuse.
- Non-finite or out-of-image detection: candidate rejected.
- Timestamp rollback: new producer epoch and cleared visual tracks.
- Missing LiDAR/TF/calibration: camera-only evidence continues.
- Ambiguous geometry: no attachment.
- Missing calibration report: production best candidate remains empty.
- Incompatible appearance descriptors: appearance term ignored, never coerced.
- Capacity overflow: deterministic truncation with a reported counter.

## 10. Non-goals

- No training or fine-tuning.
- No desktop teacher requirement.
- No JetPack, CUDA, TensorRT, ROS, Python, Torch, or global Ultralytics upgrade.
- No multilingual, relational, dialogue, negation, or multi-query claim.
- No raw feature tensor DDS transport.
- No active search motion, navigation, or base command.
- No automatic enablement of camera attachment, re-identification mutation,
  or production best-candidate output.
- No formal accuracy claim from the blue-container example.
