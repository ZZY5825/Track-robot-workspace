# Semantic Search Phase 1R-3R Visual Grounding Recovery Design

## Status

- Date: 2026-07-25
- Status: approved in conversation
- Parent architecture:
  `2026-07-13-language-conditioned-multimodal-semantic-search-design.md`
- Triggering evidence:
  `artifacts/semantic-search/phase123-blue-cylinder-2026-07-25/test_report.md`
- Deployment target: Jetson AGX Orin, JetPack 5.0.2, Ubuntu 20.04,
  ROS 2 Foxy, Python 3.8, CUDA 11.4, TensorRT 8.4.1

## 1. Decision summary

The existing OpenAI CLIP Phase 1 path remains a reproducible baseline, but it
is not promoted as the production target localizer. The recovery program
introduces a language-conditioned visual grounder that owns target
localization, a bounded DINOv3 target-crop descriptor path that owns visual
identity evidence, camera-led optional LiDAR geometry, and evidence-mode-aware
best-candidate calibration.

Model selection is measurement-driven:

1. the desktop teacher path is optional and is not a release prerequisite;
2. the Jetson runs pretrained bounded inference candidates that pass the frozen
   accuracy, latency, memory, compatibility, and licence gates;
3. Grounding DINO remains an optional desktop reference only;
4. YOLO-World-S-v2 is the active zero-training Orin candidate;
5. a DINOv3 query-conditioned localization head is considered only if the
   pretrained open-vocabulary student cannot pass the declared gates.

No model is selected merely because it imports or publishes a ROS message.

## 2. Why the current path fails

The physical test target was approximately 20 cm high, 5 cm wide, and 1.5 m
from the camera. It was visually clear, but the Phase 1 adapter encoded four
large image quadrants, the complete image, and one large center window. The
target occupied only a small fraction of each encoded crop. The best raw CLIP
similarity was approximately 0.1918 and came from the complete image, below the
configured 0.25 absolute threshold.

This evidence does not establish that the camera cannot resolve the target.
It establishes that:

- the global CLIP embedding is dominated by surrounding scene content;
- the current path has no production object-localization model;
- its proposal subscriber has no live proposal producer;
- proposal descriptors are pooled from a coarse 2x2 embedding grid rather than
  re-encoded from a target crop;
- the resulting full-image region makes camera-LiDAR projected IoU and size
  consistency poor;
- Phase 3 correctly stays fail-closed because no calibrated semantic evidence
  reaches a memory object.

Lowering the CLIP threshold would admit more scene-level false positives
without producing a defensible object boundary or stable identity descriptor.

## 3. Product boundary

### 3.1 Supported query form

The first release accepts one English description of one physical object using
visible attributes:

- category or common noun;
- colour;
- visible shape;
- visible material or texture when useful;
- short combinations such as `a tall blue cylindrical container`.

Input is restricted to printable ASCII, normalized whitespace, and a bounded
length. This is an enforceable proxy for the English-only product boundary; the
software does not claim to perform language identification from ASCII text.

The first release does not support:

- negation such as `not the blue one`;
- relational descriptions such as `the cup beside the bag`;
- actions or hidden properties;
- dialogue or pronoun resolution;
- simultaneous search for multiple target descriptions.

The system may add meaning-preserving syntactic prompt wrappers, but it does
not automatically add synonyms or remove attributes in a way that changes the
requested target.

### 3.2 Why English-only is selected

Text encoding occurs only when a query changes and is cached. English-only
operation therefore does not materially reduce per-frame visual inference
cost. It is selected because it bounds prompt validation, model comparison,
training data, calibration, operator guidance, and release claims.

## 4. Architecture

### 4.1 Runtime data flow

```text
English object query
  -> query validation and immutable query identity
  -> open-vocabulary visual grounder
  -> bounded target boxes and optional masks
  -> DINOv3 descriptor extraction for the top K target crops
  -> visual association and stable camera_track_id
  -> SemanticObservationArray
  -> optional mask/ROI-conditioned LiDAR point projection
  -> 2D-only or camera+LiDAR observation
  -> odom-local semantic memory
  -> evidence-mode-aware task relevance
  -> calibrated best candidate or explicit abstention
```

Large model tensors remain inside the model worker. ROS transports only
bounded queries, boxes, compressed masks, descriptors, scores, timestamps,
calibration identities, and object observations.

### 4.2 Offline teacher boundary

The desktop RTX environment is independent of the Jetson ROS installation. It
may run newer Python, PyTorch, CUDA, and model toolchains. It produces only
versioned artifacts:

- predictions in the project grounding-prediction schema;
- reviewed pseudo-labels;
- training and calibration splits;
- model checkpoints or exported engines;
- code revision, dependency, checkpoint checksum, licence, and provenance
  manifests;
- accuracy and resource reports.

Teacher output is not ground truth until it passes human review. Test labels
are human-authored or human-verified and are never replaced by teacher
predictions.

### 4.3 Jetson visual grounder

The first candidate family is YOLO-World-S/Seg because it supplies
open-vocabulary boxes or masks and has a smaller deployment target than the
teacher. Input sizes 640, 960, and 1280 are benchmark variants, not silently
interchangeable settings. The selected input size must pass target recall and
localization gates for the declared distance and object-size envelope.

Grounding DINO is also benchmarked as a reference and may become a Jetson
candidate only through an isolated runtime that passes the current JetPack
5.0.2 compatibility and complete-path gates. NVIDIA Jetson Platform Services
packages that require JetPack 6.x are not installed on the current robot.

If neither pretrained candidate passes, the fallback research path freezes
DINOv3 ViT-S and an English text encoder, then trains only compact projection
and localization heads from public phrase-region data plus reviewed teacher
labels. This fallback is a separate implementation decision after R0 evidence,
not an assumed part of the initial runtime.

### 4.4 Appearance and visual identity

The grounder owns semantic localization, not long-term identity. For at most
the top three candidates per processed frame:

1. enlarge the predicted ROI by a small bounded context margin;
2. preserve aspect ratio and pad the crop;
3. run the installed DINOv3 ViT-S backbone;
4. pool a unit-normalized descriptor from valid target tokens;
5. retain the model, checkpoint, preprocessing, and descriptor version.

A visual association layer uses source time, box or mask overlap, appearance
cosine, and bounded motion to maintain a stable `camera_track_id`. A new ID is
created rather than silently switching identity when association is ambiguous.
The grounder confidence, descriptor quality, and temporal stability remain
separate evidence fields.

A single-backbone variant that reuses student-detector features for appearance
is retained as an ablation. It replaces the DINO crop path only if it passes
the same identity-switch and re-entry gates.

### 4.5 ROS integration

The existing public contracts are retained:

- `/semantic_search/query`;
- `/semantic_search/regions`;
- `/semantic_memory/observations`;
- `/semantic_memory/tasks`;
- `/semantic_memory/visual_proposals`;
- `/semantic_search/perception_diagnostics`.

The model worker either runs in an isolated Python environment that can import
the ROS interfaces or behind a bounded local worker protocol. It must not
upgrade the system ROS Python environment. The selected grounder publishes
stable producer epochs, source timestamps, proposal IDs, and camera track IDs.
The current CLIP worker remains selectable as `clip_baseline`, never as an
implicit fallback after a production-model fault.

## 5. Phase 1R: visual-language localization

### 5.1 Deliverables

- English query validation shared by evaluation and runtime;
- versioned grounding dataset and prediction contracts;
- teacher and Jetson-candidate benchmark reports;
- selected open-vocabulary grounder;
- target boxes and masks mapped to original image coordinates;
- DINO target-crop appearance descriptors;
- stable camera track identities;
- explicit no-target output and fault diagnostics;
- RViz overlay of query, box or mask, language confidence, visual ID, and
  evidence state.

### 5.2 Phase 1R release gates

The initial numerical gates are:

- target-present top-1 recall at IoU 0.50: at least 0.85;
- target-absent false-accept rate: at most 0.05;
- median top-1 IoU on accepted positives: at least 0.50;
- evaluated distance envelope: 0.5 to 6 m;
- complete Jetson path P95: at most 150 ms;
- semantic output rate: at least 5 Hz;
- incremental CUDA reserve: at most 1.5 GiB;
- no duplicate query changes or timestamp rollback may reuse stale identities.

The gates are evaluated on a frozen held-out test split. Pilot results may
cause a documented design revision before the test split is opened, but test
results cannot be used to move the gates.

## 6. Phase 2R: camera-led optional LiDAR geometry

### 6.1 Role of LiDAR

LiDAR does not decide semantic identity and does not need to propose every
possible object in the scene. For each accepted visual region:

1. select the nearest point cloud by source timestamp;
2. reject pairing beyond the frozen time tolerance;
3. transform valid points into the camera frame at measurement time;
4. project points into the target mask, or a conservatively contracted ROI
   when no mask is available;
5. group compatible depths and reject background outliers;
6. estimate robust range, bearing, 3D centre, partial extent, and covariance;
7. transform the measurement into the active `odom` local-session frame;
8. optionally associate a compatible persistent LiDAR tracklet.

Insufficient points produce a valid camera-only observation with invalid 3D
geometry. They do not erase a visually strong target or fabricate depth.
Stereo depth may later provide a separately identified geometry source.

### 6.2 Phase 2R release gates

- wrong-depth attachment rate is measured and bounded before mutation is
  enabled;
- source-time, transform, and calibration failures fail geometry closed;
- supported 3D estimates report range error, bearing error, covariance, point
  count, and support provenance;
- camera-only degradation preserves the camera track and semantic evidence;
- the old all-visual/all-tracklet matrix remains available only for controlled
  comparison and diagnostics.

Numerical 3D accuracy gates are frozen after a calibration capture establishes
the sensor's achievable residual distribution; they are not invented from a
single object test.

## 7. Phase 3R: memory and best-candidate calibration

Camera track identity is the semantic authority. LiDAR tracklet identity is
optional physical support. The memory record stores both without conflating
them.

Task relevance consumes:

- visual grounding confidence;
- DINO appearance similarity and stability;
- temporal visual-track continuity;
- available 3D position continuity;
- geometry quality and sensor health;
- evidence age and prediction state.

Best-candidate calibration is stratified by support state:

- camera-only;
- camera plus LiDAR;
- prediction-only.

One threshold is not assumed to be calibrated across all three distributions.
The production profile remains fail-closed until the calibration dataset,
threshold, metrics, checksum, and software revision are frozen. An empty best
candidate is the correct result when no candidate passes.

Phase 3R reports top-1 correctness, false confirmation, identity switches,
time to confirmation, re-entry recovery, and abstention coverage.

## 8. Dataset and evaluation

### 8.1 Required coverage

The corpus includes:

- target present and absent;
- blue containers, green cups, blue bags, and other ordinary objects;
- physically distinct instances sharing similar descriptions;
- colour and shape distractors;
- cluttered and simple backgrounds;
- 0.5 to 6 m distance bins;
- centre, edge, partial, occluded, entry, and exit views;
- lighting variation;
- camera-only frames and frames with useful or missing LiDAR support.

The blue toothpaste-container observation is one regression case, not the
training objective.

### 8.2 Split integrity

Train, validation/calibration, and test data are separated by recording
session and physical object instance. A physical object or near-duplicate frame
cannot cross into the test split. Teacher-generated labels may be used for
training after review but never as unreviewed test truth.

All reports record dataset checksum, query set, model and checkpoint identity,
preprocessing, resolution, threshold configuration, hardware, software
revision, warm-up count, latency samples, and peak memory.

## 9. Failure handling

- Unsupported query syntax is rejected without replacing the active query.
- Missing or incompatible model artifacts keep the grounder unavailable.
- A model fault stops semantic inference and never launches the CLIP baseline
  silently.
- Oversized candidate sets are deterministically truncated before DINO crop
  encoding.
- Non-finite boxes, masks, scores, descriptors, or transforms reject the
  affected observation.
- Timestamp rollback advances the producer epoch and clears visual association.
- Calibration mismatch disables accepted 3D attachment.
- LiDAR absence produces camera-only evidence.
- Uncalibrated best-candidate configuration produces no selected object.

## 10. Delivery sequence

The work is split at evidence-bearing boundaries:

1. **R0A — benchmark contracts and evaluator:** build dataset integrity,
   prediction, metric, and deterministic model-selection tooling.
2. **R0B — optional desktop reference:** the runner exists, but desktop
   execution and pseudo-label review are not required.
3. **R0C — active Orin candidate comparison:** benchmark pretrained
   YOLO-World variants directly on the physical Jetson without training.
4. **R1 — Phase 1R runtime:** integrate only the selected grounder, DINO crop
   descriptors, visual tracking, ROS output, and RViz evidence.
5. **R2 — Phase 2R geometry:** add region-conditioned point projection and
   optional tracklet support.
6. **R3 — Phase 3R calibration:** collect stratified evidence, freeze
   thresholds, and enable best-candidate output.

Each stage produces usable, reviewable evidence. A downstream implementation
plan is written only after its upstream selection artifact exists.

## 11. Non-goals

- No JetPack, Ubuntu, ROS 2, or system Python upgrade in this recovery.
- No multilingual query claim.
- No relation, action, dialogue, or negation understanding.
- No LiDAR semantic classifier or learned LiDAR backbone.
- No Phase 4 cross-modal transformer or active motion.
- No release claim based solely on one blue-object test.
- No automatic GitHub release before the held-out gates pass.
