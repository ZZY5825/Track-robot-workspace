# Generalized Multi-Object 3D Semantic Memory Design

**Date:** 2026-07-15
**Status:** Approved for implementation
**Phase:** 2
**Parent design:** `2026-07-13-language-conditioned-multimodal-semantic-search-design.md`

## 1. Problem summary

Phase 1 produces query-conditioned `SemanticRegionArray` messages. It does not
yet publish query-independent proposals, visual candidate IDs, object-level
descriptors, camera track IDs, proposal provenance, or detector confidence.
Phase 2 must turn multiple visual observations and persistent LiDAR tracklets
into a persistent object-centric 3D memory without changing the existing human
tracking behavior.

The current LiDAR manager already provides clustering, Hungarian assignment,
Kalman filtering, NIS gates, and persistent tracklets. The selected-human
tracker contains useful projection, timing, ambiguity, and hysteresis logic, but
it owns one selected human and contains human-specific weights and geometry.
Phase 2 therefore reuses the LiDAR producer, extracts generic mechanisms from
the selected-human tracker, and introduces a separate multi-object memory core.

## 2. Approved architecture

The approved deployment is option C:

```text
Phase 1 Python/GPU inference worker
        |  SemanticObservationArray / SemanticTask
        v
Phase 2 C++ semantic_memory_node <--- C++ LiDAR tracklet manager
  - bounded source-time buffers
  - camera/LiDAR projector
  - CrossModalAssociator
  - MemoryCore
  - TaskRelevanceScorer
        |
        +--> active object snapshots / events / debug
        +--> query and mutation services

Independent semantic_memory_visualizer_node
```

Association and memory update are separate pure C++ modules in the same Phase
2 process. Their API must remain transport-independent so profiling can later
move the associator to a separate process without changing MemoryCore or ROS
contracts. Phase 1, LiDAR tracking, Phase 2 memory, and visualization are four
fault-isolation units.

The future Transformer replacement boundary is:

```text
bounded candidate set + modality masks -> association hypotheses/logits
```

The model never owns global IDs, lifecycle transitions, memory epochs, or hard
physical gates.

## 3. Non-negotiable invariants

1. Camera candidate, camera track, LiDAR tracklet, query, localization epoch,
   memory epoch, and global object IDs remain separate.
2. `global_object_id` is allocated only by MemoryCore.
3. A public object key is `(memory_epoch_id, global_object_id)`.
4. A global object ID is monotonic and never reused inside one memory epoch.
5. A process restart starts a new memory epoch unless an explicitly validated
   checkpoint is restored; checkpoint persistence is not part of Phase 2.
6. A LiDAR key is `(lidar_source_epoch_id, lidar_tracklet_id)`.
7. Observations from different spatial domains or localization epochs are not
   associated automatically.
8. Task relevance never overwrites permanent object semantics or identity.
9. Incompatible visual encoder/checkpoint versions are never compared by
   cosine similarity.
10. Real-time ROS messages contain bounded data only; detailed history lives in
    the process or evaluation artifacts.
11. The system prefers temporary non-association over an incorrect ID switch.
12. Existing human tracking topics, launch files, configurations, and message
    behavior remain available and unchanged by default.

## 4. Spatial memory domains

`base_link`, `odom`, and `map` have different roles:

- `base_link` is an instantaneous fusion frame only.
- `odom` is the canonical frame for `LOCAL_SESSION` memory.
- `map` is the canonical frame for `WORLD` memory only after localization
  health, covariance, continuity, and jump gates pass.

Memory modes are:

```text
OBSERVATION_ONLY
LOCAL_SESSION
WORLD
```

MemoryCore derives a spatial domain key from:

```text
(memory_mode, localization_epoch_id, canonical_frame_id)
```

A change in that key begins a new memory epoch. In particular, promotion from
`LOCAL_SESSION` to `WORLD` never silently reinterprets old odom coordinates as
map coordinates. Migration requires an explicit, validated, logged transform;
automatic migration is outside the first implementation.

The legacy human-tracking bag is observation-only. Its camera/odom TF tree and
base/LiDAR TF tree are disconnected, and it does not contain the Odometry and
IMU evidence required by localization health. Point-LIO exists but requires a
topic/frame adapter, measured LiDAR-IMU extrinsics, and field validation before
it can prove local or world memory.

## 5. ROS contracts

Existing `ObjectObservation3D`, `TrackedSemanticObject`, their arrays, and
`SearchForObject.action` remain as compatibility interfaces. Phase 2 adds new
bounded contracts rather than changing those messages in place.

### 5.1 Supporting messages

`VisualDescriptor.msg`:

- bounded encoder ID, checkpoint ID, and descriptor version;
- declared dimension;
- L2-normalized flag;
- at most 1024 float values.

`VisualProposal.msg` and `VisualProposalArray.msg`:

- bounded query-independent or query-conditioned 2D proposal input;
- source producer epoch/proposal ID, image stamp/geometry, ROI, optional bounded
  mask, proposal source, detector confidence, and bounded label evidence;
- at most 64 proposals per source result.

`SemanticObjectHistorySample.msg`:

- one short-history record containing stamp, position, uncertainty, support,
  and lifecycle. This avoids fragile parallel history arrays.

`SemanticLabelEvidence.msg`:

- bounded label and provenance strings;
- confidence;
- permanent/task-conditioned flag;
- source observation ID.

`SemanticLocalizationState.msg`:

- header;
- memory mode and localization epoch;
- mode/epoch changed flags;
- local, world, and base frame IDs;
- local/world health flags;
- bounded reason string.

`SemanticTask.msg`:

- query ID and version;
- bounded query text for provenance;
- normalized text descriptor and encoder metadata;
- source timestamp and producer epoch.

### 5.2 SemanticObservation

`SemanticObservation.msg` is one bounded evidence packet. It contains:

- header and producer epoch;
- observation and visual candidate IDs;
- optional camera track ID;
- optional LiDAR key including source epoch;
- independent camera, LiDAR, pose, and TF source stamps;
- image dimensions, ROI, optional bounded compressed mask, and proposal source;
- detector, appearance, language, geometry, and overall confidence terms;
- one optional canonical 3D position, frame, covariance, velocity, and extent;
- memory mode and localization epoch;
- calibration ID and evidence flags;
- one normalized bounded object descriptor;
- bounded semantic label evidence.

It never contains a global object ID. `SemanticObservationArray` contains at
most 64 observations from one producer result.

`SemanticLidarTrackletArray.msg` wraps the existing `LidarTracklet` record with
a producer source epoch and a bound of 256 tracklets. The human-facing
`LidarTrackletArray` remains unchanged. The LiDAR manager dual-publishes the
bounded Phase 2 array only when its semantic output is configured.

### 5.3 SemanticObject

`SemanticObject.msg` is the compact public memory record. It contains:

- memory epoch and global object ID;
- current camera and LiDAR source references;
- spatial domain, canonical position, velocity, extent, and covariance;
- lifecycle, sensor support, visibility, motion, re-identification, and
  inspection states as independent axes;
- permanent semantic evidence and current-task relevance as separate fields;
- appearance summary metadata, prototype count, and active encoder version;
- per-sensor counts and first/last observation stamps;
- association confidence, uncertainty, duplicate, and anomaly values;
- at most 16 recent position/time/uncertainty summaries.

`SemanticObjectArray` publishes a bounded full snapshot of active objects only,
at most 256 objects. Stale/lost/archived records are retrieved by paginated
services rather than republished at camera rate.

### 5.4 Explainability and events

`AssociationTerm.msg` records a named normalized term, raw value, weight,
validity, and gate result. `AssociationDebug.msg` records the candidate pair,
all terms, total score/cost, decision, margin, and reason. Arrays are bounded.

`SemanticMemoryEvent.msg` records object creation, confirmation, state change,
association attach/detach, re-identification, archival, reset, domain change,
and rejection events. Event records are reliable and bounded.

### 5.5 Services

Phase 2 provides:

- `GetSemanticObject.srv` by public object key;
- `QuerySemanticObjects.srv` using the active task or a compatible supplied
  descriptor, with bounded pagination;
- `MarkSemanticObjectInspected.srv`;
- `ResetSemanticMemory.srv`, which always creates a new memory epoch.

Raw text is encoded by the Phase 1 Python worker into `SemanticTask`; the C++
core does not embed text itself.

## 6. Visual candidate and feature policy

Phase 1 must publish candidates independently from permanent identity and task
state. Initial supported proposal sources are:

- existing language-patch regions;
- external camera detector proposals;
- manual ROI for controlled evaluation;
- future patch grouping or LiDAR-projected proposals.

The normalized adapter stamps proposal source and never treats a query label as
permanent truth.

Image encoding is independent from query scoring: an image is encoded when a
bounded external/manual proposal set is available even if there is no active
language query. With neither a query nor proposals, Phase 1 emits no visual
observation and Phase 2 continues in LiDAR-only mode.

The first object descriptor is computed by overlap-weighted pooling of the
already computed visual grid cells inside each candidate, followed by L2
normalization. Descriptor quality records cell coverage and proposal quality.
This avoids a second inference pass on Jetson. A crop re-encoding strategy may
later replace this behind the same `VisualDescriptor` contract.

The default Phase 1 `grid_size=2` is insufficient for reliable object re-ID.
Phase 2 profiling must benchmark candidate quality, latency, and memory for
supported grid sizes before selecting a deployment value.

Each object stores at most four appearance prototypes per compatible encoder
version. A confirmed, sufficiently high-quality observation updates the nearest
prototype by confidence-weighted EMA; a sufficiently different good view may
create a new prototype. Low-confidence, ambiguous, prediction-only, or
incompatible-version observations do not update the bank.

## 7. Association baseline

Association is source-time driven. Inputs are paired through bounded buffers,
and TF is queried at each evidence stamp. Arrival order never substitutes for
source time. Timestamp rollback clears affected buffers and changes the source
or memory epoch.

Hard rejection gates include:

- maximum source-time difference and evidence age;
- equal spatial/localization domain;
- TF and calibration availability;
- projected field-of-view compatibility;
- 3D innovation/Mahalanobis gate;
- impossible size or motion;
- compatible descriptor encoder/checkpoint/version.

Soft, separately logged terms include:

- 3D position consistency;
- projected centroid, inside fraction, center distance, and IoU;
- visual cosine similarity;
- extent and point-count consistency;
- motion continuity;
- previous association continuity;
- detector, geometry, and sensor confidence.

Current language relevance is excluded from permanent identity association. It
is used only by task ranking.

Ordinary matching uses global one-to-one Hungarian assignment. A new attachment
requires multi-frame confirmation. Existing attachments use margin, hysteresis,
and cooldown. An insufficient top-two margin yields ambiguity rather than a
forced match. Cluster split/merge cases create temporary hypotheses and never
immediately merge or split global IDs.

Implementation starts in shadow mode: calculate and record term distributions
without attaching camera identity. Weights and thresholds are enabled only
after inspecting the pilot bag and are stored in a versioned configuration.

## 8. Memory state model

Lifecycle is independent from sensor support, visibility, and motion:

```text
TENTATIVE -> CONFIRMED -> STALE -> LOST -> ARCHIVED
```

- `TENTATIVE`: requires repeated compatible evidence; position and features
  update conservatively and the object is not a task winner.
- `CONFIRMED`: normal measurement, semantic, and feature updates.
- `STALE`: no recent support; prediction/hold continues, covariance grows,
  features freeze, and confidence decays.
- `LOST`: removed from active snapshots but retained as a re-ID candidate.
- `ARCHIVED`: compact metadata only; no automatic online reactivation.

Sensor support is `CAMERA_LIDAR`, `CAMERA_ONLY`, `LIDAR_ONLY`,
`PREDICTION_ONLY`, or `NONE`. Visibility is `VISIBLE`, `OCCLUDED`,
`OUT_OF_FOV`, or `UNKNOWN`. Motion is `STATIC`, `DYNAMIC`, `UNCERTAIN`, or
`TEMPORARILY_MOVING`. Re-identification is an event/status rather than a
lifecycle state.

Static objects use lower process noise and longer retention and do not require
non-zero velocity. Dynamic objects use motion prediction and shorter recovery
windows. Exact timeouts remain configuration values calibrated from rosbag
evidence rather than copied from the human tracker.

## 9. Topics and QoS

Initial topics are:

```text
/semantic_memory/observations
/semantic_memory/tasks
/semantic_memory/localization_state
/semantic_memory/lidar_tracklets
/semantic_memory/active_objects
/semantic_memory/best_candidate
/semantic_memory/events
/semantic_memory/association_debug
/semantic_memory/diagnostics
/semantic_memory/markers
```

Sensor and candidate inputs use depth-one/latest semantics with explicitly
configured reliability. Object snapshots and events are reliable. Debug output
is best-effort and can be disabled. Foxy zero-copy is not assumed. Every queue,
mask, candidate set, descriptor, label list, debug-term list, result page, and
history is bounded. Overload drops the oldest unprocessed evidence and emits a
counter; it never grows a queue.

## 10. Evaluation and acceptance

Evaluation has two levels.

### 10.1 Pilot rosbag gate

Record 60-90 seconds containing one static object from three viewpoints, robot
translation and rotation, one crossing person, a camera occlusion, and if
possible a leave-and-re-enter event. Record RGB, CameraInfo, raw LiDAR,
observations, tracklets, connected TF, odometry, IMU, localization state/epoch,
task changes, events, and diagnostics.

The pilot gate requires:

- zero ID switches, duplicate objects, and incorrect merges for the annotated
  static target;
- one global ID across viewpoints and short camera loss;
- the crossing human remains a separate identity;
- re-entry restores the original ID when the event is present;
- camera-LiDAR precision at least 95% and recall at least 80%;
- Phase 2 update rate at least 5 Hz;
- P95 Phase 2 core latency at most 50 ms, excluding Phase 1 inference;
- identical IDs and association events for repeated normalized-input replay;
- no unbounded queue or history growth.

3D consistency is reported on the pilot; its numeric gate is frozen only after
the real LiDAR-centroid and localization-error distribution is observed.

### 10.2 Full completion gate

The full suite covers multiple similar static objects, a moving human crossing
static objects, temporary occlusion, camera-only and LiDAR-only periods,
prediction-only loss, leave/re-entry, LiDAR split/merge, camera false positives,
LiDAR false clusters, robot rotation/translation, and a task change without
clearing memory.

Reports include ID continuity, association precision/recall, ID switches,
duplicates, incorrect merges, fragmentation, re-ID and stale-reactivation
accuracy, 3D position consistency, memory lifetime, task ranking, module
latency, update rate, CPU/GPU/memory use, long-duration stability, and human
tracking regression evidence.

## 11. Out of scope

Phase 2 does not include end-to-end multimodal Transformer training, a learned
LiDAR backbone, VLA/control output, Nav2 integration, long-horizon planning,
arm perception, DINOv3 fine-tuning, permanent database infrastructure, or
unrestricted robot motion commands.
