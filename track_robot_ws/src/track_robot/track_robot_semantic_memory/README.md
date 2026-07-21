# Track Robot Semantic Memory

`track_robot_semantic_memory` is the bounded C++ semantic-memory process used
by Phase 2. Stage 2B supports LiDAR-only object identity, lifecycle, motion,
source/domain isolation, reliable active snapshots and a separate RViz marker
adapter. Stage 2C adds camera/LiDAR projection, explainable pair scoring,
deterministic global assignment, confirmation logic and ROS shadow debug. Stage
2D connects calibrated camera attachment to `MemoryCore` while retaining a
safe-off default. Stage 2E connects bounded appearance memory, deterministic
multi-frame re-identification and atomic identity transfer to that runtime.
Stage 2F additionally provides tested pure-policy modules for task-conditioned
ranking and bounded memory-service behavior. None of these paths bypasses the
calibration gate or mutates permanent identity from an uncalibrated camera
match.

## Run

Build the workspace, source it, then launch the dedicated Phase 2 components:

```bash
source install/setup.bash
ros2 launch track_robot_semantic_memory semantic_memory_phase2.launch.py
```

This launch starts only `semantic_memory_node` and
`semantic_memory_visualizer_node`. It does not alter the existing human
tracking launch or default topics.

## ROS interfaces

Inputs:

- `/semantic_memory/localization_state` — reliable typed localization/domain
  state;
- `/semantic_memory/lidar_tracklets` — best-effort bounded semantic LiDAR
  tracklets with a producer source epoch;
- `/semantic_memory/observations` — reliable bounded Phase 1 visual
  observations with source timestamps, ROI, descriptor provenance and camera
  calibration identity;
- `/semantic_memory/tasks` — reliable depth-one task updates with a public
  query ID/version, bounded ASCII text and compatible appearance descriptor;
- `/zed/zed_node/left/camera_info` — best-effort camera intrinsics used by the
  generic projector.

Outputs:

- `/semantic_memory/active_objects` — reliable transient-local snapshot,
  bounded to 256 active objects and enriched with the active task/inspection
  overlay;
- `/semantic_memory/best_candidate` — reliable transient-local zero-or-one
  object snapshot; an empty array means there is no calibrated valid winner;
- `/semantic_memory/events` — reliable lifecycle, reset and rejection events;
- `/semantic_memory/diagnostics` — readiness, counts and rejection reasons;
- `/semantic_memory/association_debug` — reliable bounded per-pair hard gates,
  soft terms, total score, margin and shadow decision;
- `/semantic_memory/markers` — reliable transient-local bounded RViz cubes.

Every public object key is `(memory_epoch_id, global_object_id)`. LiDAR source
epoch and tracklet ID remain separate producer identifiers.

Live services are `/semantic_memory/get_object`,
`/semantic_memory/query_objects`, `/semantic_memory/mark_inspected` and
`/semantic_memory/reset`. Queries use pages of at most 64 objects and reset can
require an exact expected epoch.

## Stage 2F task and service policy

Task relevance is a separate, replaceable overlay. It uses the best compatible
appearance prototype plus bounded permanent semantic evidence; incompatible
encoder/checkpoint/version/dimension evidence is ignored. Changing or clearing
the task leaves object keys, lifecycle, prototypes and permanent labels
unchanged.

The service core enforces public object keys, pages of at most 64 records,
relevance-descending deterministic order with object-key tie breaking, and
distinct stale-epoch/not-found results. Inspection changes and resets emit
bounded events. Reset advances the epoch before indexes are cleared. Best
candidate selection fails closed until its relevance threshold is explicitly
calibrated, and then permits only confirmed, active, uninspected objects.

The Stage 2F policy is live in `semantic_memory_node`. A task received before
the first memory snapshot is held in one bounded pending slot; source-time
rollback clears it fail-closed. Invalid/unhealthy localization and spatial
domain changes immediately invalidate all four services and publish empty
active/best snapshots until a successful LiDAR snapshot re-establishes the
runtime view.

Production winner selection remains deliberately safe-off:

```yaml
best_candidate_threshold_calibrated: false
best_candidate_minimum_relevance: 1.0
```

The node still publishes the best-candidate topic, but it contains zero objects
until a later calibrated profile explicitly enables a threshold. Stage 2F does
not claim a physical task-ranking pilot or threshold calibration.

## Normalized deterministic replay

`semantic_memory_replay INPUT.json OUTPUT.json` replays bounded serialized
localization domains, LiDAR observations, assignment matrices, compatible task
evidence and expected event sequences without ROS transport or GPU inference.
It emits canonical JSON with sorted public objects and deterministic assignment
tie-breaking. The Python `semantic_search_phase2_replay` wrapper executes it
twice and accepts evidence only when the output bytes are identical.

## Spatial-domain behavior

- `OBSERVATION_ONLY / base_link` is instantaneous. Each input batch starts a
  new memory epoch so robot motion cannot mix coordinates across frames.
- `LOCAL_SESSION / odom` preserves IDs within a healthy localization epoch.
- `WORLD / map` preserves IDs only while the typed world-localization state is
  healthy.

A mode, localization epoch or canonical-frame change starts a new memory
epoch. Localization state must also be source-time-close to the LiDAR batch.
LiDAR geometry is transformed into the canonical frame at each tracklet's
`last_measurement_stamp`.

Camera-to-LiDAR projection also queries TF at that exact LiDAR source stamp.
Missing TF, calibration, domain, time, FOV or incompatible evidence remains an
explicit invalid/failed term rather than silently becoming a zero score.

## Stage 2C calibration and Stage 2D runtime attachment

The default configuration remains safe-off and continues to publish shadow
debug:

```yaml
association_shadow_mode: true
camera_attachment_enabled: false
association_calibration_status: calibrated
```

`config/phase2_association_baseline.yaml` contains the calibrated human-pilot
ranges, normalized soft-term weights, match threshold, and ambiguity margin.
The checked-in `config/phase2_camera_attachment.yaml` profile enables the
Stage 2D runtime path with `association_shadow_mode: false` and
`camera_attachment_enabled: true`. Startup then parses the named calibration
report and fails closed unless it says `status=calibrated`, explicitly permits
attachment, contains at least 20 samples per class, meets 95% precision / 80%
recall, and exactly matches the installed scoring contract: camera calibration,
all hard gates, all twelve normalized weights, threshold, ambiguity margin and
confirmation policy. The report is installed under this package's `share`
directory, so the profile does not depend on the caller's working directory.

Runtime association builds one complete source-time score matrix, rejects an
oversized matrix rather than assigning from truncated evidence, applies
deterministic global Hungarian assignment, and requires three compatible
frames before memory mutation. Rejected frames cannot partially advance
confirmation state, and LiDAR source epochs are isolated in the source-time
buffer. The stable confirmation identity is a camera
track key when present, otherwise an upstream proposal key. A one-shot
`visual_candidate_id` without either stable key remains tentative and cannot
write memory.

Delayed visual results supplement the object selected by its LiDAR producer
key. They may update visual association metadata, bounded non-task semantic
labels, sensor counts, visibility, support and confidence. They never rewind
position, velocity, covariance or the metric `state_stamp`. Compatible, finite,
normalized current-frame appearance evidence updates the Stage 2E prototype
bank. Invalid optional appearance evidence does not roll back an otherwise
valid Stage 2D attachment. One stable visual identity has exactly one memory
owner; a confirmed replacement atomically detaches the prior owner and emits
detach/attach events.

To enable attachment, launch with the installed calibrated profile as
`config_file`. An absolute `association_calibration_report` remains supported
for controlled experiments. To roll back, set only:

```yaml
camera_attachment_enabled: false
```

With the attachment profile's `association_shadow_mode: false`, that single
switch disables both memory mutation and runtime association output. Set
`association_shadow_mode: true` separately only when shadow diagnostics are
desired.

Reproduce the calibration report from captured debug pairs and human
positive/negative labels with:

```bash
python3 scripts/export_phase2_association_samples.py \
  --debug-jsonl association_debug.jsonl \
  --annotations-jsonl association_annotations.jsonl \
  --output-jsonl phase2_association_samples.jsonl \
  --report-json phase2_association_calibration.json \
  --dataset-id pilot-session
```

The exporter requires at least 20 positive and 20 negative labels by default,
at least 20 positive samples that pass all valid hard gates, separable
normalized soft scores, precision at least 95%, and recall at least 80%.
The checked-in report contains 22 positive and 79 negative pairs, with 100%
precision and 81.8% recall at the selected threshold.

## Stage 2E appearance memory and re-identification

`MemoryCore` owns at most four compatible appearance prototypes per object.
Only a descriptor accepted from the current visual observation can make an
active object eligible as a re-identification candidate; an older matching
prototype alone cannot advance confirmation. The runtime rejects prediction-
only, stale, ambiguous, incompatible and invalid evidence without partial
mutation.

Each complete re-identification frame is bounded to 64 active candidates, 256
lost targets and 1,024 candidate/target pairs. It uses deterministic Hungarian
assignment, checks ambiguity along both matrix axes and requires the same pair
to win three consecutive increasing frames. A confirmed transfer preserves
the lost object's global key and cumulative counts, adopts the replacement's
current metric/source state, deletes the temporary duplicate and emits exactly
one `EVENT_REIDENTIFIED` event.

Production re-identification is deliberately safe-off:

```yaml
reidentification_shadow_mode: true
reidentification_mutation_enabled: false
reidentification_calibration_status: uncalibrated
```

The synthetic DDS leave/re-entry test exercises the runtime transaction, but
it is not physical-robot evidence. No production re-identification calibration
report or field leave/re-entry result is claimed at this checkpoint.

## Configuration and limits

Defaults are in `config/semantic_memory.yaml`. Object count, history, input
queues and marker state are bounded. Static and dynamic lifecycle profiles,
motion thresholds, TF timeout and localization-state freshness are separately
configurable. An `initial_memory_epoch_id` of zero derives a deterministic
nonzero epoch from the localization domain and LiDAR source epoch; a nonzero
value is intended for controlled tests.

## Verification

The normal package tests cover deterministic replay, identity isolation,
lifecycle, transforms, covariance, projection, hard/soft association terms,
descriptor compatibility, Hungarian assignment, confirmation/cooldown,
shadow export, all lifecycle/support multi-sensor update policies, bounded
appearance prototypes, conservative re-identification, visualization
collisions and deletion, task-overlay isolation, service pagination, epoch
reset, calibrated best-candidate selection and normalized deterministic replay.
Stage 2D runtime association and delayed visual supplementation, Stage 2E
appearance memory/re-identification, and Stage 2F live task/service/reset
transport are connected and tested. The opt-in DDS fixture covers pending task
delivery, zero/one winner publication, get/query/inspection, epoch reset,
unhealthy-localization invalidation, pending-task rollback and domain changes.
Threshold calibration and the physical task-ranking pilot remain Stage 2G
work; this checkpoint is not Phase 2 completion.
The executable DDS probe is opt-in because it requires local network-interface
access:

```bash
RUN_ROS_RUNTIME_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  colcon test --packages-select track_robot_semantic_memory \
  --ctest-args -R test_ros_runtime --output-on-failure
```

Run that probe on a normal ROS host; restricted sandboxes that deny
`getifaddrs` cannot execute DDS discovery.
