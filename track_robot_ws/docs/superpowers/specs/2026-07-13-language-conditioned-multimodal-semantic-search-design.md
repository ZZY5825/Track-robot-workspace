# Language-Conditioned Multimodal Outdoor Semantic Search

## Approved Architecture Design

- Date: 2026-07-13
- Status: architecture approved in conversation; written-spec review pending;
  implementation has not started
- Platform: NVIDIA Jetson AGX Orin, JetPack 5.0.2, Ubuntu 20.04, ROS 2 Foxy
- Workspace: /home/track-robot/track_robot_ws
- First demonstration: find a fallen branch or path obstruction

## 1. Executive summary

This design generalizes the current gesture-selected human tracker into a
language-conditioned, object-centric outdoor semantic-search system. The
recommended destination is a modular research architecture with:

- DINOv3 as the dense visual representation backbone;
- a cached CLIP or SigLIP-family text representation;
- a lightweight trainable query-conditioned localization head;
- existing LiDAR ground removal, clustering, tracklets, and geometric
  uncertainty as the initial LiDAR branch;
- object-level multimodal fusion;
- multi-object 3D semantic memory with explicit identifier separation;
- calibrated uncertainty and abstention;
- bounded, rotation-only active perception through the existing classical
  safety and control chain.

The design deliberately avoids running two full image encoders on every camera
frame. The real-time hot path runs DINOv3 once per selected frame, caches the
text encoding when a query changes, and consumes dense features inside one
model-worker process. Large feature tensors are not published through ROS.

The first release guarantees evaluation from 0.5 to 6 metres, processes
semantic observations at at least 5 Hz, and permits only bounded in-place
rotation. Eight to ten metres is an extension measurement, not a first-release
guarantee. The robot must explicitly abstain when evidence is inadequate.

Architecture B, described below, is the recommended destination. It is
delivered in Architecture-A-style slices so that every learned component has a
simple baseline and every phase produces a usable result.

## 2. Problem, scope, and success boundary

### 2.1 Problem

Given a free-form positive description of one visible target and synchronized
outdoor sensor data, the robot must:

1. find relevant visual and LiDAR candidates;
2. rank candidates using language, appearance, geometry, and history;
3. estimate a 2D region and, when evidence permits, a 3D position and extent;
4. maintain candidate identity through multiple observations;
5. preserve explicit uncertainty and source history;
6. request a safer viewpoint by bounded in-place rotation when confidence is
   intermediate;
7. confirm, reject, or report uncertainty without silently switching targets;
8. report the result through typed ROS interfaces.

### 2.2 First-release scope

The first release supports:

- a fallen branch or path obstruction as the primary outdoor demonstration;
- one active language query at a time;
- free descriptions of one target and its visible attributes;
- a UTF-8 query string received through SearchForObject.action; keyboard,
  speech, or an upstream VLM may create that string without changing the
  perception contract;
- multiple candidate objects in memory;
- camera-only, LiDAR-only, camera-LiDAR, prediction, ambiguous, and lost
  evidence states;
- 2D region, 3D centre, 3D extent, covariance, confidence, and evidence source;
- 0.5 to 6 metre guaranteed evaluation range;
- passive observation and bounded in-place rotation;
- explicit not-found and uncertain results.

### 2.3 Out of scope for this specification

The following require separate later specifications:

- autonomous approach, circling, or Nav2 global search;
- open-ended anomaly discovery without a positive target query;
- guarantees for damaged trees, people-attribute search, relational targets,
  or other categories not represented in the declared held-out test manifest;
- guarantees beyond 6 metres;
- unrestricted multi-turn dialogue, negation, and complex relational language;
- a learned LiDAR backbone in the real-time path;
- a temporal foundation model or learned continuous-action policy;
- direct continuous velocity output from any learned component;
- manipulation, grasp planning, arm control, or wrist-camera perception.

Speech recognition, dialogue management, and upstream VLM task decomposition
are outside this package. They may submit the same typed action goal as a human
operator; the semantic-search server does not infer trust or safety permission
from the source of the string.

The architecture preserves extension points for these capabilities but does not
claim them as first-release functionality.

## 3. Current repository assessment

### 3.1 Repository shape

The current workspace does not contain a usable top-level README or usable Git
metadata. The authoritative project documentation is distributed across:

- src/track_robot_perception/README.md;
- src/track_robot_perception/docs;
- rosbags/human_tracking_rosbag_test_guide.md;
- package-specific configuration and launch files.

The working source packages relevant to this feature are:

- track_robot_perception;
- track_robot_lidar_tracking;
- track_robot_interfaces;
- track_robot_decision;
- track_robot_control;
- track_robot_safety;
- track_robot_bringup.

### 3.2 Current perception and tracking flow

The current human-tracking flow is:

    ZED RGB
      -> person detector and camera tracker
      -> gesture trigger
      -> logical camera target lock
      -> camera target

    RoboSense point cloud
      -> ground and range filtering
      -> generic candidate clusters
      -> persistent LiDAR tracklets

    camera target + LiDAR tracklets + camera-guided points
      -> selected human target tracker
      -> IMM/Kalman-filtered TargetState
      -> behavior decision
      -> target controller
      -> local trajectory planner
      -> motion safety supervisor
      -> velocity gate
      -> Bunker base

The camera owns semantic identity. LiDAR supplies geometry, persistent physical
tracklets, and short continuation. Association retains ambiguity instead of
globally selecting a replacement cluster. This principle is directly reusable.

### 3.3 Reusable modules

The new system reuses, with independent configuration where required:

- ZED rectified RGB and CameraInfo inputs;
- RoboSense point-cloud bringup;
- LiDAR-camera transforms and projection utilities;
- adaptive ground removal;
- generic candidate clusters;
- persistent LiDAR tracklets and timestamp-aware prediction;
- covariance, quality, and point-density fields;
- bounded nearest-timestamp queues and the existing 80 ms pairing limit;
- selected-target IMM/Kalman concepts;
- explicit logical-ID versus physical-tracklet-ID separation;
- perception health, safety state, and typed decision messages;
- behavior-tree, controller, trajectory-planning, safety-supervisor, velocity
  gate, emergency-stop, and RC-override layers;
- rosbag replay, regression monitoring, and replay-rate comparison patterns.

### 3.4 Modules to generalize

The following elements are currently human-specific and must not be reused
unchanged:

- person-only detector and gesture selection;
- torso keypoints and torso-depth extraction;
- human height, width, and motion limits;
- selected-human naming and topic semantics;
- one-human identity state machine;
- FollowDecision paths that permit forward following.

The generic LiDAR tracker can be launched with a separate semantic-search
configuration. The human-tracking defaults remain unchanged.

### 3.5 Relevant technical debt and mismatches

1. DINOv3 is a feature encoder, not a detector or semantic mask generator.
2. The current DINOv3 node publishes only a debug heatmap and JSON metadata;
   it does not expose a production feature contract.
3. Current DINO preprocessing stretches a 1280 by 720 image to 512 by 512,
   distorting thin targets.
4. A 32 by 32 by 384 float feature grid is about 1.5 MiB. Publishing it through
   DDS at runtime would create unnecessary serialization and copying.
5. CLIP, open_clip, transformers, and ONNX Runtime are not installed in the
   working ROS Python environment.
6. The available RF-DETR wrapper targets a package whose current official
   runtime requirements exceed Python 3.8, PyTorch 1.13, and TensorRT 8.4.
7. Existing LiDAR tracklet limits are tuned for humans, including roughly
   1.2 metre width and depth limits.
8. TargetState already has map-validity and covariance fields, but the active
   selected-target tracker currently publishes position_map_valid as false.
9. Point-LIO and FAST-LIO integrations exist, and an external IMU node exists,
   but physical LiDAR-IMU extrinsics and field localisation behaviour are not
   fully validated.
10. The four existing human-tracking bags contain RGB, CameraInfo, LiDAR, TF,
    and TF-static, but not IMU, odometry, language queries, target labels, or
    active-search actions.
11. The current bags therefore validate sensor processing and human identity,
    not language search, world memory, or safe active rotation.
12. Nav2 packages may be installed on the host, but the repository has no
    validated Nav2 integration for this robot.
13. The existing human-search defaults combine a 120 degree sector, 4 second
    timeout, and 0.20 rad/s angular limit. The speed-time product permits only
    0.80 radian, or 45.8 degrees, before acceleration and settling margin, so
    those values do not constitute a completed 120 degree sweep and must not be
    copied as one. The existing target controller also limits angular
    acceleration to 0.25 rad/s squared, reducing the reachable envelope.

### 3.6 Scope corrections

- The live robot has an IMU node. The limitation is that current human-tracking
  bags do not record IMU or odometry.
- A TF topic in a bag is not proof of a stable map-to-base localisation chain.
- DINO feature magnitude is not a semantic segmentation mask.
- Public datasets cannot prove accuracy for the Helios-32, ZED2i, robot
  viewpoint, extrinsics, or Bunker motion.
- A VLM or VLA label does not remove the need for classical motion safety,
  calibrated uncertainty, or explicit stop behaviour.

## 4. Measured platform evidence

### 4.1 DINOv3 benchmark

The approved design uses a direct Jetson benchmark, not a desktop estimate.
The test used DINOv3 ViT-S+/16, a 512 by 512 input, the local Python-3.8
compatibility checkout, and the installed checkpoint.

- cold first inference: 2197.61 ms;
- 30 measured iterations after three warm-up iterations;
- preprocessing and host-to-GPU P95: 10.40 ms;
- model inference mean: 33.72 ms;
- model inference P95: 37.66 ms;
- feature copy P95: 1.47 ms;
- complete measured path mean: 43.48 ms;
- complete measured path P95: 46.60 ms;
- peak CUDA allocation: 178 MiB;
- CUDA reserved memory: 202 MiB.

Consequences:

- the model loads and warms during node activation, not when a query arrives;
- semantic inference targets 5 Hz rather than the full camera rate;
- the queue keeps only the latest frame;
- a second full image encoder is not run on every frame;
- an occasional high-resolution verification crop is allowed only while the
  complete path remains below the 150 ms P95 gate.

### 4.2 Recorded sensor rates

Across the four current bags:

- camera rate is approximately 14.39 to 14.98 Hz;
- LiDAR rate is approximately 19.20 to 20.02 Hz.

The existing controller and local trajectory planner run at 20 Hz. The motion
safety supervisor and velocity gate run at 50 Hz. A 5 Hz semantic path and
15 Hz minimum LiDAR path therefore have measurable scheduling margin.

## 5. Architecture alternatives

### 5.1 Architecture A: minimum viable system

Modules:

- existing detector or LiDAR clusters as proposals;
- one of two mutually exclusive image-scoring variants:
  - a frozen open-vocabulary image-text encoder scores a bounded batch of
    proposal crops; or
  - frozen DINO ROI features use a trained shallow projection into the cached
    text space and also provide appearance-association evidence;
- cached CLIP or SigLIP text features in the compatible scoring variant;
- rule-based 3D fusion;
- Kalman and feature-bank memory;
- passive reporting.

Data flow and required data:

- proposals -> bounded crop or ROI scoring -> rule-based 3D association ->
  feature-bank memory -> passive result;
- public open-vocabulary validation data and the robot calibration split select
  the scoring variant;
- no robot-specific training is required for the zero-shot variant; the DINO
  projection variant requires a small phrase-region training split;
- both variants use the same held-out robot bags for fair evaluation and are
  benchmark configurations, not two simultaneous per-frame image towers.

Training:

- zero-shot scoring first;
- optional small projection MLP;
- no foundation-backbone fine-tuning.

Jetson and ROS:

- lowest deployment risk;
- accepted only if the selected single-image-tower variant fits the 150 ms P95
  complete-path budget on Jetson;
- simple new ROS package and typed observations.

Risks:

- weak localisation of non-canonical text descriptions;
- proposal recall limits overall recall;
- simple fusion may not exploit cross-modal interactions.

Research, employment, and portfolio value:

- credible systems integration and baseline;
- limited novelty by itself.

Migration:

- every interface is retained by Architecture B;
- scoring and memory implementations are replaceable.

### 5.2 Architecture B: recommended research system

Modules:

- DINOv3 dense visual tokens;
- cached CLIP or SigLIP-family text tokens;
- a lightweight query-conditioned localisation head;
- object tokens from image regions and LiDAR clusters;
- concatenation, gated-fusion, and object-level cross-attention variants;
- calibrated uncertainty;
- multi-object semantic memory;
- high-level bounded active perception.

Training:

- frozen backbones initially;
- public phrase-region and outdoor pretraining;
- pseudo-labels from offline teachers;
- small robot-specific fusion and calibration set;
- optional parameter-efficient tuning only after ablation evidence.

Data flow and required data:

- query -> cached text tokens; RGB -> DINO dense tokens -> localisation head;
  LiDAR -> generic clusters and tracklets; object tokens -> fusion -> calibrated
  observation -> semantic memory -> passive result or bounded rotation request;
- public phrase-region data supports language grounding, public outdoor
  multimodal data supports representation and geometry experiments, and
  offline teachers provide versioned pseudo-labels;
- a small robot-specific calibration/training split fits the lightweight heads;
  site-, date-, and object-held-out robot bags provide acceptance evidence.

Jetson and ROS:

- one visual hot path;
- text encoded only when a query changes;
- internal tensor consumption inside one worker;
- target P95 below 150 ms and incremental CUDA reserve below 1.5 GiB;
- typed, model-independent ROS boundaries.

Risks:

- query-conditioned localisation requires a trained adapter;
- outdoor branch-like objects are underrepresented in public language data;
- cross-attention can overfit small robot datasets;
- runtime export and licensing must be checked per checkpoint.

Research, employment, and portfolio value:

- strong, measurable contributions in multimodal fusion, identity, uncertainty,
  object memory, active perception, ablation, and embedded deployment;
- supports truthful high-level VLA-like decision claims without claiming an
  unsafe end-to-end VLA.

Migration:

- implement Architecture A baselines first;
- replace scoring, fusion, and memory behind stable interfaces;
- add active rotation only after passive gates pass.

### 5.3 Architecture C: ambitious research branch

Modules:

- learned point-cloud or sparse-voxel backbone;
- dense vision-language model;
- temporal transformer or graph memory;
- learned active-search policy;
- later Nav2 goal selection or VLM planner.

Training:

- substantially larger multimodal data;
- more synthetic data and teacher inference;
- offline multi-GPU training;
- domain adaptation and policy evaluation.

Data flow and required data:

- RGB and learned LiDAR tokens -> temporal transformer or graph memory ->
  learned high-level search policy -> the same classical motion boundary;
- requires substantially more labelled or pseudo-labelled multimodal sequences,
  object identities, localisation epochs, and action-outcome records than the
  current workspace contains;
- policy learning additionally requires simulation or offline-safe action data
  and a separate real-robot safety-validation protocol.

Jetson and ROS:

- uncertain compatibility and latency;
- likely requires TensorRT engines, aggressive pruning, or a separate runtime;
- expected to exceed the first-release implementation and validation budget;
- no real-time claim is made until a complete exported path is measured on the
  Jetson against the same 150 ms and memory gates.

Risks:

- large data gap;
- difficult causal ablation;
- safety-validation burden;
- high chance of producing a demo that is less reliable than Architecture B.

Research, employment, and portfolio value:

- potentially high after Architecture B is established;
- low credibility if attempted before reproducible baselines and real-robot
  evidence.

Migration:

- learned encoders and policies plug into Architecture-B interfaces later.

### 5.4 Recommendation

Use Architecture B as the destination and Architecture A as the staged delivery
path. Architecture C is postponed. This choice maximises research depth without
sacrificing repeatability, Jetson feasibility, or real-robot safety.

## 6. Approved decision log

The following decisions are final for this specification:

1. Spatial memory is dual-mode:
   - world memory only when localisation health passes;
   - local-session memory otherwise, provided local odometry remains
     continuous;
   - no silent association across localisation epochs.
2. The first outdoor target is a fallen branch or path obstruction.
3. First-release active motion is in-place rotation only.
4. Language supports a free positive description of one target and visible
   attributes.
5. The model route combines DINOv3 visual features with a CLIP or SigLIP-family
   text representation.
6. The deployed hot path uses one visual backbone. A vision-language image
   tower is an offline teacher or low-rate benchmark, not a second per-frame
   encoder.
7. Output includes a 2D region, 3D centre, 3D extent, coordinate validity, and
   uncertainty.
8. Intermediate confidence triggers bounded rotation; unresolved evidence
   produces an uncertain or not-found result.
9. The guaranteed first-release range is 0.5 to 6 metres.
10. Eight to ten metres is reported as an extension test only.
11. First-release semantic output targets at least 5 Hz.
12. Existing human tracking, control, and safety defaults remain intact.

## 7. Target architecture and data flow

### 7.1 Runtime hot path

    SearchForObject goal
      -> validate and normalise query
      -> text encoder once
      -> cache query tokens and query version

    latest RGB frame
      -> aspect-preserving resize and padding
      -> DINOv3 once
      -> dense patch tokens
      -> query-conditioned localisation head
      -> image regions and scores

    LiDAR cloud
      -> ground removal
      -> permissive generic clusters and tracklets
      -> cluster geometry and uncertainty

    image regions + projected points + LiDAR object tokens
      -> multimodal association and fusion
      -> ObjectObservation3DArray
      -> temporal association
      -> semantic object memory
      -> confidence decision
      -> confirm, rotate for verification, or abstain

### 7.2 Offline and training path

    public datasets + robot manifests + pseudo-labels
      -> model-independent dataset adapters
      -> frozen feature extraction cache
      -> baseline and fusion training
      -> calibration and ablation
      -> export bundle
      -> Jetson compatibility and performance gates
      -> versioned runtime deployment

Training code does not import rclpy or depend on ROS callbacks. ROS inference
loads a versioned export bundle and contains no training loop.

## 8. Runtime modules

### 8.1 object_search_server

Responsibilities:

- accept, validate, cancel, and time out language searches;
- assign a query ID and query version;
- report progress and the final status;
- ensure only one active first-release query;
- reject a second goal as busy instead of implicitly pre-empting the first.

Dependencies:

- semantic perception health;
- memory query API;
- active-search coordinator.

### 8.2 semantic_perception_worker

Responsibilities:

- own the image backbone, text encoder, localisation head, and GPU tensors;
- keep a latest-frame queue of depth one;
- use a timestamp-based target-rate scheduler rather than fixed frame-count
  skipping;
- preserve image aspect ratio;
- run text encoding only on query change;
- publish not-ready health until model loading and warm-up complete;
- generate image candidates and score components;
- publish compact semantic regions, not raw embeddings.

Dependencies:

- RGB and CameraInfo;
- model export bundle;
- active query.

### 8.3 semantic_geometry_fusion

Responsibilities:

- pair image candidates and point clouds by sensor timestamp;
- look up camera-LiDAR and robot transforms;
- project points and associate generic clusters;
- estimate centre, extent, covariance, measurement age, and evidence flags;
- retain 2D-only candidates when geometry is inadequate.

Dependencies:

- SemanticRegionArray;
- PointCloud2;
- CameraInfo;
- TF;
- generic LiDAR clusters or tracklets.

### 8.4 object_memory

Responsibilities:

- preserve global semantic object IDs separately from raw detections, camera
  tracks, and LiDAR tracklets;
- associate observations using geometry, appearance, semantics, time, and
  uncertainty;
- maintain world or local-session memory;
- manage tentative, confirmed, occluded, prediction, lost, and stale states;
- expose candidate, duplicate, last-seen, and inspection queries.

### 8.5 active_search_coordinator

Responsibilities:

- evaluate confidence and health;
- choose confirm, bounded rotate, not-found, uncertain, cancelled, or fault;
- limit rotation angle, duration, and angular speed;
- terminate rather than automatically resume after a safety or health fault.

### 8.6 search_motion_bridge

Responsibilities:

- translate a high-level search rotation intent into the existing classical
  decision and control boundary;
- force forward_permitted to false;
- force maximum linear speed and requested linear velocity to zero;
- expose no direct learned-to-cmd_vel connection.

### 8.7 semantic_search_evaluator

Responsibilities:

- collect versioned metrics;
- compare baselines on an identical manifest;
- produce JSON reports and machine-readable pass/fail gates;
- extend existing identity, rate, synchronisation, and position-jump reports.

### 8.8 Replaceable internal module contracts

Runtime implementations are selected by a versioned configuration registry.
The interfaces below are model-independent and have matching online and offline
adapters:

- ImageEncoder: an aspect-preserved image tensor, valid-token mask, and
  preprocessing transform -> dense or ROI visual tokens plus encoder metadata;
- TextEncoder: normalised query text and query version -> cached global and
  token-level language representations plus encoder metadata;
- ProposalGenerator: valid image/text tokens and optional projected LiDAR
  support -> bounded SemanticRegion candidates with provenance;
- LiDAREncoder: timestamped generic cluster or tracklet geometry -> one bounded
  object token per physical candidate with covariance and physical IDs;
- CrossModalAssociator: image candidates, LiDAR candidates, timestamps, and
  transforms -> scored candidate edges, rejected-edge reasons, and ambiguity;
- FusionModule: associated object tokens, query representation, and observation
  health -> separated semantic, geometry, identity, and overall evidence;
- TemporalMemory: fused observations, pose health, and localisation epoch ->
  identity-preserving object updates and a read-only memory snapshot;
- TaskRelevanceScorer: query representation and memory snapshot -> ranked
  object IDs with relevance and inspection priority;
- UncertaintyEstimator: component evidence, calibration version, and available
  shift indicators -> calibrated confidence, uncertainty, and abstention flag;
- ActivePerceptionPolicy: ranked candidates, health snapshot, action budget, and
  current search state -> a bounded high-level intent and optional target
  bearing, never continuous velocity;
- ROS adapters: typed ROS messages and sensor inputs <-> the contracts above;
  they contain no training loop and expose no model-specific tensor topic.

The required configuration keys are `image_encoder`, `text_encoder`,
`proposal_generator`, `lidar_encoder`, `cross_modal_associator`,
`fusion_module`, `memory_module`, `task_relevance_scorer`,
`uncertainty_estimator`, `active_perception_policy`, and `export_manifest`.
Each resolves to a registered implementation plus a versioned parameter block;
unknown implementations or incompatible manifest versions fail activation.

The registry preserves these comparison families without promising all of them
in the first release:

- image encoder: DINOv3 primary, then DINOv2, CLIP/SigLIP image towers,
  grounding/segmentation-derived features, a lightweight custom encoder, or an
  exported TensorRT implementation;
- LiDAR encoder: handcrafted geometry first, then PointNet-family, range-image,
  sparse-voxel, or other learned object encoders;
- fusion: concatenation MLP, gated fusion, object-level cross-attention, then
  temporal or confidence-aware variants;
- memory: Kalman plus feature bank first, then learned temporal, transformer,
  or graph memory;
- policy: rule-based priority first, then learned high-level classifier or
  planner behind the same no-continuous-velocity boundary.

Only one registered image encoder is in the deployed per-frame hot path. Every
candidate replaces the current implementation for a comparable run; registry
support is not permission to stack multiple full backbones.

Tensor shape, dtype, coordinate convention, checkpoint checksum, preprocessing
version, and calibration version are declared in the export manifest. A module
replacement is valid only when contract, replay, accuracy, latency, and memory
tests pass; changing a backbone does not change the external ROS messages.

The first-release memory-query interface is internal to the search server and
supports: all objects matching the active query, highest-priority candidate,
not-yet-inspected candidates, objects last seen in a bounded region, uncertain
candidates requiring another observation, possible duplicates, and the current
sensor-supported versus prediction-only state. A public ROS memory-query
service is deferred until its access and persistence semantics have a separate
approved specification.

## 9. ROS contracts

All new runtime topics use the /semantic_search namespace. Backbones and fusion
modules may change without changing these external contracts.

### 9.1 SearchForObject.action

Goal fields:

- query_text;
- timeout;
- allow_rotation;
- maximum_rotation_angle;
- client_request_id.

Feedback fields:

- query_id;
- phase;
- elapsed time;
- searched angle;
- candidate count;
- best candidate score;
- current reason.

Result fields:

- status: confirmed, not found, uncertain, cancelled, safety rejected, sensor
  unavailable, model unavailable, or timeout;
- query_id;
- selected global object ID;
- selected ObjectObservation3D;
- reason and evidence summary.

The wire-level action uses the following concrete ROS types and sentinels:

- goal: `string query_text`, `builtin_interfaces/Duration timeout`,
  `bool allow_rotation`, `float32 maximum_rotation_angle`, and
  `string client_request_id`;
- feedback: `uint64 query_id`, `uint8 phase`,
  `builtin_interfaces/Duration elapsed`, `float32 searched_angle`,
  `uint32 candidate_count`, `float32 best_candidate_score`, and
  `string current_reason`;
- result: `uint8 status`, `uint64 query_id`,
  `bool selected_object_valid`, `uint64 selected_global_object_id`,
  `bool selected_observation_valid`,
  `ObjectObservation3D selected_observation`, and
  `string evidence_summary`;
- query ID zero is reserved as invalid; a zero goal timeout or maximum angle
  requests the configured bounded default, never an unbounded search;
- the first-release configured default and hard maximum rotation angle are both
  0.50 radian; a smaller positive goal value further restricts the action;
- the first-release default overall action timeout is 8 seconds and the hard
  maximum is 10 seconds; a smaller positive goal timeout further restricts the
  action;
- `phase` constants are INITIALISING, PASSIVE_OBSERVATION,
  ROTATION_VERIFICATION, FINALISING, and TERMINAL;
- `status` constants are CONFIRMED, NOT_FOUND, UNCERTAIN, CANCELLED,
  SAFETY_REJECTED, SENSOR_UNAVAILABLE, MODEL_UNAVAILABLE, and TIMEOUT.

The server rejects empty queries, unsupported first-release language operators,
non-finite limits, and requests outside configured safety bounds.

### 9.2 SemanticRegion and SemanticRegionArray

Each region contains:

- header and original image dimensions;
- query ID, query version, and observation ID;
- bounding box and optional compressed mask;
- DINO appearance score;
- language score;
- localisation-head score;
- fused region score;
- preprocessing transform;
- model and checkpoint versions.

### 9.3 ObjectObservation3D and ObjectObservation3DArray

Each observation contains:

- associated semantic region;
- base-frame position and validity;
- local-session position and validity;
- world position and validity;
- 3D extent;
- extent-is-partial flag for sparse or one-sided LiDAR support;
- 3 by 3 position covariance;
- range and bearing;
- projected point count;
- associated LiDAR cluster or tracklet ID;
- image, LiDAR, TF, and pose ages;
- evidence-source flags;
- identity, semantic, geometry, and overall confidence.

### 9.4 TrackedSemanticObject and TrackedSemanticObjectArray

Each object contains:

- global semantic object ID;
- active query IDs and relevance;
- raw observation IDs;
- current camera track ID when available;
- current LiDAR tracklet ID when available;
- memory mode and localisation epoch;
- position, velocity, extent, and covariance;
- appearance, geometry, and semantic feature summaries;
- bounded top-k semantic labels and description evidence with model provenance;
- first-seen, last-seen, last-camera, last-LiDAR, observation-count, and
  source-history fields;
- bounded position and uncertainty history with timestamps;
- association, visibility, re-identification, inspection, and stale states;
- duplicate and uncertainty scores;
- task relevance and inspection priority;
- anomaly score and validity fields reserved for later experiments, not used as
  first-release acceptance gates.

### 9.5 SearchMotionIntent

Fields include:

- query ID;
- state;
- target bearing;
- maximum rotation angle;
- maximum angular speed;
- deadline;
- rotation permission;
- explicit forward permission, which is always false in this release;
- reason.

Raw DINO, text, and attention tensors are internal implementation details and
are never ROS interface requirements.

## 10. Visual-language perception

### 10.1 Preprocessing

- preserve aspect ratio;
- pad to a multiple of the backbone patch size;
- record scale and padding;
- map every region and mask back to original image coordinates;
- exclude padded tokens from scoring.

### 10.2 Candidate generation

The initial proposal set is the union of:

- query-conditioned connected regions from the lightweight localisation head;
- permissive generic LiDAR clusters projected into the image;
- optional image-independent regions used by simple baselines.

The system does not require a heavy learned LiDAR backbone. Human-specific
dimension rejection is disabled in the semantic-search configuration. Loose
physical and sensor-range bounds remain to reject impossible measurements.

### 10.3 Text model selection

CLIP and SigLIP-family candidates are compared using:

- phrase-region ranking and target recall;
- English and required operational-language prompts;
- query initialisation latency;
- memory usage;
- Python-3.8 compatibility or exportability;
- supported operations in TensorRT 8.4 or the selected isolated runtime;
- checkpoint and code licence;
- redistribution constraints.

The selected text model is the highest-recall candidate that passes all
compatibility, licence, memory, and complete-pipeline latency gates. SigLIP 2 B
is the first benchmark candidate because of its multilingual and localisation
capabilities. A smaller CLIP-family text tower is the fallback candidate. The
choice is a measured Phase-1 output, not an unbounded implementation choice.
The ROS environment is not upgraded in place to accommodate a model; only a
compatible adaptation, export, isolated runtime, or replacement candidate may
pass selection.

### 10.4 Lightweight localisation head

The initial trainable head:

- projects DINO patch tokens to a compact multimodal dimension;
- projects cached text tokens to the same dimension;
- computes query-conditioned patch scores;
- optionally uses one or two shallow cross-attention blocks;
- predicts a dense relevance map and candidate confidence.

DINOv3 and the text encoder remain frozen initially. Feature concatenation,
gating, and cross-attention share the same object-token input and output
contracts.

### 10.5 High-resolution verification

When the full-frame result is intermediate and a bounded region is available,
the worker may run one aspect-preserving verification crop. It cannot create an
unbounded crop loop and cannot violate the 150 ms complete-path P95 gate.

## 11. LiDAR and 3D fusion

### 11.1 Pairing

- pair by source timestamp, not callback arrival time;
- reject image-cloud offsets over 80 ms;
- reject data older than configured maximum age;
- do not apply out-of-sequence state corrections;
- clear temporal state on bag timestamp rollback.

### 11.2 Geometric estimation

For each image region:

1. project valid LiDAR points at the observation timestamp;
2. select points inside the valid image region or mask;
3. combine them with compatible generic clusters;
4. reject depth outliers and inconsistent components;
5. estimate robust median centre and 3D extent;
6. propagate point scatter, timing, and transform uncertainty into covariance.

Insufficient points produce a 2D-only observation. The system never invents a
3D position from an unsupported image score.

ZED stereo depth is an optional secondary geometry source behind the same
observation contract. Its measurements are reported with separate provenance
and are not counted as independent confirmation of LiDAR evidence.

Field acceptance requires measured camera-LiDAR calibration, recorded
calibration provenance, and a residual check. A static fallback transform may
support visualisation of old bags, but cannot support accepted field
localisation results.

### 11.3 Fusion scores

The fusion input includes:

- language-region relevance;
- DINO appearance stability;
- LiDAR point and cluster support;
- geometric plausibility;
- temporal continuity;
- timestamp and transform health;
- localisation health.

The output separates semantic, geometry, identity, and overall confidence.
Thresholds are calibrated on the validation split rather than copied from the
human tracker.

### 11.4 Confirmation

The first-release default requires consistent evidence in at least two of the
latest three eligible observations. Candidate switching uses a margin and
hysteresis. Conflicting candidates become ambiguous rather than causing an
identity switch.

## 12. Multi-object 3D semantic memory

### 12.1 Identifier invariant

The following identifiers are never aliases:

- query ID;
- raw observation ID;
- camera track ID;
- LiDAR cluster ID;
- LiDAR tracklet ID;
- global semantic object ID;
- localisation epoch ID.

Every association stores its evidence and confidence.

### 12.2 Memory modes

World mode:

- requires fresh, continuous map-to-base pose;
- requires acceptable localisation covariance;
- requires no detected reset or jump;
- supports persistent world position and later global queries.

Local-session mode:

- used when global localisation is unavailable but odom-to-base and IMU yaw are
  continuous;
- stores positions in an epoch-anchored local frame;
- supports current-session association only;
- is not silently promoted to world memory.

Observation-only degradation:

- used when continuous local pose is unavailable;
- returns current 2D or instantaneous 3D evidence;
- writes no actionable spatial memory;
- permits no active motion.

### 12.3 Epoch changes

Create a new epoch on:

- localisation reset or large pose jump;
- covariance or health-gate failure requiring reset;
- TF discontinuity;
- bag timestamp rollback;
- explicit localisation reset event.

Objects from different epochs are not automatically associated. Reprojection
requires a separately verified transform and an auditable offline or explicit
migration operation.

### 12.4 Association

Within one epoch, association combines:

- Mahalanobis position distance;
- extent compatibility;
- DINO appearance similarity;
- language and semantic similarity;
- LiDAR geometry similarity;
- elapsed time and motion model;
- top-two association margin.

Static, moving, occluded, split-cluster, merged-cluster, prediction-only, and
stale cases remain explicit states.

## 13. Active-search state machine

    IDLE
      -> query accepted
    OBSERVE
      -> high confidence: CONFIRMED
      -> intermediate confidence: VERIFY_GATE
      -> low confidence or timeout: ABSTAIN
      -> fault: FAULT_STOP
    VERIFY_GATE
      -> health and safety pass: ROTATE_VERIFY
      -> otherwise: SAFETY_REJECTED
    ROTATE_VERIFY
      -> consistent evidence: CONFIRMED
      -> angle or time exhausted: ABSTAIN
      -> cancel or fault: FAULT_STOP

### 13.1 Rotation bounds

- first-release motion is one candidate-directed verification rotation, not a
  multi-leg left-right sweep;
- maximum requested yaw offset: 0.50 radian, approximately 29 degrees;
- maximum rotation sub-budget: 4 seconds;
- maximum angular speed: 0.20 rad/s;
- requested and permitted linear speed: exactly zero.

The 0.50-radian release bound is deliberately below both the 0.80-radian naive
speed-time product and the approximately 0.64-radian symmetric
acceleration-limited envelope implied by the existing 0.25 rad/s-squared ramp.
This reserves time for proportional slowdown, sensor/control delay, and
settling. The coordinator stops on measured yaw tolerance, deadline,
cancellation, or any health/safety fault. The effective commanded angle is the
minimum of the goal request, the 0.50-radian release bound, and the angle still
reachable within the remaining deadline. Phase 5 may lower this bound after
motion profiling; raising it requires a reviewed update with measured
acceleration, stopping, and overshoot evidence. A 120 degree exploratory sweep
is a later behavior requiring separately coherent speed, acceleration, dwell,
and timeout bounds.

The default eight-second action budget includes initial passive evidence,
optional rotation, post-rotation confirmation, and terminal reporting. Model
loading and warm-up are activation work and never consume a goal budget. The
rotation sub-budget cannot borrow time beyond four seconds, and every phase is
also bounded by the remaining overall goal deadline.

### 13.2 Required gates

Rotation requires:

- fresh and usable camera;
- fresh and usable LiDAR;
- fresh IMU and odometry yaw or equivalent validated angular feedback;
- required TF availability;
- measured linear speed within the configured stationary deadband;
- a healthy base;
- no emergency stop;
- no RC override;
- no blocked safety state;
- an armed safety supervisor;
- fresh control and safety feedback.

Any failing gate ends the search and publishes a zero intent. The search does
not automatically resume after a fault.

First-release active trials run only in a surveyed, level, controlled area with
an operator holding RC override and emergency-stop access. The top-mounted
LiDAR does not establish drop-off safety or reliable coverage of every very low
obstacle, so those conditions remain outside the autonomous safety claim.

### 13.3 Classical control boundary

The learned system selects only a semantic action and target bearing. Existing
classical nodes retain responsibility for:

- behaviour transitions;
- angular command generation;
- local obstacle checks;
- footprint collision checks;
- watchdogs;
- emergency stop;
- RC override;
- final cmd_vel output.

## 14. Data and training strategy

### 14.1 Public pretraining and evaluation

Recommended sources and roles:

- WildScenes: natural-environment image, LiDAR, semantic, calibration, and pose
  data for outdoor representation and 2D-3D transfer;
- RELLIS-3D: multimodal off-road semantic evaluation;
- TartanDrive 2.0: off-road multimodal self-supervision;
- LVIS: long-tail categories and instance masks;
- Flickr30k Entities and RefCOCO-family data: phrase-region grounding;
- SegmentMeIfYouCan: road-obstacle and anomaly segmentation.

Public data does not prove robot-specific geometry or safety.

### 14.2 Synthetic and weak supervision

Permitted uses:

- language paraphrases and attribute variants;
- pasted or rendered obstruction examples with explicit provenance;
- geometric and photometric augmentation;
- teacher-generated regions and descriptions;
- hard-negative mining from vegetation, shadows, path edges, and logs;
- temporal consistency pseudo-labels.

Pseudo-label confidence and teacher version are stored. Pseudo-labels never
enter the held-out robot test split.

### 14.3 Robot-specific data

The first-release test distribution covers:

- 0.5, 1, 2, 4, and 6 metre guaranteed distances;
- 8 and 10 metre extension distances;
- at least ten physical branches or obstructions;
- at least three sites or lighting sessions;
- front, side, partial occlusion, and vegetation overlap;
- hard negatives;
- pre-rotation and post-rotation identity.

Splits are by site, date, and physical object. Adjacent frames from one
sequence cannot cross splits.

New bags record:

- RGB and CameraInfo;
- LiDAR;
- TF and TF-static;
- IMU;
- odometry and localisation health;
- language-query events;
- typed semantic observations and memory states;
- search decisions;
- safety states and planned/safe commands;
- ground-truth event or annotation identifiers.

Raw feature tensors are not recorded by default.

### 14.4 Training order

1. freeze DINOv3 and the text encoder;
2. establish zero-shot and shallow baselines;
3. train only projection, localisation, fusion, and uncertainty heads;
4. compare concatenation, gating, and cross-attention;
5. add parameter-efficient tuning only with a measured validation gain;
6. unfreeze final backbone blocks only after the adapter limit is demonstrated;
7. export and re-run all Jetson gates.

### 14.5 Training and deployment hardware boundary

- foundation-model inference used for teaching, adapter training, and fusion
  training runs offline on a separate workstation or compute service when
  needed;
- the design does not require one specific training GPU, but every experiment
  manifest records GPU model, count, driver, CUDA, framework, precision, seed,
  wall time, and peak memory;
- the Jetson remains the authoritative inference and acceptance platform;
- absence of larger training hardware does not block the frozen zero-shot and
  rule-based baselines, but it does block claims about unrun fine-tuning or
  Architecture-C training;
- no training dependency is installed by upgrading the working ROS runtime in
  place.

## 15. Required baselines and ablations

Every baseline uses the same manifests, distance ranges, proposals where
applicable, and evaluation code.

1. Camera-only pretrained fixed-category detector, no LiDAR and no memory.
2. LiDAR proposals with fixed geometric scoring, no language.
3. Open-vocabulary language-camera model, no LiDAR fusion.
4. Camera, LiDAR, and language feature concatenation, no attention.
5. Camera, LiDAR, and language object-level cross-attention, no temporal
   memory.
6. Cross-attention plus 3D semantic object memory.
7. Optional incremental experiment: memory plus active rotation.

Required ablations disable one item at a time:

- DINO features;
- language embedding;
- LiDAR geometry;
- temporal memory;
- uncertainty;
- dynamic token or candidate selection;
- active observation.

The cross-attention design is retained only if it improves task-relevant
metrics over concatenation at an acceptable compute cost.

## 16. Evaluation and acceptance gates

### 16.1 Minimum held-out coverage

- five guaranteed distances;
- ten distinct positive objects;
- three sites or lighting sessions;
- at least 30 positive trials;
- at least 30 hard-negative trials;
- at least 20 bounded-rotation trials.

### 16.2 First-release hard gates

Perception:

- task-relevant candidate recall at 0.5 to 6 metres: at least 90 percent;
- confirmed-result precision: at least 85 percent;
- hard-negative false confirmation: at most 5 percent.

Geometry:

- median range error: at most 0.30 metre;
- P95 range error: at most 0.60 metre;
- P95 bearing error: at most 7 degrees.

Identity and memory:

- no unconfirmed global-object switch in controlled trials;
- memory duplicate rate: at most 5 percent;
- correct re-association after FOV exit and re-entry: at least 80 percent.

Uncertainty:

- high-confidence false confirmation: at most 5 percent;
- expected calibration error: at most 0.10 on at least 500 temporally
  subsampled held-out observations, rather than only trial-level results.

Efficiency:

- semantic output: at least 5 Hz;
- complete semantic-path latency P95: at most 150 ms;
- LiDAR processing path: at least 15 Hz;
- image-cloud offset: at most 80 ms;
- incremental CUDA reserved memory: at most 1.5 GiB;
- no sustained queue growth;
- stable operation for at least 30 minutes.

Active search and safety:

- positive bounded-rotation trials ending in correct confirmation: at least
  80 percent;
- negative bounded-rotation trials ending in false confirmation: at most
  5 percent;
- linear.x is exactly zero throughout every search;
- measured absolute yaw displacement never exceeds the accepted commanded
  bound by more than 3 degrees;
- emergency stop, RC override, stale sensor, or safety fault produces zero
  output within 200 ms;
- every terminal action state produces zero angular intent within 200 ms;
- no motion when required health evidence is missing.

Any motion-safety failure fails the release regardless of average accuracy.
Thresholds are frozen before final-test inspection. A threshold may change only
from independent calibration evidence recorded before the final run.

### 16.3 Verification ladder

1. pure unit and contract tests;
2. offline images and public datasets;
3. deterministic robot-bag replay;
4. full state machine with drive output disabled;
5. controlled rotation with the safety chain armed;
6. held-out outdoor 0.5 to 6 metre demonstration;
7. separately reported 8 to 10 metre extension.

Every run emits a versioned JSON report containing data-manifest, model,
configuration, commit when available, timing, resource, accuracy, identity, and
safety fields.

### 16.4 Mandatory reported metrics

In addition to the hard gates, comparable reports include:

- perception: candidate recall, confirmed precision, false-positive rate,
  phrase-region ranking, range and bearing error, track continuity, target-loss
  rate, re-identification accuracy, duplicate rate, and uncertainty curves;
- efficiency: camera ingest and selected-frame rate, LiDAR rate, per-module
  mean/P50/P95/maximum latency, CPU usage, system RAM, CUDA allocation and
  reserve, input and retained token counts, dropped frames, queue depth, Jetson
  power mode, power draw, temperature, and throttle events;
- active search: time to first candidate, time to confirmation, searched angle,
  rotation duration, active-observation count, false-stop count, safety
  interventions, cancellation outcome, and reacquisition outcome;
- experimental context: public versus robot dataset, site/object split, model
  and export versions, calibration provenance, software environment, and random
  seed.

These are mandatory report fields even when they are not first-release hard
gates. Public-dataset results, extension-range results, and report-only metrics
cannot replace any failed hard gate in Section 16.2. Existing 20 Hz
controller/planner and 50 Hz safety/gate diagnostics must remain within their
current rate gates, and an accepted field run records no thermal-throttle event.

## 17. Implementation stages

### Phase 0: contracts, replay, and profiling

Deliver:

- new messages and SearchForObject action;
- data and annotation manifest schema;
- semantic-search bag-recording guide;
- model-independent evaluator;
- passive launch skeleton;
- baseline latency and resource report.

Gate:

- workspace builds;
- existing human-tracking launch and replay remain unchanged;
- contracts round-trip;
- no motion topics are produced.

### Phase 1: passive language baseline

Deliver:

- aspect-preserving DINO runtime;
- text-model benchmark and deterministic selection;
- cached text encoding;
- simple region proposal and open-vocabulary scoring;
- camera-only and LiDAR-only baselines.

Gate:

- at least 5 Hz semantic output;
- no feature-tensor DDS path;
- baseline 1 to 3 reports;
- model licence and export review complete.

### Phase 2: generic 3D fusion

Deliver:

- independent semantic-search LiDAR configuration;
- image-region and generic-cluster union;
- robust 3D centre, extent, covariance, and validity;
- 2D-only degradation.

Gate:

- timing, pairing, range, and bearing gates pass;
- human-tracking parameters remain unchanged.

### Phase 3: multi-object semantic memory

Deliver:

- identifier-separated object records;
- local-session and world modes;
- epoch handling;
- appearance and geometry re-association;
- memory queries.

Gate:

- no cross-epoch association;
- identity, duplicate, and re-entry gates pass.

### Phase 4: trainable fusion and uncertainty

Deliver:

- concatenation, gated, and cross-attention variants;
- lightweight localisation head;
- calibration and abstention;
- baselines 4 to 6 and required ablations.

Gate:

- selected model beats simpler baselines on predeclared metrics;
- complete Jetson path remains within resource gates.

### Phase 5: active rotation

Deliver:

- action server;
- active-search state machine;
- search-motion bridge;
- health, safety, timeout, cancel, and fault launch tests;
- controlled outdoor rotation evaluation.

Gate:

- every safety hard gate passes;
- active-search success gate passes;
- forward output remains exactly zero.

### Phase 6: deployment optimisation

Deliver:

- candidate and token pruning;
- batching and memory profiling;
- optional distillation;
- ONNX, TensorRT, or isolated-runtime export when compatible;
- 30-minute stability and power report.

Gate:

- all first-release gates pass on the exported runtime;
- no in-place platform upgrade.

## 18. Repository impact

### 18.1 New runtime package

Create:

- src/track_robot_semantic_search/package.xml
- src/track_robot_semantic_search/setup.py
- src/track_robot_semantic_search/setup.cfg
- src/track_robot_semantic_search/resource/track_robot_semantic_search
- src/track_robot_semantic_search/track_robot_semantic_search/
- src/track_robot_semantic_search/config/
- src/track_robot_semantic_search/launch/
- src/track_robot_semantic_search/rviz/
- src/track_robot_semantic_search/test/

The Python package contains focused libraries and thin ROS adapters rather than
one large callback file.

### 18.2 Offline research package

Create:

- research/semantic_search/datasets/
- research/semantic_search/models/
- research/semantic_search/training/
- research/semantic_search/evaluation/
- research/semantic_search/export/
- research/semantic_search/configs/
- research/semantic_search/tests/

This tree is not discovered by colcon and does not import ROS inference nodes.

### 18.3 Interfaces

Add under src/track_robot/track_robot_interfaces:

- action/SearchForObject.action
- msg/SemanticRegion.msg
- msg/SemanticRegionArray.msg
- msg/ObjectObservation3D.msg
- msg/ObjectObservation3DArray.msg
- msg/TrackedSemanticObject.msg
- msg/TrackedSemanticObjectArray.msg
- msg/SearchMotionIntent.msg

Modify only:

- src/track_robot/track_robot_interfaces/CMakeLists.txt
- src/track_robot/track_robot_interfaces/package.xml

### 18.4 Targeted reuse changes

Potential backward-compatible modification:

- src/track_robot_perception/track_robot_perception/dinov3_runtime.py:
  add aspect-preserving preprocessing and coordinate metadata while preserving
  the existing function path.

Prefer a separate semantic-search LiDAR configuration before modifying:

- src/track_robot/track_robot_lidar_tracking/src/lidar_tracklet_manager_node.cpp.

Do not change by default:

- existing human messages;
- human_tracking_simplified.launch.py;
- human-tracking YAML defaults;
- current human behavior-tree path;
- current safety and velocity-gate defaults.

### 18.5 Documentation and data

Create:

- rosbags/semantic_search/semantic_search_rosbag_guide.md
- rosbags/semantic_search/manifests/
- docs/superpowers/specs/

Large bags and model weights remain external artifacts referenced by checksums
and manifests.

## 19. Testing strategy

### 19.1 Unit tests

- aspect-ratio resize, padding, and inverse coordinate mapping;
- padded-token exclusion;
- query normalisation and caching;
- text and image adapter shapes;
- candidate connected components and mask compression;
- robust depth, extent, and covariance;
- identifier invariants;
- association gating and hysteresis;
- memory mode and epoch transitions;
- uncertainty calibration;
- action validation and status mapping.

### 19.2 Launch and integration tests

- SearchForObject success, not-found, uncertainty, cancel, and timeout;
- missing camera, LiDAR, TF, IMU, and odometry;
- safety disarmed, blocked, RC override, and emergency stop;
- timestamp rollback and localisation jump;
- search bridge never permits forward motion;
- over-limit rotation requests are rejected and terminal states publish zero
  angular intent;
- the bridge supplies every finite geometry field required by the existing
  controller while keeping target distance and linear limits safely at zero;
- existing human stack starts without semantic-search nodes.

### 19.3 Replay tests

- same bag at 0.5, 1.0, and 2.0 replay rate;
- logical object sequence and association evidence;
- maximum 80 ms pairing;
- no unconfirmed identity switch;
- deterministic report fields;
- no temporal leakage across loop boundaries.

### 19.4 Performance tests

- cold load and warm-up;
- per-module mean, P50, P95, and maximum latency;
- CUDA allocation and reserve;
- CPU and system memory;
- power from Jetson telemetry;
- dropped-frame and queue-depth counts;
- control and safety loop-rate preservation;
- 30-minute stability.

## 20. Error handling

- Model load failure: publish model-unavailable health and reject queries.
- CUDA out-of-memory: stop inference, clear pending search, emit fault, and
  request no motion.
- Stale image or cloud: reject the observation.
- Missing TF: retain a 2D candidate only; do not create 3D memory or rotate.
- Missing local pose: observation-only degradation.
- Global localisation failure: close the world epoch and use local-session mode
  only if local odometry remains healthy.
- Timestamp rollback: clear temporal queues and create a new epoch.
- Candidate conflict: mark ambiguous and verify or abstain.
- Action cancel or timeout: stop rotation and return a terminal result.
- Safety or RC fault: publish zero intent, terminate the search, and require a
  new explicit action after recovery.

## 21. Migration and rollback

### 21.1 Compatibility

- new topics live under /semantic_search;
- new launches are opt-in and default off;
- the existing human stack does not subscribe to new topics;
- the motion bridge is the only connection to the existing control chain;
- model names and implementations are configuration values behind stable
  interfaces.

### 21.2 Rollback

- disable the semantic-search launch to restore the existing system;
- select concatenation if gating or cross-attention fails;
- select passive mode if active rotation fails;
- select the simpler compatible text encoder if the preferred model fails
  runtime or licence gates;
- use an offline teacher or isolated runtime instead of upgrading the robot;
- retain the last passing export bundle and configuration manifest.

No rollback requires replacing the existing human-tracking launch or changing
the platform environment.

## 22. Risks and mitigations

Thin branch missed by image patches:

- preserve aspect ratio;
- use query-conditioned dense scores and LiDAR proposal union;
- allow one bounded high-resolution verification crop;
- collect hard examples at every guaranteed distance.

Sparse or merged LiDAR geometry:

- keep 2D-only state;
- use permissive candidates and robust components;
- never invent unsupported depth;
- calibrate at 0.5, 1, 2, 4, and 6 metres.

Very low obstacle or drop-off outside top-LiDAR coverage:

- limit first-release motion trials to surveyed, level, controlled areas;
- require an operator with RC override and emergency-stop access;
- make no drop-off-safety claim;
- require downward-looking safety sensing before unsupervised operation near
  edges or steps.

Public-to-robot domain gap:

- freeze general backbones;
- train small heads;
- split robot data by scene and object;
- report public and robot metrics separately.

Small-data overfitting:

- require simple baselines and ablations;
- use site-held-out validation;
- add capacity only after measured need.

Runtime incompatibility:

- do not upgrade the host in place;
- test local adaptations, exports, containers, or replacement models;
- include runtime and licence gates in model selection.

Localisation discontinuity:

- dual-mode memory;
- explicit epochs;
- no automatic cross-epoch association or promotion.

Unsafe learned output:

- high-level intent only;
- classical controller and safety chain;
- zero-forward invariant;
- fail-closed health gates.

Unclear research claim:

- preserve experiment manifests;
- compare all baselines;
- report failures and resource costs;
- claim only capabilities that pass real-robot gates.

## 23. Definition of done for the first release

The first release is complete only when:

1. all Phase-0 through Phase-5 artifacts exist and are versioned;
2. all six required baselines have comparable reports;
3. the selected model passes every perception, geometry, identity,
   uncertainty, performance, and safety hard gate;
4. the held-out 0.5 to 6 metre outdoor demonstration passes;
5. the existing human-tracking regression remains green;
6. no in-place JetPack, ROS, Python, PyTorch, CUDA, or TensorRT upgrade was
   required;
7. documentation includes build, replay, field-test, rollback, model-version,
   and data-manifest procedures;
8. unsupported capabilities are not presented as completed.

Phase 6 is required before claiming optimised Jetson deployment, but an
unoptimised first release may pass earlier phases if it already meets all
resource gates.

## 24. Specification decomposition and next step

This master design is too large for one safe implementation plan. After the
user reviews this written specification, implementation planning is decomposed
in this order:

1. contracts, replay, data manifests, localisation health, and profiling;
2. passive language-conditioned perception and model benchmark;
3. generic 3D fusion, semantic memory, training, baselines, and uncertainty;
4. bounded active rotation and deployment optimisation.

Each implementation plan has its own tests, rollback, and approval boundary.
The next workflow step after written-spec approval is a detailed plan for item
1 only. No implementation begins from this design document alone.

## 25. Primary evidence and references

Local evidence:

- src/track_robot_perception/README.md
- src/track_robot_perception/docs/human_tracking_progress.md
- src/track_robot_perception/docs/human_tracking_reinforcement.md
- src/track_robot_perception/docs/human_tracking_fusion_refactor_log_2026-07-09.md
- rosbags/human_tracking_rosbag_test_guide.md
- src/track_robot/track_robot_interfaces/msg/TargetState.msg
- src/track_robot/track_robot_interfaces/msg/FollowDecision.msg
- src/track_robot/track_robot_lidar_tracking/config/lidar_tracklets.yaml
- src/track_robot/track_robot_decision/config/outdoor_decision.yaml
- src/track_robot/track_robot_safety/docs/obstacle_safety.md
- src/track_robot_perception/config/point_lio_rshelios.yaml
- src/track_robot_perception/config/fast_lio_rshelios.yaml

External primary sources:

- DINOv3: https://github.com/facebookresearch/dinov3
- ROS 2 Foxy intra-process communication:
  https://docs.ros.org/en/foxy/Tutorials/Demos/Intra-Process-Communication.html
- SigLIP: https://arxiv.org/abs/2303.15343
- SigLIP 2: https://arxiv.org/abs/2502.14786
- WildScenes: https://csiro-robotics.github.io/WildScenes/
- RELLIS-3D: https://github.com/unmannedlab/RELLIS-3D
- TartanDrive 2.0: https://theairlab.org/TartanDrive2/
- LVIS: https://www.lvisdataset.org/
- Flickr30k Entities: https://arxiv.org/abs/1505.04870
- SegmentMeIfYouCan: https://segmentmeifyoucan.com/
