# Track Robot Semantic Search

This opt-in ROS 2 Foxy package owns Phase 0 semantic-search manifests,
localisation-mode health, replay evaluation, and the passive Phase 1 language
baseline.

It deliberately contains no action server, controller, planner, or cmd_vel
publisher. See the approved design and the semantic-search rosbag guide before
enabling later phases.

## Grounding Benchmark Artifacts

R0A provides model-independent dataset, prediction, evaluation, and candidate
selection contracts. It installs no grounding model or checkpoint and does not
run model inference. Follow the
[grounding model evaluation guide](../../docs/guides/semantic-search/grounding-model-evaluation.md)
to produce evaluation and selection artifacts with the installed
`semantic_search_grounding_evaluate` and
`semantic_search_grounding_select` commands.

Model-specific desktop RTX and Jetson runners belong to separately planned R0B
and R0C work. They must emit the portable R0A prediction artifact.

## Phase 1 Passive Language Baseline

Phase 1 adds a default-off perception worker that accepts a compact query event,
runs one aligned image-text visual backbone, and publishes bounded
`SemanticRegionArray` observations. It does not run DINOv3 and OpenCLIP on the
same frame: each comparable run owns one visual backbone. The separately
corrected DINOv3 runtime supplies aspect-preserving features for later trained
alignment work; raw DINO features are never compared directly with unrelated
text vectors.

The Jetson workspace has an isolated external runtime for official OpenAI CLIP
and a released ViT-B/32 checkpoint outside the ROS Python environment. They are
intentionally ignored by Git and loaded only when the launch is explicitly
enabled:

    ros2 launch track_robot_semantic_search semantic_search_phase1.launch.py \
      start_perception:=true \
      adapter_implementation:=openai_clip \
      model_name:=ViT-B/32 \
      runtime_path:=/home/track-robot/track_robot_ws/models/phase1_runtime/python \
      checkpoint_path:=/home/track-robot/track_robot_ws/models/phase1/ViT-B-32.pt

The checkpoint SHA-256 is
`40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af`;
the pinned OpenAI CLIP source revision is
`d05afc436d78f1c48dc0dbf8e5980a9d471f35f6`. The earlier `grid_only` 2x2
configuration measured a complete-path P95 of 103.76 ms on 32 evenly spaced
real rosbag frames. The 4x4 configuration measured 173.75 ms and is not a
replacement for context at different target scales.

The checked `multiscale_v1` profile addresses large targets and objects split
by a grid boundary with one bounded six-window GPU batch: the unchanged four
2x2 crops, one letterboxed whole image, and one centered crop covering 60% of
the source width and height. The center crop participates as a local region;
overlapping local results are merged deterministically. A passing whole image
is published only as a whole-frame fallback when no local result passes. It is
evidence that the queried concept may be present, not an object-accurate box.
Use `window_strategy: grid_only` for direct legacy comparisons.

No dependency installer or implicit model download runs inside the node. A
missing runtime, checkpoint, unsupported model, or failed warm-up leaves the
worker alive but unavailable and publishes the reason on
`/semantic_search/perception_diagnostics`.

Send a Phase 1 replay query with a monotonically managed version:

    ros2 topic pub --once /semantic_search/query std_msgs/msg/String \
      '{data: "{\"query_text\":\"fallen branch\",\"query_id\":1,\"query_version\":1}"}'

The worker keeps the latest image only, schedules by image source timestamp at
5 Hz by default, caches text encoding until the normalised query or version
changes, and transports no image or language feature tensor through DDS.

## Text Query CLI Portal

After sourcing the built workspace, submit a one-shot query without writing
JSON by hand:

    ros2 run track_robot_semantic_search semantic_search_query \
      "a red backpack"

The command allocates a positive query ID, publishes to
`/semantic_search/query`, waits for a correlated response on
`/semantic_search/perception_diagnostics`, and reports acceptance, rejection,
a missing subscriber, or an acknowledgement timeout. Deterministic tests and
replays may set the key explicitly:

    ros2 run track_robot_semantic_search semantic_search_query \
      "a red backpack" --query-id 10 --query-version 1 --timeout 5

Omit the query text to enter the interactive portal:

    ros2 run track_robot_semantic_search semantic_search_query

Plain text starts a new query. The interactive commands are `:new TEXT`,
`:revise TEXT`, `:status`, `:help`, and `:quit`. `:revise` retains the current
query ID and increments its version; a new query receives a new ID and version
one. Topic names can be changed with `--query-topic` and
`--diagnostics-topic` when the robot uses an explicit namespace.

One-shot exit codes are zero for accepted, two for local/rejected input, three
for no query subscriber, four for acknowledgement timeout, and 130 for an
operator interrupt. All subscriber and acknowledgement waits are bounded.

The current official OpenAI CLIP ViT-B/32 checkpoint should be tested with
English descriptions. Unicode transport is supported, but Chinese retrieval
quality has not been validated. This portal performs passive observation only:
it is not a `SearchForObject` action client and never publishes motion intent,
`Twist`, or `cmd_vel`.

## Live RViz Console

`semantic_search_live_overlay` keeps at most eight images and eight region
arrays and combines them only when their source timestamps match exactly. It
publishes `/semantic_search/overlay_image` with every valid candidate,
descending score rank, query ID/version, and the permanent warning
`CANDIDATES - NOT GROUND TRUTH`. It never falls back to drawing current boxes
on the latest unrelated image.

The normal operator entry point starts the overlay together with a saved RViz
view:

```bash
export ROS_DOMAIN_ID=20
ros2 run track_robot_bringup semantic_search_ctl visualize phase1
ros2 run track_robot_bringup semantic_search_ctl visualize phase2
```

Use the RViz **Semantic Search** panel to enter an English description. **New
Query** allocates a new positive query ID and **Revise** keeps the ID while
incrementing its version. The panel shows correlated acknowledgement/model
state, image candidate count, active 3D object count, and the fail-closed best
candidate. Cyan marks the highest-scored 2D candidate and amber marks the
remaining candidates; neither colour asserts semantic correctness.

Closing RViz stops the visualization launch and overlay node. It does not stop
the semantic-search stack or externally owned hardware in another terminal.

## Formal Legacy Replay Evidence

Formal legacy evidence uses evaluation report schema `1.1.0` and exactly three
fresh runs. Launch the evaluator with
`timing_policy:=foxy_wall_time_scaled` and
`freshness_time_base:=arrival_monotonic`. The accepted replay-rate and
wall-duration pairs are exactly 0.5x/90.0 seconds, 1.0x/45.0 seconds, and
2.0x/22.5 seconds. Each required topic must contain at least two messages and
cover at least `0.90` of the 45.0-second source-time target.

Compare the three reports against the one immutable manifest they reference:

    ros2 run track_robot_semantic_search semantic_search_compare_reports \
      --manifest artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json \
      /tmp/semantic_search_rate_05.json \
      /tmp/semantic_search_rate_10.json \
      /tmp/semantic_search_rate_20.json

The comparator validates the manifest checksum and capabilities, enforces the
exact policy and provenance, and recomputes all five hard gates. Stored gates
or `passed` values are evidence only when they equal that recomputation.

## Phase 2 evaluation and deterministic replay

Phase 2 adds optional strict annotation and manifest fields for public object
keys, LiDAR source keys, support/visibility, 3D position, task relevance,
ignore regions, the selected memory frame and the twelve required scenarios.
Existing Phase 0/1 manifests remain valid.

`semantic_search_phase2_evaluate` emits strict schema `2.0.0` reports covering
identity continuity, switches,
duplicates, merges, fragmentation, camera/LiDAR precision and recall, re-ID,
stale reactivation, 3D consistency, memory lifetime, task ranking, update
rate, core and complete-path latency, 30-minute stability and Jetson resources.
It recomputes percentiles from raw samples. Missing annotations or profiles are
reported as unavailable with null metrics, never as measured zeroes.

`semantic_search_phase2_calibrate_task_threshold` freezes a best-candidate
threshold from an independent calibration JSONL. It requires at least 30
positive and 30 hard-negative candidates, task recall at least 0.90 and
hard-negative false confirmation at most 0.05. Final task predictions must use
that exact threshold; uncalibrated production profiles stay fail-closed.

`semantic_search_phase2_replay` runs the C++ normalized replay executable twice
and requires byte-equivalent canonical JSON. The checked synthetic fixture
proves deterministic ID, lifecycle, assignment and task-overlay mechanics; it
does not replace the annotated pilot bag or physical resource evidence. Follow
`docs/guides/semantic-search/phase2-recording-and-evaluation.md` for field collection.
