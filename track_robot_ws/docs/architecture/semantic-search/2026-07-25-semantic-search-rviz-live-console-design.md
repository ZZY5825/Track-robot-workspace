# Semantic Search RViz Live Console Design

**Date:** 2026-07-25

**Status:** Approved for implementation

## 1. Goal

Turn the existing passive Phase 1 and Phase 2 test outputs into one direct,
live RViz view. An operator must be able to enter or revise an English language
query, see correlated image candidates, inspect 3D semantic-memory objects, and
understand model and pipeline state without reading raw ROS messages.

The console remains observation-only. It never publishes an action goal,
`SearchMotionIntent`, `Twist`, or `/cmd_vel`.

## 2. Chosen architecture

Use RViz rather than a separate browser dashboard because the required camera,
point-cloud, TF, robot-frame, and MarkerArray displays already belong in the ROS
visualization process. The solution has four bounded components:

1. `track_robot_semantic_search.live_overlay` correlates source images and
   `SemanticRegionArray` messages by exact source stamp and publishes an
   annotated image on `/semantic_search/overlay_image`.
2. The existing semantic-memory visualizer publishes a cube, readable
   `TEXT_VIEW_FACING` label, and optional best-candidate highlight for each
   bounded object record on `/semantic_memory/markers`.
3. A new C++/Qt package,
   `track_robot_semantic_search_rviz_plugins`, provides a passive RViz panel
   using the existing query and diagnostic topics.
4. `track_robot_bringup` installs Phase 1 and Phase 2 RViz configurations and a
   foreground `semantic_search_ctl visualize phase1|phase2` command.

No feature tensor, image crop, or unbounded history is transported through DDS.

## 3. Live image overlay

The overlay node subscribes to:

- `/zed/zed_node/left/image_rect_color`;
- `/semantic_search/regions`.

It publishes:

- `/semantic_search/overlay_image`.

Only an image and region array with the same header timestamp may be combined.
The node keeps at most eight images and eight pending region arrays. Stale or
unmatched entries are evicted; the node must never draw a current box on an
unrelated frame.

All valid candidate regions are drawn. The highest-scored candidate is cyan and
the remaining candidates are amber. Labels show rank and fused score. The
header shows query ID/version and explicitly says `CANDIDATES - NOT GROUND
TRUTH`. Empty correlated arrays still produce an annotated image saying that no
candidates passed. Invalid or out-of-bounds ROIs are rejected or clipped
without crashing the node.

## 4. 3D semantic-memory rendering

Each active semantic object produces:

- one state-coloured cube using its validated position and extent;
- one white text label above the cube containing object ID, lifecycle, support,
  motion class, and task relevance when an active task exists.

The existing lifecycle colours remain:

- yellow: tentative;
- orange: stale;
- blue: dynamic;
- green: confirmed static.

Prediction-only objects remain translucent. If
`/semantic_memory/best_candidate` contains one calibrated winner, the same
object receives a magenta translucent halo and `BEST CANDIDATE` label. An empty
winner array removes the old highlight. Marker namespaces are separate and
every removed object emits explicit deletes for cube, label, and highlight.

Best-candidate output remains fail-closed. The visualizer does not invent a
winner from uncalibrated scores.

## 5. RViz query and status panel

The plugin class is
`track_robot_semantic_search_rviz_plugins/SemanticSearchPanel`. It provides:

- a query text field;
- **New Query** and **Revise** buttons;
- current query ID and version;
- acceptance/model status and reason;
- latest region count;
- active semantic-object count;
- best-candidate ID and relevance, or an explicit unavailable state;
- a permanent passive-observation safety statement.

It publishes canonical JSON to `/semantic_search/query` and subscribes to:

- `/semantic_search/perception_diagnostics`;
- `/semantic_search/regions`;
- `/semantic_memory/active_objects`;
- `/semantic_memory/best_candidate`.

New queries receive a positive microsecond-derived ID and version one. IDs are
strictly increasing within the panel process. Revise keeps the ID and
increments the version. Text is Unicode NFKC-normalized, whitespace-collapsed,
non-empty, and at most 512 characters. Diagnostics only acknowledge the
matching ID/version. The panel shows missing model readiness but never blocks
the RViz UI thread while waiting for a response.

## 6. RViz configurations and operator flow

`semantic_search_phase1.rviz` uses
`zed_left_camera_optical_frame` and shows the live overlay plus the query panel.

`semantic_search_phase2.rviz` uses `odom` and shows TF, the ZED overlay,
`/rslidar_points`, `/semantic_memory/markers`, and the query panel.

The operator starts the already modular stack in one terminal and visualization
in another:

```bash
ros2 run track_robot_bringup semantic_search_ctl visualize phase1
ros2 run track_robot_bringup semantic_search_ctl visualize phase2
```

The visualization command execs one foreground launch. Closing RViz or pressing
Ctrl+C shuts down the overlay node and launch process. It does not stop
externally owned sensors or the separately managed semantic-search stack.

## 7. Failure handling

- Missing image, region, object, or diagnostic topics leave the corresponding
  display in a clear waiting state.
- Malformed diagnostics are ignored and shown as a bounded status reason.
- Image conversion failures are throttled ROS warnings; no stale overlay is
  published.
- Invalid semantic snapshots continue to be rejected by the bounded marker
  registry.
- Closing RViz terminates only the visualization launch group.
- All UI updates from ROS callbacks are queued onto the Qt thread.

## 8. Acceptance criteria

1. A Phase 1 query can be submitted and revised from the panel.
2. The panel never publishes to motion, action, reset, or inspection APIs.
3. Exact-stamp camera overlays display every valid candidate and never combine
   unrelated frames.
4. Phase 2 markers show stable object IDs, state text, explicit deletion, and
   calibrated-winner highlighting.
5. Saved Phase 1 and Phase 2 RViz configurations load the intended topics and
   panel.
6. `semantic_search_ctl visualize` is foreground, Domain 20, bounded to the two
   supported stages, and cleans up visualization children when RViz exits.
7. Pure overlay, query-session, marker, launch-contract, and CLI tests pass.
8. All affected packages build on ROS 2 Foxy without new runtime downloads.

## 9. Out of scope

- active search and motion control;
- `SearchForObject` action-client behavior;
- persistent query history;
- accepting a candidate as ground truth;
- changing model thresholds or association calibration;
- replacing the formal JSON evidence and rosbag evaluation workflow.
