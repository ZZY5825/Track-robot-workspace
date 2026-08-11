# ZED-Only Semantic 3D Position Design

Date: 2026-08-11

Status: approved in conversation (Approach A)

## 1. Objective

Stabilize the Phase 1–5 semantic target position before changing Nav2. For the
current semantic-search runtime, the canonical target position shall come only
from the ZED registered depth image. Semantic Camera–LiDAR association shall
not create, replace, or update the target position or identity.

LiDAR remains active and unchanged for obstacle filtering, the independent
safety grid, Nav2 costmaps, and motion safety. Human-tracking Camera–LiDAR
fusion and generic non-semantic-search profiles are outside this change.

## 2. Current failure

The current Phase 4A profile sends ZED-depth-enriched observations to semantic
memory while also enabling semantic LiDAR attachment. A visual object can
therefore receive geometry from two independent sources. The live 2026-08-11
test showed one visible target changing from approximately 4.64 m to 0.00 m
and then to invalid position within less than one second, before later
stabilizing near 1.50 m.

The depth adapters also retain only the latest depth image. The Phase 4A
selector fallback checks depth age against the current time and requests the
latest TF, rather than selecting a depth image and transform for the camera
observation timestamp. These paths can attach unrelated depth or omit valid
depth during inference delay.

## 3. Considered approaches

### A. ZED-only semantic geometry (selected)

Use a bounded timestamped ZED depth buffer, select the closest registered
depth frame for each camera observation, and transform the resulting point at
that same timestamp. Disable semantic LiDAR attachment in the Phase 1–5
profile. This is the smallest architecture that provides one geometry owner
without removing LiDAR from navigation safety.

### B. ZED primary with LiDAR shadow comparison

Compute both sources but forbid LiDAR mutation. This provides experimental
comparison data but retains unnecessary runtime work and state for the current
test objective. Deferred.

### C. Retune Camera–LiDAR association

Keep both sources and adjust association thresholds. This cannot remove
duplicate geometry ownership and is not supported by the observed instability.
Rejected.

## 4. Runtime data flow

```text
/zed/zed_node/left/image_rect_color
  -> YOLO-World detection and stable camera_track_id
  -> /semantic_memory/observations

/zed/zed_node/depth/depth_registered
  + /zed/zed_node/left/camera_info
  + timestamped TF base_link <- depth_frame
  -> semantic_depth_enricher
  -> /semantic_memory/spatial_observations
  -> semantic_memory
  -> Phase 3 selection
  -> Phase 4 approach goal
  -> Phase 4B/5 Nav2 supervision

/rslidar_points
  -> obstacle filter / safety grid / Nav2 costmaps / safety supervisor only
```

`MemoryCore` remains the sole owner of memory epochs, global IDs, lifecycle,
and temporal object state. The change does not create a second memory owner.

## 5. ZED depth association

### 5.1 Bounded timestamp matching

The depth adapter retains a small bounded deque of registered depth frames.
For each observation it uses `camera_stamp` when valid, otherwise the array
header stamp. It selects the depth frame with the smallest absolute timestamp
difference and rejects it when the configured maximum delta is exceeded.

The buffer has both a count bound and an age bound. Timestamp rollback clears
the affected buffer so frames from different camera epochs cannot mix.

### 5.2 ROI depth estimate

The accepted depth is derived from the centre portion of the full-resolution
YOLO ROI, never from one pixel. Invalid, non-finite, too-near, and too-far
samples are removed. At least the configured minimum sample count is required.
The robust median of the accepted centre samples remains the initial estimator;
this change does not introduce segmentation, LiDAR fusion, or a learned depth
model.

If later live evidence shows that the centre median still selects background,
a separately measured depth-cluster estimator may replace it. That is not part
of this change because semantic LiDAR mutation must first be removed as the
independent variable.

### 5.3 Timestamped projection

Camera intrinsics convert the ROI centre and accepted range into the registered
depth optical frame. TF is queried at the selected depth timestamp and projects
the point into `base_link`. The resulting observation carries
`EVIDENCE_CAMERA | EVIDENCE_STEREO_DEPTH`, `position_valid=true`, its
localization epoch, and the existing conservative covariance.

Missing depth, insufficient samples, stale timestamp, unavailable TF, or a
non-finite transform publishes the original 2D observation exactly once with
`position_valid=false`. `(0, 0, 0)` must never be presented as valid geometry.

## 6. Semantic LiDAR isolation

The Phase 1–5 semantic-search launch/configuration shall:

- keep the RoboSense driver and `/rslidar_points` active;
- keep `local_obstacle_map_node`, Nav2 costmaps, and the safety supervisor
  unchanged;
- disable semantic-memory camera attachment and test camera attachment;
- not launch the semantic LiDAR tracklet manager for this profile;
- prevent LiDAR-only tracklets from entering semantic ranking or changing a
  semantic object's support, position, lifecycle, or global ID.

The generic semantic-memory defaults and human-tracking packages are preserved.
No public topic or message definition changes.

## 7. Diagnostics

The depth adapter shall expose bounded counters and the latest rejection reason:

- `matched_depth`;
- `no_matching_depth`;
- `depth_delta_exceeded`;
- `insufficient_depth_samples`;
- `depth_out_of_range`;
- `tf_unavailable`;
- `invalid_transformed_position`;
- depth/observation timestamp delta;
- valid-depth sample count and quality.

Diagnostics must not suppress the normal observation output.

## 8. Validation and regression gates

Automated tests shall prove:

1. nearest timestamp selection is deterministic;
2. a depth frame outside the permitted delta is rejected;
3. timestamp rollback cannot attach an old frame;
4. TF lookup uses the selected depth timestamp;
5. valid ZED depth produces finite `base_link` geometry and stereo-depth
   evidence;
6. invalid depth produces a 2D-only observation, never valid zero geometry;
7. the Phase 1–5 profile disables semantic LiDAR mutation while keeping the
   LiDAR obstacle and Nav2 inputs;
8. existing semantic-memory ID/lifecycle tests remain unchanged and pass.

The live green-bottle gate is a stationary 30–60 second observation:

- no valid 0 m output;
- no multi-metre single-frame jump without a corresponding raw depth change;
- valid 3D output continuity and dropout reasons are recorded;
- one camera track retains one global ID through short depth dropouts;
- powering semantic LiDAR association off does not remove ZED target geometry;
- `/rslidar_points` continues to update costmaps and safety diagnostics.

No Nav2 planner tuning is included. Nav2 investigation resumes only after this
gate is reproduced successfully.

## 9. Rollback

The change is profile-scoped. Rollback restores the previous Phase 4A semantic
LiDAR launch include and association parameters, plus the previous latest-depth
adapter. Public interfaces and stored data formats remain compatible.
