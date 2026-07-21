# Phase 1 Multiscale Window Robustness Design

## Status

Approved for implementation on 2026-07-20.

## Problem

The Phase 1 OpenAI CLIP baseline currently letterboxes each image to a square,
splits that square into a non-overlapping 2x2 grid, and encodes only those four
crops. A large or nearby object that crosses a grid boundary can therefore be
split into semantically incomplete parts even when the source image is clear.
The same failure mode applies to large artificial objects, fallen branches,
tree trunks, partially occluded targets, objects entering the field of view,
and targets whose apparent scale changes while the robot approaches.

This is a general scale-and-boundary robustness defect. It is not an
object-specific adjustment for the blue-chair test that exposed it.

## Goals

- Preserve the existing ROS topics and message contracts.
- Preserve the query-independent 2x2 descriptors used by the current proposal
  pooling path.
- Add whole-image and boundary-crossing semantic context without introducing a
  second visual backbone.
- Encode all windows in one bounded CLIP batch.
- Suppress obvious duplicate regions deterministically.
- Retain the Phase 1 target rate of 5 Hz and complete-path P95 gate of 150 ms.
- Keep the existing threshold at 0.25 for the first controlled comparison so
  window-strategy effects are not confused with threshold changes.

## Non-goals

- No chair-specific prompts, rules, labels, or thresholds.
- No 4x4 default grid.
- No detector, segmenter, LiDAR proposal generator, or new model dependency.
- No temporal smoothing, threshold hysteresis, or threshold recalibration in
  this change.
- No Phase 2 message or association-policy changes.

## Considered approaches

### Dense overlapping sliding windows

Dense windows provide broad scale and boundary coverage but multiply Jetson
inference cost and duplicate candidates. They are rejected for the first
implementation because the current 4x4 measurement already exceeds the
complete-path latency target.

### Detector- or LiDAR-proposal crops

Object proposals can provide better object boundaries and should later be
encoded directly by CLIP. They are not the first fix because proposal recall
would become an additional hard limit and the current normalized proposal path
only pools the coarse 2x2 descriptors.

### Fixed bounded multiscale windows

This is the selected approach. It adds two crops to the current four-crop batch:
one whole-image crop and one centered crop. It is deterministic, model-agnostic,
bounded, and directly targets the discovered failure mode.

## Window contract

`multiscale_v1` uses exactly six source-image windows in one batch:

1. the four legacy 2x2 quadrants, in row-major order;
2. the complete source image;
3. a centered window covering 60% of source width and 60% of source height.

The four legacy quadrants remain crops of the existing globally letterboxed
square, preserving their current descriptors exactly. The whole-image crop is
that same complete letterboxed square. The centered source-image crop is
independently padded to a square before the installed CLIP preprocessing
transform, so none of its non-square content is center-cropped away.

Window coordinates are integer, clipped to the source image, non-empty, and
deterministic. The centered ROI uses rounded dimensions and symmetric placement;
any odd-pixel remainder is placed on the right or bottom.

The existing `grid_only` strategy remains available for controlled comparison
and compatibility. The checked Phase 1 profile selects `multiscale_v1`.

## Internal interfaces

`WindowEncoding` owns a source ROI, a bounded window kind, and one normalized
CLIP image vector. `ImageGridEncoding` retains its regular 2x2 `embeddings`,
valid-patch mask, and geometry fields and gains a bounded tuple of extra window
encodings. This preserves `pool_roi_descriptor()` for current Phase 2 proposal
inputs.

Both OpenAI CLIP and OpenCLIP adapters accept:

- `window_strategy`: `grid_only` or `multiscale_v1`;
- `center_window_scale`: fixed to 0.60 in the checked profile and constrained to
  the finite interval `[0.25, 1.0]`.

The adapters perform one model call for the complete crop batch. In
`multiscale_v1`, the first four output vectors rebuild the legacy 2x2 tensor;
the final two vectors become whole-image and center-window encodings.

## Scoring and duplicate handling

The cached text vector is compared against the four grid vectors and the two
extra window vectors in the same aligned CLIP space.

The legacy grid continues to use thresholding and four-connected component
merging. The center window becomes a local `RegionCandidate` when its score
passes the same cutoff. Deterministic duplicate suppression applies to local
candidates using both:

- IoU at least 0.50; or
- intersection divided by the smaller candidate area at least 0.80.

Candidates are ordered by descending mean score, descending peak score, then
source coordinates. The first candidate wins; suppressed candidates cannot
alter its descriptor or score. Output remains bounded by `max_regions`.

The whole-image score is a fallback presence signal. It is published as a
whole-image region only when no grid or center candidate passes the cutoff. It
is never published alongside a local candidate, preventing the most obvious
global/local duplicate.

Absolute mode uses the configured threshold for every window. Quantile mode
computes one cutoff over every valid grid and extra-window score from the same
frame, so scale sources are compared consistently.

## ROS behavior

The node adds bounded parameters:

```yaml
window_strategy: multiscale_v1
center_window_scale: 0.60
duplicate_iou_threshold: 0.50
duplicate_containment_threshold: 0.80
```

The following interfaces remain unchanged:

- input image: `/zed/zed_node/left/image_rect_color`;
- input query: `/semantic_search/query`;
- output regions: `/semantic_search/regions`;
- output observations: `/semantic_memory/observations`;
- output tasks: `/semantic_memory/tasks`;
- diagnostics: `/semantic_search/perception_diagnostics`.

Published regions continue to carry source-image ROIs, the aligned encoder and
checkpoint identity, query identity/version, scores, and bounded descriptors.
No image or embedding tensor is transported through DDS.

## Failure handling

- Invalid strategy or center scale fails model initialization closed and emits
  the existing `not_ready` diagnostic.
- Invalid or empty generated ROIs fail before model inference.
- Non-finite embeddings or scores stop inference through the existing `fault`
  diagnostic path.
- An oversized or malformed result batch is rejected rather than partially
  mapped to windows.
- A frame with no score above the cutoff publishes an empty region array, as it
  does today.

## Verification

Unit coverage must prove:

- exact six-window layout at 1280x720 and odd image dimensions;
- independent square padding without source-content loss;
- one image-model call per frame and batch size six;
- legacy four-vector grid reconstruction;
- full-image fallback behavior;
- center/grid duplicate suppression, deterministic ties, and bounded output;
- quantile cutoff uses grid and extra-window scores together;
- `grid_only` behavior remains compatible;
- invalid strategy, scale, geometry, and batch size fail closed.

The same recorded frames must compare `grid_only` and `multiscale_v1` for:

- a small target within one quadrant;
- a large target crossing both center boundaries;
- a target at an image edge;
- partial occlusion;
- two similar objects;
- target absence and hard-negative queries;
- target scale change from far to near.

Runtime evidence must report mean and P95 complete-path latency, observation
rate, empty-region share, candidate count, and peak CUDA reservation. The change
passes only if the package test suite is green, the model loads and warms, the
new strategy improves boundary/scale coverage on the controlled comparison,
the output rate remains at least 5 Hz, and complete-path P95 remains at most
150 ms.

If the six-window batch misses the latency gate, the implementation remains
safe and configurable but does not become the official profile until a separate
measured optimization is approved. It must not silently reduce target rate or
relax the latency gate.
