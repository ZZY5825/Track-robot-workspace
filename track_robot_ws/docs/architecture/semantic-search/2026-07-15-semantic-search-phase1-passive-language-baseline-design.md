# Semantic Search Phase 1 Passive Language Baseline Design

**Date:** 2026-07-15

**Status:** Approved by the parent semantic-search design and selected for implementation

**Parent design:** `docs/architecture/semantic-search/2026-07-13-language-conditioned-multimodal-semantic-search-design.md`

## 1. Outcome

Phase 1 adds an opt-in, passive camera semantic path that can accept a text
query, cache its text representation, score a bounded set of image regions,
and publish compact `SemanticRegionArray` messages. It also produces comparable
reports for the first three required baselines:

1. fixed-category camera detector;
2. LiDAR geometry without language;
3. open-vocabulary language-camera scoring without LiDAR fusion.

The phase does not add 3D fusion, temporal semantic memory, active rotation, or
any connection to motion commands.

## 2. Constraints carried from the parent design

- Deployment remains ROS 2 Foxy on Python 3.8, PyTorch 1.13 NVIDIA, CUDA 11.4,
  TensorRT 8.4, JetPack 5.0.2/L4T R35.1.
- The working ROS Python environment is not upgraded in place.
- Existing human-tracking launches, parameters, messages, controllers, safety
  gates, and velocity topics remain unchanged.
- The semantic launch is opt-in and publishes no raw feature tensor through
  DDS.
- The per-frame runtime owns at most one visual backbone.
- DINOv3 remains a visual feature encoder; raw DINO features are not claimed to
  share a language embedding space.
- The semantic output target is at least 5 Hz with complete-path P95 at or below
  150 ms on the Jetson.
- Missing model artifacts produce explicit `unavailable` results and prevent
  activation; they never produce synthetic success evidence.

## 3. Options considered

### Option A: compare raw DINO patch tokens with CLIP/SigLIP text tokens

Rejected. The embeddings were learned in different spaces, so cosine scores
would have no defensible semantic meaning even if their dimensions happened to
match.

### Option B: run DINOv3 and a complete CLIP/SigLIP image tower every frame

Rejected as the deployed Phase 1 path. It violates the one-visual-backbone
decision and spends Jetson budget twice. This remains an offline comparison
only.

### Option C: aligned image-text zero-shot baseline plus a separately verified
DINO runtime

Selected. A compatible CLIP/SigLIP-family adapter supplies both image-region
and text representations for baseline 3. For any comparable run it is the only
per-frame visual tower. In parallel, the existing DINO runtime is corrected to
preserve aspect ratio and expose internal coordinate metadata for later
DINO-text projection work. A trained DINO-to-text projection is not introduced
until suitable training data exists.

## 4. Runtime boundaries

### 4.1 Pure-Python core

The core package contains no ROS callback logic and defines:

- aspect-preserving image geometry and inverse coordinate mapping;
- query normalisation and versioned text-cache semantics;
- model descriptors and deterministic candidate selection;
- bounded region proposal, score-map thresholding, and connected components;
- baseline report assembly and gate evaluation.

Unit tests use deterministic fake encoders. Fakes are test tools only and can
never be selected by a production launch.

### 4.2 Optional model adapters

Every adapter reports:

- implementation and checkpoint identifiers;
- image and text embedding dimensions;
- preprocessing version;
- code/checkpoint licence and redistribution status;
- Python, PyTorch, device, latency, and memory evidence;
- whether it is available in the current runtime.

The first benchmark order is SigLIP 2 B, then a smaller CLIP-family fallback.
Selection is deterministic: among candidates that pass compatibility, licence,
memory, and latency gates, choose highest held-out phrase-region recall, then
lowest P95 latency, then lexical candidate ID. If no candidate passes, the
selection command exits non-zero and writes an `unavailable` report.

The initial production adapter boundary supports an optional `open_clip`
runtime loaded from an isolated external path. Import failure, missing weights,
or incompatible embedding shapes fail closed. No package installer is invoked
by a ROS node.

### 4.3 ROS node

`semantic_perception_worker`:

- subscribes to RGB and an active query topic;
- keeps only the latest image;
- processes by source timestamp at a configured target rate, default 5 Hz;
- encodes text only after normalised query/version changes;
- publishes `SemanticRegionArray` under `/semantic_search/regions`;
- publishes compact JSON health/metrics under `/semantic_search/perception_diagnostics`;
- publishes no image/text feature arrays;
- remains not-ready until model load and warm-up pass.

The query input is a typed internal dataclass in the core and a compact JSON
transport for Phase 1 replay tooling. The public `SearchForObject` action is not
activated until the later search-server phase.

## 5. Image geometry

For a source image of width `W` and height `H` and a square model canvas `S`:

1. compute `scale = min(S/W, S/H)`;
2. resize to rounded positive dimensions no greater than `S`;
3. centre-pad to `S x S`;
4. record scale, left/top padding, resized dimensions, and valid patch mask;
5. exclude every patch intersecting only padding from scoring;
6. clip inverse-mapped regions to the original image bounds.

The existing stretching helper remains available for backward compatibility,
but all Phase 1 paths use the new transform.

## 6. Region scoring

The zero-shot adapter returns a dense or coarse image embedding grid in the same
space as its cached text vector. The core L2-normalises both and computes cosine
similarity. It then:

1. masks padded tokens;
2. thresholds by configured absolute score or top quantile;
3. finds 4-connected components;
4. removes components below a minimum valid-token area;
5. maps component bounds back to source pixels;
6. sorts by score and retains at most the configured maximum regions.

`SemanticRegion` score fields remain separated. Baseline 3 sets
`language_score` and `localization_score`; `appearance_score` is reserved for
later DINO association evidence and `fused_score` equals the calibrated
baseline score. Empty evidence publishes an empty array rather than a fabricated
full-frame box.

## 7. Baselines and evidence

All three reports use the same manifest and schema extension:

- Baseline 1 consumes recorded fixed-category detector observations if present.
- Baseline 2 consumes recorded LiDAR candidates and applies only fixed geometric
  bounds; it does not use text.
- Baseline 3 consumes the selected aligned image-text adapter output and does
  not use LiDAR.

Each report distinguishes `passed`, `failed`, `unavailable`, and `not_evaluated`.
Missing annotations make accuracy metrics `not_evaluated`; they do not become
zero or pass. Phase completion requires runtime/contract evidence and three
machine-readable reports. Accuracy release claims still require the held-out
coverage defined in the parent design.

## 8. Failure handling

- Empty or whitespace-only query: reject without changing the current cache.
- Duplicate normalised query/version: reuse the cached tensor.
- Model import/checkpoint/licence failure: keep node not-ready and emit reason.
- CUDA OOM or inference exception: stop semantic inference and emit fault; do
  not affect the human stack.
- Stale image or timestamp rollback: drop the frame and reset the scheduler.
- Invalid or non-finite score map: reject the observation.
- No candidate passes threshold: publish an empty region array.

## 9. Acceptance

Phase 1 is complete when:

- aspect-preserving geometry and inverse mapping unit tests pass;
- query cache and deterministic model selection tests pass;
- bounded proposal/scoring tests pass;
- the opt-in ROS launch contains no motion/control node or topic;
- no raw feature tensor is present in a ROS message or publisher;
- baseline 1, 2, and 3 reports validate against the report schema;
- an available real model is benchmarked on Jetson, or the phase checkpoint
  explicitly records the external model artifact as unavailable without making
  a performance/accuracy claim;
- the complete workspace build and pre-existing regression suite pass.

The final bullet separates engineering completion from model-artifact
availability. A missing externally licensed checkpoint may block a release
claim, but it must not force unsafe environment changes or dishonest evidence.
