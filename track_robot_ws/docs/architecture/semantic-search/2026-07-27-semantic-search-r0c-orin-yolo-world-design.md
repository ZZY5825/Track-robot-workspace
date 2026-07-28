# Semantic Search R0C Orin YOLO-World Design

## Status

- Date: 2026-07-27
- Status: engineering implementation complete; model-quality evaluation pending
- Upstream: R0A contracts and evaluator are complete
- Supersedes: mandatory R0B desktop-teacher execution
- Target: Jetson AGX Orin 32 GB, JetPack 5.0.2, Python 3.8

## 1. Decision

The active recovery path no longer requires a desktop teacher. R0B-1 remains
an optional, inert reference tool, but R0B-2 is cancelled. R0C runs a
pretrained YOLO-World candidate directly on the physical Orin without model
training.

The first candidate is `yolov8s-worldv2.pt` in PyTorch FP16 at 640 pixels.
TensorRT is optional and begins only after PyTorch correctness and R0A evidence
exist. Grounding DINO is not installed on the Orin.

## 2. Current compatibility boundary

The measured target has:

- aarch64 Linux 5.10.104-tegra, L4T 35.1;
- JetPack 5.0.2, CUDA 11.4, TensorRT 8.4.1;
- NVIDIA PyTorch `1.13.0a0+936e9305.nv22.11`;
- torchvision `0.14.1a0+5e8e2f1`;
- system-user Ultralytics `8.0.239`, which has no `YOLOWorld` API;
- an existing isolated OpenAI CLIP runtime and `ViT-B-32.pt`.

The existing Ultralytics installation must not be upgraded because other robot
features use it. R0C adds only `ultralytics==8.2.103` and a compatible
`ultralytics-thop` under `models/r0c_runtime/python`, with dependency
installation disabled. That wheel supports Python 3.8 and Torch 1.13 and
contains the YOLO-World implementation.

## 3. Runtime data flow

```text
R0A image + normalized English query
  -> local CLIP encodes the query only when it changes
  -> YOLO-World caches the single-query vocabulary
  -> local YOLOv8s-World-v2 FP16 image inference on Orin CUDA
  -> strict original-image XYXY validation and clamping
  -> deterministic score/geometry/label ordering
  -> at most 256 detections
  -> existing R0A prediction JSON
  -> existing R0A evaluator and selector
```

No ROS process is needed for R0C. Runtime ROS integration remains R1 and occurs
only after the candidate produces acceptable offline evidence.

## 4. Dependency and download policy

- All model paths are explicit local regular files.
- R0C never invokes Ultralytics automatic model downloads.
- R0C never invokes Ultralytics automatic CLIP installation.
- `models/r0c_runtime/python` precedes the global Ultralytics path only inside
  the R0C process.
- `models/phase1_runtime/python` provides the already installed `clip`, `ftfy`,
  and `regex` modules.
- The local `ViT-B-32.pt` is passed to a temporary `clip.load` wrapper while
  YOLO-World updates its vocabulary.
- The system NVIDIA Torch and torchvision remain unchanged.
- The YOLO-World and CLIP checkpoint SHA-256 values are recorded separately in
  preflight and combined deterministically for the R0A model identity.
- Network access is allowed only in explicit operator setup commands.

## 5. Candidate policy

The adapter receives exactly one normalized printable-ASCII query. It calls
`set_classes([query])` only when the query changes. It does not add synonyms,
remove attributes, or add generic fallback classes.

The first benchmark configuration is:

- input size: 640 by 640 maximum inference canvas;
- proposal confidence floor: 0.05;
- NMS IoU: 0.70;
- maximum detections: 256;
- device: CUDA device 0;
- precision: FP16;
- augmentation: disabled.

The 0.05 confidence is a proposal-retention floor. R0A selects the release
threshold from validation evidence. An empty result is valid.

## 6. Evidence and release status

R0C records complete-path latency from image opening through postprocessing,
with CUDA synchronization before and after each case. It records peak
incremental CUDA reserved memory relative to the reservation before model
loading.

The candidate sets:

- `runtime_available=true` only after successful Orin inference;
- `platform_compatible=true` for the measured Orin PyTorch path;
- `licence_approved=false` unless the operator explicitly records approval.

Ultralytics code is AGPL-3.0. Licence approval remains a release gate rather
than an inferred technical property.

## 7. Commands

The R0C CLI provides:

- `probe`: inventory Orin, isolated runtime, CUDA, and both checkpoints;
- `smoke`: run one local image/query and write bounded diagnostic JSON;
- `predict`: process every case in an R0A dataset and atomically publish a
  validated R0A prediction artifact.

Every failure produces one bounded error line and no partial final output.
Probe may report unavailable without importing or mutating the ROS runtime.

## 8. Verification

Pure tests use fake Torch, Ultralytics, CLIP, model, and result objects. They
prove:

- exact runtime isolation;
- no eager heavy imports;
- no implicit download path;
- query-change vocabulary caching;
- exact inference parameters;
- finite coordinate validation and deterministic truncation;
- CUDA synchronization and peak-memory accounting;
- R0A artifact compatibility and atomic failure behavior.

Physical verification then proves:

1. CUDA reports one Orin device;
2. the isolated Ultralytics exposes YOLO-World;
3. both local checkpoints pass SHA-256 verification;
4. one real image/query completes without training or network access;
5. no ROS node was started.

Formal model-quality claims remain blocked until a real, human-reviewed R0A
dataset exists. A smoke result proves execution, not semantic accuracy.

## 9. Non-goals

- No desktop teacher requirement.
- No training, fine-tuning, or pseudo-label generation.
- No system Ultralytics, Torch, JetPack, Ubuntu, ROS, or Python upgrade.
- No TensorRT requirement in R0C-1.
- No ROS topic publication or live-camera integration before R1.
- No LiDAR, semantic-memory, navigation, or motion changes.
- No quality conclusion from one blue-object smoke image.
