# Semantic Search R0B Zero-Shot Teacher Design

## Status

- Date: 2026-07-27
- Status: R0B-1 retained as an optional tool; mandatory R0B-2 cancelled
- Upstream: R0A grounding benchmark contracts are complete
- Target host: a separate x86_64 desktop with an NVIDIA RTX GPU
- Current host: Jetson AGX Orin, aarch64, JetPack 5.0.2; implementation and
  pure tests only, with no Grounding DINO installation

## 1. Decision summary

R0B starts with zero-shot inference. It does not train or fine-tune Grounding
DINO. The first teacher candidate uses the Apache-2.0
`IDEA-Research/grounding-dino-tiny` checkpoint through the Hugging Face
Transformers Grounding DINO API in a desktop-only virtual environment.

This route is selected because it:

- avoids the official repository's custom CUDA/C++ extension during the first
  comparison;
- keeps the desktop model environment independent of ROS 2 Foxy and JetPack;
- supports local, offline checkpoint loading;
- returns text-conditioned boxes in original image coordinates;
- lets the existing R0A evaluator compare predictions without importing model
  libraries.

The official IDEA repository remains a later reference adapter if the
Transformers implementation produces materially different evidence. A hosted
API is not used because it weakens reproducibility, privacy, and complete-path
latency measurement.

## 2. Scope

This implementation slice, R0B-1, delivers:

1. a desktop eligibility and runtime preflight;
2. a model-independent teacher runner core;
3. a lazy Hugging Face Grounding DINO adapter;
4. a command-line runner that consumes an R0A dataset and emits the frozen
   R0A grounding-prediction artifact;
5. deterministic output validation and atomic writing;
6. a guide for moving the workspace to the desktop and running the preflight.

R0B-1 does not:

- install or download a model on the Jetson;
- update any model weight;
- import Grounding DINO, Transformers, Torch, or Pillow from ROS nodes;
- publish a ROS topic or start a ROS process;
- generate human-reviewed pseudo-labels;
- convert RefCOCO, Visual Genome, LVIS, or another public dataset;
- claim that a teacher model is suitable for Jetson deployment.

Public-dataset import and actual desktop inference are no longer required
checkpoints. The active path is the Orin-only R0C design in
`2026-07-27-semantic-search-r0c-orin-yolo-world-design.md`.

## 3. Host boundary

The current workspace host is an aarch64 Jetson with JetPack 5.0.2. It is not
an R0B execution host. The R0B CLI therefore has two levels of readiness:

- `host_eligible`: x86_64, `nvidia-smi` available, and at least one visible
  NVIDIA GPU;
- `runtime_ready`: host eligible plus importable Torch, Transformers, Pillow,
  OpenCV, CUDA available to Torch, a local model directory, and a verified
  local checkpoint file.

The preflight reports every failed condition. Formal `predict` execution
requires both levels. There is no override that turns the Jetson into a
desktop teacher host.

## 4. Runtime data flow

```text
R0A grounding_dataset.json
  -> strict R0A dataset loader and image verification
  -> one normalized English query + one image per case
  -> local Hugging Face Grounding DINO processor/model
  -> original-image XYXY boxes, scores, labels
  -> finite/bounded XYWH conversion
  -> deterministic score/geometry/label ordering
  -> at most 256 detections per case
  -> R0A grounding_predictions.json
  -> strict R0A prediction loader validation
  -> atomic rename to the requested output
```

Every train, validation, and test case is processed so that the prediction
case set exactly matches the dataset case set. Teacher output does not modify
dataset labels.

## 5. Prompt and detection policy

The runner sends exactly one validated, normalized English query to the model.
It does not add synonyms, remove attributes, or combine different target
descriptions. The adapter may apply only the punctuation convention required
by its processor.

Inference uses two fixed proposal floors:

- box score threshold: `0.05`;
- text threshold: `0.05`.

These are proposal-retention floors, not the release operating threshold.
R0A selects the operating threshold from validation evidence. The candidate ID
and run guide record both floors so different settings are not treated as the
same candidate.

The adapter clamps finite boxes to image bounds, discards boxes that collapse
after clamping, sorts detections by descending score then geometry and label,
and retains at most 256 detections. Non-finite scores or coordinates fail the
entire run; they are not silently repaired.

## 6. Timing and resource evidence

`complete_path_ms` covers image opening, preprocessing, device transfer,
inference, postprocessing, coordinate conversion, sorting, and truncation.
CUDA is synchronized immediately before and after the measured path.

The adapter records incremental CUDA reserved memory relative to the CUDA
reservation immediately before model loading. The peak covers model loading
and the complete inference run. R0B is a desktop reference, so
`platform_compatible` is `false`; the teacher report is evaluable but cannot
win the Jetson candidate selector.

`licence_approved` is false unless the operator supplies an explicit approval
flag after reviewing the model and code licences.

## 7. Model and artifact identity

The runner accepts only a local model directory and uses
`local_files_only=True`. It never downloads a checkpoint implicitly.

The prediction artifact records:

- implementation: `huggingface_transformers_grounding_dino`;
- exact model revision supplied by the operator;
- relative checkpoint filename;
- computed lowercase SHA-256 of the checkpoint;
- declared licence;
- desktop hardware, OS, Python, Torch, device, and candidate role;
- candidate ID including model family and proposal-floor configuration.

The output is rejected before publication unless the existing R0A prediction
loader accepts it.

## 8. Failure handling

- A non-desktop host makes preflight unavailable and blocks prediction.
- Missing GPU, CUDA, runtime modules, model directory, or checkpoint produces
  one bounded CLI error and no prediction output.
- Dataset image, checksum, split, or query failures are reported by the R0A
  loader.
- A model exception, non-finite result, malformed result, or case mismatch
  aborts the run.
- Temporary output is removed after any failure.
- Existing final output is not replaced until the complete new document
  validates.

## 9. Verification

Jetson verification is limited to pure tests:

- desktop and Jetson preflight classification;
- deterministic detection conversion;
- exact prompt forwarding;
- all-case orchestration and timing;
- R0A schema compatibility;
- atomic-output failure handling;
- absence of eager Torch/Transformers imports.

Desktop verification later adds:

- actual preflight report;
- local checkpoint checksum;
- one-image smoke inference;
- complete corpus prediction artifact;
- R0A evaluation report;
- confirmation that no training operation ran.

## 10. Archived checkpoint

R0B-1 is complete when the runner and pure tests pass on the current workspace,
the current Jetson is correctly rejected as a desktop teacher host, and the
desktop guide identifies the exact next command. This condition was met.
R0B-1 remains available for future optional comparison; R0B-2 does not block
R0C or any later stage.
