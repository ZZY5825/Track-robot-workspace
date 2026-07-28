# R0C Orin YOLO-World

R0C runs a pretrained open-vocabulary detector directly on the Jetson AGX
Orin. It does not require a desktop, training, LiDAR, or ROS.

The first candidate is YOLOv8s-World-v2, PyTorch FP16, input size 640. It uses
the existing local OpenAI CLIP checkpoint to encode one English query and
caches that vocabulary until the query changes.

## Safety boundary

- Do not upgrade the global Ultralytics `8.0.239`.
- Do not replace NVIDIA Torch `1.13.0a0+936e9305.nv22.11`.
- Do not run pip without `--no-deps`.
- Do not pass a remote model name to the R0C commands.
- Do not interpret a successful smoke as an accuracy pass.
- R0C starts no ROS node. `ROS_DOMAIN_ID` is irrelevant to offline inference.

## Local paths

```text
models/r0c_runtime/python/             isolated Ultralytics 8.2.103
models/r0c/yolov8s-worldv2.pt          local visual checkpoint
models/phase1_runtime/python/           existing local OpenAI CLIP package
models/phase1/ViT-B-32.pt               existing local text checkpoint
```

The `models/` tree is excluded from Git. A fresh Orin therefore needs the
explicit setup below even after cloning the repository.

## One-time isolated setup

Run from the workspace root:

```bash
mkdir -p models/r0c_runtime/python models/r0c

python3 -m pip install \
  --no-deps \
  --target models/r0c_runtime/python \
  ultralytics==8.2.103 \
  ultralytics-thop==2.0.18

curl --fail --location \
  --output models/r0c/yolov8s-worldv2.pt \
  https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s-worldv2.pt
```

Expected checkpoint SHA-256 values for the implemented candidate:

```text
9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792  yolov8s-worldv2.pt
40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af  ViT-B-32.pt
```

Confirm that the global package was not replaced:

```bash
python3 -m pip show ultralytics | grep -E '^(Version|Location):'
```

Expected global version: `8.0.239`.

## Probe the physical Orin

The CUDA check must run with normal device access:

```bash
cd ~/track_robot_ws
export PYTHONPATH=src/track_robot_semantic_search:.

python3 -m tools.semantic_search_grounding.orin_cli probe \
  --runtime-path models/r0c_runtime/python \
  --clip-runtime-path models/phase1_runtime/python \
  --world-checkpoint models/r0c/yolov8s-worldv2.pt \
  --clip-checkpoint models/phase1/ViT-B-32.pt \
  --output artifacts/semantic_search/reports/r0c_orin_inventory.json

echo $?
```

Exit code `0` requires:

- `host_eligible: true`;
- `runtime_ready: true`;
- L4T release 35;
- CUDA device `Orin`;
- isolated Ultralytics `8.2.103`;
- a working `YOLOWorld` API;
- local CLIP and both verified checkpoints.

Exit code `2` means the report was written but at least one reason blocks
inference.

## Run one smoke image

```bash
export MPLCONFIGDIR=/tmp/matplotlib-r0c
export YOLO_CONFIG_DIR=/tmp/ultralytics-r0c

python3 -m tools.semantic_search_grounding.orin_cli smoke \
  --runtime-path models/r0c_runtime/python \
  --clip-runtime-path models/phase1_runtime/python \
  --world-checkpoint models/r0c/yolov8s-worldv2.pt \
  --clip-checkpoint models/phase1/ViT-B-32.pt \
  --image /absolute/path/to/image.png \
  --query "blue bottle" \
  --confidence-floor 0.05 \
  --iou-threshold 0.70 \
  --input-size 640 \
  --output artifacts/semantic_search/reports/r0c_orin_smoke.json
```

Expected: exit `0` and a bounded JSON report. `detections: []` is structurally
valid and means the candidate abstained at the proposal floor.

The first query includes CLIP text encoding and Ultralytics predictor
initialization. On the 2026-07-27 physical smoke it took about 6.8 seconds.
Repeating the same query in the same process used the cached vocabulary and
took about 52.2 ms per frame. Cold-query and cached-frame latency must remain
separate metrics.

## Produce an R0A prediction artifact

This command requires a real `grounding_dataset.json` and its adjacent image
tree:

```bash
python3 -m tools.semantic_search_grounding.orin_cli predict \
  --runtime-path models/r0c_runtime/python \
  --clip-runtime-path models/phase1_runtime/python \
  --world-checkpoint models/r0c/yolov8s-worldv2.pt \
  --clip-checkpoint models/phase1/ViT-B-32.pt \
  --dataset /absolute/path/to/grounding_dataset.json \
  --candidate-id yolov8s-worldv2-fp16-640-c005-i070 \
  --confidence-floor 0.05 \
  --iou-threshold 0.70 \
  --input-size 640 \
  --output artifacts/semantic_search/reports/r0c_predictions.json
```

Do not add `--licence-approved` until the operator has reviewed and accepted
the AGPL-3.0 deployment obligations. Without it the artifact remains
evaluable, but cannot pass release selection.

Evaluate the output:

```bash
PYTHONPATH=src/track_robot_semantic_search \
python3 -m track_robot_semantic_search.grounding_evaluation_cli \
  --dataset /absolute/path/to/grounding_dataset.json \
  --predictions artifacts/semantic_search/reports/r0c_predictions.json \
  --output artifacts/semantic_search/reports/r0c_evaluation.json
```

## Interpreting the current checkpoint

R0C execution readiness is proven on the physical Orin. The current blue
container smoke returned no box at the fixed 0.05 proposal floor for both
`blue cylindrical container` and `blue bottle`. This is useful negative
evidence, not a reason to silently lower the threshold.

Quality remains unproven until the human-reviewed R0A corpus covers target
present/absent images, distractors, distances, lighting, and multiple object
descriptions. R1 ROS integration must wait for that candidate evidence.
