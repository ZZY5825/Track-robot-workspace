# R0B Zero-Shot Grounding DINO Teacher

R0B answers one narrow question before any training: can a pretrained
text-conditioned detector locate the described object much more reliably than
the OpenCLIP window baseline?

Grounding DINO runs here as a desktop reference teacher. It receives one
English query and one image, and returns boxes with confidence scores. It does
not replace the Jetson runtime yet, and it does not train or modify weights.

## 1. Know which machine does what

- Jetson AGX Orin: keeps ROS 2 Foxy and the robot runtime; it can edit and run
  the pure R0B tests, but must not receive this desktop model environment.
- RTX desktop: runs Grounding DINO prediction and produces portable JSON.
- Either machine: runs the R0A evaluator after the JSON and dataset images are
  present.

No ROS node, sensor, LiDAR, motor driver, or `ROS_DOMAIN_ID` is involved in
R0B. This is an offline image/query benchmark.

## 2. Put the same workspace and corpus on the RTX desktop

Copy or clone the workspace onto an x86_64 Ubuntu desktop with an NVIDIA RTX
GPU. Preserve the R0A dataset JSON and its relative image directory together;
the loader verifies every image checksum and rejects changed or missing
images.

The current workspace has the R0A contracts but no real
`grounding_dataset.json`. Before formal comparison, create or import a
human-reviewed corpus that follows
`src/track_robot_semantic_search/schemas/grounding_dataset.schema.json`.
Validation and test images must not share a recording session, physical
object, or image checksum.

## 3. Create an isolated desktop environment

Do this only on the RTX desktop:

```bash
cd ~/track_robot_ws
python3 -m venv ~/.venvs/track-robot-r0b
source ~/.venvs/track-robot-r0b/bin/activate
python -m pip install --upgrade pip
```

Install the CUDA-enabled PyTorch wheel selected for the desktop's installed
NVIDIA driver from the official PyTorch installation selector. Then install
the model-facing Python packages:

```bash
python -m pip install \
  transformers pillow opencv-python-headless safetensors huggingface_hub pytest
```

Do not source ROS or install these packages into the Jetson system Python.
After the first successful desktop run, save `python -m pip freeze` beside the
run report so the working environment can be pinned.

## 4. Download and pin the model explicitly

Review the model card and licences first. Resolve and record one immutable
revision commit for `IDEA-Research/grounding-dino-tiny`, then download that
revision explicitly:

```bash
hf download IDEA-Research/grounding-dino-tiny \
  --revision REVISION_COMMIT_SHA \
  --local-dir ~/models/grounding-dino-tiny
```

The directory must contain `model.safetensors`. The prediction command never
uses the network and will not silently fetch missing files.

## 5. Run the desktop preflight

From the desktop workspace:

```bash
cd ~/track_robot_ws
source ~/.venvs/track-robot-r0b/bin/activate
export PYTHONPATH="$PWD/src/track_robot_semantic_search:$PWD"

python3 -m tools.semantic_search_grounding.cli probe \
  --model-dir "$HOME/models/grounding-dino-tiny" \
  --checkpoint-file model.safetensors \
  --output artifacts/semantic_search/reports/r0b_desktop_inventory.json

echo $?
```

Expected result: exit code `0`, `host_eligible: true`, `runtime_ready: true`,
one visible NVIDIA GPU, `torch.cuda_available: true`, and a non-empty
checkpoint SHA-256. Exit code `2` means the report was written but at least one
condition is not ready; read its `reasons` array before proceeding.

## 6. Generate predictions without training

Replace `REVISION_COMMIT_SHA` with the exact revision used for the download:

```bash
python3 -m tools.semantic_search_grounding.cli predict \
  --dataset /absolute/path/to/grounding_dataset.json \
  --model-dir "$HOME/models/grounding-dino-tiny" \
  --checkpoint-file model.safetensors \
  --model-revision REVISION_COMMIT_SHA \
  --candidate-id grounding-dino-tiny-box005-text005 \
  --licence Apache-2.0 \
  --licence-approved \
  --box-threshold 0.05 \
  --text-threshold 0.05 \
  --output artifacts/semantic_search/reports/r0b_grounding_dino_predictions.json
```

Expected result: exit code `0` and one prediction record for every dataset
case. The two `0.05` values only retain proposals; R0A chooses the release
operating threshold from validation evidence. Do not tune these values on the
test split.

## 7. Evaluate the teacher

The evaluator needs no model:

```bash
PYTHONPATH=src/track_robot_semantic_search \
python3 -m track_robot_semantic_search.grounding_evaluation_cli \
  --dataset /absolute/path/to/grounding_dataset.json \
  --predictions artifacts/semantic_search/reports/r0b_grounding_dino_predictions.json \
  --output artifacts/semantic_search/reports/r0b_grounding_dino_evaluation.json
```

The important evidence is validation-selected threshold, held-out test recall,
false-accept rate, median IoU, complete-path P95 latency, and per-scenario
failures. Because this is a desktop teacher,
`release_evidence.platform_compatible` remains `false`; it is a reference for
R0C Jetson candidates, not a deployable winner.

## 8. What happens next

R0B-1 is complete when the runner and current-host report exist. R0B-2 starts
when an RTX desktop and a real R0A corpus are available:

1. run the desktop preflight;
2. run a one-image smoke case;
3. generate the complete prediction artifact;
4. evaluate it with R0A;
5. review failure images and decide whether zero-shot quality is sufficient.

Only after that evidence should R0C implement and compare the pretrained
YOLO-World Jetson candidate. Training remains optional; it is considered only
if both pretrained routes miss the required quality gates.
