# Grounding Model Evaluation

R0A evaluates portable prediction artifacts; it does not install or run
Grounding DINO, YOLO-World, or any other model. Use this workflow after a
model-specific runner has produced predictions for every dataset case.

## Artifact flow

```text
grounding_dataset.json
  + candidate_predictions.json
  -> candidate_evaluation.json

one or more candidate_evaluation.json files
  -> grounding_selection.json
```

The version `1.0.0` dataset contract is defined by
`src/track_robot_semantic_search/schemas/grounding_dataset.schema.json`. It
records train, validation, and test cases, image identities, queries, boxes,
scenario tags, and label review status. Dataset loading verifies each image's
regular-file status, SHA-256 digest, decoded dimensions, and declared box
bounds.

Keep recording sessions, non-empty physical object IDs, and image SHA-256
digests within exactly one split. The loader rejects any of these identities
when they cross train, validation, or test boundaries. Test cases must use
`human_verified` labels to pass candidate selection; teacher output is not test
truth until a person has verified it.

Each model runner writes the version `1.0.0` artifact defined by
`src/track_robot_semantic_search/schemas/grounding_predictions.schema.json`.
The artifact contains model and checkpoint identity, a checkpoint checksum,
platform and input-size metadata, resource and release evidence, and bounded
per-case boxes, scores, labels, and complete-path latency. It contains neither
an absolute checkpoint path nor raw model tensors, so evaluation evidence can
move between the desktop, robot, and review systems without depending on a
model installation.

## Evaluate one candidate

Build the workspace first, then source ROS 2 and the workspace installation.
The ROS package installs its console scripts in its package-specific `lib`
directory, so add that directory to `PATH` before using the standalone command
names:

```bash
source /opt/ros/foxy/setup.bash
source /home/track-robot/track_robot_ws/install/setup.bash
export PATH=/home/track-robot/track_robot_ws/install/track_robot_semantic_search/lib/track_robot_semantic_search:$PATH

semantic_search_grounding_evaluate \
  --dataset /absolute/path/to/grounding_dataset.json \
  --predictions /absolute/path/to/candidate_predictions.json \
  --output /absolute/path/to/candidate_evaluation.json
```

The evaluator requires the dataset and predictions to have the same
`dataset_id` and exactly the same case IDs. It chooses the highest score
threshold that passes the quality gates using validation cases only. That
threshold is then frozen for held-out test metrics and per-scenario test
metrics. Test results never move or rescue the threshold; if validation cannot
select one, test quality metrics remain unavailable.

The evaluation report also records a content-derived dataset checksum, model
and platform identity, latency and memory evidence, release gates, and explicit
failure reasons.

## Select a candidate

Pass every candidate report from the same dataset. The selector requires both
the dataset ID and the canonical dataset content checksum to match, so reports
from altered corpus contents cannot be ranked together:

```bash
semantic_search_grounding_select \
  --report /absolute/path/to/yolo_world_s_1280_evaluation.json \
  --report /absolute/path/to/grounding_dino_evaluation.json \
  --output /absolute/path/to/grounding_selection.json
```

Selection first requires evidence that validation produced a finite frozen
threshold in `[0, 1]`. It then applies all accuracy, false-accept,
localization, latency, semantic rate, CUDA-memory, runtime, platform, licence,
and human-review gates. Passing candidates are ranked deterministically by
recall, false-accept rate, median IoU, P95 latency, then candidate ID. When none
pass, the output status is `unavailable` and includes rejection reasons.

Both commands write output atomically after complete validation. Exit code `0`
means the evaluator wrote a complete report or the selector chose a candidate.
Exit code `2` means invalid input or output evidence; for selection it also
means no candidate passed. Argument parsing errors use the standard
`argparse` non-zero exit.

## Model-runner boundary

R0B and R0C model runners emit the prediction artifact above rather than
calling model code from R0A. The desktop R0B runner is optional. The active R0C
runner executes pretrained YOLO-World directly on the Orin from an isolated
runtime. Both keep model dependencies and checkpoints outside the ROS 2 Foxy
benchmark package.
