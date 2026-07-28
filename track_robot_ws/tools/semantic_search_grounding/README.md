# R0B Grounding Teacher Tools

This directory contains the desktop-only, zero-training Grounding DINO
teacher runner. It is deliberately separate from ROS packages and must not be
installed into the Jetson ROS environment.

## Commands

Run commands from the workspace root with the R0A package on `PYTHONPATH`:

```bash
export PYTHONPATH="$PWD/src/track_robot_semantic_search:$PWD"

python3 -m tools.semantic_search_grounding.cli probe \
  --model-dir /absolute/path/to/grounding-dino-tiny \
  --checkpoint-file model.safetensors
```

`probe` returns:

- `0` when an x86_64 NVIDIA desktop, Python dependencies, CUDA runtime, local
  model directory, and checkpoint are all ready;
- `2` when it wrote a valid report but the host or runtime is not ready.

Once the probe returns `0`, run all R0A cases:

```bash
python3 -m tools.semantic_search_grounding.cli predict \
  --dataset /absolute/path/to/grounding_dataset.json \
  --model-dir /absolute/path/to/grounding-dino-tiny \
  --checkpoint-file model.safetensors \
  --model-revision REVISION_COMMIT_SHA \
  --candidate-id grounding-dino-tiny-box005-text005 \
  --licence Apache-2.0 \
  --licence-approved \
  --output /absolute/path/to/grounding_dino_predictions.json
```

The model directory is always loaded with `local_files_only=True`. `predict`
does not download a model, start ROS, train weights, or publish a partial
result. It verifies the final JSON through the frozen R0A loader before an
atomic rename.

Only pass `--licence-approved` after the operator has reviewed the checkpoint
and dependency licences. Omitting it produces valid comparison evidence but
prevents release selection.

## Pure tests

These tests run on the Jetson without Torch, Transformers, Pillow, a desktop
GPU, or a model:

```bash
PYTHONPATH=src/track_robot_semantic_search:. python3 -m pytest -q \
  tools/semantic_search_grounding/test
```

For the complete desktop setup and evaluation sequence, see
`docs/guides/semantic-search/r0b-zero-shot-teacher.md`.
