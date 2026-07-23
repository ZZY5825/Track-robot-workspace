# Semantic Search Evidence

This directory contains small, reviewable evidence files that are intentionally
versioned with the source code. It does not contain raw rosbag databases,
model checkpoints, runtime logs, images, or generated feature tensors.

## Layout

- `manifests/` describes immutable recording inputs, topic inventories,
  checksums, capabilities, queries, and dataset provenance.
- `annotations/` is reserved for reviewed JSONL labels referenced by a
  manifest.
- `calibration/` contains measured or manually reviewed calibration evidence.
- `reports/` contains benchmark, replay, runtime-checkpoint, and evaluation
  results with their provenance and gate status.

Raw recordings live under `rosbags/`. A report must not claim hardware,
accuracy, resource, or safety evidence that its bound manifest and recorded
inputs do not contain.

See the
[semantic-search recording and evaluation guide](../../docs/guides/semantic-search/phase2-recording-and-evaluation.md)
before creating or replacing evidence.
