# Raw Rosbag Recordings

This directory contains local rosbag2 recordings. Database payloads are large
runtime data and are ignored by Git; small `metadata.yaml` files may be
versioned to document available recordings.

## Layout

- `human_tracking/recordings/` contains camera/LiDAR human-tracking sessions.
- `semantic_search/recordings/` is the destination for Phase 0–2
  semantic-search sessions.

Versioned manifests, annotations, calibration evidence, and evaluation reports
live under `artifacts/semantic_search/`. Operator procedures live under
`docs/guides/`.

Before replaying a bag, stop live camera or LiDAR drivers that publish the same
topics on the same ROS domain. Follow the relevant replay guide:

- [Human-tracking replay](../docs/guides/human-tracking/rosbag-replay.md)
- [Semantic-search rosbag workflow](../docs/guides/semantic-search/rosbag-workflow.md)
