# Human-following replay evidence

`human-following-replay-evidence.png` and its vector PDF were captured from
`human_tracking_lidar_20260706_145900`. The figure shows a confirmed camera–
LiDAR binding to active tracklet `T22`; the fused state reports
`CAMERA_LIDAR_TRACKED`, a 2.24 m target range, and no live sensor was started.

The capture utility requires the camera overlay, raw cloud, tracklet array,
and fused target state to be within 120 ms of one another. In its default
strict mode it also requires:

- `lidar_visible = true`;
- `selected_tracklet_id >= 0`; and
- the selected ID to be active in the synchronized tracklet array.

Those conditions passed on the recorded replay, and the selected LiDAR cluster
was visually checked against the person inside the camera target box. The JSON
sidecar preserves the bag, state, selected ID, confidence, range, and topic
provenance used for the figure.
