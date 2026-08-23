# Human-following replay evidence

No publication result is currently committed in this directory. The earlier
unbound diagnostic composite was withdrawn because its camera overlay and
LiDAR tracklet snapshot were not paired by sensor timestamp.

`tools/visualization/capture_human_following_evidence.py` now requires the
camera overlay, raw cloud, tracklet array, and fused target state to be within
120 ms of one another. In its default strict mode it also requires:

- `lidar_visible = true`;
- `selected_tracklet_id >= 0`; and
- the selected ID to be active in the synchronized tracklet array.

The next PNG/PDF result should only be committed after those conditions pass
on recorded replay and the selected cluster is visually checked against the
camera target.
