# Human-following replay evidence

`human-following-replay-evidence.png` and its PDF counterpart were generated
from the recorded ROS 2 bag `human_tracking_lidar_20260706_145900`. The figure
combines the live perception overlay with the corresponding LiDAR point cloud,
active tracklets, and target-state diagnostics.

This replay confirms gesture-authorized camera target lock and LiDAR tracklet
generation. It does **not** show a confirmed camera–LiDAR association: the
captured state is `CAMERA_ONLY / CAMERA_LOCKED`, with
`selected_tracklet_id = -1`. The figure labels that limitation explicitly so it
is suitable as diagnostic evidence without implying a fusion result that the
recording did not produce.

Machine-readable provenance and captured state are stored in
`human-following-replay-evidence.json`. The reproducible capture utility is
`tools/visualization/capture_human_following_evidence.py`.
