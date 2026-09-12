# Person and pose overlays for the project README

These panels run the project's local YOLOv8n-pose weights on the two existing README frames. Boxes, confidence values and keypoints are actual model predictions. The full camera images are embedded unchanged; labels and geometry are editable SVG layers. PNG copies support GitHub display.

- [Original raised-hand frame](../human-tracking-rosbag-start-gesture.png) · [Annotated panel](human-start-pose-v1.png)
- [Original later frame](../human-tracking-rosbag-later-position.png) · [Annotated panel](human-later-pose-v1.png)
- [Numerical predictions](human-pose-measurements.json) · [Model hash and settings](human-pose-provenance.json)

Teal marks the largest detected person and its confident body keypoints. Gold marks wrists. Other detected people receive pale boxes. The pose cue compares each wrist with its shoulder; it is not a gesture-state or target-lock label. A confirmed start gesture requires temporal evidence in the runtime. No tracking IDs, fusion state or gesture confirmation are inferred from these two isolated images.

From the repository root, render the stored measurements:

```bash
python3 docs/tools/render_readme_pose.py
```

To repeat inference and rendering with the local checkpoint:

```bash
python3 docs/tools/render_readme_pose.py --model /path/to/yolov8n-pose.pt
```

Rendering requires PyGObject with Rsvg and pycairo. Inference additionally requires the project's PyTorch and Ultralytics environment. The provenance records the versions used. No model weights are added to this repository.
