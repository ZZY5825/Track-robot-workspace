# Human Tracking Dependencies

This project runs on ROS2 Foxy, Python 3.8, JetPack/L4T R35.1, and the NVIDIA
PyTorch build:

```text
torch==1.13.0a0+936e9305.nv22.11
torchvision==0.14.1a0+5e8e2f1
```

Do not replace Torch or TorchVision with generic PyPI wheels.

## Installed Python Packages

The human tracking node uses:

```text
ultralytics==8.0.239
lap==0.5.13
filterpy==1.4.5
```

`lap` is used by the YOLO tracker association path. `filterpy` is available for
tracking/filter experiments and future Kalman-filter recovery work.

`ultralytics==8.0.239` is the newest tested compatible version on this system.
Newer tested versions failed with the current Torch 1.13 runtime:

```text
ultralytics==8.3.0   failed loading weights
ultralytics==8.4.82  failed loading weights
```

Both newer versions call a `torch.load` keyword unsupported by the installed
NVIDIA Torch build.

## Model Weights

The default model is:

```text
/home/track-robot/track_robot_ws/models/human_tracking/yolov8n-pose.pt
```

Use the pose model for this pipeline because gesture triggering needs shoulder
and wrist keypoints. A plain detection model such as `yolov8n.pt` can track
people, but it cannot drive the current gesture heuristic.

## Reinstall Command

```bash
python3 -m pip install --user \
  -r ~/track_robot_ws/src/track_robot_perception/requirements-human-tracking.txt
```
