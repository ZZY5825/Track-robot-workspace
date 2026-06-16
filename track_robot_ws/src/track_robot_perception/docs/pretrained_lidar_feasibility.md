# Pretrained LiDAR Segmentation Feasibility

## Environment Findings

Checked on June 8, 2026:

```text
Platform:              NVIDIA Jetson AGX Orin, aarch64
Jetson Linux:          R35.1
OS / ROS:              Ubuntu 20.04 / ROS 2 Foxy
Python:                3.8.10
PyTorch:               1.13.0a0+936e9305.nv22.11
Torchvision:           0.14.1a0+5e8e2f1
PyTorch CUDA:          available, one device named Orin
PyTorch CUDA version:  11.4
CUDA toolkits:         11.4 and 11.8
cuDNN:                 8.4.1
TensorRT:              8.4.1
NumPy:                 1.24.4
OpenCV:                4.13.0
Open3D:                not installed
scikit-learn:          not installed
```

No new dependency was installed. The CUDA test must run with access to the
Jetson device nodes; a restricted shell can incorrectly report CUDA as
unavailable.

The machine already contains RangeNet++ and SalsaNext code copies below
`~/dynamic_object_removal_tools/LiDAR-MOS/`, including SemanticKITTI label
configuration and inference entry points. No `.pth`, `.pt`, `.ckpt`, `.tar`, or
TensorRT model weight was found there. These copies are part of a moving-object
segmentation project; code presence alone is not a usable pretrained human
semantic model.

At inspection time `/rslidar_points` was not present in `ros2 topic list`.
Attempting `ros2 topic hz` also produced DDS deserialization errors and a
`std::bad_alloc`. Start or restart the RoboSense driver before exporting frames
and make sure only compatible publishers use `/rslidar_points`.

## Model Feasibility

| Model | Task | Pretrained | Human label | Input / output | Main dependencies | Jetson difficulty | ROS 2 difficulty | Helios-32 domain risk | Recommendation |
|---|---|---:|---:|---|---|---|---|---|---|
| RangeNet++ / LiDAR-Bonnetal | Semantic | Yes | `person`, `bicyclist`, `motorcyclist` | SemanticKITTI `.bin`; per-point semantic labels | PyTorch, NumPy, YAML | Medium | Low-medium | High: trained on roof-mounted 64-beam HDL-64E and range projection | **First offline test** |
| `rangenet_lib` | Semantic | Yes | Same SemanticKITTI classes | Single scan; per-point probabilities/labels | C++, CUDA, TensorRT | High | High | High | Do not use original build: official model requires TensorRT 5, device has 8.4 |
| SalsaNext | Semantic | Repository supports evaluation; checkpoint setup is less direct | SemanticKITTI person classes | 5-channel range image; per-point labels | Old Conda/CUDA environment, PyTorch | Medium-high | Medium | High, especially 32 vs 64 scan lines | Second range-image candidate |
| Cylinder3D | Semantic | Yes | SemanticKITTI or nuScenes human-related classes | `.bin` point cloud; cylindrical voxels; per-point labels | `spconv 1.2.1`, `torch-scatter`, Cython | Very high on ARM64 | Medium | Medium-high; less tied to scan rows but still dataset-specific | Not first on this Jetson |
| Panoptic-PolarNet | Panoptic | Yes | SemanticKITTI thing classes include person | `.bin`; semantic labels plus class-agnostic instances | `torch-scatter`, numba, Cython, dropblock | High | Medium | Medium-high | Best later panoptic experiment |
| DS-Net | Panoptic | Yes | SemanticKITTI thing classes include person | `.bin`; semantic and instance IDs | old `spconv`, HDBSCAN, torch cluster/scatter | Very high | High | Medium-high | Not recommended for first deployment |
| Mask3D | Instance | Yes | Dataset-dependent | Dense voxelized indoor scene; masks/classes | MinkowskiEngine, Hydra, sparse convolutions | Very high | High | Very high for sparse spinning LiDAR | Future indoor reference only |
| Human3D | Human instance/body parts | Yes | Human-specific | Dense EgoBody-style 3D scenes | MinkowskiEngine and Mask3D stack | Very high | High | Very high | Research reference, not Helios-32 baseline |

## Recommendation

Test **RangeNet++ through the PyTorch LiDAR-Bonnetal implementation** first,
using a smaller pretrained SemanticKITTI model such as `darknet21` or
`darknet53-512`.

Reasons:

1. It directly produces a semantic label for every input point.
2. Official pretrained SemanticKITTI weights include person-related classes.
3. A single frame is already represented as `[x, y, z, intensity]`.
4. Its network mostly uses ordinary PyTorch 2D operations and avoids old sparse
   convolution packages that are difficult to compile on Jetson ARM64.
5. It is a fast baseline for deciding whether the SemanticKITTI-to-Helios
   domain gap is acceptable before spending time on panoptic frameworks.

This recommendation does not mean the old C++ `rangenet_lib` should be built.
That repository explicitly targets TensorRT 5, which is incompatible with the
installed TensorRT 8.4 without a port. LiDAR-Bonnetal is archived, so it must
also be kept isolated from the ROS workspace and evaluated offline first.

## SemanticKITTI Human Labels

Predictions in original SemanticKITTI label space use:

```text
30   person
31   bicyclist
32   motorcyclist
253  moving-bicyclist
254  moving-person
255  moving-motorcyclist
```

SemanticKITTI `.label` values pack the semantic label into the lower 16 bits
and the instance ID into the upper 16 bits. Semantic-only models normally leave
the instance part at zero. In the 20-class learning space, person, bicyclist,
and motorcyclist are indices `6`, `7`, and `8`; moving variants are merged by
the dataset learning map.

## Export Helios-32 Frames

Start the RoboSense driver and verify:

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 topic list | grep rslidar
ros2 topic info /rslidar_points --verbose
ros2 topic hz /rslidar_points
```

Export 20 frames, saving every tenth incoming cloud:

```bash
ros2 run track_robot_perception export_lidar_frames_node --ros-args \
  -p lidar_topic:=/rslidar_points \
  -p output_dir:=/home/track-robot/track_robot_ws/lidar_pretrained_test_frames \
  -p save_every_n_frames:=10 \
  -p max_saved_frames:=20
```

Each frame produces matching `.npy`, `.bin`, and `.json` files. The `.bin` is
little-endian `float32 [x, y, z, intensity]`.

## Prepare RangeNet++ Input

Select one exported frame:

```bash
FRAME=$(find ~/track_robot_ws/lidar_pretrained_test_frames \
  -maxdepth 1 -name '*.bin' | sort | head -n 1)

python3 ~/track_robot_ws/src/track_robot_perception/scripts/prepare_lidar_for_pretrained_model.py \
  --model-type rangenet \
  --input-frame "$FRAME" \
  --output-dir ~/track_robot_ws/lidar_pretrained_rangenet_input \
  --sequence 08 \
  --frame-index 0
```

The prepared scan is:

```text
~/track_robot_ws/lidar_pretrained_rangenet_input/
└── sequences/08/velodyne/000000.bin
```

## Run RangeNet++ If Already Installed

No semantic model weight is currently installed. The existing LiDAR-MOS
RangeNet++ code can be used as `--model-root` after an appropriate official
semantic model directory is supplied:

```bash
python3 ~/track_robot_ws/src/track_robot_perception/scripts/run_pretrained_lidar_inference.py \
  --model-type rangenet \
  --input-frame "$FRAME" \
  --dataset-root ~/track_robot_ws/lidar_pretrained_rangenet_input \
  --model-root ~/dynamic_object_removal_tools/LiDAR-MOS/mos_RangeNet \
  --checkpoint ~/lidar_models/pretrained/darknet21 \
  --output-dir ~/track_robot_ws/lidar_pretrained_results/rangenet
```

The wrapper calls the repository's semantic inference entry point and saves:

```text
predicted_labels.npy
predicted_instances.npy
debug.json
```

If the repository API differs from the archived official layout, run its
official inference command, then convert its SemanticKITTI `.label` file:

```bash
python3 ~/track_robot_ws/src/track_robot_perception/scripts/run_pretrained_lidar_inference.py \
  --model-type rangenet \
  --input-frame "$FRAME" \
  --dataset-root ~/track_robot_ws/lidar_pretrained_rangenet_input \
  --prediction-label /path/to/000000.label \
  --label-space original \
  --output-dir ~/track_robot_ws/lidar_pretrained_results/rangenet
```

## Visualize Offline Prediction

```bash
python3 ~/track_robot_ws/src/track_robot_perception/scripts/visualize_lidar_prediction.py \
  --input-frame "$FRAME" \
  --labels ~/track_robot_ws/lidar_pretrained_results/rangenet/predicted_labels.npy \
  --instances ~/track_robot_ws/lidar_pretrained_results/rangenet/predicted_instances.npy \
  --output ~/track_robot_ws/lidar_pretrained_results/rangenet/colored_prediction.ply \
  --debug-json ~/track_robot_ws/lidar_pretrained_results/rangenet/visualization_debug.json
```

Person-related labels are red. The PLY is ASCII and does not require Open3D to
create; it can be opened in CloudCompare or another PLY viewer.

## Evaluation Criteria

Evaluate at least 20 frames containing standing, walking, partially occluded,
near, and distant people:

1. Are any true human points assigned `person` or `moving-person`?
2. What proportion of each visible person is covered by the predicted points?
3. Do buildings, poles, vegetation, and robot parts become false people?
4. Are labels stable on the same person across adjacent frames?
5. Do predicted points spatially align with the actual object?
6. Does this independent LiDAR evidence agree with the projected Detectron2
   `person` mask often enough to improve fusion?

Expected domain gap is substantial: 32 instead of 64 channels, different
vertical angles and scan pattern, lower mounting height, tracked-robot motion,
campus/lab scenes rather than road driving, different density, and different
class frequency. Useful output is possible, but accuracy must be measured and
must not be promised before this offline test.
