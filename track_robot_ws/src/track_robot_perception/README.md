# Track Robot Perception

## Pretrained LiDAR Model Evaluation

The offline-first pretrained LiDAR evaluation workflow is documented in
[`docs/pretrained_lidar_feasibility.md`](docs/pretrained_lidar_feasibility.md).
It includes the Jetson environment report, model comparison, RangeNet++
recommendation, Helios-32 frame export, SemanticKITTI conversion, inference
wrapper, label mapping, visualization, and evaluation criteria.

## LiDAR Human Candidate Segmentation

`lidar_human_segment_node` highlights geometrically plausible human clusters in
red. The combined launch starts adaptive ground segmentation first, then runs:

```text
non-ground points
-> voxel sampling
-> DBSCAN spatial clustering
-> human height and footprint limits
-> PCA verticality test relative to the estimated ground normal
-> local ground-contact test
-> red human-candidate points
```

Run:

```bash
cd ~/track_robot_ws
source install/setup.bash
ros2 launch track_robot_perception lidar_human_segment.launch.py
```

RViz2:

```text
Fixed Frame: rslidar
Display: PointCloud2
Topic: /lidar_human_segmented_points
Color Transformer: RGB8 or RGB
```

The output preserves every input point and field. It keeps the green ground and
the original non-ground colors, turns candidate human clusters red, and adds:

```text
is_human         UINT8, 1 for candidate-human points
human_cluster_id INT32, candidate ID or -1
```

Useful tuning:

```bash
ros2 launch track_robot_perception lidar_human_segment.launch.py \
  cluster_tolerance:=0.35 \
  max_sample_points:=25000 \
  min_human_height:=0.7 \
  max_human_height:=2.4 \
  max_human_width:=1.2 \
  min_verticality:=0.55
```

Inspect candidate measurements:

```bash
ros2 topic echo /lidar_human_candidates_debug
```

This is intentionally a high-recall candidate generator, not a learned human
classifier. Poles, narrow vegetation, mannequins, and bicycles can have similar
LiDAR geometry. The intended next reliability step is to confirm each red 3D
cluster with the existing Detectron2 `person` mask after camera projection.

The design follows established building blocks:

- [PCL Euclidean cluster extraction](https://pointclouds.org/documentation/tutorials/cluster_extraction.html)
  uses spatial connectivity after plane removal.
- [Autoware Euclidean clustering](https://autowarefoundation.github.io/autoware_universe/pr-10075/perception/autoware_euclidean_cluster/)
  uses Euclidean or voxel-grid Euclidean clustering to produce object clusters.
- [PCL ground-based people detection](https://pointclouds.org/documentation/classpcl_1_1people_1_1_ground_based_people_detection_app.html)
  demonstrates ground-relative people detection, but its RGB-D classifier is
  not directly suitable for the RoboSense-only input here.
- [Combining LiDAR Space Clustering and CNNs](https://arxiv.org/abs/1710.06160)
  supports using LiDAR clusters as pedestrian proposals before visual
  classification.
- [LiDAR dense pedestrian detection](https://www.mdpi.com/2076-3417/12/4/1799)
  uses upright-human and body width/height geometry as pedestrian constraints.

## Adaptive Ground Highlighting

`lidar_ground_segment_node` estimates a new ground plane for every LiDAR cloud.
The default `ransac_plane` method is a lightweight baseline inspired by the
lowest-point seed selection used by Patchwork:

```text
finite/range filtering
-> divide XY into cells and keep each cell's lowest point
-> RANSAC plane estimation with a maximum tilt constraint
-> SVD plane refinement using the RANSAC inliers
-> classify points by perpendicular distance to the fitted plane
```

Because the plane is estimated in the current LiDAR frame on every callback,
robot pitch and moderate sensor shaking change the fitted plane instead of
invalidating a fixed Z threshold.

No points are removed. The node copies every original PointCloud2 point and its
existing fields, including fields such as `x`, `y`, `z`, `intensity`, `ring`,
and `timestamp`. It adds:

```text
rgb        packed RGB for RViz
is_ground  UINT8, 1 for ground and 0 for non-ground
```

Ground points use one configurable color, bright green by default. Non-ground
points use grayscale derived from the original intensity, while retaining all
original point data.

Run:

```bash
cd ~/track_robot_ws
source install/setup.bash
ros2 launch track_robot_perception lidar_ground_segment.launch.py
```

Tune the adaptive plane:

```bash
ros2 launch track_robot_perception lidar_ground_segment.launch.py \
  method:=ransac_plane \
  ransac_distance_threshold:=0.18 \
  seed_grid_size:=0.5 \
  ground_fit_max_range:=20.0 \
  max_ground_tilt_deg:=45.0
```

`ransac_distance_threshold` controls the ground band thickness. Increase it for
rough ground or noisy points; decrease it when object bottoms are being marked
as ground. `seed_grid_size` controls the density of lowest-point candidates.

The old fixed-height method remains available as a fallback:

```bash
ros2 launch track_robot_perception lidar_ground_segment.launch.py \
  method:=height \
  ground_z_threshold:=-0.7
```

Change the RGB ground color:

```bash
ros2 launch track_robot_perception lidar_ground_segment.launch.py \
  ground_color:=35,255,80
```

RViz2:

```text
Fixed Frame: rslidar
Display: PointCloud2
Topic: /lidar_ground_segmented_points
Color Transformer: RGB8 or RGB
```

This implementation fits one dominant plane, so it is intended for basic
locally planar ground. Strongly uneven terrain, curbs, multiple road levels, or
long steep transitions need regional plane fitting such as Patchwork/Patchwork++
rather than one global plane.

References used for this baseline and the likely next upgrade:

- [Patchwork paper](https://arxiv.org/abs/2108.05560): concentric zones and
  region-wise ground plane fitting.
- [Patchwork++ paper](https://arxiv.org/abs/2207.11919): adaptive ground
  likelihood estimation, temporal ground reversion, and noise removal.
- [Patchwork++ open-source implementation](https://github.com/url-kaist/patchwork-plusplus)
  with C++, Python, and ROS 2 examples.
- [PCL planar segmentation](https://pointclouds.org/documentation/tutorials/planar_segmentation.html)
  for the standard RANSAC plane model.
- [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) as a
  scan-line/range-image alternative for organized rotating LiDAR data.

## LiDAR-Only Clustering Baseline

`lidar_cluster_baseline_node` independently segments `/rslidar_points` into 3D
object candidates. It does not use ZED images or Detectron2 results.

This branch is needed because camera-mask projection and LiDAR clustering answer
different questions:

```text
Camera-mask projection:
camera recognition -> assign camera labels to projected LiDAR points

LiDAR clustering:
LiDAR geometry -> independently discover 3D object candidates
```

The baseline performs:

```text
PointCloud2
-> finite/range/ROI filtering
-> simple ground-height removal
-> voxel downsampling
-> SciPy cKDTree DBSCAN or Euclidean clustering
-> cluster shape filtering
-> 3D boxes, centroids, labels, and optional colored cloud
```

No additional heavy Python dependency is required. SciPy is already installed
on the Jetson. The node implements both `dbscan` and `euclidean` methods with
`scipy.spatial.cKDTree`.

Outputs:

```text
/lidar_cluster_markers   visualization_msgs/MarkerArray
/lidar_clusters_debug    std_msgs/String JSON
/lidar_clustered_points  sensor_msgs/PointCloud2, optional
```

Run:

```bash
cd ~/track_robot_ws
source install/setup.bash
ros2 launch track_robot_perception lidar_cluster_baseline.launch.py
```

Enable the colored clustered point cloud:

```bash
ros2 launch track_robot_perception lidar_cluster_baseline.launch.py \
  publish_clustered_cloud:=true
```

Use Euclidean connected-component clustering instead of DBSCAN:

```bash
ros2 launch track_robot_perception lidar_cluster_baseline.launch.py \
  method:=euclidean \
  euclidean_tolerance:=0.35
```

### LiDAR Clustering RViz

Open RViz2:

```bash
rviz2
```

Use `rslidar` as the fixed frame. Add:

```text
PointCloud2  /rslidar_points
MarkerArray  /lidar_cluster_markers
PointCloud2  /lidar_clustered_points  # when enabled
```

For `/lidar_clustered_points`, select `RGB8` or `RGB` as the color transformer.

Debug:

```bash
ros2 topic hz /rslidar_points
ros2 topic echo /lidar_clusters_debug
ros2 topic hz /lidar_cluster_markers
```

### LiDAR Clustering Tuning

Tune in this order:

1. `ground_z_threshold`: raise it if ground fragments remain; lower it if object
   bottoms disappear.
2. `voxel_size`: increase for speed and noise reduction; decrease for small or
   distant objects.
3. `dbscan_eps` or `euclidean_tolerance`: increase if one object splits; decrease
   if nearby objects merge.
4. `dbscan_min_samples`: increase to reject sparse noise; decrease to retain
   distant sparse objects.
5. `min_cluster_points`: increase to remove tiny false clusters.
6. ROI and range limits: reduce the search space for the robot's operating area.

Typical first adjustment:

```bash
ros2 launch track_robot_perception lidar_cluster_baseline.launch.py \
  ground_z_threshold:=-0.6 \
  dbscan_eps:=0.30 \
  dbscan_min_samples:=6 \
  min_cluster_points:=15 \
  voxel_size:=0.08
```

Expected baseline limitations include ground fragments, walls becoming large
clusters, nearby objects merging, sparse distant objects disappearing, and
cluster jitter during robot motion. RANSAC ground removal and temporal tracking
are later improvements.

The later fusion stage will compare:

```text
Detectron2 camera mask
+ LiDAR 3D cluster
+ projection/bounding-box overlap
= object candidate confirmed by both sensors
```

## ZED2i Mask R-CNN Instance Segmentation

`zed_mask_rcnn_node` runs Detectron2 Mask R-CNN inference on the ZED2i left RGB
image and publishes:

- `/mask_rcnn/annotated_image` as a `sensor_msgs/Image` with masks, boxes,
  class labels, and confidence scores drawn on the image.
- `/mask_rcnn/detections_text` as `std_msgs/String` JSON containing compact
  detection metadata.

The pipeline is:

```text
ZED image -> Detectron2 Mask R-CNN -> instance masks + boxes + labels -> annotated ROS image
```

The prototype uses the COCO-pretrained Detectron2 model zoo config:

```text
COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml
```

## ZED2i Human Pose / Skeleton Estimation

`zed_pose_rcnn.launch.py` runs the same Detectron2 ROS node with a COCO
Keypoint R-CNN model. It detects people and draws the human skeleton/keypoints
on the output image.

The pose pipeline is:

```text
ZED image -> Detectron2 Keypoint R-CNN -> person boxes + keypoints + skeleton -> annotated ROS image
```

The launch file uses:

```text
COCO-Keypoints/keypoint_rcnn_R_50_FPN_1x.yaml
```

Run:

```bash
ros2 launch track_robot_perception zed_pose_rcnn.launch.py
```

Visualize:

```bash
ros2 run rqt_image_view rqt_image_view /pose_rcnn/annotated_image
```

Inspect keypoint metadata:

```bash
ros2 topic echo /pose_rcnn/detections_text
```

The text output includes named COCO keypoints such as `nose`, `left_shoulder`,
`right_elbow`, `left_wrist`, `right_hip`, `left_knee`, and `right_ankle`.

## ZED2i RF-DETR Small Detection

`zed_rfdetr_small_node` applies the COCO-pretrained `RFDETRSmall` detector to
the ZED2i left rectified RGB stream. It publishes:

- `/rfdetr/annotated_image`: bounding boxes, class labels, and scores.
- `/rfdetr/detections_text`: JSON detection metadata with original-image
  bounding-box coordinates.

The official Small model uses 512x512 inference internally and returns boxes,
confidence scores, and COCO class IDs. This is the detection model, not the
RF-DETR segmentation model, so it does not publish pixel masks.

Build the ROS package:

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select track_robot_perception
source install/setup.bash
```

Run from an environment where the official `rfdetr` package is installed:

```bash
ros2 launch track_robot_perception zed_rfdetr_small.launch.py
```

For a ZED node named `zed2i`:

```bash
ros2 launch track_robot_perception zed_rfdetr_small.launch.py \
  image_topic:=/zed2i/zed_node/left/image_rect_color
```

Visualize and inspect results:

```bash
ros2 run rqt_image_view rqt_image_view /rfdetr/annotated_image
ros2 topic echo /rfdetr/detections_text
```

Useful launch overrides:

```bash
ros2 launch track_robot_perception zed_rfdetr_small.launch.py \
  score_threshold:=0.6 \
  run_every_n_frames:=5 \
  max_detections:=20
```

The latest official RF-DETR package requires Python 3.10+, PyTorch 2.2+, and
torchvision 0.17+. Its TensorRT extra requires TensorRT 8.6.1+. The current
JetPack 5.0.2 ROS Foxy host uses Python 3.8, PyTorch 1.13, and TensorRT 8.4, so
do not install RF-DETR into that working host environment. In particular,
`pip` on Python 3.8 exposes an unrelated placeholder package named
`rfdetr==0.0.1`; that is not Roboflow RF-DETR. The node intentionally imports
RF-DETR only at runtime so the remaining ROS package continues to build.

## DINOv3 ViT-S+/16 Feature Extraction

`zed_dinov3_feature_node` runs the pretrained DINOv3 ViT-S+/16 visual backbone
on ZED2i RGB images. DINOv3 is a feature extractor, not a Mask R-CNN-style
detector. This baseline publishes:

- `/dinov3/debug_image`: patch-feature norm heatmap overlaid on the input image.
- `/dinov3/feature_debug`: JSON with tensor shapes, device, and inference time.
- Optional `.npy` class-token and patch-token files for offline experiments.

At the default 512x512 input, the ViT-S+/16 output is expected to contain:

```text
class token:  [384]
patch tokens: [32, 32, 384]
```

The heatmap is only a visualization of feature magnitude. It is not a semantic
or instance segmentation mask. Dense features can later support clustering,
semantic similarity, image-LiDAR association, tracking, and downstream
segmentation adapters.

The exact official model entry point is:

```text
dinov3_vits16plus
```

This Jetson uses ROS Foxy with Python 3.8 and PyTorch 1.13. The current official
DINOv3 repository requires a newer Python/PyTorch environment for its complete
training and evaluation stack. Do not upgrade the working ROS PyTorch/CUDA
environment in place. Use `local_repo` with an approved compatible checkout and
checkpoint, or deploy an exported model in a separate runtime.

The official `torch.hub` load was tested on this machine. Repository download
succeeded, but import stopped before weight download because Python 3.8 cannot
evaluate the repository's Python 3.10 union annotations (`float | None`).
Therefore pretrained ViT-S+/16 weights have not yet been loaded on this Jetson.

Build:

```bash
cd ~/track_robot_ws
colcon build --symlink-install --packages-select track_robot_perception
source install/setup.bash
```

Run with the installed ViT-S+/16 checkpoint:

```bash
ros2 launch track_robot_perception zed_dinov3_feature.launch.py
```

The Python 3.8-compatible backbone checkout is stored at:

```text
/home/track-robot/track_robot_ws/src/track_robot_core/third_party/dinov3_py38
```

The launch file uses this path by default. It contains narrowly scoped
compatibility changes for postponed annotations, optional training-only imports,
and an unfused attention fallback for PyTorch 1.13. Architecture-only inference
was verified with 28,697,472 parameters, 384 feature dimensions, four storage
tokens, and the expected patch-token output.

For a ZED node named `zed2i`:

```bash
ros2 launch track_robot_perception zed_dinov3_feature.launch.py \
  image_topic:=/zed2i/zed_node/left/image_rect_color \
  weights_path:=/home/track-robot/track_robot_ws/models/dinov3_vits16plus_pretrain_lvd1689m.pth
```

Visualize with `rqt_image_view`, or add an Image display in RViz2:

```bash
ros2 run rqt_image_view rqt_image_view /dinov3/debug_image
ros2 topic echo /dinov3/feature_debug
```

Offline test:

```bash
python3 ~/track_robot_ws/src/track_robot_perception/scripts/test_dinov3_on_image.py \
  --image /path/to/test.jpg \
  --model-source local_repo \
  --local-repo /home/track-robot/track_robot_ws/src/track_robot_core/third_party/dinov3_py38 \
  --weights-path /home/track-robot/track_robot_ws/models/dinov3_vits16plus_pretrain_lvd1689m.pth \
  --device cuda \
  --input-size 512 \
  --output-dir ~/track_robot_ws/dinov3_feature_outputs/test
```

## LiDAR Mask Projection

`lidar_mask_projector_node` runs Detectron2 Mask R-CNN internally, projects
RoboSense points into the ZED2i rectified image, samples the instance masks, and
publishes a semantic point cloud:

```text
ZED image -> Detectron2 instance masks
LiDAR cloud -> LiDAR-to-camera TF -> image projection
projected pixel + mask lookup -> class ID + instance ID + confidence
```

Output topic:

```text
/lidar_semantic_points
```

The output `PointCloud2` remains in the input LiDAR frame by default and contains:

```text
x, y, z         FLOAT32
intensity       FLOAT32
rgb             FLOAT32 packed RGB for RViz
class_id        INT32
instance_id     INT32
confidence      FLOAT32
```

Unknown points use `class_id=-1`, `instance_id=-1`, and `confidence=0.0`.
Detectron2 COCO class IDs are zero-based. The `instance_id` is only stable inside
one inference result; it is not a tracked object ID across frames.

### Run Mask Projection

Start the ZED2i, RoboSense LiDAR, and the calibrated LiDAR-to-camera TF first.
Then launch:

```bash
cd ~/track_robot_ws
source install/setup.bash
ros2 launch track_robot_perception lidar_mask_projector.launch.py
```

This launch reuses the prototype static transform from
`lidar_camera_colorizer.launch.py` and enables it by default:

```text
zed_camera_link -> rslidar
x=-0.05, y=0.0, z=0.20
yaw=1.08, pitch=-0.03, roll=0.0
```

Disable it when another calibrated TF publisher already provides this transform:

```bash
ros2 launch track_robot_perception lidar_mask_projector.launch.py \
  publish_static_tf:=false
```

For a lighter Jetson test:

```bash
ros2 launch track_robot_perception lidar_mask_projector.launch.py \
  run_inference_every_n_images:=10 \
  project_every_n_clouds:=2 \
  resize_width:=640 \
  max_mask_age_sec:=1.0
```

If the ZED namespace is `zed2i`:

```bash
ros2 launch track_robot_perception lidar_mask_projector.launch.py \
  image_topic:=/zed2i/zed_node/left/image_rect_color \
  camera_info_topic:=/zed2i/zed_node/left/camera_info \
  camera_frame:=zed2i_left_camera_optical_frame
```

The node uses `CameraInfo.K` for this first prototype. The selected image and
camera-info topics must describe the same rectified left image.

### Visualize Semantic LiDAR

Check the output:

```bash
ros2 topic hz /lidar_semantic_points
ros2 topic echo /lidar_semantic_points --once
```

Open RViz2:

```bash
rviz2
```

Use the input LiDAR frame, normally `rslidar`, as the fixed frame. Add a
`PointCloud2` display, select `/lidar_semantic_points`, and choose `RGB8` or
`RGB` as the color transformer.

To inspect only labelled points:

```bash
ros2 launch track_robot_perception lidar_mask_projector.launch.py \
  publish_only_labelled_points:=true \
  max_mask_age_sec:=1.0
```

Projection quality depends directly on the LiDAR-to-ZED optical-frame
calibration. A wrong TF causes shifted labels even when Detectron2 masks are
correct. The first version uses the latest mask result and rejects it when its
timestamp differs from the LiDAR cloud by more than `max_mask_age_sec`.

`timestamp_mode` defaults to `auto`. It uses message header timestamps when they
are compatible. If the LiDAR stamp is zero or differs from the camera clock by
more than 60 seconds, it automatically uses local receipt times and the latest
TF. Available values are `auto`, `header`, and `receipt`.

### Jetson Environment Status

Installed and verified on this Jetson AGX Orin:

```text
JetPack 5.0.2 / L4T R35.1
Python 3.8.10
torch 1.13.0a0+936e9305.nv22.11
torch.cuda.is_available() True
torch.version.cuda 11.4
torchvision 0.14.1a0+5e8e2f1
detectron2 0.6
Detectron2 CUDA compiler 11.4
Detectron2 arch flags 8.7
cv2 4.13.0
rclpy ok
cv_bridge ok
```

The installed PyTorch wheel came from NVIDIA's JetPack 5.0.2 `jp/v502` wheel
index. `torchvision` and Detectron2 were built from source with:

```text
CUDA_HOME=/usr/local/cuda-11.4
TORCH_CUDA_ARCH_LIST=8.7
FORCE_CUDA=1
```

The node imports PyTorch before OpenCV/cv_bridge to avoid a Jetson OpenMP TLS
loader issue:

```text
libgomp.so.1: cannot allocate memory in static TLS block
```

### Check ZED Image Topic

```bash
ros2 topic list | grep image
ros2 topic hz /zed/zed_node/left/image_rect_color
```

If your ZED node uses `camera_name:=zed2i`, the image topic may be:

```text
/zed2i/zed_node/left/image_rect_color
```

### Start the ZED2i Camera

Source the ROS2 workspace and launch the installed ZED wrapper:

```bash
cd ~/track_robot_ws
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Confirm that the RGB stream is available:

```bash
ros2 topic hz /zed/zed_node/left/image_rect_color
```

View the raw ZED image:

```bash
ros2 run rqt_image_view rqt_image_view /zed/zed_node/left/image_rect_color
```

The optional `zed_display_rviz2` package is not installed in this workspace.
Standard `rviz2` can still be used for ZED depth and point-cloud topics:

```bash
rviz2
```

In RViz2, set the fixed frame to `zed_camera_link`, then add an `Image`,
`PointCloud2`, or `DepthCloud` display as needed. A useful point-cloud topic is:

```text
/zed/zed_node/point_cloud/cloud_registered
```

### Native ZED SDK Tools

To test the camera directly through the Stereolabs SDK, stop the ROS ZED launch
first and run:

```bash
/usr/local/zed/tools/ZED_Explorer
```

Other installed SDK tools:

```bash
/usr/local/zed/tools/ZED_Depth_Viewer
/usr/local/zed/tools/ZED_Sensor_Viewer
/usr/local/zed/tools/ZED_Diagnostic
```

The native SDK tools and the ROS ZED node should not access the same camera at
the same time.

### Run

Build and source the package:

```bash
cd ~/track_robot_ws
colcon build --symlink-install --packages-select track_robot_perception
source install/setup.bash
```

Launch with automatic device selection:

```bash
ros2 launch track_robot_perception zed_mask_rcnn.launch.py
```

Use the alternate ZED2i topic and reduce GPU/CPU load:

```bash
ros2 launch track_robot_perception zed_mask_rcnn.launch.py \
  image_topic:=/zed2i/zed_node/left/image_rect_color \
  score_threshold:=0.6 \
  run_every_n_frames:=5 \
  resize_width:=960
```

If CUDA memory is tight or inference is too slow, increase
`run_every_n_frames` or set a smaller `resize_width`.

### Parameters

- `image_topic`: default `/zed/zed_node/left/image_rect_color`
- `output_image_topic`: default `/mask_rcnn/annotated_image`
- `output_text_topic`: default `/mask_rcnn/detections_text`
- `model_config`: default `COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml`
- `score_threshold`: default `0.5`
- `device`: direct node default is `cuda` when PyTorch CUDA is available,
  otherwise `cpu`; launch default is `auto`
- `publish_annotated_image`: default `true`
- `publish_text`: default `true`
- `run_every_n_frames`: default `1`
- `resize_width`: default `0`, which keeps the original image size
- `max_detections`: default `20`

## LiDAR Camera Colorizer

`lidar_camera_colorizer` colorizes the RoboSense LiDAR cloud with the ZED2i
rectified RGB image. The LiDAR cloud provides the 3D geometry. The camera image
provides color only. The camera info message provides pinhole intrinsics, and TF
provides the extrinsic transform from the LiDAR frame into the ZED optical frame.

For each LiDAR point, the node:

1. Reads `x`, `y`, and `z` from `/rslidar_points`.
2. Transforms the point into the configured camera optical frame.
3. Projects it with `u = fx * X / Z + cx` and `v = fy * Y / Z + cy`.
4. Samples the RGB image at `(u, v)` when the point is in front of the camera and
   inside the image.
5. Publishes `/lidar_colored_points` with fields `x`, `y`, `z`, and `rgb`.

The output points stay in the original LiDAR frame by default.

## Build

```bash
cd ~/track_robot_ws
colcon build --symlink-install --packages-select track_robot_perception
source install/setup.bash
```

## Run

Start the ZED2i and RoboSense LiDAR first. Then run:

```bash
ros2 launch track_robot_perception lidar_camera_colorizer.launch.py
```

If your ZED node uses `camera_name:=zed2i`, use:

```bash
ros2 launch track_robot_perception lidar_camera_colorizer.launch.py \
  image_topic:=/zed2i/zed_node/left/image_rect_color \
  camera_info_topic:=/zed2i/zed_node/left/camera_info \
  camera_frame:=zed2i_left_camera_optical_frame
```

The launch file can publish a rough prototype static transform, but it is
disabled by default because the transform direction must be verified on the
robot:

```bash
ros2 launch track_robot_perception lidar_camera_colorizer.launch.py \
  publish_static_tf:=true \
  static_tf_parent_frame:=zed_camera_link \
  static_tf_child_frame:=rslidar \
  static_tf_x:=-0.05 \
  static_tf_y:=0.0 \
  static_tf_z:=0.20 \
  static_tf_yaw:=1.08 \
  static_tf_pitch:=-0.03 \
  static_tf_roll:=0.0
```

For the static transform, translation is expressed in the parent frame
(`zed_camera_link` by default), and rotation follows the ROS2
`static_transform_publisher` Euler order: yaw, pitch, roll. The colorizer then
uses TF to project points into `zed_left_camera_optical_frame`, whose convention
is +Z forward, +X right, and +Y down.

## RViz2

Use fixed frame:

```text
rslidar
```

Add:

```text
/lidar_colored_points -> PointCloud2
```

Set the PointCloud2 color transformer to `RGB8` or `RGB`.

## Debug Commands

```bash
ros2 topic list | grep zed
ros2 topic list | grep image
ros2 topic list | grep camera_info
ros2 topic echo /zed/zed_node/left/camera_info --once
ros2 topic echo /rslidar_points --once
ros2 run tf2_ros tf2_echo rslidar zed_left_camera_optical_frame
ros2 run tf2_ros tf2_echo zed_left_camera_optical_frame rslidar
ros2 topic hz /lidar_colored_points
```

## Common Failure Cases

- Wrong TF direction: projected colors appear shifted, mirrored, or absent.
- Wrong camera frame: projection must use an optical frame where +Z points forward.
- Timestamps are not synchronized: colors lag or smear during motion.
- Image topic is not rectified: `CameraInfo.P` no longer matches the image.
- Most LiDAR points are outside camera FOV: output remains mostly fallback gray.
- RGB/BGR channel mismatch: colors look swapped.
- RViz color transformer is wrong: select `RGB8` or `RGB` for the `rgb` field.

## References

- `leo-drive/color-point-cloud`: ROS2 point cloud colorization from camera images.
- Mindkosh colorized LiDAR article: projection-based LiDAR/camera colorization.
- Stereolabs ZED ROS2 docs: ZED image topics, camera info, and optical frame names.
