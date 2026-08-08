# YOLO-World 绿色瓶子置信度受控实验

本实验只采集 ZED RGB/注册深度并离线运行与生产系统共享的
`YoloWorldBackend`。它不会发布查询、目标、导航或速度命令，也不会修改生产阈值。

## 实验回答的问题

通过固定目标、距离和背景，分别检查：

- 距离增加、框像素面积减小时，绿色瓶子置信度是否系统性下降；
- 绿色盒子、绿色纸巾盒、黄色圆柱体等干扰物是否与绿色瓶子分数重叠；
- 是否存在同时满足目标召回率不低于 90%、干扰物误接受率不高于 5% 的单一
  YOLO 阈值；
- 在候选 ROI 上增加 CLIP 正样本/困难负样本 margin 后，能否保持 4–5 m
  目标召回率并提高 precision；
- 选定一张人工确认的目标 ROI 后，DINOv3 图像特征相似度是否有额外区分能力。

一次静态 burst 内的帧高度相关，因此只能用于诊断；生产参数必须至少再用另一天或
另一段独立采集验证。

## 受控场景矩阵

保持 ZED 分辨率、光照和相机高度不变。每次画面只放一个测试物体，物体中心尽量在
同一水平位置，实际距离用卷尺从相机近似光心量取。

| 真值 | 标签 | 距离 | 每格帧数 |
|---|---|---|---:|
| target | `green_bottle` | 1、2、3、4、5 m | 10 |
| distractor | `green_box` | 1、2、3、4、5 m | 10 |
| distractor | `green_tissue_box` | 1、2、3、4、5 m | 10 |
| distractor | `yellow_cylinder` | 1、2、3、4、5 m | 10 |
| distractor | `other_similar_object` | 1、2、3、4、5 m | 10 |

完整建议矩阵为 250 帧。工具的最低完整性门槛是：目标覆盖全部五个距离，至少三个
干扰物类别覆盖全部五个距离，每格至少五帧。不满足时报告明确标记为
`NOT_EVALUATED_INCOMPLETE_MATRIX`，不输出“可分离”的生产结论。

## 1. 构建与启动 ZED

```bash
cd ~/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select track_robot_semantic_search
source install/setup.bash
export TRACK_ROBOT_WS=~/track_robot_ws
export ROS_DOMAIN_ID=20
```

另开终端，仅启动 ZED：

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

确认两个输入存在：

```bash
ros2 topic hz /zed/zed_node/left/image_rect_color
ros2 topic hz /zed/zed_node/depth/depth_registered
```

## 2. 采集每个静态 trial

```bash
export BENCH=~/track_robot_ws/artifacts/yolo_world_green_bottle_2026-08-08
mkdir -p "$BENCH"

ros2 run track_robot_semantic_search semantic_search_confidence_capture \
  --dataset "$BENCH/dataset.json" \
  --trial-id target-green-bottle-1m \
  --kind target \
  --label green_bottle \
  --distance-m 1 \
  --samples 10 \
  --interval-sec 0.5 \
  --interactive-roi
```

弹窗出现后，只框住物体本体并按 Enter。相机和物体在十帧采集期间必须保持静止。
依次把 `trial-id`、`kind`、`label`、`distance-m` 替换为矩阵对应值。若 Jetson 无
桌面，也可把最后一行替换为人工量取的：

```bash
  --roi X Y WIDTH HEIGHT
```

同一个 `trial-id` 不允许覆盖；RGB PNG 与 float32 深度 NPY 都保存 SHA-256。

查看矩阵是否完整：

```bash
ros2 run track_robot_semantic_search semantic_search_confidence_benchmark \
  status --dataset "$BENCH/dataset.json"
```

## 3. 离线运行同一 YOLO-World 后端

基础 YOLO 运行：

```bash
ros2 run track_robot_semantic_search semantic_search_confidence_benchmark \
  infer \
  --dataset "$BENCH/dataset.json" \
  --output-dir "$BENCH/run_yolo"

ros2 run track_robot_semantic_search semantic_search_confidence_benchmark \
  report \
  --dataset "$BENCH/dataset.json" \
  --run-dir "$BENCH/run_yolo"
```

默认参数与当前生产候选一致：`yolov8s-worldv2.pt`、输入 640、CUDA FP16；为观察
低分候选，离线检测 floor 固定为 0.05。每张图记录完整 YOLO 推理耗时，调用 CUDA
synchronize 后再计时。

## 4. 可选 ROI 语义验证

下面命令在同一候选 ROI 上增加现有 OpenAI CLIP ViT-B/32，并可同时加入 DINOv3。
`target-green-bottle-1m-000` 必须是数据集中人工框选并确认的目标样本：

```bash
ros2 run track_robot_semantic_search semantic_search_confidence_benchmark \
  infer \
  --dataset "$BENCH/dataset.json" \
  --output-dir "$BENCH/run_roi_verification" \
  --enable-clip-verifier \
  --clip-hard-negative-prompt "green box" \
  --clip-hard-negative-prompt "green tissue box" \
  --clip-hard-negative-prompt "yellow cylindrical object" \
  --enable-dino-verifier \
  --dino-reference-sample-id target-green-bottle-1m-000

ros2 run track_robot_semantic_search semantic_search_confidence_benchmark \
  report \
  --dataset "$BENCH/dataset.json" \
  --run-dir "$BENCH/run_roi_verification"
```

CLIP margin 定义为：`green bottle` 相似度减去最相似困难负样本文本的相似度。
DINOv3 只做候选 ROI 与确认目标参考 ROI 的图像特征余弦相似度，不承担文本理解。

## 5. 输出格式

采集目录：

```text
dataset.json                    严格数据清单、真值与 provenance
images/<sample_id>.png          原始全分辨率 ZED 左目图像
depth/<sample_id>.npy           对齐图像的 float32 注册深度
```

每个运行目录：

```text
run.json                        checkpoint/run 状态、P50/P95、显存增量
frames.jsonl                    每帧检测/漏检、真值距离和推理耗时
candidates.jsonl                每个候选的分数、bbox、3D 距离、ROI 指标
crops/*.png                     每个检测候选的原分辨率 ROI crop
ground_truth_crops/*.png        人工真值 ROI crop
samples.csv                     候选平表
summary.json                    阈值扫描、相关性和诊断结论
report.md                       短报告
confidence_vs_distance.png
confidence_vs_bbox_area.png
score_distributions.png
failure_cases.png
roi_margin_distributions.png    仅启用 CLIP 时生成
```

候选与人工 ROI 的 IoU 不低于 0.30 时，继承该 trial 的 `target` 或 `distractor`
真值；其他候选标记为 `background`。3D 距离取候选框中心 50% 区域内有效注册深度的
中位数，并同时记录有效像素比例。

## 结论边界

- `report.md` 只陈述测得分布、相关性和阈值扫描结果，不自动修改生产参数。
- 若目标与干扰物分数重叠，不应仅凭均值声称存在可用阈值。
- ROI 验证只有在不降低 4–5 m recall 的前提下提高 precision，才标记为改善。
- 缺少完整矩阵、人工真值、模型文件或有效注册深度时，必须保留未评估状态。
