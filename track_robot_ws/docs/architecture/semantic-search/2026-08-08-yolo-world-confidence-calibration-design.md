# YOLO-World 距离与语义混淆离线标定设计

## 1. 目的与边界

本实验回答两个可测问题：

1. `green bottle` 的 YOLO-World 分数是否主要随距离增加、目标像素面积减小而下降；
2. 绿色盒子、绿色纸巾盒、黄色圆柱体等 hard negative 是否在相同像素尺度下仍与
   目标分数重叠，说明存在语义混淆。

实验只新增采集、离线推理、分析和报告工具。生产 perception node、模型权重、
query、阈值、Phase 2/3 排名和导航行为均不修改。实验输出不能直接成为生产阈值；
任何集成决定必须在看过真实数据报告后另行设计。

## 2. 方案比较与选择

- 直接订阅 `/semantic_search/regions` 最接近线上输出，但目标漏检时没有候选记录，
  也不能在完全相同图像上重复运行 ROI 验证。
- 完整 rosbag 可以保留全部 ROS 证据，但数据量大，解码与标注流程复杂。
- 同步保存 ZED RGB 和注册深度，再调用生产共用的 `YoloWorldBackend` 离线推理，
  可以记录漏检、重复推理并对同一候选比较 CLIP 和 DINOv3。

采用第三种；rosbag 仅作为可选旁证，不是运行分析的必需输入。

## 3. 受控数据矩阵

主矩阵包含一个 target 和四个 hard negative：

- `green_bottle`，ground-truth kind 为 `target`；
- `green_box`；
- `green_tissue_box`；
- `yellow_cylindrical_object`；
- `other_visually_similar_object`。

每种物体分别放置在相机光轴距离 1、2、3、4、5 m；每个静态 trial 采集 10 组
同步 RGB+Depth，共 250 帧。相机姿态、分辨率、曝光策略和场景照明在一个 session
中保持不变；画面中一次只放一个受测物体。每个 trial 在第一帧人工框选一次真实
ROI，并将同一像素 ROI 传播到该 trial 的其余静态帧。物体或相机移动后必须新建
trial，不能继续沿用 ROI。

最小完整性门槛为：target 的五个距离均有样本，至少三类 hard negative 各自覆盖
五个距离，每格至少 5 帧。未达到门槛时仍可生成采集清单和原始图，但结论状态必须
为 `NOT_EVALUATED_INCOMPLETE_MATRIX`。

## 4. 数据协议

数据集根文件为 `dataset.json`，版本
`semantic_confidence_dataset/1.0.0`。它记录：

- dataset/session/trial/sample ID；
- query `green bottle`；
- `target`、`distractor` 或 `background` ground-truth kind；
- 人工标签、名义距离、人工 ROI；
- ROS Domain、RGB/Depth topic、source stamp、frame ID；
- 原始 PNG、float32 depth NPY 的相对路径、尺寸和 SHA-256；
- git commit 和采集备注。

离线推理输出 `candidates.jsonl`，每行对应一个检测候选，记录：

- sample/candidate ID 和与人工 ROI 的 IoU；
- `target`、`distractor` 或 `background` 匹配标签；
- YOLO-World confidence、XYWH、宽、高、像素面积和画面面积比例；
- 名义距离、候选 ROI 中心区域的 ZED 深度中位数及有效深度比例；
- 原始分辨率候选 crop 的相对路径和 SHA-256；
- 可选 CLIP 正提示相似度、最大 hard-negative 相似度及 margin；
- 可选 DINOv3 与一个人工确认 target reference crop 的余弦相似度；
- YOLO、CLIP 和 DINO 的分项耗时。

每个 sample 另写一行 frame summary。目标帧没有与人工框 IoU `>=0.30` 的候选时，
该帧成为 false-negative 候选；hard-negative 帧有重叠候选时成为 false-positive
候选。背景中的其他框仍保存，但不冒充受测物体标签。

## 5. 模型复用

- YOLO 使用生产共用 `YoloWorldBackend.from_local_model`，保持
  YOLOv8s-World-v2、640 输入、FP16、CUDA、0.70 IoU；为了观察低分重叠，离线
  confidence floor 固定为 0.05。
- ROI-CLIP 使用现有 `OpenAIClipAdapter` 的同一 ViT-B/32 本地 checkpoint。
  正提示为 `green bottle`；负提示来自数据集中 hard-negative 的英文标签。
  每个原始候选 crop 只编码一次，计算
  `positive_similarity - max(hard_negative_similarity)`。
- DINOv3 继续只做 image-to-image 身份证据。操作者显式指定一个近距离、人工确认
  的 target sample；其 ground-truth crop 形成 reference，候选 crop 与其计算余弦。
  不计算 CLIP-text-to-DINO cosine。

CLIP 和 DINO 均可关闭。工具必须在输出中明确记录 `disabled` 或错误原因，不能用
0.0 冒充可用分数。

## 6. 分析与图表

报告固定生成：

1. `confidence_vs_distance.png`：target/hard-negative 散点和每距离箱线统计；
2. `confidence_vs_bbox_area.png`：confidence 对数像素面积散点；
3. `score_distributions.png`：target 与 hard-negative 原始分数分布；
4. `failure_cases.png`：在诊断阈值下最高分 false positive 与最低分/漏检 target；
5. 启用 CLIP 时增加 `roi_margin_distributions.png`。

`summary.json`、`samples.csv` 和 `report.md` 同时记录：

- 每距离样本数、检测召回、confidence P50/P95、bbox area P50；
- target 与 hard-negative 分数范围及重叠区间；
- confidence 对距离和 log(pixel area) 的 Spearman 相关；
- 0.00–1.00、步长 0.01 的单阈值扫描；
- 是否存在同时达到 target frame recall `>=0.90` 且 hard-negative frame false
  accept rate `<=0.05` 的诊断阈值；
- 可选的 YOLO threshold + CLIP margin 二维扫描是否提高 precision，同时保持不低于
  原始方案的 far-target（4–5 m）recall。

这些门槛只用于判断分数能否分离，不是生产验收门槛。来自同一静态 burst 的帧高度
相关，因此报告必须声明该结果是受控诊断，不是泛化精度。

## 7. 命令与安全

采集工具只订阅 ZED topic 并写入用户指定目录，不发布 query、速度、goal 或任何
机器人控制消息。离线工具不依赖 ROS graph。所有路径使用相对路径和哈希，JSON
采用原子写入。

测试分三层：纯数据协议/指标单元测试、伪模型离线流水线测试、真实本地模型 preflight。
真实 250 帧结论只有在操作者完成受控摆放和人工 ROI 后产生；代码测试不得用合成
图片声称真实模型性能。
