# Phase 1 query 切换导致 overlay 停止更新：故障分析与修复

日期：2026-08-12
范围：Jetson AGX Orin，ROS 2 Foxy，Phase 1 YOLO-World 感知节点
状态：代码修复及真实模型离线回归通过；下一次完整实机启动仍需确认 RViz
overlay 连续更新

## 1. 故障现象

运行 Phase 1–5A 后，首个英文 query 可以正常产生候选框。通过 RViz 面板
切换到另一个 query 后，Phase 1 overlay 停留在最后一帧，不再更新候选框。

初始 ROS graph 检查发现：

- `/semantic_search/query` 仍有发布者；
- `/semantic_search/regions` 不再产生新消息；
- overlay 本身没有独立推理能力，只能保留最后收到的图像/候选结果。

受控复现进一步确认，故障发生时 perception 诊断持续报告：

```text
state=degraded
reason=frame_processing_failed
error=RuntimeError: YOLO-World vocabulary update failed
```

因此问题不在 RViz、ZED、DINOv3、Camera–LiDAR 关联或 Nav2，而在
YOLO-World 更新文本类别的路径。

## 2. 根因

当前管线使用：

- YOLOv8s-World-v2；
- OpenAI CLIP ViT-B/32 文本编码器；
- CUDA FP16 图像推理；
- Ultralytics 8.2.103 的动态 `set_classes()`。

第一个 query 调用 `set_classes()` 时，Ultralytics 加载并缓存 CLIP 文本
编码器。随后第一次 YOLO FP16 推理会对整个 YOLO module tree 执行半精度
转换，缓存于其中的 CLIP 模型也因此变成 FP16。

第二个 query 再次调用 `set_classes()` 时，CLIP 使用该缓存模型重新编码文本。
其 LayerNorm 路径将输入按 FP32 处理，但权重已被前一次推理转为 FP16，产生
真实底层异常：

```text
RuntimeError: expected scalar type Float but found Half
```

异常发生在：

```text
YoloWorldBackend._set_query()
  -> YOLOWorld.set_classes()
  -> WorldModel.set_classes()
  -> CLIP.encode_text()
  -> LayerNorm
```

原实现把底层异常统一包装成 `YOLO-World vocabulary update failed`，导致现场
只能看到 overlay 停止，无法直接看到 dtype 不匹配。

## 3. 修复内容

修改文件：

```text
track_robot_ws/src/track_robot_semantic_search/
  track_robot_semantic_search/yolo_world_backend.py
  test/test_yolo_world_backend.py
```

修复位于 `YoloWorldBackend._set_query()`：

1. query 未变化时仍直接复用当前文本特征；
2. query 变化时，检查 YOLO 内是否已有缓存的 `clip_model`；
3. 仅在重新编码文本前调用 `cached_clip.float()`，将 CLIP 恢复为 FP32；
4. 调用原有 `set_classes([query])`；
5. YOLO 图像推理继续使用原有 CUDA FP16，不改变输入尺寸、阈值或检测逻辑；
6. 若更新仍失败，诊断现在保留底层异常类型和文本。

该修改不改变任何公共 topic、message、query ID/version、候选 ID 或 Phase 2–5
接口。

## 4. 回归测试

### 4.1 自动化测试

新增两项回归覆盖：

- 词表更新失败时保留底层异常；
- 首次推理已把缓存 CLIP 转成 FP16 后，切换 query 前恢复 FP32。

执行：

```bash
cd ~/track_robot_ws/.worktrees/main-test/track_robot_ws
source /opt/ros/foxy/setup.bash
source ../install/setup.bash
source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_yolo_world_backend.py \
  src/track_robot_semantic_search/test/test_yolo_world_perception_core.py \
  src/track_robot_semantic_search/test/test_yolo_world_node_contract.py \
  src/track_robot_semantic_search/test/test_live_overlay.py
```

验证结果：

```text
28 passed in 0.30 seconds
```

其中 backend 定向测试结果为：

```text
9 passed in 0.11 seconds
```

### 4.2 真实 checkpoint 验证

使用生产路径中的本地模型文件：

```text
models/r0c/yolov8s-worldv2.pt
models/phase1/ViT-B-32.pt
```

在 CUDA FP16 图像推理之间连续执行：

```text
green bottle -> yellow cylinder -> green bottle
```

每次 query 更新后均完成一次实际 YOLO-World 推理，结果：

```text
PASS query sequence: green bottle -> yellow cylinder -> green bottle
```

## 5. 实机复验方法

完整栈启动后：

1. 输入 `green bottle` 并确认 Phase 1 overlay 持续更新；
2. 不重启 perception 节点，切换为另一个英文 query；
3. 确认 overlay 在模型重新编码期间短暂停顿后恢复；
4. 再切回 `green bottle`；
5. 确认 `/semantic_search/regions` 仍有一个发布者并继续更新；
6. 检查 `/semantic_search/perception_diagnostics` 中没有
   `vocabulary update failed` 或 `Float but found Half`。

检查命令：

```bash
ros2 topic info /semantic_search/regions --verbose
ros2 topic hz /semantic_search/regions
ros2 topic echo /semantic_search/perception_diagnostics
```

## 6. 已验证边界

本次验证证明真实 YOLO-World/CLIP 模型可以在同一进程中连续切换 query 并继续
推理。由于最后一次验证时完整 ZED/RViz 栈已由操作者停止，尚未记录修复后的
完整实机 RViz 视频或长时间连续切换统计；这些属于下一轮实机复验，不应写成
已经完成。
