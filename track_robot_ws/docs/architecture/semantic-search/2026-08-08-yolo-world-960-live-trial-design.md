# YOLO-World 960 实机试验设计

## 决策

将语义搜索生产共用配置中的 YOLO-World `input_size` 默认值从 640 改为 960。
Phase 1、4A、4B 和 5A 均通过同一个
`track_robot_semantic_search/config/semantic_search_yolo_world.yaml` 启动感知，
因此不增加新的 launch 参数或并行配置文件。

## 依据

在同一批 50 组真实 ZED RGB/注册深度图上，960 输入相对于 640：

- 2 m 检出从 0/10 恢复为 10/10；
- 4 m 检出从 0/10 恢复为 10/10；
- 1 m 和 3 m 保持 10/10；
- 5 m 仍为 0/10；
- YOLO 推理 P50/P95 从 40.56/45.76 ms 增至 49.47/51.08 ms。

1280 输入只恢复 2/10 的 5 m 样本，同时将 3 m 降为 5/10，因此不采用。

## 范围

本次只改变 YOLO-World 输入尺寸默认值。以下内容保持不变：

- YOLOv8s-World-v2 checkpoint、CUDA FP16、confidence floor、IoU 和 max detections；
- DINOv3 224×224 输入、checkpoint、context crop 和身份描述符；
- 文本 query、ROS topic/message、Phase 2 global ID、Phase 3 排名；
- Phase 4/5、Nav2、安全链和底盘控制；
- 离线 benchmark 默认的 640 基线，便于继续与生产候选进行可复现对比。

## 验证与回滚

增加配置契约测试，确认生产 YAML 为 960，且 Phase 1、4A、4B、5A 继续引用该集中
配置。运行语义搜索包与 bringup 的相关测试及完整回归。

实机试验使用原标准启动流程，检查 perception diagnostics、regions/observations rate、
YOLO 延迟、GPU 显存和远距离候选。该变更不以一次实机试验作为永久性能结论。

回滚只需把集中配置中的 `input_size` 恢复为 640；公共接口和数据无需迁移。
