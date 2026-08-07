# Phase 4B/5A 实机小问题修正方案

## 范围

本轮只处理 2026-08-06 实机问题日志中可以低风险独立修改的项目：

1. 降低动态行人离开后单个残留代价格继续被视为障碍的概率；
2. 在 Phase 5A RViz 中恢复规划路径显示；
3. 避免 Nav2 把 Bunker 的线速度降到无法克服静摩擦的范围；
4. 每次主动旋转完成后提供明确、更长的固定视觉观察时间。

近距离角落停滞的几何原因仍需下一次实机日志确认；YOLO-World 与 DINOv3
架构本轮完全不变。

## 设计决定

### 动态轨迹残留

继续使用现有 Foxy `VoxelLayer`、当前帧 marking 和原始点云 clearing，不安装新的
costmap 插件，也不周期性清空整张地图。周期性清图会短暂移除真实障碍，因此不
接受。

局部三维体素层由 `mark_threshold: 0` 调整为 `mark_threshold: 1`：同一二维栅格
必须保留至少两个被占用的高度体素才能继续成为致命障碍。人在场时会形成多体素
支撑；清除射线已经移除大部分旧体素后，单个未清除体素不再维持一个孤立粉色格。
这是一项保守的残留抑制，而不是声称已经证明所有残留的生命周期根因。全局层不
修改，避免同时改变全局规划语义。

### 路径可视化

Phase 5A RViz 增加两条只读 Path display：

- `/plan`：Nav2 当前执行路径；
- `/semantic_search/phase4/planned_path`：Phase 4A 接近参考路径。

不加入 RViz Nav2 goal 工具，不改变操作者授权或运动接口。

### 最低有效线速度

Phase 5A 实际复用 `nav2_phase4b.yaml`。将 RPP 控制器的
`min_approach_linear_velocity` 与 `regulated_linear_scaling_min_speed` 从
`0.03 m/s` 提高到已由操作者确认可克服 Bunker 静摩擦的 `0.10 m/s`。最高速度
仍为 `0.15 m/s`，安全 supervisor 与最终 cmd_vel gate 保持不变。同步更新独立
`nav2_phase5a.yaml`，防止备用启动入口出现不一致行为。

### 固定观察等待

将 `settle_duration_sec` 从 `0.75 s` 提高到 `2.5 s`。状态机逻辑不扩展：每次
Nav2 Spin 报告完成后固定等待 2.5 秒，之后才允许观察阶段继续决定下一步。按当前
约 2 Hz 的视觉处理目标，这提供约五个处理周期的时间预算。`observation_timeout`
和 ranking 去重本轮不改。

## 回归门槛

- 配置测试必须证明两套 Nav2 配置的最低线速度均为 `0.10 m/s`，最高速度仍为
  `0.15 m/s`；
- 局部 costmap 必须继续使用原始点云 clearing 与过滤点云 marking，并要求
  `mark_threshold == 1`；
- Phase 5A RViz 必须包含两条 Path，且仍不包含手动 Nav2 goal 工具；
- Phase 5A 配置必须固定等待 `2.5 s`；
- 相关 Python 测试和三个受影响 ROS package 必须构建通过；
- 动态轨迹是否在实机消失、角落停滞是否改善只能标记为“待实机验证”，不能由
  静态配置测试宣称修复完成。

## 回滚

四项修改互不依赖，可分别恢复 `mark_threshold`、两项最低速度、RViz display 或
`settle_duration_sec`，不改变 topic、message、action、global ID 或安全命令链。
