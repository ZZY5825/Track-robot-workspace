# 绿色瓶 Phase 0–4A 固定底盘综合验证报告

测试日期：2026-07-28
ROS Domain：`20`
查询：`green bottle`
Query ID/version：`2026072813 / 1`
测试时长：25 秒
模式：固定底盘、planning-only、advisory-only

## 结论

Phase 0、1、2A、3、4 和 4A 均达到本次“最小可工作系统”的 PASS 条件。
系统从英文查询生成了有效视觉目标，为目标维持了稳定全局 ID，在
`base_link` 下估计了三维位置，选定目标，并在 LiDAR 局部障碍图上生成了
无碰撞接近路径和人类可读建议。测试期间没有速度或运动目标发布者，机器人
没有收到运动命令。

## 分阶段结果

| 阶段 | 状态 | 主要证据 |
|---|---|---|
| Phase 0 | PASS | 251/251 条固定底盘定位状态健康；frame=`base_link`；未使用 IMU |
| Phase 1 | PASS | 19 条非空语义区域/观测；分数 0.333–0.424；query 引用一致 |
| Phase 2A | PASS | 目标全局 ID 始终为 `17`；50/50 个目标样本具有 `base_link` 三维位置 |
| Phase 3 | PASS | 34 个已选目标样本；ID=`17`；最低置信度 0.503；最大不确定度 0 |
| Phase 4 | PASS | 17 次成功规划；最大路径 37 个 pose；规划 p50=5.042 ms、p95=105.809 ms |
| Phase 4A | PASS | 12 条 `READY` 建议；目标、query、epoch 和全局 ID 与上游一致 |
| 安全门 | PASS | `cmd_vel` 发布者 0；其他运动接口发布者 0；planning/advisory-only 均为 true |
| 跨阶段一致性 | PASS | global ID、query ID/version、定位 epoch、memory epoch 和 planning frame 无冲突 |

## 代表性成功输出

```text
READY target="green bottle"
position=front 2.28m,left 0.08m
range=2.28m bearing=2.1deg
approach=front-left
goal=(1.48,0.08)m standoff=0.80m
path=clear path_length=1.54m
confidence=0.53 uncertainty=0.00
ADVISORY_ONLY
```

在成功窗口中，目标估计约为前方 2.27–2.57 m、左侧 0.08–0.09 m；建议目标点
约为前方 1.47–1.77 m，并保持 0.80 m 停靠距离。以上仅是建议，不会被发送给底盘。

## 数据链与职责

- Phase 1：YOLO-World/DINO 将英文查询和 ZED 图像关联。
- Phase 2A：ZED 注册深度给出目标三维位置；语义记忆维持 `global_object_id=17`。
- LiDAR：生成 240×240、0.05 m/格的局部障碍图，供 Phase 4 碰撞检查。
- Phase 3：保守选择符合当前 query、具有稳定三维位置的对象。
- Phase 4/4A：生成接近候选、选定 goal、路径和文本建议，但不执行运动。

## 已知限制和剩余工作

1. **绝对距离尚未标定。** 布置说明约为 1.6 m，而本次 ZED 深度估计约为
   2.3 m。端到端功能成立，但必须用卷尺和多距离样本验证误差，之后才能用于运动。
2. **相机–LiDAR直接语义绑定未通过。** 运行诊断显示
   `accepted_camera_attachments=0`。现有 prototype 外参和粗 tracklet 投影不足以
   稳定绑定；本次采用 ZED 深度定位目标、LiDAR 检查障碍的工程降级路径。
3. **目标输出仍有间歇。** 125 条建议中有 12 条 `READY`，其余窗口包含
   `no_target`/`missing_target`。因此当前适合固定底盘验证，不适合实际运动。
4. **RViz/驱动关闭缺陷。** RViz 可正常显示，但退出时观察到 OpenGL sampler
   警告并可能以 `-11` 结束；RSLiDAR 在 Ctrl-C 时可能以 `-6` 结束。这些发生在
   测试关闭阶段，没有破坏验证数据，但需要单独修复退出行为。

## 可审计证据

完整机器可读报告：
`phase4a_validation.json`

报告中的固定标识：

- `global_object_id=17`
- `localization_epoch_id=1785248776301029482`
- `memory_epoch_id=4012842245455624155`
- `planning_frame=base_link`
- `query_id=2026072813`
- `query_version=1`
