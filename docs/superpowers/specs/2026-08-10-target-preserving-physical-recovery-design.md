# Phase 4B/5A 目标保持式物理恢复设计

**日期：** 2026-08-10  
**目标分支：** `main`（实施时创建独立 feature 分支）  
**范围：** 在现有 Phase 4B/5A Nav2 监督导航中加入受控旋转和短距离后退恢复；不改变 Phase 1–3 目标选择、Phase 2 ID 所有权、Nav2 正常规划器/控制器或最终速度安全链。

## 1. 目标

当前恢复只执行清理 costmap、等待和重新规划。新恢复流程必须在机器人被局部障碍、控制器进展检查或短时 Nav2 abort 卡住时，依次尝试：

```text
NORMAL_NAVIGATION
  -> CLEAR_AND_REPLAN
  -> SPIN_AND_REPLAN
  -> SAFE_BACKUP_AND_REPLAN
  -> HOLD_AND_RETRY
```

所有恢复步骤都继续追求同一个已授权静态目标。清理路径、控制器和 costmap 绝不能清除正在追踪的目标位置、目标身份、文本查询或操作者授权。

## 2. 现状与约束

现有实现已经具备：

- `semantic_navigation_supervisor` 保存已授权目标引用，并在 Nav2 abort 后保留授权；
- 静态目标在同一 memory/localization/query 域内，可按 `odom` 锚点和 `0.45 m` 半径重新关联；
- Nav2 最多执行两次有界重试，每次间隔 `2.0 s`，重试周期耗尽后仍可保留授权；
- `navigate_supervised.xml` 在失败后清理 local/global costmap、等待 `1 s` 并重新规划一次；
- Nav2 速度输出固定进入 `/nav2/cmd_vel_raw -> motion_safety_supervisor -> cmd_vel_gate -> Bunker`；
- Phase 5A 使用独立 Spin action 主动寻找目标，但该动作不是 Phase 4B 接近任务的故障恢复。

以下约束保持不变：

- 不改变 Phase 2 global ID 和生命周期所有权；
- 不让 Phase 3 学习输出直接控制机器人；
- 不绕过 motion safety supervisor 或 cmd_vel gate；
- `PLANNING_ONLY` 和 `SEMANTIC_SHADOW` 不允许产生运动；
- RC override、E-stop、base fault 和明确的 operator cancel 始终拥有最高终止优先级；
- 恢复不得因目标暂时离开相机画面而更换任务目标。

## 3. 方案选择

采用混合职责方案：

- `semantic_navigation_supervisor` 是语义任务状态的唯一所有者；
- Nav2 Behavior Tree 和 recoveries server 负责路径执行失败后的物理恢复；
- safety supervisor 和 velocity gate 继续负责所有最终运动许可。

不采用独立自研路径跟随或直接发布 `/cmd_vel` 的恢复节点。这样可以复用 Nav2 Foxy 的碰撞预测、生命周期管理和 action 取消语义，同时避免把目标身份状态放进会被重启的 Nav2 行为树黑板。

## 4. 任务状态所有权

恢复期间必须冻结一个 `MissionContext` 语义概念，其数据继续由现有 supervisor 字段持有，不新增公开消息：

- query ID、query version 和 memory/localization domain；
- 已授权目标 global ID；
- 已锁定静态目标的 `odom` 位置和目标引用；
- operator authorization；
- 当前 approach goal；
- recovery cycle、stage、attempt 和最近失败原因。

Nav2 action abort、costmap 清理、Spin、BackUp 和重新规划只能改变 recovery stage、Nav2 goal handle、path 和局部控制状态。以下事件才允许清除任务：

- operator cancel；
- RC override；
- E-stop 或 base hard fault；
- localization domain/reset 导致原 `odom` 锚点失效；
- 目标位置本身被判定为 invalid；
- 成功到达并完成任务。

目标短时不可见、感知帧间隔、Nav2 abort、controller progress timeout、清图、旋转和后退都不是清除任务的理由。

## 5. 恢复状态机

### 5.1 CLEAR_AND_REPLAN

正常导航首次失败后：

1. 停止当前 NavigateToPose action；
2. 清理 local 和 global costmap；
3. 等待 `1.0 s` 让传感器重新标记真实障碍；
4. 对原 approach goal 重新规划和执行。

该步骤保留现有行为，作为最低风险恢复。

### 5.2 SPIN_AND_REPLAN

清图重规划仍失败时，调用 Nav2 Spin action：

- 默认角度 `30 deg`；
- 同一 mission 选定一个方向后保持不变，不在相邻尝试间左右切换；
- 最大角速度沿用受控值 `0.30 rad/s`；
- Nav2 根据当前 footprint 和 local costmap 预测旋转碰撞；
- 完成后清理 costmap，并对原目标重新规划。

Spin 只用于恢复局部视野和控制器几何关系，不重新运行目标选择，也不等同于 Phase 5A 的主动搜索任务。

### 5.3 SAFE_BACKUP_AND_REPLAN

旋转恢复仍失败时，调用 Nav2 BackUp action：

- 默认距离 `0.25 m`；
- 可配置范围限制为 `0.20–0.30 m`；
- 默认速度 `0.10 m/s`；
- 只允许后退，不允许在第一版中自动前冲或执行任意轨迹；
- local costmap、LiDAR obstacle source 和 odometry 必须处于现有 freshness 限制内；
- Nav2 使用完整 `0.88 m x 0.80 m` footprint 预测整个后退轨迹的碰撞；
- 后方未知、被占用、碰撞预测失败或传感器过期时跳过 BackUp，不发送运动；
- 完成后清理 costmap，再对原目标重新规划。

### 5.4 HOLD_AND_RETRY

本轮三种恢复均失败时：

- 发布零速度并保持目标、授权和 approach goal；
- 进入有界 cooldown；
- 传感器、costmap 和 odometry 恢复有效后启动下一轮；
- 不要求操作者重复点击 Start Approaching；
- 诊断信息明确区分 `spin_blocked`、`backup_blocked`、`stale_costmap`、`stale_odom`、`replan_failed` 等原因。

第一版不实现返回历史安全位姿。该能力需要可信的已行驶轨迹缓存和可验证的 odom 连续性，应在短距离 BackUp 实机验证后另立阶段。

## 6. 安全和命令链

所有恢复动作必须复用 Phase 4B Nav2 recoveries server，并将速度 remap 至 `/nav2/cmd_vel_raw`：

```text
Nav2 Spin/BackUp
  -> /nav2/cmd_vel_raw
  -> motion_safety_supervisor
  -> cmd_vel_gate
  -> Bunker base
```

不得新增任何直连最终 `/cmd_vel` 的 publisher。恢复期间：

- RC override 或 E-stop 立即 cancel 当前 recovery action；
- base fault、stale odom、stale costmap/LiDAR 或 TF 失败禁止开始新的物理动作；
- footprint 碰撞预测失败立即停止；
- safety supervisor 的 hard stop 仍可拦截 Nav2 输出；
- motion mode 不是 `SEMANTIC_ACTIVE` 时不发送 Spin 或 BackUp goal。

## 7. 接口与诊断

保持现有公开 topics、messages、services 和 action 接口不变。第一版不新增手动 `/recovery` service。恢复由已授权 approach mission 内部自动触发。

在现有 `/semantic_navigation/diagnostics` 中增加不破坏接口的 key/value：

- `recovery_stage`；
- `recovery_cycle`；
- `recovery_attempt`；
- `recovery_last_failure`；
- `mission_target_global_id`；
- `mission_target_anchor_x/y`；
- `mission_authorization_preserved`；
- `backup_permitted`。

RViz 继续显示原目标和重新规划后的 Path；诊断面板显示当前 recovery stage 和阻断原因。没有必要为第一版创建新的 RViz 按钮。

## 8. 配置

新增参数均采用保守默认值，并限制合法范围：

- `physical_recovery_enabled: false`：行为变化由 feature flag 控制，初始默认关闭；
- `recovery_spin_angle_rad: 0.523599`；
- `recovery_spin_clockwise: false`；
- `recovery_backup_distance_m: 0.25`；
- `recovery_backup_speed_mps: 0.10`；
- `recovery_cooldown_sec: 2.0`；
- `maximum_physical_recovery_cycles: 2`。

测试启动时显式开启 `physical_recovery_enabled`。验证通过并经批准前，不改变现有正式启动的默认行为。

## 9. 测试与验收

### 9.1 单元与合同测试

- Nav2 配置加载 Wait、Spin 和 BackUp，且不存在绕过安全链的 remap；
- 行为树恢复顺序严格为 Clear、Spin、BackUp、Hold；
- 相邻恢复动作保持相同旋转方向；
- BackUp 距离、速度和最大循环次数受到参数边界约束；
- 任一恢复结果不清除目标引用、`odom` 锚点或 operator authorization；
- operator cancel、RC override、E-stop、localization reset 会清除/终止相应任务；
- `PLANNING_ONLY` 和 `SEMANTIC_SHADOW` 不发送 Spin、BackUp 或速度。

### 9.2 集成测试

- 模拟首次规划失败后完成清图重规划；
- 模拟重复失败后执行单方向 Spin，再向同一目标规划；
- 模拟 Spin 失败且后方为空时执行 `0.25 m` BackUp，再向同一目标规划；
- 模拟后方障碍、未知空间、stale odom 和 stale LiDAR，确认 BackUp 被拒绝且速度为零；
- 恢复期间目标观测消失，任务仍保持原目标位置；
- recovery action 执行时触发 RC/E-stop，确认立即 cancel 并阻止后续重试。

### 9.3 实机验收

按顺序执行：

1. 空旷环境仅开启诊断，确认 feature flag 关闭时行为完全不变；
2. 开启恢复但只触发 Clear，确认目标和路径一致；
3. 在可安全原地旋转环境触发 Spin；
4. 在后方至少留出 `1.0 m` 已观测净空时触发 BackUp；
5. 在后方放置障碍，确认 BackUp 被拒绝；
6. 恢复过程中执行 RC override 和 E-stop；
7. 重复至少三轮同一静态目标，确认无需再次点击 Start Approaching，且 global ID/目标锚点没有因 recovery 改变。

实机通过条件：没有安全链旁路、没有错误目标切换、没有未授权运动、没有后方阻挡时后退，且恢复完成后 Nav2 自动继续追求原目标。

## 10. 文件范围

预计修改：

- `track_robot_ws/src/track_robot/track_robot_navigation/behavior_trees/navigate_supervised.xml`；
- `track_robot_ws/src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`；
- `track_robot_ws/src/track_robot/track_robot_navigation/config/semantic_navigation.yaml`；
- `track_robot_ws/src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py`；
- 对应的 Nav2 config、launch、authorization、mission 和 no-motion contract 测试；
- Phase 4B/5A 操作与测试文档。

若 Foxy Behavior Tree 无法在不污染任务状态的情况下表达该恢复顺序，允许新增一个只管理 recovery stage 的内部 Python 模块；它不得拥有目标选择或速度输出职责。

不会修改：

- Phase 1–3 感知和目标排序算法；
- Phase 2 semantic memory ID/lifecycle 所有权；
- Bunker/传感器/PiPER URDF 和 TF；
- Nav2 正常 GridBased A* planner 和 Regulated Pure Pursuit controller；
- motion safety supervisor 和 cmd_vel gate 的公开接口；
- 最终底盘驱动接口。

## 11. 回滚

设计按独立提交实施：配置/BT、任务状态、诊断文档分别经过回归门禁。任何阶段出现目标切换、授权丢失、无障碍后退失败、被阻挡仍后退、no-motion 模式产生速度或现有 Phase 1–5 回归时，立即撤销该独立提交。将 `physical_recovery_enabled` 保持为 `false` 可在不回退代码的情况下恢复当前已测试行为。
