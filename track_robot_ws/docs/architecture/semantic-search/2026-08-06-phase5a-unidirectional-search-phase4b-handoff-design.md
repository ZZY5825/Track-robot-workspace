# Phase 5A 单向主动搜索与 Phase 4B 连续接近设计

## 目标

修复两个已在实机测试中出现的问题：Phase 5A 在没有新证据时沿相反方向来回旋转；Phase 5A 找到目标后，RViz 的 `Start Approach` 无法调用 Phase 4B 接近链路。

## 已确认根因

当前默认绝对航向序列为 `+45°, +90°, 0°, -45°, -90°`。相对于机器人当前航向，它产生 `+45°, +45°, -90°, -45°, -45°` 的旋转指令，因此第三步必然反向旋转 90°。

当前 Phase 5A launch 使用 `phase5a_rotation.launch.py`，只启动 controller、recoveries、搜索运动适配器、安全监督和速度门。它没有启动 Phase 4B 的 planner、BT navigator 与 `semantic_navigation_supervisor`，因此 `/semantic_navigation/authorize_approach` 不存在，Finding 的确认结果没有可执行的 Phase 4B 接收端。

## 选定架构

### 单向搜索

默认搜索航向改为 `+45°, +90°, +135°, +180°, +225°, +270°`。初始视角仍先做被动观察；证据不足时，每次只追加 `+45°` 的 Nav2 Spin，并在每个新航向稳定后收集证据。搜索总旋转预算仍为 `270°`，单次旋转仍不超过 `90°`，最大角速度仍为 `0.30 rad/s`，始终禁止线速度。

`SearchForObject.maximum_rotation_angle` 作为单次旋转上限使用；累计预算继续由 `maximum_cumulative_rotation_deg` 独立限制。配置允许使用负数航向序列切换为顺时针搜索，但默认只使用同一个正方向，不在一次任务中反向。

### Finding 到 Approach 的 handover

Phase 5A 监督运行不再启动独立的 rotation-only Nav2 实例，而是启动完整且唯一的 Phase 4B Nav2 栈：planner server、controller server、recoveries server、BT navigator、semantic navigation supervisor、现有 safety supervisor 和 cmd_vel gate。Phase 5A 只额外增加 active-search manager 与 search-motion adapter。

Phase 4B 的 recoveries server 同时加载 `wait` 和 `spin`，使搜索适配器通过 `/spin` 原地搜索，语义接近通过原有 NavigateToPose 链路执行。系统中不得出现第二个 controller、recoveries、安全监督或速度门。

数据流为：

```text
Start Finding
→ SearchForObject
→ 单向 SearchMotionIntent
→ Nav2 Spin
→ safety supervisor → cmd_vel gate → Bunker
→ Phase 3 确认完整目标引用
→ SearchForObject CONFIRMED
→ RViz 保留同一 selected target 引用
→ Start Approach
→ /semantic_navigation/authorize_approach
→ Phase 4B Nav2 NavigateToPose
→ safety supervisor → cmd_vel gate → Bunker
```

Finding 终止时仍停止 Spin 并 disarm。随后点击 `Start Approach` 使用已有 Phase 4B 授权服务重新 arm，不自动开始平移。

## 接口与兼容性

- 保持 `/semantic_search/search_for_object`、`/semantic_search/search_motion_intent`、`/semantic_search/phase4a/selected_target` 和 `/semantic_navigation/authorize_approach` 不变。
- 保持 memory epoch、global object ID、localization epoch、query ID/version 和 snapshot sequence 不变。
- 保持 `/nav2/cmd_vel_raw` → safety supervisor → `/nav2/cmd_vel_safe` → cmd_vel gate → `/cmd_vel` 不变。
- `PASSIVE_ONLY` 与 `SEARCH_SHADOW` 仍不启动可执行运动栈。
- `Start Finding` 不自动触发 `Start Approach`。

## 失败处理

- 单向搜索到达 `270°` 或超时仍无有效目标时，返回既有 NOT_FOUND、UNCERTAIN 或 SEARCH_SPACE_EXHAUSTED 结果。
- Spin、TF、odometry、RC override、E-stop 或 safety fault 继续使用既有 fail-closed 处理。
- Finding 确认后，若 Phase 4B planner 或目标引用尚未同步，`Start Approach` 保守拒绝并显示原有精确原因；不得绕过引用校验。
- 任何时刻不得同时存在 Search Spin 与 NavigateToPose 执行。

## 验收标准

1. 确定性策略测试证明所有非零旋转 delta 同号，序列为六次 `+45°`，累计不超过 `270°`。
2. Phase 5A 监督 launch 中恰好各有一个 planner、controller、recoveries、BT navigator、semantic supervisor、安全监督与速度门，并存在一个 search-motion adapter。
3. Phase 4B Nav2 配置同时提供 `wait` 与 `spin`，Spin 仍限制为 `0.30 rad/s`。
4. `Start Finding` 实机搜索期间线速度为零；目标确认后 Spin 停止。
5. 无需重启 launch，`Start Approach` service 可用，并能接受同一目标引用进入既有 Phase 4B 接近流程。
6. 受影响的语义搜索、导航、RViz 和 bringup 测试零回归。

