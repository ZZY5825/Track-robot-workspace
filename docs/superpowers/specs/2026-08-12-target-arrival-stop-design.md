# Phase 4B/5A 目标到达停止设计

**日期：** 2026-08-12  
**范围：** 只修改 Phase 4B/5A 语义导航监督器的任务终止条件；不改变 Phase 1–3 感知、目标 ID、目标位置生成、Nav2 插件、机器人模型或最终速度安全链。

## 1. 问题

现有 approach planner 使用 `0.8 m` standoff 生成 Nav2 goal，而 Nav2 使用该 goal 的 `0.10 m` XY 容差判断到达。机器人已经非常接近目标时，goal、控制误差或重新规划仍可能促使控制器继续修正，表现为左右摇摆。

系统需要独立于 Nav2 goal checker 的目标相对到达条件：机器人中心到已锁定静态目标的平面距离小于 `0.70 m` 时，任务应停止，不再为同一已授权任务重新规划或进入 recovery。

## 2. 采用方案

到达判定放在 `semantic_navigation_supervisor`，因为该节点已经拥有：

- 已授权目标引用；
- 已冻结的目标 `odom` 锚点；
- 实时 `/odom`；
- 当前 Nav2 action handle；
- safety arm/disarm 客户端。

计算使用冻结的 `odom` 目标锚点与机器人 `/odom` 平面位置，不使用实时视觉置信度、实时 depth 或 LiDAR。这样目标暂时不可见、置信度变化或新候选出现都不会改变到达距离。

## 3. 判定规则

- 参数 `target_arrival_distance_m` 默认 `0.70`，合法范围 `(0.0, 2.0] m`；
- 参数 `target_arrival_confirmation_cycles` 默认 `3`，合法范围 `[1, 20]`；
- 只在已经授权并锁定静态目标、目标 `odom` 锚点有效、odometry 新鲜时判定；
- 平面距离严格满足 `distance < 0.70 m` 才累计一次；
- 任一周期距离不满足或数据无效，连续计数归零；
- 连续三个监督周期满足后，任务进入 `target_reached`；10 Hz 监督频率下确认时间约 `0.3 s`。

`0.70 m` 是机器人参考中心到目标的位置距离，不是机器人外壳到目标的净空距离。

## 4. 到达动作

判定到达时按现有安全链执行：

1. 取消当前 Nav2 `NavigateToPose` action；
2. 清除本次 approach authorization，阻止自动重新 dispatch；
3. 请求现有 safety supervisor disarm，使最终速度归零；
4. 发布 `target_reached` 诊断，并记录最终目标距离；
5. 保留上游语义记忆、global ID 和目标三维位置，不修改 Phase 2/3 数据。

到达逻辑不得直接发布 `/cmd_vel`。新 query 或操作者之后重新发起 approach 可建立新任务，但如果仍小于阈值，新任务也会立即停住。

## 5. 失败与兼容行为

- 没有有效目标锚点、没有 odometry、odometry 过期或数值非有限时，不声明到达，保持现有 fail-safe 行为；
- `SEMANTIC_SHADOW` 仍然不产生运动；
- RC override、E-stop、base fault 和 operator cancel 优先级不变；
- recovery 不能在 `target_reached` 后启动；
- 公共 topic、message、service 和 action 名称不变，只增加配置参数和诊断 KeyValue。

## 6. 验收

- 单元测试覆盖阈值严格小于、连续三周期、计数复位、无效 anchor/odom；
- supervisor 测试证明到达时 cancel、clear authorization、disarm 各执行一次；
- 测试证明到达前仍正常 NAVIGATE，且不会清除上游目标对象；
- navigation、bringup 合同测试与现有 Phase 0–5 回归保持通过；
- 实机将静态目标放置在可控距离，确认大于等于 `0.70 m` 时仍按 Nav2 工作，小于 `0.70 m` 并稳定约 `0.3 s` 后停止且不左右摇摆。
