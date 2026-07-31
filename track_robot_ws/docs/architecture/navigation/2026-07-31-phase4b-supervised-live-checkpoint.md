# Phase 4B 监督语义接近：实机开发检查点

## 1. 检查点范围

- 分支：`feature/phase4b-nav2-supervised`
- 本轮改动前提交：`42ea396`
- 平台：Jetson、ROS 2 Foxy、Bunker、ZED 2i、RoboSense LiDAR
- ROS Domain：`20`
- 目标：静态 `green bottle`
- 控制模式：Nav2 监督执行；IMU 不参与；RC、E-stop、安全监督器和
  `cmd_vel_gate` 保持有效

本检查点记录 Phase 4B 从“可启动”到“可由 RViz 人工授权并尝试接近”的
实际工程改动。它不是完整运动验收报告，也不把尚未关闭的安全阻塞问题描述成
成功结果。

## 2. 当前执行链

```text
English query
  -> YOLO-World / Phase 1 observations
  -> Phase 2 semantic memory and camera-depth position
  -> Phase 3 selected_target
  -> Phase 4A standoff goal and path
  -> semantic_navigation_supervisor
  -> Nav2 NavigateToPose
  -> /nav2/cmd_vel_raw
  -> motion_safety_supervisor_node
  -> /nav2/cmd_vel_safe
  -> cmd_vel_gate
  -> /cmd_vel
  -> Bunker
```

RViz 的 `Start Approach` 只提交带引用的操作员请求，不发布速度。最终速度仍
必须依次通过安全监督器和速度门控。

## 3. 已完成的工程改动

### 3.1 单命令启动与 RViz

- 修复 Phase 4A/4B include 共享 `start_rviz` 参数导致 Phase 4B RViz 未按预期
  启动的问题；Phase 4B 使用独立的 `start_phase4b_rviz` 参数。
- 标准入口同时启动传感器、Phase 0–4B、Nav2、底盘和 RViz，并固定 Domain 20、
  禁用 IMU。
- RViz 只以 Phase 3 `selected_target` 作为按钮就绪条件；supervisor 仍负责最终
  引用、规划和安全校验。
- 移除 `safety_arm_pending` 作为长期界面状态，按钮显示
  `starting approach` / `approach enabled (supervised)`。
- Phase 4B 不启动全量 semantic-memory marker，也不显示未选中对象的“语义盒”；
  对象记忆本身仍正常运行。

### 3.2 静态目标连续性

- Camera tracker 加入最长 2 s 的有界常速度框预测，补偿机器人运动或检测间隔；
  检测标签不同的目标不能共享 camera track ID。
- Phase 3 selector 优先保留已经确认的同一目标，即使竞争候选短暂排到第一；
  保留时使用最新目标位置，不重复发布旧快照。
- 阈值按当前实机验证配置调整为：新目标 `0.26`，已确认目标保留阈值 `0.24`。
- 针对实测最大 3.51 s 的 Orin 语义输出间隔，静态测试 profile 将 Phase 2、3、
  4A、4B 的证据期限统一限制在 4.0 s。该 profile 有显式开关和硬上限，不用于
  声明动态目标能力。
- 已授权静态目标允许最长 1.0 s 的目标输入中断。odom、RC、E-stop、底盘健康
  和安全状态不使用该宽限。
- 上游 global ID 重建时，只有 memory/localization/query 域一致且目标在 `odom`
  锚点 0.45 m 内，才重关联为同一静态目标。

### 3.3 授权与安全启动

- RViz 快照序号允许小于等于 supervisor 当前序号，解决约 10 Hz 快照在服务
  往返期间自然前进造成的误拒绝；目标身份仍必须完全一致，未来/零序号仍拒绝。
- 安全层允许武装后以零速度等待第一条 Nav2 命令，修复旧的生命周期零命令被
  立即判为 `planned_command_stale` 的启动竞态。
- 一旦收到第一条真实 Nav2 命令，原有 0.15 s command watchdog 恢复严格生效。
- RC override、E-stop、odom 陈旧、底盘故障和安全硬停止没有被绕过。

### 3.4 动态障碍显示与代价地图

- `/safety/filtered_obstacle_points` 在 local/global costmap 中同时用于 marking
  和 clearing；persistence 保持为零，用于消除行人离开后的旧痕迹。
- Nav2 inflation radius 从 `0.1625 m` 降为 `0.105625 m`；Bunker 实体 footprint
  仍为 `1.20 x 1.00 m`，没有缩小实体碰撞模型。
- 蓝色/粉色区域是 Nav2 costmap 与 inflation，不是语义地图或目标分类结果。

## 4. 遇到的挑战及处理结果

| 挑战 | 已确认原因 | 处理 | 当前状态 |
| --- | --- | --- | --- |
| Phase 4B RViz 空白或进入错误 ROS 图 | include 参数名冲突、手动 RViz 可能未继承同一 DDS 环境 | 独立 `start_phase4b_rviz`，由受管入口统一启动 | 已修复，需继续回归 |
| `Start Approach` 显示候选不相关 | UI 同时要求 best candidate 与 selected target 瞬时一致 | UI 只使用 selected target；supervisor 保留最终校验 | 已修复 |
| 点击后长期显示 `safety_arm_pending` | 武装与第一条 Nav2 命令之间存在零命令 freshness 竞态 | 加入 bounded waiting-for-first-command 状态 | 已修复 |
| 机器人运动时 global ID 变化导致取消 | 视觉轨迹和上游 memory ID 会因推理间隔重建 | 有界框预测、selector stickiness、odom 静态锚点重关联 | 已实现；实机仍观测到 87→102→125，需继续验证稳定性 |
| 行人走过后 costmap 留下“蜗牛痕迹” | filtered source 只 marking、不 clearing | filtered source 同时 clearing + marking，persistence=0 | 已实现，需动态场景复测 |
| 空旷区域仍停车 | 安全监督器报告 `safety_obstacle_blocked`，Nav2 无法产生有效位移 | 已完成日志和 rosbag 取证，尚未降低或绕过安全判定 | **未修复** |

## 5. 2026-07-31 实机证据

当前手动测试成功启动全部节点，Phase 1 overlay、best candidate、Phase 4A 路径和
RViz 操作入口可用。导航日志记录了多次操作员授权以及 Nav2 goal 接收。

最后一次可复现事件：

1. `1785530663.032`：安全武装请求被接受；
2. `1785530663.036`：操作员授权 object 125；
3. `1785530663.059`：supervisor 报 `NAVIGATE (goal_accepted)`；
4. `1785530663.078`：Nav2 controller 接收 goal；
5. `1785530663.158` 起：supervisor 报 `safety_obstacle_blocked`；
6. controller 仍约每秒接收更新路径，但机器人没有达到 0.10 m progress 条件；
7. `1785530693.079`：controller 报 `Failed to make progress` 并 abort；
8. supervisor 随后清除授权并回到 disarmed。

同轮点云诊断约有 25,000 个原始点、8,700 个过滤后障碍点；最近稳定点约位于
`(0.63, -0.90, 0.42–0.64) m`，footprint clearance 约 `0.39–0.40 m`。该数据说明
当前停止来自安全碰撞预测，但尚不足以断言该点是实体障碍、自车回波、外参误差
还是地面过滤残留。下一步必须用同步的 safety state、planned command 和过滤点云
复现后分类，不能直接删除安全逻辑。

## 6. 软件回归证据

本轮提交前，在 Phase 4B worktree 中对以下 6 个相关 package 执行完整
`colcon test`：

- `track_robot_semantic_search`
- `track_robot_semantic_memory`
- `track_robot_navigation`
- `track_robot_safety`
- `track_robot_bringup`
- `track_robot_semantic_search_rviz_plugins`

结果为 `1327 tests, 0 errors, 0 failures, 4 skipped`。代码按职责拆分为：

| 提交 | 内容 |
| --- | --- |
| `20365bb` | 静态语义目标连续性、global ID 位置重关联和短时目标丢失宽限 |
| `955f7ee` | 安全监督器等待首条 Nav2 命令的有界启动状态 |
| `121f51f` | 动态障碍 clearing、膨胀范围和非目标语义 marker 清理 |
| `3eeecc3` | Phase 4B 统一启动入口和 RViz 人工授权交互 |

这些结果证明软件回归测试通过；它们不替代机器人运动与障碍识别的实机验收。

## 7. 当前正确手动启动命令

```bash
cd ~/track_robot_ws/.worktrees/phase4b-nav2/track_robot_ws

source /opt/ros/foxy/setup.bash
source install/setup.bash

export TRACK_ROBOT_WS=~/track_robot_ws
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/track_robot_ws/src/track_robot/track_robot_bringup/config/fastdds_semantic_search.xml

ros2 run track_robot_bringup semantic_search_ctl run phase4b
```

## 8. 发布边界

本检查点可以作为 Phase 4B 功能分支供代码审查和继续调试，但不能标记为完整
运动验收或正式稳定 release。下一项阻塞是关闭空旷环境的错误
`safety_obstacle_blocked`，然后重复验证连续移动、障碍移除后恢复、RC override
和 E-stop。
