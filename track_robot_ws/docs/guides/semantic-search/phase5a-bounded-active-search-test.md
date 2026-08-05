# Phase 5A 有界主动搜索测试指南

本文用于测试静态目标的“先被动观察，必要时原地转向换视角，再把同一目标引用交回 Phase 4”的最小系统。Phase 5A 不执行平移搜索，不自动开始 Phase 4 接近，也不发布 `Twist`。

## 当前证据状态

冻结的 Phase 0–4B 功能基线为 `76c8ead`；Phase 5A 实现分支为 `feature/phase5a-active-search`，本文编写前的实现提交为 `a153c40f3abd6b8afbba1da3931bdd9ca0c56aba`。

| 能力 | 当前证据 | 状态 |
|---|---|---|
| 被动、shadow、监督旋转三种模式及独立执行门 | 源码、launch/config 契约测试 | 已实现、单元测试通过 |
| 有界航向序列 `+45°, +90°, 0°, -45°, -90°` | 确定性策略测试 | 已实现、单元测试通过 |
| 单次转角不超过 `90°`、累计不超过 `270°`、角速度不超过 `0.30 rad/s` | 策略、适配器和 Nav2 配置测试 | 已实现、单元测试通过 |
| 多视角证据绑定完整目标引用键 | 24 场景确定性回放 | 回放测试通过 |
| Phase 5A 不生成线速度、不直接连接 `/cmd_vel` | 源码契约和回放 | 测试通过 |
| Nav2 Spin、现有安全监督和速度门可加载 | ROS Domain 91 无硬件冒烟测试 | 通过；缺少 TF 时按预期等待 |
| PASSIVE_ONLY/SEARCH_SHADOW 实机运行 | 尚未执行 | NOT EVALUATED |
| 支撑架原地旋转、地面旋转、RC 接管、E-stop | 尚未执行 | NOT EVALUATED |
| 实机发现视角、转角误差、停止延迟、总任务延迟 | 尚无测量 | NOT EVALUATED |

软件回归结果：语义搜索 856、语义记忆 10、导航 78、bringup 186、安全 5，共 `1135 passed, 4 skipped`。七个受影响 package 已构建成功。跳过项为原有语义记忆测试，不视为 Phase 5A 实机证据。

## 安全边界

- 默认 `PASSIVE_ONLY`，不启动底盘、Nav2 旋转服务器或速度安全链。
- `SEARCH_SHADOW` 只发布建议航向，`rotation_permitted=false`。
- 只有显式 `--rotation-supervised` 才启动底盘和 Nav2 Spin；每个新查询仍必须调用一次授权 service。
- 即使已授权，Nav2 输出仍依次经过 `/nav2/cmd_vel_raw` → motion safety supervisor → `/nav2/cmd_vel_safe` → cmd_vel gate → `/cmd_vel`。
- Phase 5A 的搜索意图始终 `forward_permitted=false`。目标确认后只返回目标引用，不自动开始 Phase 4B 接近。
- 首次物理旋转必须把机器人可靠架空，遥控器和 E-stop 保持可用。不得从本指南直接跳到地面测试。
- 本阶段不使用 IMU 作为搜索或安全判定依据。

## 共同准备

所有终端从同一工作树启动：

```bash
cd ~/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export TRACK_ROBOT_WS=~/track_robot_ws
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/track_robot_ws/src/track_robot/track_robot_bringup/config/fastdds_semantic_search.xml
```

先做只读检查：

```bash
ros2 run track_robot_bringup semantic_search_ctl doctor phase3
```

必须确认相机、LiDAR、模型文件、TF 和 Phase 1–3 输入没有硬错误。测试模式之间切换时，先在另一个已配置终端执行：

```bash
ros2 run track_robot_bringup semantic_search_ctl stop
```

## RViz 一键 Finding 验收（需人工监督）

本节是 Phase 5A RViz 面板的操作验收，不属于自动化测试，也不声称已
完成硬件运动验收。首次物理旋转前必须可靠架空机器人，保持 RC 和 E-stop
可用。

在同一工作树中启动监督旋转模式：

```bash
cd ~/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export TRACK_ROBOT_WS=~/track_robot_ws
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 run track_robot_bringup semantic_search_ctl run phase5a --rotation-supervised
```

在 RViz 面板中输入 `green bottle`。先把瓶子放在初始相机视野外、但位于
有界原地旋转可见的范围内；点击 **Start Finding**。确认按钮变为
**Stop Finding**，并确认机器人只原地旋转。结果应显示已确认的全局对象 ID，
或一个有界的终止失败原因；不得自动开始接近。

第二次运行时，在正在旋转期间点击 **Stop Finding**。确认动作和待执行旋转
意图均被取消，取消后没有命令的旋转。最后，在没有目标的情况下再运行一次：
它必须在 60 秒内结束，并且整个过程不得平移。

这些项目须由操作者观察并记录；自动 build/test 只能验证软件契约，不能验证
底盘的实际运动、停止延迟、RC 接管或 E-stop。

## 测试 1：PASSIVE_ONLY

目标先放在相机视野内。启动：

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a
```

另一个终端发送一次 action：

```bash
ros2 action send_goal /semantic_search/search_for_object \
  track_robot_interfaces/action/SearchForObject \
  "{query_text: 'green bottle', timeout: {sec: 60, nanosec: 0}, allow_rotation: false, maximum_rotation_angle: 0.0, client_request_id: 'phase5a-passive-1'}" \
  --feedback
```

预期：

- 初始视角证据足够时返回 `CONFIRMED=0`；不足时保守返回 `NOT_FOUND=1` 或 `UNCERTAIN=2`。
- `searched_headings_deg` 为空，不发送可执行旋转。
- `/nav2/cmd_vel_raw`、`/nav2/cmd_vel_safe` 和 `/cmd_vel` 没有 Phase 5A 发布者。
- 确认结果的 `evidence_summary.target_reference` 同时包含 `memory_epoch_id`、`global_object_id`、`localization_epoch_id`、`query_id` 和 `query_version`。

## 测试 2：SEARCH_SHADOW

停止上一轮后，把目标移出初始视野，然后启动：

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a --search-shadow
```

发送允许搜索但不允许执行的任务：

```bash
ros2 action send_goal /semantic_search/search_for_object \
  track_robot_interfaces/action/SearchForObject \
  "{query_text: 'green bottle', timeout: {sec: 60, nanosec: 0}, allow_rotation: true, maximum_rotation_angle: 1.5708, client_request_id: 'phase5a-shadow-1'}" \
  --feedback
```

检查：

```bash
ros2 topic echo /semantic_search/search_motion_intent
ros2 topic echo /semantic_search/active_search/diagnostics
```

预期：建议视角确定且有界，每条建议均为 `rotation_permitted: false`、`forward_permitted: false`；系统不启动底盘，三层速度 topic 不产生 Phase 5A 运动命令。shadow 只验证决策，不声称目标确实从新视角被看到。

## 测试 3：支撑架监督旋转

只有在机器人可靠架空后才执行：

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a --rotation-supervised
```

终端 B 发送任务：

```bash
ros2 action send_goal /semantic_search/search_for_object \
  track_robot_interfaces/action/SearchForObject \
  "{query_text: 'green bottle', timeout: {sec: 60, nanosec: 0}, allow_rotation: true, maximum_rotation_angle: 1.5708, client_request_id: 'phase5a-support-stand-1'}" \
  --feedback
```

等 diagnostics 显示 `WAITING_FOR_AUTHORIZATION` 后，在终端 C 授权一次：

```bash
ros2 service call /semantic_search/active_search/authorize_rotation std_srvs/srv/Trigger '{}'
```

这次授权只绑定当前 query，但在同一 query 的后续航向间保持有效。新 query 必须重新授权。随时停止：

```bash
ros2 service call /semantic_search/active_search/cancel std_srvs/srv/Trigger '{}'
ros2 service call /safety/disarm std_srvs/srv/Trigger '{}'
```

支撑架验收：

- 授权前无旋转；授权后只允许原地 Spin。
- 任意 `/nav2/cmd_vel_raw`、`/nav2/cmd_vel_safe`、`/cmd_vel` 的 `|linear.x| <= 0.001 m/s`。
- `|angular.z| <= 0.30 rad/s`，单次旋转 `<=90°`，累计旋转 `<=270°`。
- 每次 Spin 完成后，角速度低于 `0.03 rad/s` 并稳定至少 `0.75 s` 才收集新证据。
- 目标确认后返回完整目标引用，但不启动 Phase 4 接近。

## 测试 4：地面安全与失败场景

支撑架全部通过后，才逐项独立测试取消、RC 接管、E-stop、base fault、stale odom、stale TF、Nav2 Spin 失败、目标不存在、持续歧义和定位 epoch 重置。每项重新启动干净任务，禁止一次同时注入多个故障。

记录以下实测值，不允许估计：

- 每个停靠航向的角度误差，目标 `<=5°`；
- 任务总时长，目标 `<=60 s`；
- cancel、RC、E-stop 的停止延迟，目标 `<=0.30 s`；
- 三层速度 topic 的最大绝对线速度；
- 发现目标的视角及完整 handoff 引用键；
- 精确终止状态和 `terminal_reason`。

建议报告保存为：

```text
artifacts/semantic_search/phase5a/phase5a_validation_<YYYY-MM-DD>.md
```

在尚未完成物理测试前，对应字段必须写 `NOT EVALUATED`。

## 测试结束

```bash
ros2 run track_robot_bringup semantic_search_ctl stop
ros2 node list
```

确认被本次受管启动拥有的进程已经退出。若仍存在外部启动的传感器节点，先确认其来源，不要用全局 `pkill` 清理。

## 当前剩余阻塞项

- Foxy Nav2 Spin 在 Bunker 履带底盘上的实际角度误差和停止延迟尚未测量。
- `4.5 s` 单视角观察窗口是否覆盖 Jetson 上最慢的 Phase 1 推理间隔，需通过 shadow 实测决定。
- prototype Camera–LiDAR 外参只能用于功能验证，不能支持正式绝对定位精度结论。
- Phase 5A 不含平移搜索、全局探索、移动目标追踪或自动 Phase 4 接近授权。
