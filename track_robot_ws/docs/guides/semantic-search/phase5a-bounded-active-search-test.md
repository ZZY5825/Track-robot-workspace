# Phase 5A 有界主动搜索测试指南

本文用于测试静态目标的“先被动观察，必要时沿同一方向原地转向换视角，再把同一目标引用交回 Phase 4”的最小系统。Phase 5A Finding 不执行平移搜索、不自动开始 Phase 4 接近，也不直接发布 `Twist`。监督模式与 Phase 4B 共用同一套 Nav2 runtime，因此找到目标后无需重启 launch。

## 当前证据状态

冻结的 Phase 0–4B 功能基线为 `76c8ead`；Phase 5A 实现分支为 `feature/phase5a-active-search`，本文编写前的实现提交为 `a153c40f3abd6b8afbba1da3931bdd9ca0c56aba`。

| 能力 | 当前证据 | 状态 |
|---|---|---|
| 被动、shadow、监督旋转三种模式及独立执行门 | 源码、launch/config 契约测试 | 已实现、单元测试通过 |
| 单向有界航向序列 `+45°, +90°, +135°, +180°, +225°, +270°` | 确定性策略测试 | 已实现、单元测试通过 |
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
- 只有显式 `--rotation-supervised` 才启动底盘和 Nav2 Spin。RViz 中点击 **Start Finding** 本身就是当前有界旋转任务的操作员授权，不需要第二次授权。
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
unset FASTRTPS_DEFAULT_PROFILES_FILE

sudo -v
sudo ip addr replace 192.168.1.102/24 dev eth0
sudo ip link set eth0 up
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

Phase 4B/5A 的受管启动固定使用 `configure_network:=false` 和
`ROS_LOCALHOST_ONLY=0`：Foxy 的 localhost-only 模式会令多进程 `/tf_static`
发现不完整。LiDAR 网卡和 Bunker CAN 必须在
ROS 节点启动前一次配置完成，运行期间不得 flush/reconfigure。当前本机 RViz
测试不使用旧远程面板 Fast DDS profile；即使操作者 shell 曾设置该变量，控制
CLI 也会显式移除，避免 Foxy 中出现只发现部分 TF、点云或 odom 端点的 ROS 图。

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
unset FASTRTPS_DEFAULT_PROFILES_FILE

sudo -v
sudo ip addr replace 192.168.1.102/24 dev eth0
sudo ip link set eth0 up
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ros2 run track_robot_bringup semantic_search_ctl run phase5a --rotation-supervised
```

该标准命令保持原有行为，Phase 4B 的物理恢复默认关闭。完成默认回归后，如需测试
“Finding 确认目标 → Start Approach → 保留目标的 Nav2 Spin/BackUp 恢复”，使用：

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a \
  --rotation-supervised --physical-recovery
```

`--physical-recovery` 不改变 Start Finding 的搜索策略，也不会自动点击 Start
Approach。它只在目标已经确认、操作者独立点击 Start Approach、冻结静态目标
`odom` goal 后，允许 Phase 4B 在 Nav2 abort 时执行有界恢复。PASSIVE_ONLY 和
SEARCH_SHADOW 即使误传该标志，仍保持 `PLANNING_ONLY`、不启动底盘运动。

在 RViz 面板中输入 `green bottle`。先把瓶子放在初始相机视野外、但位于
有界原地旋转可见的范围内；点击 **Start Finding**。确认按钮变为
**Stop Finding**，查询输入框、**New Query** 和 **Revise Query** 均禁用，并确认
机器人只原地旋转。面板顶部须明确显示：平移/接近只会在点击
**Start Approach** 后开始；**Start Finding** 可能触发有界原地旋转，RC 接管和
E-stop 始终具有最高权限。结果应显示已确认的全局对象 ID，或一个有界的终止
失败原因；不得自动开始接近。

若初始视角没有确认目标，后续搜索必须始终沿一个方向进行，目标绝对航向依次
为 `+45°、+90°、+135°、+180°、+225°、+270°`；每次 Spin 为 `+45°`，
不得在相邻观察窗口之间正负反转。Finding 显示 `confirmed; object <ID>` 后，
确认 **Start Approach** 已启用，且 `/semantic_navigation/authorize_approach`
存在。点击 **Start Approach** 后应显示 `starting approach`，随后显示
`approach enabled (supervised)`；目标的 memory epoch、global object ID、
localization epoch、query ID/version 必须与 Finding 确认结果一致。该按钮仍是
独立的平移授权，Finding 不得自动调用它。

启用物理恢复时，RViz 的 **Navigation recovery** 行应显示 recovery stage、
cycle、attempt、最近失败和被冻结的 target ID。恢复中的 Spin 是 Phase 4B
路径失败恢复，不是 Phase 5A 的新视角搜索；两者均由 Nav2 执行并走相同安全链，
但状态机和触发条件不同。

第二次运行时，在正在旋转期间点击 **Stop Finding**。确认动作和待执行旋转
意图均收到一次取消请求。取消期间按钮保持禁用；收到 action 终态后恢复为
**Start Finding**，查询控件恢复可用。面板不得出现 **Retry Stop**，也不需要
再次点击。若底盘未及时停止，立即使用 RC 或 E-stop；它们始终具有最高权限。
最后，在没有目标的情况下再运行一次：它必须在 60 秒内结束，并且整个过程
不得平移。

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

使用 RViz 时，输入查询后只点击一次 **Start Finding**；这次点击同时提交任务并
授权该 query 的有界原地旋转。同一 query 的后续航向不再要求额外授权。使用
终端 action 测试时，提交上述 action 即代表同等的操作员授权。随时停止：

```bash
ros2 service call /semantic_search/active_search/cancel std_srvs/srv/Trigger '{}'
ros2 service call /safety/disarm std_srvs/srv/Trigger '{}'
```

支撑架验收：

- 任务提交前无旋转；点击 **Start Finding** 或发送监督 action 后只允许原地 Spin。
- 任意 `/nav2/cmd_vel_raw`、`/nav2/cmd_vel_safe`、`/cmd_vel` 的 `|linear.x| <= 0.001 m/s`。
- `|angular.z| <= 0.30 rad/s`，单次旋转 `<=90°`，累计旋转 `<=270°`。
- 每次 Spin 完成后固定等待 `2.5 s`，并确认角速度低于 `0.03 rad/s` 后才继续
  当前视角的证据评估或下一次旋转。
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
- Phase 4B 目标保持 Spin/BackUp 恢复的软件回归已完成，但尚未完成 Bunker
  实机后退走廊、RC/E-stop 中断和重复循环验收。
