# Phase 4B Nav2 监督导航测试

Phase 4B 把 Phase 4A 的语义接近点交给 Nav2，但保留原有独立安全链。初始范围只包括静态目标、`odom` 内短距离、低速和人工监督。

## 1. 不可绕过的速度链

```text
Nav2 controller/recoveries
  -> /nav2/cmd_vel_raw
  -> motion_safety_supervisor_node
  -> /nav2/cmd_vel_safe
  -> cmd_vel_gate
  -> /cmd_vel
  -> bunker_base
```

Nav2 controller 和 recovery server 都重映射到 `/nav2/cmd_vel_raw`。最终 `/cmd_vel` 的唯一合法发布者是 `cmd_vel_gate`。

## 2. 四种运行模式

| 模式 | Nav2 能力 | 是否可能输出运动 |
| --- | --- | --- |
| `PLANNING_ONLY` | planner server | 否 |
| `MANUAL_NAV2_ACTIVE` | 手动 Nav2 goal、规划和控制 | 只有安全链武装后 |
| `SEMANTIC_SHADOW` | 语义 goal 调用 `ComputePathToPose` | 否 |
| `SEMANTIC_ACTIVE` | 语义 goal 调用 `NavigateToPose` | 默认禁用；双开关并武装后才可能 |

组合入口默认是 `SEMANTIC_SHADOW`、`start_base=false`、`enable_semantic_execution=false`。

## 3. 标准重复测试流程（首选）

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to \
  track_robot_bringup \
  track_robot_navigation \
  track_robot_semantic_search_rviz_plugins
source install/setup.bash
sudo -v
ros2 run track_robot_bringup semantic_search_ctl run phase4b
```

该命令固定执行以下策略，不再逐个手工启动节点：

- ROS Domain 固定为 `20`；
- 启动 ZED、LiDAR、Bunker、Phase 0–4B 和 RViz；
- 不使用 IMU；
- 运行 `SEMANTIC_ACTIVE`，但启动后没有操作员授权，因此机器人保持静止；
- 所有速度仍走 Nav2 → safety supervisor → cmd_vel gate → Bunker；
- `Ctrl-C` 或 `semantic_search_ctl stop` 会先请求
  `/semantic_navigation/cancel_and_disarm`，再停止自己管理的进程组。

RViz 内按以下固定顺序操作：

1. 输入英文目标，例如 `green bottle`，点击 `New Query`；
2. 确认 Phase 1 overlay 持续有正确目标框；
3. 确认 `Best candidate` 显示一个稳定 object/global ID；
4. 确认地图中目标位置和 Nav2 路径合理；
5. 只有 `Start Approach` 可点击后，才点击一次开始接近；
6. 任何异常点击 `Cancel & Disarm`；遥控器接管和 E-stop 始终优先。

`Start Approach` 不是直接发速度。它把当前
`memory/global/localization/query/version/snapshot` 精确引用交给监督器；
监督器再次核对 Phase 4A planner 和当前目标，安全链成功武装后才允许
Nav2 goal。目标或 query 改变、RC 接管、E-stop、底盘故障都会撤销授权。

需要 LiDAR 时，确认 `eth0` 是 `192.168.1.102/24`。组合 launch 会沿用已有的网络配置入口。

## 4. RViz 蓝色/粉色区域是什么

蓝色和粉色区域是 Nav2 global/local costmap 中的障碍代价与 inflation，
不是语义目标框。它们会影响 Phase 4B 是否可规划、是否暂停：

- 原始 `/rslidar_points` 只用于 raytracing clearing；
- `/safety/filtered_obstacle_points` 只用于 marking；
- 两个 observation source 的 persistence 均为 `0`；
- 人离开后，后续 LiDAR 射线应清除旧代价，不应永久留下脚印。

若痕迹持续不清除，先检查原始点云和 costmap 更新率；这属于
Phase 4B 动态障碍清除失败，不能靠解锁绕过。

## 5. 分离 Gate A：影子模式（故障排查用，绝不运动）

实机需要 Bunker 里程计和 `odom -> base_link`，因此使用 `start_base:=true`。这只启动底盘驱动；影子模式没有 controller、BT navigator、安全执行链或速度发布者。

```bash
ros2 launch track_robot_bringup semantic_search_phase4b.launch.py \
  runtime_mode:=SEMANTIC_SHADOW \
  start_base:=true
```

另开终端输入英文目标：

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
ros2 run track_robot_bringup semantic_search_ctl query "green bottle"
```

期望：

- Phase 1 overlay 显示候选框；
- Phase 2 保持同一个 `(memory_epoch_id, global_object_id)`；
- Phase 4A 显示接近候选、橙色建议路径；
- `/semantic_navigation/shadow_path` 显示青色 Nav2 路径；
- `/semantic_navigation/diagnostics` 从 `confirming_target_reference` 进入 `goal_accepted` 或 `goal_already_dispatched`；
- `/cmd_vel` 和 `/nav2/cmd_vel_raw` 均不存在。

验证：

```bash
ros2 topic echo /semantic_navigation/diagnostics
ros2 topic hz /semantic_navigation/shadow_path
ros2 topic info /cmd_vel --verbose
ros2 topic info /nav2/cmd_vel_raw --verbose
```

影子模式出现任何速度发布者均判为 FAIL。

## 6. 分离 Gate B：手动 Nav2（故障排查用）

只在轮子离地或清空的受控测试通道运行：

```bash
ros2 launch track_robot_bringup semantic_search_phase4b.launch.py \
  runtime_mode:=MANUAL_NAV2_ACTIVE \
  start_base:=true
```

启动后安全监督器保持 `DISARMED`。先确认速度链：

```bash
ros2 topic info /nav2/cmd_vel_raw --verbose
ros2 topic info /nav2/cmd_vel_safe --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic echo /safety/state
```

必须看到：

- raw：Nav2 controller/recoveries 发布，安全监督器订阅；
- safe：安全监督器发布，gate 订阅；
- final：`cmd_vel_gate` 是 `/cmd_vel` 唯一发布者。

在 RViz 使用 `Nav2 Goal` 工具选择 0.5–1.0 m 内的可见空闲点。未武装时机器人必须保持静止。确认 E-stop、RC、Bunker CAN 模式、LiDAR、odom 全部正常后：

```bash
ros2 service call /safety/arm std_srvs/srv/Trigger "{}"
```

当前 Phase 4B 限制：

- 起步线速度目标为 0.10 m/s，最高线速度不超过 0.15 m/s；
- 角速度不超过 0.50 rad/s；
- controller 10 Hz；
- NavFn A* 约 1 Hz 重规划；
- Regulated Pure Pursuit 跟踪。

随时停止：

```bash
ros2 service call /safety/disarm std_srvs/srv/Trigger "{}"
ros2 service call /safety/emergency_stop std_srvs/srv/Trigger "{}"
```

急停是锁存的；只在现场重新确认安全后执行：

```bash
ros2 service call /safety/reset_emergency_stop std_srvs/srv/Trigger "{}"
```

## 7. 分离 Gate C：语义主动模式（等价底层命令）

只有 Gate A、Gate B 和失败注入全部通过后才能运行：

```bash
ros2 launch track_robot_bringup semantic_search_phase4b.launch.py \
  runtime_mode:=SEMANTIC_ACTIVE \
  enable_semantic_execution:=true \
  start_base:=true
```

少任意一个条件都不得派发 `NavigateToPose`：

- 同一目标引用至少连续两个 snapshot；
- Phase 4A planner 为 `PASS/planned`；
- memory/global/localization/query ID 全部一致；
- 目标、goal、diagnostics 和 odom 均新鲜；
- `base_link` goal 能转换到 `odom`；
- 安全监督器已武装；
- 当前精确目标已由操作员通过 RViz 按钮授权；
- 安全状态为 CLEAR、SLOWDOWN 或 AVOIDING。

目标丢失/变化、无效位置、TF 失败、odom 陈旧、BLOCKED、RC 接管、Bunker 故障或 E-stop 会拒绝或取消语义 Nav2 goal。SLOWDOWN 和 AVOIDING 保留安全监督器的限速控制，不直接绕开 Nav2。

## 8. 失败测试

按顺序执行，每项都必须安全停止或拒绝：

1. 不输入目标：`waiting_for_correlated_inputs`，无 motion；
2. 用手遮挡/移走目标超过 1 s：取消语义 goal；
3. 停止 odom：0.25 s 后监督器取消，安全链输出零；
4. 在通道放置障碍：Nav2 重规划；无路时 clear、wait 后 abort；
5. 操作 RC：进入 `RC_OVERRIDE` 并取消；
6. 调用 E-stop：立即锁存零速度并取消；
7. 改变 localization epoch 或 TF：旧目标引用不得继续执行。

每次测试结束按 `Ctrl-C`，再确认：

```bash
ros2 node list
ps -eo pid,ppid,stat,cmd | grep -E \
  'nav2|semantic_navigation|motion_safety|cmd_vel_gate' | grep -v grep
```

## 9. 当前验收状态

- 软件构建、单元/配置/launch 合同测试：PASS；
- `PLANNING_ONLY` 运行时启动：PASS；
- `MANUAL_NAV2_ACTIVE` ROS 图安全链：PASS；
- `SEMANTIC_SHADOW` 运行时零运动图验证：PASS；
- 本次 Camera+Stereo Phase 2、costmap 清除和 RViz 授权改动：离线回归通过后仍需按本页流程做一次实机验收，不能仅凭代码宣称实机通过。

初始 footprint `1.20 x 1.00 m` 来自现有配置，不替代实物复测。Phase 4B 仍使用现有 prototype Camera–LiDAR 外参；在主动实机验收前必须用实际安装外参复核目标位置和障碍投影。
