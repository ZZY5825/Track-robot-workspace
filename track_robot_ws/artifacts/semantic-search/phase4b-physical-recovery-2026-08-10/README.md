# Phase 4B 目标保持物理恢复验收记录

日期：2026-08-10
实现分支：`feature/target-preserving-physical-recovery`
实现基线：`bc75c02`（文档提交前）
ROS Domain：`20`

## 范围

本记录对应显式 `--physical-recovery` 功能：Nav2 `NavigateToPose` abort 后，在
不更换静态语义目标、不清除冻结 `odom` goal 和 operator authorization 的前提
下，尝试一次 `30 deg` Spin、重新规划、一次 `0.25 m @ 0.10 m/s` BackUp、再次
重新规划，并限制物理恢复循环次数。默认启动仍关闭该功能。

所有 recovery 速度必须经过：

```text
/nav2/cmd_vel_raw
-> motion_safety_supervisor
-> /nav2/cmd_vel_safe
-> cmd_vel_gate
-> /cmd_vel
```

## 自动化证据

执行命令：

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_navigation track_robot_bringup \
  track_robot_semantic_search_rviz_plugins --symlink-install
source install/setup.bash
colcon test --packages-select \
  track_robot_navigation track_robot_bringup \
  track_robot_semantic_search_rviz_plugins --event-handlers console_direct+
colcon test-result --verbose
```

最终结果：1541 tests，0 errors，0 failures，4 skipped。该结果仅证明软件构建与自动化回归通过；实机 Spin/BackUp 恢复与安全链路仍为 NOT EVALUATED，不得写成实机验证。

## 默认回归命令

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a \
  --rotation-supervised
```

期望：`physical_recovery_enabled=false`，行为与已测试 Phase 4B/5A 基线一致。

## 实机 opt-in 命令

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a \
  --rotation-supervised --physical-recovery
```

必须先按照 Phase 5A 指南配置 LiDAR 网卡、ROS Domain 20、模型路径和受管启动
环境。首次运动必须有操作员持续在场，RC 和 E-stop 随时可用。

## 实机场景记录

| 场景 | 期望 | 实测证据 | 状态 |
|---|---|---|---|
| flag 关闭的绿色瓶子接近 | 行为等于基线，无自动 Spin/BackUp | 尚未执行 | NOT EVALUATED |
| clear/replan | target ID、anchor、goal、authorization 不变 | 尚未执行 | NOT EVALUATED |
| 空旷旋转 | 同一方向 `30 deg` Spin | 尚未执行 | NOT EVALUATED |
| 后方 `>=1.0 m` 已观测净空 | `0.25 m @ 0.10 m/s` BackUp | 尚未执行 | NOT EVALUATED |
| 后方可见障碍 | BackUp 被拒绝，最终 `/cmd_vel=0` | 尚未执行 | NOT EVALUATED |
| Spin/BackUp 中 RC override | 动作立即取消、任务终止 | 尚未执行 | NOT EVALUATED |
| Spin/BackUp 中 E-stop | 动作立即取消并解除任务 | 尚未执行 | NOT EVALUATED |
| 同一静态目标 3 次恢复 | 无需再次 Start Approach，ID/anchor 稳定 | 尚未执行 | NOT EVALUATED |

## 采集命令

```bash
ros2 topic echo /semantic_navigation/diagnostics
ros2 topic echo /safety/state
ros2 action list -t
ros2 topic hz /odom
```

关键诊断字段：`recovery_stage`、`recovery_cycle`、`recovery_attempt`、
`recovery_last_failure`、`mission_target_global_id`、
`mission_target_anchor_x/y`、`mission_authorization_preserved`、
`backup_permitted`。其中 `backup_permitted` 只表示前置 health/freshness 门通过，
不代表 Nav2 已确认后方走廊无碰撞。

## 停止与回滚

```bash
ros2 run track_robot_bringup semantic_search_ctl stop
ros2 node list
```

异常时立即 RC 接管或 E-stop。软件回滚首先移除命令中的
`--physical-recovery`；该开关默认关闭，因此无需改动公共 topic、message、ID
或现有 Start Approach 流程即可恢复基线行为。
