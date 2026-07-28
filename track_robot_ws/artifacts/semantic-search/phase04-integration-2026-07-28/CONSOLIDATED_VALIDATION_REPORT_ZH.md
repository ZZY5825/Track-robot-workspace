# Phase 0–4 集成验证报告

日期：2026-07-28
ROS Domain：20
查询：`green bottle`
查询键：`query_id=2026072801, query_version=1`
现场采样窗口：25 秒
总体结论：**FAIL**

## 结论摘要

本次没有达到“语言查询 → 稳定 3D 全局对象 → 选定目标 → 安全接近路径”的完整成功链。Phase 1 现场通过；Phase 2 的 epoch 崩溃已修复且进程保持运行，但底盘里程计失败使 Phase 0 进入 observation-only，导致 Phase 2 无法形成稳定 memory/global identity；Phase 3 因此没有合法目标；Phase 4 按 fail-closed 规则拒绝规划。

Phase 4 独立确定性功能合同通过全部 8 个成功/失败场景，但这不能替代现场端到端通过。

| 阶段 | 现场状态 | 关键证据 | 延迟/周期 |
| --- | --- | --- | --- |
| Phase 0 | **FAIL** | 252 条定位状态，0 条 healthy；固定 `base_link`、localization epoch 1；原因 `local_pose_stale` | 发布周期约 99.2 ms；端到端延迟未评估 |
| Phase 1 | **PASS** | 50/50 非空 region，50 observations；frame `zed_left_camera_optical_frame`；query `2026072801/1`；分数 min/mean/max = 0.3979/0.4323/0.4570 | 输出周期约 500 ms；图像到观察延迟未评估 |
| Phase 2 | **FAIL** | 264 条 active-object 消息，global ID 数值为 1，214 个有效位置样本；但 265 个 memory epoch 连续变化，query reference 同时出现 0 和 `2026072801` | 快照周期约 94.7 ms；融合延迟未评估 |
| Phase 3 | **FAIL** | 263 条 best-candidate 消息，选定目标总数 0，无 confidence/uncertainty/ID reference | 发布周期约 95.1 ms；选择延迟未评估 |
| Phase 4 | **FAIL（现场）/ PASS（确定性功能合同）** | 500 张 240×240、0.05 m/cell 的 `base_link` costmap；125 条诊断和空路径；原因 `localization_unhealthy` | 现场 planner P50/P95 = 4.911/7.992 ms；确定性成功 17.843 ms |
| 安全边界 | **PASS** | `planning_only=true`，`cmd_vel` 发布者为 0，Phase 4 motion interfaces 为空 | 不适用 |

## Phase 0

**状态：FAIL**

- Bunker base node 启动后以 `std::system_error: Resource deadlock avoided`、exit code -6 退出。
- 定位健康节点持续发布，但全部 252 个样本为 `local_pose_stale`。
- `localization_epoch_id=1` 和 `canonical_frame_id=base_link` 本身一致，但健康条件不满足。
- ZED 2i 成功以 HD720@15 打开；启动时 depth 为 `NONE`，并报告 self-calibration failed。
- RoboSense LiDAR 正常发布；IMU 最终成功连接并完成陀螺仪归零。

## Phase 1

**状态：PASS**

- CLI 明确返回查询 `ACCEPTED`。
- 25 秒内得到 50 条 region 消息，全部非空；同时得到 50 条 semantic observations。
- region 与 observation 都只引用 `query_id=2026072801, version=1`。
- 相机 frame 始终为 `zed_left_camera_optical_frame`。
- 分数范围 0.397949–0.457031，均值 0.432290。PASS 表示接口、查询关联和连续观察有效，不表示这些分数已完成任务阈值标定。

## Phase 2

**状态：FAIL**

修复前，semantic memory 因同一 observation-only 批次跨越多个 memory epoch 而抛出 `runtime task reset epoch does not match memory core` 并退出。修复后新增 skipped-epoch 回归测试，服务可原子同步到核心 epoch；现场进程在完整采样窗口内保持运行并发布 264 条 active-object 消息。

仍然失败的原因：

- Phase 0 不健康，系统必须使用 observation-only；
- 25 秒内 memory epoch 从 `7579124794573525093` 变化到 `7579124794573525357`，共 265 个，复合全局键不稳定；
- 虽然数值 `global_object_id=1` 重复出现，但 `(memory_epoch_id, global_object_id)` 每次不同，不能称为同一全局对象；
- 214 个位置样本有效，但顶层目标样本并非全部有效；
- query reference 同时出现 0 和 `2026072801/1`；
- collector 检出 active-object 时间戳不单调。

## Phase 3

**状态：FAIL**

- 收到 263 条 fail-closed best-candidate 消息，但没有任何选定对象。
- 因此无法验证 memory/global/localization ID、query reference、confidence 和 uncertainty 的跨阶段一致性。
- 当前测试配置仍保持 `best_candidate_threshold_calibrated=false`；即使恢复定位，也必须用独立标注数据完成阈值标定，不能为得到路径而临时打开。

## Phase 4

**现场状态：FAIL**

- costmap 有效且持续更新，frame 为 `base_link`。
- 125 次规划均输出空路径，诊断原因均为 `localization_unhealthy`。
- 这是正确的安全拒绝；没有 goal 或可执行运动输出。
- RViz 成功加载 Phase 4 配置和 240×240 costmap，但当前 Jetson OpenGL 驱动报告 sampler 类型冲突；退出时 RViz 以 -11 结束。由于上游无目标，现场无法显示目标、候选位姿、goal 和 path。

**确定性 Phase 4 合同：PASS**

| 场景 | 预期/实际原因 | 功能状态 | 延迟 |
| --- | --- | --- | --- |
| success | `planned`，16 candidates，13 path poses | PASS | 17.843 ms |
| no target | `no_target` | PASS | 0.015 ms |
| ambiguous target | `ambiguous_target` | PASS | 0.018 ms |
| target lost | `target_lost` | PASS | 0.007 ms |
| invalid position | `invalid_position` | PASS | 0.007 ms |
| blocked path | `blocked_path`，空路径 | PASS | 702.997 ms |
| stale map | `stale_map` | PASS | 0.025 ms |
| localization reset | `localization_reset` | PASS | 0.022 ms |

确定性 success 保持 `memory_epoch=11, global_object_id=42, localization_epoch=7, query=1234/2, frame=base_link` 一致。blocked-path 的 702.997 ms 高于 5 Hz 的 200 ms 预算，是明确的 Phase 4 性能阻塞项。

## 跨阶段一致性

**状态：FAIL**

- Phase 1 query ID/version 一致。
- localization epoch 数值稳定，但健康状态失败。
- Phase 2 memory epoch 不稳定，复合 global object ID 不稳定。
- Phase 2 同时存在 query 0 和已接受 query，Phase 3/4 没有目标引用可供核对。
- frame 链是 `zed_left_camera_optical_frame → base_link`，但 camera-to-base 仍为 prototype 外参；不能据此声称 3D 几何已标定。
- Phase 2 active-object 时间戳不单调；Phase 3 没有选定目标时间戳。因此完整 timestamp continuity 不通过。

## 剩余阻塞项（按依赖顺序）

1. 修复 Bunker base 的 CAN 启动死锁，恢复连续 `/odom` 和 `odom → base_link`。
2. 重新运行 Phase 0，要求 25 秒窗口所有 localization samples healthy 且 epoch 不变化。
3. 使用测量后的 `base_link → zed_camera_link` 外参替换 prototype；处理 ZED self-calibration 警告。
4. 在健康定位下验证 camera/LiDAR association，使 Phase 2 在一个 memory epoch 内保持同一复合全局 ID、有效 3D 位置和一致 query reference。
5. 用独立标注集完成 Phase 3 best-candidate threshold 校准，之后验证 confidence、uncertainty 和 ambiguity margin。
6. 优化 blocked-path 搜索至 P95 小于 200 ms。
7. 解决 Jetson 上 RViz occupancy-grid GLSL sampler 冲突和退出 -11。
8. 完成上述项目后，按同一标准重新运行现场 Phase 0–4；只有届时才能把总体状态改为 PASS。

## 证据文件

- `live_report_after_epoch_fix.json`：修复后 25 秒现场证据。
- `live_report_before_epoch_fix.json`：修复前 semantic memory 崩溃证据。
- `phase4_contract.json`：Phase 4 八场景确定性合同。
- ROS 日志目录：`~/.ros/log/2026-07-28-14-04-57-760820-ubuntu-23222`。

## 回归验证

- `track_robot_semantic_search`：764 tests passed。
- `track_robot_semantic_memory`：222 tests passed，4 skipped（需要显式 ROS runtime 环境）。
- `track_robot_bringup`：166 tests passed。
- 新 skipped-epoch 回归：PASS。
- 相关四包构建：PASS。
- 测试结束后已确认没有遗留 ROS、RViz、ZED、LiDAR 或 local-obstacle-map 测试进程。
