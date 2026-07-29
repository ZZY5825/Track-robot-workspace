# Phase 0–3 官方 ZED2i 复测报告

- 日期：2026-07-27
- ROS Domain：20
- 场景：绿色瓶子放在椅子上，距相机约 1.6 m
- 查询：`green bottle on a chair`
- 相机：官方 `zed_wrapper`，ZED 2i SN37617639，HD720@15
- 测试性质：被动感知；未启动导航或运动控制
- 标定状态：`prototype` + `allow_degraded=true`
- 总体结果：**Phase 0 降级通过，Phase 1 部分通过，Phase 2 子链通过但跨模态关联未通过，Phase 3 未通过**

本报告替代同目录初次测试报告中“官方 ZED 没有发布首帧”的错误判断。
本轮未使用 UVC。

## 启动方式与相机门禁

统一控制器的首次启动在 120 秒后被其顺序就绪探针误判并主动关闭。
随后使用同一个 aggregate launch 直接启动：

```bash
ros2 launch track_robot_bringup semantic_search_live.launch.py \
  stage:=phase3 \
  start_camera:=true \
  start_lidar:=true \
  start_base:=true \
  start_imu:=true \
  extrinsic_mode:=prototype \
  allow_degraded:=true
```

官方 ZED 三项门禁均通过：

1. 日志出现 `Camera successfully opened`；
2. 主动订阅实测图像约 `11.4–14.1 Hz`，高于 5 Hz 门限；
3. `base_link -> zed_left_camera_optical_frame` 存在：
   - translation：`[0.260, 0.060, 0.635]`
   - quaternion：`[-0.500, 0.500, -0.500, 0.500]`

相机 SDK 同时报告 self-calibration warning，因此不能把本轮视为测量标定
证据；几何相关结论仍是 DEGRADED。

## 安全与清场

- 测试前 `/cmd_vel` 不存在。
- 运行中 `/cmd_vel` 为 0 个发布者、1 个订阅者。
- 测试未启动导航、规划、运动控制或速度发布节点。
- RViz、ZED、LiDAR、底盘、IMU、Phase 0–3 节点均已关闭。
- 清理 ROS CLI daemon 的陈旧 LiDAR 图缓存后，Domain 20 节点列表为空。
- 精确进程检查未发现 ZED、LiDAR、Bunker、语义搜索、语义内存或 RViz
  残留。

## Phase 0：传感器与 TF

| 数据 | 实测频率 | 判定 |
| --- | ---: | --- |
| 官方 ZED 左目图像 | 约 11.4–14.1 Hz | PASS |
| `/rslidar_points` | 稳态约 15.4 Hz | PASS |
| `/imu/data_raw` | 约 250 Hz | PASS |
| `/odom` | 约 50 Hz | PASS |

`base_link -> rslidar` 也正常：

- translation：`[0.000, 0.000, 0.700]`
- quaternion：`[0.000, 0.000, 0.000, 1.000]`

Phase 0 判定：**DEGRADED PASS**。数据链和 TF 正常，降级原因是相机外参
仍为 prototype。

## Phase 1：文本条件视觉候选

感知层实际接受的查询为：

```text
query_id      = 1785173186512252
query_version = 1
state         = active
model_ready   = true
```

完整诊断持续报告每帧 4 个候选，推理延迟样本约 `37–46 ms`。
`/semantic_search/regions` 持续输出约 `2.0 Hz`，不是一次性偶发结果。

与画面中绿色瓶子位置和尺寸相符的最稳定候选为：

```text
ROI ≈ x=589–590, y=395–396, width=28–30, height=60–62
最高采样 fused/language score = 0.3452
camera_track_id = 7
```

同帧还有背景或干扰候选，例如左侧大框和右侧窄框。这说明 YOLO-World
已经稳定形成文本条件候选，但当前输出仍不能独立证明物体语义绝对正确。

Phase 1 仍有两个缺陷：

1. DINO 外观模型未启用：

   ```text
   TypeError: 'weights_only' is an invalid keyword argument for Unpickler()
   ```

   因此 `appearance_available=false`、`appearance_score=0.0`。

2. CLI portal 最终返回：

   ```text
   TIMEOUT ... no correlated acknowledgment arrived before timeout
   ```

   但同一查询 ID 已在感知诊断中处于 active，且 regions 持续发布。
   所以这是 portal acknowledgment 缺陷，不是模型拒绝查询。

Phase 1 判定：**PARTIAL PASS**。

## Phase 2：定位、LiDAR 与跨模态关联

定位状态：

```text
memory_mode    = 1 (LOCAL_SESSION)
canonical_frame = odom
local_healthy  = true
world_healthy  = false
reason         = world_disabled
```

LiDAR 采样结果：

- 10 个候选聚类；
- 8 条确认轨迹；
- `dropped_tracklet_count=0`；
- 最近的正常确认轨迹约在 3.15 m；
- 距离约 2.37 m 的最近聚类尺寸约
  `13.85 × 12.56 × 2.49 m`，被正确标记为 `oversized=true`；
- 所有轨迹均为 `position_map_valid=false`，与 world mode 未启用一致。

来自相机的目标观察仍为：

```text
camera_track_id_valid = true
camera_track_id       = 7
lidar_tracklet_id_valid = false
position_valid        = false
evidence_flags        = 1 (CAMERA)
```

因此 LiDAR 聚类/轨迹子链和本地定位子链工作，但没有把绿色瓶子的相机 ROI
关联到 LiDAR tracklet 或 3D 位置。

Phase 2 判定：**PARTIAL / ASSOCIATION FAIL**。

## Phase 3：语义内存与最佳候选

官方 ZED 图像和完整光学 TF 存在时，`semantic_memory_node` 仍稳定复现：

```text
terminate called after throwing an instance of 'std::invalid_argument'
what(): visual shortlist pair is invalid or duplicate
```

进程退出码为 `-6`。退出后：

- `/semantic_memory/active_objects`：0 个发布者；
- `/semantic_memory/diagnostic_ranking`：0 个发布者；
- `/semantic_memory/best_candidate`：0 个发布者。

这证明 Phase 3 崩溃不是初次测试的 UVC 光学 TF 缺失造成的，而是独立的
semantic-memory shortlist 输入/验证缺陷。

Phase 3 判定：**FAIL**。

## RViz

直接运行：

```bash
ros2 launch track_robot_bringup \
  semantic_search_visualization.launch.py stage:=phase3
```

时，RViz 正常打开，`/semantic_search/overlay_image` 有 1 个发布者并约
`0.6 Hz` 更新，因此官方 ZED 图像与 Phase 1 候选框叠加可见。

通过 `semantic_search_ctl visualize phase3` 启动时曾出现叠加进程存活、
但 overlay topic 为 0 个发布者的情况。窗口打开本身不能作为可视化通过
证据；当前已验证路径是直接 visualization launch。

由于 Phase 3 节点崩溃，RViz 无法显示有效 active object、ranking 或
best candidate。RViz 判定：**PARTIAL PASS**。

## 总体判定

| 阶段 | 判定 | 关键证据 |
| --- | --- | --- |
| Phase 0 | **DEGRADED PASS** | 官方 ZED、LiDAR、IMU、odom 与两条 TF 均有效；外参仅 prototype |
| Phase 1 | **PARTIAL PASS** | 稳定 2 Hz 候选；目标 ROI 分数最高 0.345；DINO 和 portal ack 失败 |
| Phase 2 | **PARTIAL / ASSOCIATION FAIL** | 本地定位、聚类和轨迹有效；相机目标无 LiDAR ID/3D 位置 |
| Phase 3 | **FAIL** | semantic memory shortlist 异常退出；三个最终输出均无发布者 |
| RViz | **PARTIAL PASS** | 直接 launch 的 overlay 约 0.6 Hz；Phase 3 图层因上游崩溃为空 |

## 下一步修复顺序

1. 在 semantic-memory 输入边界复现、去重并验证 visual shortlist pair；
   非法单条 observation 不得终止节点。
2. 修复 DINO checkpoint 与当前 PyTorch 的 `weights_only` 兼容性。
3. 修复 query portal 的 correlated acknowledgment 判定。
4. 设计并验证相机 ROI 射线/时间戳/外参门控的 LiDAR 关联，避免从全场景
   聚类中直接选目标。
5. 把 `semantic_search_ctl start` 改为条件式、可重复的并行就绪检查，
   不让第一轮冷启动快照耗尽总 deadline。
6. 修复 `semantic_search_ctl visualize` 的 overlay 注册一致性。
7. 完成测量相机—LiDAR 外参后，重复同一物理场景并要求 Phase 3
   `active_objects` 与 `diagnostic_ranking` 持续输出。

## 关闭阶段的既有驱动缺陷

主动停止时仍观察到：

- `rslidar_sdk_node`：退出码 `-6`，`std::system_error`；
- `bunker_base_node`：退出码 `-11`；
- Python 感知/定位节点：收到 Ctrl-C 后退出码 `-2`。

这些发生在测试主动关闭阶段，不改变运行期数据率，但应独立修复。
