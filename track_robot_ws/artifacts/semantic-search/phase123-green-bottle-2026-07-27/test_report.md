# Phase 0–3 绿色瓶子实机测试报告

> 本文件记录第一次测试，其中关于“官方 ZED 未发布首帧”的判断已被
> 官方 ZED2i 复测推翻。当前结论请以
> [official_zed_rerun_report.md](official_zed_rerun_report.md) 为准。

- 日期：2026-07-27
- ROS Domain：20
- 场景：绿色瓶子放在椅子上，距相机约 1.6 m
- 主查询：`green bottle on a chair`
- 对照查询：`green bottle`
- 测试性质：被动感知；没有启动导航或运动控制
- 总结：**Phase 0 降级通过，Phase 1 部分通过，Phase 2 独立雷达链通过但融合未通过，Phase 3 未通过**

## 安全边界

- 测试期间 `/cmd_vel` 始终为 `0` 个发布者、`1` 个订阅者。
- 工作区没有测量所得的相机外参，本轮显式使用
  `extrinsic_mode=prototype` 与 `allow_degraded=true`。
- 测试结束后，RViz、相机发布器、底盘、IMU、LiDAR 和 Phase 0–3
  节点均已停止。
- 进程检查没有发现本轮 ROS/RViz 残留；Domain 20 的节点列表为空，
  `/cmd_vel` 已不存在。

## 测试配置

静态检查结果：

| 检查项 | 结果 |
| --- | --- |
| YOLO-World 运行时及 checkpoint | PASS |
| OpenAI CLIP 运行时及 checkpoint | PASS |
| DINOv3 源码及 checkpoint | PASS |
| CUDA 运行时 | PASS |
| Fast DDS 配置 | PASS |
| Phase 3 被动安全配置 | PASS |
| 相机外参 | DEGRADED：仅 prototype |

硬件启动状态：

- Bunker 通过 `can0`（500000 bit/s）识别为 `AGX_V2`。
- PhidgetSpatial 序列号 `166154` 成功附着，配置为 250 Hz。
- RoboSense RSHELIOS 成功发布 `/rslidar_points`。
- `eth0` 配置为 `192.168.1.102/24`。

## Phase 0：传感器与基础数据链

官方 ZED ROS wrapper 能打开 ZED 2i（SN37617639），但没有发布首帧，
因此官方 ZED 路径本轮仍为 **FAIL**。

为了继续验证后续阶段，本轮使用明确标记为 DEGRADED 的临时 UVC 输入：

- 从 `/dev/video0` 读取 2560×720 左右目拼接图；
- 取左侧 1280×720；
- 使用 `/usr/local/zed/settings/SN37617639.conf` 的
  `LEFT_CAM_HD` 参数矫正；
- 发布 `/zed/zed_node/left/image_rect_color` 与
  `/zed/zed_node/left/camera_info`。

实测数据率：

| 数据 | 实测频率 | 结论 |
| --- | ---: | --- |
| UVC 发布器内部统计 | 约 2.51 Hz | DEGRADED，可持续发布 |
| ROS 相机订阅端 | 约 1.49 Hz | DEGRADED，满载时存在掉帧 |
| `/rslidar_points` | 约 16.9 Hz | PASS |
| `/imu/data_raw` | 约 249.7 Hz | PASS |
| `/odom` | 约 50.0 Hz | PASS |

Phase 0 结论：**DEGRADED PASS**。LiDAR、IMU、里程计有效，但官方
ZED 图像链和测量外参尚未通过。

## Phase 1：英文文本到视觉候选

### 已验证结果

查询 `green bottle on a chair` 被接受，查询 ID 为
`1785170150544702`。首次模型预热约 `7186 ms`；预热后多数帧约
`31–55 ms`，偶发约 `165 ms`。

在一次可靠 QoS 的 10 秒采样中，共观察到 9 组
`SemanticRegionArray`、26 个候选：

- 每帧通常有 2–3 个候选；
- 最稳定的小框约为
  `x=544–545, y=394, width=24–26, height=58–59`；
- 同一区域还出现较大的重叠候选；
- 捕获到的最高语言/融合分数约为 `0.1260`；
- 其他较高分数约为 `0.1097、0.1089、0.1082、0.1060`；
- 当前 R0C 配置的检测下限为 `0.05`，因此这些候选会被发布。

该 ROI 与画面中绿色瓶状目标的位置、尺寸相符，说明 YOLO-World
已经形成真实的文本条件目标框；但最高分仍偏低，不能把这次结果视为
高置信目标确认。

### 降级项与缺陷

1. DINOv3 外观编码未启用。初始化错误为：

   ```text
   TypeError: 'weights_only' is an invalid keyword argument for Unpickler()
   ```

   因此 `appearance_available=false`，所有候选的
   `appearance_score=0.0`。本轮只有 YOLO-World 语言/检测证据，
   缺少用于复杂背景区分和稳定重识别的外观描述。

2. 运行中把查询切换为 `green bottle` 时，连续出现：

   ```text
   RuntimeError: YOLO-World vocabulary update failed
   ```

   热切换失败还使当前模型留在降级状态，必须重启感知进程才能恢复。
   这属于查询生命周期缺陷，不是“绿色瓶子未被识别”的证据。

3. Foxy/Fast DDS 下，默认 `ros2 topic echo` 的 BEST_EFFORT 订阅没有
   收到自定义语义输出；显式使用 `--qos-reliability reliable` 后可以
   稳定收到数据。自动 live-test 收集器本轮也产生了空报告，不能作为
   管线无输出的证据。

Phase 1 结论：**PARTIAL PASS**。主查询产生了合理位置的候选框，但
DINO 外观证据缺失、分数偏低、查询热切换失败。

## Phase 2：LiDAR 候选、轨迹与视觉关联

LiDAR 独立链持续工作：

- `/semantic_memory/lidar_candidate_clusters` 每帧约 9–12 个聚类；
- `/semantic_memory/lidar_tracklets_legacy` 持续输出稳定轨迹；
- 轨迹位于 `base_link`，部分长期轨迹 `confidence=1.0`；
- 所有采样轨迹均为 `position_map_valid=false`。

场景中最近的大块候选之一约为：

```text
centroid = (1.55, -1.75, 1.09) m
size     = (13.82, 12.56, 2.49) m
oversized = true
```

它混入了大量背景，不能作为 1.6 m 处绿色瓶子的可靠 LiDAR 对应物。
其余稳定候选主要位于约 3 m 或更远。对于瓶子这种细小物体，
LiDAR 更适合提供相机射线附近的空间约束，而不是要求形成独立完整物体簇。

本轮没有完成视觉—LiDAR 关联，确定原因包括：

- UVC 降级发布器使用
  `zed_left_camera_optical_frame`，但 TF 树中只有
  `base_link -> zed_camera_link`；
- `zed_camera_link -> zed_left_camera_optical_frame` 缺失；
- `tf2_echo base_link zed_left_camera_optical_frame` 持续报告该光学
  frame 不存在；
- 语义内存进程在处理视觉 shortlist 时崩溃，无法完成融合。

Phase 2 结论：**LiDAR 子链 PASS，跨模态融合 FAIL**。

## Phase 3：语义内存与最佳候选

`semantic_memory_node` 收到视觉 observation 后异常退出：

```text
terminate called after throwing an instance of 'std::invalid_argument'
what(): visual shortlist pair is invalid or duplicate
```

该异常发生在视觉 shortlist 处理路径，未被节点边界捕获，导致进程
退出码 `-6`。退出后：

- `/semantic_memory/active_objects`：0 个发布者；
- `/semantic_memory/best_candidate`：0 个发布者；
- 无法产生有效 ranking 或最终最佳候选；
- visualizer 报告语义内存快照无法安全显示。

Phase 3 结论：**FAIL**。不能声称 Phase 0→1→2→3 端到端链路已通过。

## RViz 结果

Phase 3 RViz 面板成功启动：

- `semantic_search_live_overlay` 正常运行；
- RViz 使用 OpenGL 3.1 正常打开；
- 相机、LiDAR 和语义图层均已加载；
- Phase 1 主查询期间有实际 region 数据可供叠加。

由于相机光学 TF 缺失且 semantic memory 崩溃，RViz 没有形成可靠的
相机目标—LiDAR 位置—Phase 3 best candidate 完整可视化。

## 本轮发现并实施的代码修复

1. 在总 launch 中提前解析 YOLO、CLIP、checkpoint 和 DINO 路径，
   修复 Foxy 嵌套 launch 参数泄漏。
2. 为感知进程预加载 `/lib/aarch64-linux-gnu/libgomp.so.1`，修复
   Jetson 上 `cannot allocate memory in static TLS block`。
3. 删除 `max_detections` 的重复参数声明，修复节点启动崩溃。
4. 相机订阅改用 ROS sensor-data QoS，修复 BEST_EFFORT UVC 图像与
   RELIABLE 订阅不兼容导致的无图像问题。
5. YOLO-World 依赖错误保留底层异常类型与消息，便于诊断。
6. readiness 子进程启用无缓冲输出，避免 Foxy CLI 探针丢失输出。
7. Phase 1–3 测试配置显式保留相机测试附件和降级标定许可。

对应回归测试均已新增并在修改时通过；最终完整验证结果见本报告末尾。

## 总体判定

| 阶段 | 判定 | 关键依据 |
| --- | --- | --- |
| Phase 0 | **DEGRADED PASS** | LiDAR/IMU/odom 稳定；官方 ZED 与测量外参未通过 |
| Phase 1 | **PARTIAL PASS** | 主查询产生正确区域候选；最高分约 0.126；DINO 和查询热切换有缺陷 |
| Phase 2 | **PARTIAL / FUSION FAIL** | 雷达聚类和轨迹工作；缺少光学 TF，未完成视觉—LiDAR 关联 |
| Phase 3 | **FAIL** | semantic memory 因重复/非法 shortlist pair 崩溃，无 best candidate |

## 下一轮修复顺序

1. 给 UVC/官方 ZED 路径提供完整、经过验证的
   `zed_camera_link -> zed_left_camera_optical_frame` TF。
2. 在 semantic memory 输入边界对 shortlist pair 去重/验证；单个坏
   observation 不得让整个节点崩溃。
3. 修复 YOLO-World 查询热切换的事务性：失败时回滚模型和 active query，
   并保留底层异常。
4. 修复 DINOv3 与当前 PyTorch 的 `weights_only` 加载兼容性，恢复外观证据。
5. 使用相机 ROI 射线与深度门控缩小 LiDAR 候选，而不是在全场景
   9–12 个候选中直接选择。
6. 修复 live-test readiness/collector 的 Foxy DDS 与 QoS 兼容性。
7. 完成上述项目后，用相同场景复测：
   `green bottle`、`green bottle on a chair`，并要求稳定输出
   active object、ranking 和 best candidate。

## 最终软件验证

测试结束并关闭所有 ROS 节点后，重新执行了软件验证：

| 验证项 | 结果 |
| --- | --- |
| `track_robot_semantic_search` Python 测试 | `740 passed` |
| `track_robot_bringup` Python 测试 | `157 passed`，6 个依赖弃用警告 |
| `track_robot_semantic_memory` Python 测试 | `10 passed, 3 skipped` |
| 三个相关 ROS 包的 `colcon build --symlink-install` | PASS，3 个包完成 |

第一次把三个测试目录放入同一个 pytest 命令时，因为不同 ROS 包中存在
同名 `test_launch_contract.py`，pytest 发生模块收集冲突。按包隔离执行后，
上述三组测试均正常完成。该收集冲突不计为产品测试失败。

## 关闭观察

测试节点均已停止，但关闭过程中再次观察到已有驱动清理缺陷：

- `rslidar_sdk_node` 停止时抛出 `std::system_error`，退出码 `-6`；
- `bunker_base_node` 停止时退出码 `-11`；
- Python 感知/健康节点收到 Ctrl-C 后以 `KeyboardInterrupt`
  退出码 `-2`。

这些退出码发生在主动停止阶段，不影响前述运行期数据率，但需要在后续
工程清理中修复。
