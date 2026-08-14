# Bunker Pro 2 传感器 TF 与 RViz 数据链设计

日期：2026-08-09  
状态：待实现  
适用范围：Track Robot Phase 1–5 语义感知、规划与 RViz 测试流程

## 1. 背景与已确认事实

当前系统同时存在两套传感器安装关系：

- `bunker_pro2.urdf` 已描述 `robot_bottom -> base_link -> sensor_station_link -> camera_link/lidar_link`；
- 旧 bringup 仍分别发布 `base_link -> zed_camera_link` 与 `base_link -> rslidar`。

这会形成平行的相机和 LiDAR TF 分支，使 RViz、点云关联和后续坐标变换无法确定唯一机械关系。现场运行还确认：

- `/rslidar_points` 有订阅者但没有发布者；LiDAR 驱动以退出码 255 终止；
- `eth0` 处于 UP 状态，但缺少驱动所需的 `192.168.1.102/24` IPv4 地址；
- RViz 报告没有收到地图，当前配置没有明确写出与 Nav2 costmap 发布者匹配的 QoS；
- ZED 官方描述已提供 `zed_camera_link` 以下的中心、左右相机和 optical frame，不应在 Bunker URDF 中重复定义这些内部 frame。

本设计将 URDF 作为机械安装 TF 的唯一事实来源，同时保留 ZED 官方内部 TF。

## 2. 目标

1. Phase 1–5 全栈运行时，每个机械 TF 只有一个发布者。
2. RoboSense 设备直接使用 `lidar_link`，点云消息的 `header.frame_id` 也为 `lidar_link`。
3. 自定义 `camera_link` 位于左右相机之间的前缘参考点，并通过一条确定的固定关节连接 ZED 官方 TF 树。
4. 保留已校准的 `base_link -> sensor_station_link` 以及传感器站到原相机安装点、LiDAR 安装点的变换，不修改数值。
5. 完整测试启动前检查 LiDAR 网络条件，避免启动一个缺失 LiDAR 的不完整栈。
6. RViz 稳定显示机器人、LiDAR 点云、规划路径和代价地图。
7. 不改变现有语义 topic、message、global ID、Nav2 规划接口或运动安全链。

## 3. 非目标

- 不重新标定 Camera–LiDAR 外参。
- 不修改 ZED 官方左右相机和 optical frame 的内部几何关系。
- 不改变 Nav2 的机器人控制基准 frame；Nav2 继续使用 `base_link`。
- 不让启动脚本自动使用 `sudo` 修改系统网络。
- 不在本次工作中调整感知模型、目标选择、路径算法或底盘控制逻辑。

## 4. 唯一 TF 树与所有权

Phase 1–5 的规范 TF 树为：

```text
odom
└── robot_bottom
    └── base_link
        └── sensor_station_link
            ├── lidar_link
            └── camera_mount_link
                └── camera_link
                    └── zed_camera_link
                        └── ZED 官方内部 frames
```

所有权规则：

- Bunker odometry 发布 `odom -> robot_bottom`。
- Bunker Pro 2 `robot_state_publisher` 发布 `robot_bottom -> base_link`、传感器站和自定义传感器安装 TF。
- ZED 自身的 `robot_state_publisher` 只负责 `zed_camera_link` 以下的内部 frame。
- 旧的 `base_to_zed_camera_tf` 和 `base_to_rslidar_tf` 不得出现在 Phase 1–5 全栈中。
- 不保留 `rslidar` 作为第二个 LiDAR frame alias，防止再次形成平行分支。

现有且必须原样保留的机械关系：

- `robot_bottom -> base_link`: `xyz = 0 0 0.45`；
- `base_link -> sensor_station_link`: `xyz = -0.2125 0 0.016`，`rpy = 1.57079632679 0 3.14159265359`；
- `sensor_station_link ->` 原相机安装点：`xyz = -0.2212 0.318 0`，`rpy = 1.57079632679 0 3.14159265359`；
- `sensor_station_link -> lidar_link`: `xyz = 0 0.4 0`，`rpy = 1.57079632679 0 3.14159265359`。

原相机安装点改名为 `camera_mount_link`，仅用于明确区分“安装基准”与“用户要求的相机参考点”。数值本身不变。

## 5. 相机参考点的几何定义

用户要求 `camera_link` 相对原相机安装点沿相机自身 X 轴移动 `-0.185 m`。为避免传感器站旋转导致符号或轴混淆，位移通过独立子关节表达：

```text
camera_mount_link -> camera_link
xyz = -0.185 0 0
rpy = 0 0 0
```

这里的 X 轴明确是 `camera_mount_link` 的局部 X 轴，也就是相机轴，而不是 `sensor_station_link` 或 `base_link` 的 X 轴。

ZED 官方几何关系为：

- `zed_camera_link -> zed_camera_center = (0, 0, 0.015)`；
- `zed_camera_center -> zed_left_camera_frame = (-0.01, +0.06, 0)`；
- `zed_camera_center -> zed_right_camera_frame = (-0.01, -0.06, 0)`。

因此添加：

```text
camera_link -> zed_camera_link
xyz = 0.01 0 -0.015
rpy = 0 0 0
```

组合后应满足：

- `camera_link -> zed_left_camera_frame = (0, +0.06, 0)`；
- `camera_link -> zed_right_camera_frame = (0, -0.06, 0)`。

也就是说，`camera_link` 位于左右镜头的横向中点和前缘参考平面，而 ZED 官方 optical frame 方向保持不变。

## 6. LiDAR 数据链

机器人专用 RoboSense 配置将 `ros_frame_id` 从 `rslidar` 改为 `lidar_link`。规范数据链为：

```text
RoboSense driver
  -> /rslidar_points [PointCloud2, header.frame_id=lidar_link]
  -> RViz / obstacle map / Nav2 costmaps / perception association
```

Phase 1–5 不再发布 `base_link -> rslidar`。所有消费者通过 URDF 中的 `base_link -> sensor_station_link -> lidar_link` 获得外参。

## 7. 启动前检查与失败处理

标准全栈命令在创建 ROS 节点前执行只读 readiness check：

1. `eth0` 存在且为 UP；
2. `eth0` 包含 `192.168.1.102/24`；
3. 若条件不满足，立即终止并输出用户可以复制的修复命令；
4. 不在工具内部隐藏执行 `sudo`；
5. LiDAR 驱动启动后，在有限等待时间内检查 `/rslidar_points` 是否有且仅有一个发布者、是否收到消息；
6. 若 LiDAR 未就绪，整套 motion-enabled 测试标记为 `NOT READY`，不进入可执行导航状态。

网络准备命令继续由操作者明确执行：

```bash
sudo ip addr flush dev eth0
sudo ip addr add 192.168.1.102/24 dev eth0
sudo ip link set eth0 up
```

这种 fail-fast 行为只用于需要 LiDAR 的 Phase 1–5 全栈；纯相机 Phase 1 测试不应被该检查阻塞。

## 8. RViz 与地图 QoS

Phase 1–5 的 RViz 配置必须显式包含并启用：

- RobotModel；
- `/rslidar_points` PointCloud2；
- Phase 1 semantic overlay；
- Nav2 计划路径；
- local/global costmap 或当前安全障碍栅格。

QoS 原则：

- PointCloud2 使用与 RoboSense 实际发布端兼容的 sensor-data QoS；实现时以运行时 `ros2 topic info --verbose` 结果为准，通常为 best-effort、volatile、小队列；
- Nav2 costmap 的 RViz Map 显示显式配置为 reliable、transient-local，使稍晚上线的 RViz 也能获取最近一帧；
- 不通过增大无界队列掩盖数据中断；所有队列保持有限。

“No map received” 只有同时满足以下证据后才算修复：

1. 对应 costmap topic 有发布者并持续收到消息；
2. RViz 对该 topic 有有效订阅；
3. RViz 画面实际显示地图；
4. LiDAR 输入存在且 TF 可转换。

## 9. 兼容性策略

- 保留通用相机 launch 的旧参数接口，避免破坏单独使用该 launch 的外部流程；
- 为完整机器人栈引入明确的 TF 来源选择，默认使用 `robot_description`；
- Phase 1–5 顶层 launch 固定选择 URDF 所有权，并禁止旧静态传感器 TF 发布者；
- 现有语义 topic、service、message、ID 生命周期和默认算法行为均不变；
- Bunker odometry 的 child frame 保持 `robot_bottom`，Nav2 `robot_base_frame` 保持 `base_link`。

## 10. 涉及文件范围

预计仅修改以下边界内的文件：

- `src/bunker_pro2/urdf/bunker_pro2.urdf`：拆分 `camera_mount_link`、`camera_link` 并连接 ZED 根 frame；
- `track_robot_sensor_bringup` 的 RoboSense launch/config：点云 frame 与旧 TF 发布开关；
- `track_robot_bringup` 的相机、传感器和 Phase 1–5 顶层 launch：选择唯一 TF 所有权并接入 readiness check；
- `track_robot_bringup` 的 RViz 配置：显示项与 QoS；
- 对应 launch、TF、配置和回归测试；
- 标准测试流程文档。

不做无关目录整理、算法重构或控制参数修改。

## 11. 验收标准

### 11.1 静态与单元验证

- URDF 可被解析，所有 joint/link 唯一；
- Phase 1–5 launch 中不存在 `base_to_rslidar_tf` 或 `base_to_zed_camera_tf` 节点；
- RoboSense 配置的 `ros_frame_id` 为 `lidar_link`；
- 缺失 LiDAR IPv4 时，标准全栈命令在启动 ROS 图之前给出明确错误；
- 现有 Phase 0–5 合约与回归测试继续通过。

### 11.2 运行时 TF 验证

- `/tf_static` 中只有一条连接完整的机械传感器树；
- `odom -> robot_bottom -> base_link -> sensor_station_link` 可解析；
- `base_link -> lidar_link` 可解析；
- `base_link -> zed_left_camera_optical_frame` 可解析；
- `camera_link -> zed_left_camera_frame` 为 `(0, +0.06, 0)`；
- `camera_link -> zed_right_camera_frame` 为 `(0, -0.06, 0)`；
- 上述相机平移允许浮点解析误差，但不允许额外旋转或平行 TF 分支。

### 11.3 数据与 RViz 验证

- `/rslidar_points` 恰有一个发布者；
- 点云 `header.frame_id == lidar_link`；
- 现场速率应恢复到该设备已观察到的约 16–17 Hz，验收下限设为持续高于 10 Hz；
- RViz 同时显示机器人模型、LiDAR 点云、规划路径和有效代价地图；
- costmap topic 有发布者、有消息，RViz 有匹配订阅；
- 不发送任何自动运动命令完成本次 TF/RViz 验证。

## 12. 回归与回滚

实施按独立小提交进行：

1. URDF 和 TF 合约；
2. LiDAR frame 与启动前检查；
3. 顶层 launch 的 TF 所有权；
4. RViz QoS 与显示；
5. 文档与运行时证据。

每一步运行受影响测试及 Phase 1–5 launch/TF 回归。若出现公共接口变化、重复 TF、点云丢失、costmap 退化或导航链异常，则只回滚对应独立提交。由于现有提交保持小而独立，不需要覆盖用户的其他工作。

## 13. 实施约束

- 使用测试先行方式为 TF 数值、单一发布者、配置和 fail-fast 行为建立回归保护；
- 不凭 RViz 画面单独判断 TF 正确，必须同时检查消息 frame、TF 数值和发布者数量；
- 不使用自动运动验证本设计；
- 不修改已校准变换，除明确新增的相机局部 X 轴 `-0.185 m` 位移和由 ZED 官方几何推导出的连接变换；
- 不覆盖或清理与本工作无关的本地修改。
