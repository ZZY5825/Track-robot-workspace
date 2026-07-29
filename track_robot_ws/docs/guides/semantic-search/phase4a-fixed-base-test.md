# Phase 0–4A 固定底盘最小工作系统测试

本流程用于验证：

```text
英文查询
→ Phase 1 视觉检测
→ Phase 2A ZED 注册深度定位 + 语义记忆稳定对象 ID
→ Phase 3A 保守目标选择
→ Phase 4 LiDAR 局部障碍图上的碰撞检查和接近路径
→ Phase 4A 人类可读接近建议
```

机器人必须全程停在原地。该启动文件不会启动 Bunker、IMU、里程计、运动控制器
或导航执行接口，也不会发布任何速度或导航目标。Phase 4A 的结果只是建议。

Phase 4A 当前采用清晰的职责划分：

- ZED 2i 的注册深度负责把视觉目标 ROI 转换为 `base_link` 下的三维位置；
- 语义记忆负责分配并维持稳定的 `global_object_id`；
- LiDAR 负责生成局部障碍栅格，供 Phase 4 检查接近路径是否碰撞；
- 相机–LiDAR 的直接语义 tracklet 绑定仍保留为诊断能力，但在正式外参标定完成前，
  不作为 Phase 4A 成功的必要条件。

## 测试布置

- ROS Domain 固定为 `20`。
- ZED 2i 和 RoboSense LiDAR 已连接。
- 机器人在整个测试期间不能被移动。
- 把绿色瓶子放在相机前方约 `1.6 m`，避免被其他物体明显遮挡。
- 当前相机外参是 `prototype` 工程估计，只能用于最小系统集成测试，不能作为
  精确测量或正式标定结果。
- 使用卷尺记录瓶子相对 `base_link` 的真实距离，以便判断深度绝对误差；当前
  PASS 只证明端到端数据链成立，不等于测距已经完成标定。

## 1. 构建

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup
source install/setup.bash
```

## 2. 启动整个固定底盘测试栈

如果网口尚未配置，先在单独终端执行一次 `sudo -v`。然后：

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_bringup semantic_search_phase4a.launch.py
```

这个命令一次启动 ZED（`PERFORMANCE` 深度模式）、LiDAR、YOLO-World/DINO、
LiDAR tracklet、语义记忆、固定底盘会话、局部障碍图、Phase 3A 选择器、
Phase 4 规划器、Phase 4A 建议器和 RViz。ZED positional tracking 不会启用，
系统也不会订阅 IMU。

## 3. 输入英文查询

模型启动和第一次推理可能需要一些时间。另开终端：

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run track_robot_semantic_search semantic_search_query \
  "green bottle" \
  --query-id 2026072801 \
  --query-version 1 \
  --timeout 20 \
  --subscriber-timeout 10
```

成功时命令显示 `ACCEPTED`。同一次验证必须使用相同的 query ID 和 version。

## 4. 查看直观结果

RViz 应逐步出现：

- ZED 画面中的目标区域；
- 由 ZED 注册深度得到的、位于 `base_link` 坐标中的语义目标；
- 局部 LiDAR 障碍栅格；
- 蓝色接近候选；
- 橙色选定接近位姿；
- 不穿越障碍物的规划路径。

文字建议：

```bash
ros2 topic echo /semantic_search/phase4a/advice
```

成功例子：

```text
READY target="green bottle" position=front 1.60m,right 0.20m
range=1.61m bearing=-7.1deg approach=front-right
goal=(0.81,-0.10)m standoff=0.80m path=clear
confidence=0.72 uncertainty=0.24 ADVISORY_ONLY
```

`front/right` 是目标相对机器人 `base_link` 的粗略位置；`goal` 是机器人如果未来
允许运动时应接近的局部位姿。本测试不会执行该位姿。

若条件不安全或数据不完整，输出为：

```text
NOT_READY reason=<具体原因> ADVISORY_ONLY
```

常见原因包括 `no_target`、`ambiguous_target`、`stale_target`、
`unstable_position`、`blocked_path` 和 `stale_map`。

## 5. 生成统一验证报告

```bash
ros2 run track_robot_semantic_search semantic_search_phase4a_validate \
  --query "green bottle" \
  --query-id 2026072801 \
  --query-version 1 \
  --duration-sec 25 \
  --output /tmp/phase4a_green_bottle.json
```

报告分别记录 Phase 0、1、2、3、4 和 Phase 4A 建议的
`PASS`、`FAIL` 或 `NOT EVALUATED`，以及时间戳、frame、memory/global ID、
query reference、置信度、不确定度、路径、延迟和任何运动接口发布者。

Phase 2 的目标空间输出可单独检查：

```bash
ros2 topic echo /semantic_search/phase4a/spatial_objects
```

有效对象必须具有当前 query ID/version、稳定 `global_object_id`、`base_link`
位置和有效三维坐标。运行时诊断里的
`accepted_camera_attachments=0` 表示直接相机–LiDAR语义绑定尚未成功，并不表示
ZED 深度定位或 LiDAR 障碍图失效。

## 6. 结束测试

在启动终端按 `Ctrl-C`。确认测试启动的 ZED、LiDAR、RViz 和语义搜索进程均已
停止。不要让测试进程在后台继续运行。
