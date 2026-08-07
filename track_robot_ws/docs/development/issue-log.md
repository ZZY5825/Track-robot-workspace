# Track Robot 实机开发问题日志

本文件记录尚未关闭的实机现象、当前代码证据和待验证假设。它不是修复计划；
在取得同步日志或 rosbag 证据前，不把候选原因写成已确认根因。

## 2026-08-06：Phase 4B/5A 联合测试

测试范围为当前 `main` 工作树的 Phase 1–5A 管线、ROS Domain 20、Jetson 本机
RViz、ZED 2i、RoboSense LiDAR 和 Bunker。以下现象来自操作者实机观察；本轮
没有为这些事件保存同步 rosbag，因此除明确标注为“代码确认”的项目外，均需
下一轮定向采证。

### ISSUE-2026-08-06-01：代价地图存在孤立粉色像素或短暂轨迹残留

- **状态：** 已实施低风险抑制，待实机复测
- **实机现象：** 行人经过后，代价地图偶尔留下与其他障碍簇不相邻的 1–2 个
  粉色栅格；观察时对应位置没有明显 LiDAR 点。
- **代码确认：** `local_obstacle_map_node` 对通过高度、距离和自车过滤的每个点
  直接进行 0.07 m 体素去重，没有最小邻居数、连通域面积或多帧支持要求。Nav2
  的 `filtered_mark` 会对每个过滤点执行 marking。`observation_persistence: 0.0`
  只表示观测缓冲不保留旧消息，不保证已经标记的每个栅格会自动消失；清除仍
  依赖后续点云射线穿过该栅格。
- **操作者复核：** 该现象只在行人走过后沿其运动轨迹出现，因此本轮按“正常
  行人点被标记后，少量高度体素没有被后续 clearing ray 清除”处理，不再把
  单点噪声、地面边界或 TF 偏差作为本轮主要方向。
- **已实施：** Phase 4B 及独立 Phase 5A 局部 `VoxelLayer` 的
  `mark_threshold` 从 0 调整为 1。一个二维格至少需要两个仍被占用的高度体素
  才继续成为致命障碍；人在场时的多体素障碍仍保留，清除后仅剩一个体素的孤立
  残留不再维持粉色格。未周期性清空整张 costmap。
- **待采证：** 同时记录 `/rslidar_points`、`/safety/filtered_obstacle_points`、
  `/local_costmap/costmap_raw`、`/global_costmap/costmap_raw`、`/tf`、`/odom` 和
  `/safety/obstacle_map_debug`，追踪孤立栅格首次出现时的原始点及其清除过程。

### ISSUE-2026-08-06-02：Phase 5A RViz 不显示已执行的规划路径

- **状态：** 已实施，待 RViz 实机确认
- **实机现象：** 机器人按 Nav2 逻辑移动，但 Phase 5A RViz 中看不到规划路径。
- **代码确认根因：** 当前 Phase 5A 已复用完整 Phase 4B Nav2 栈，但
  `semantic_search_phase5a.rviz` 没有任何 `rviz_default_plugins/Path` display。
  Phase 4B 配置则显示 `/semantic_search/phase4/planned_path`、
  `/semantic_navigation/shadow_path` 和 `/plan`。因此“机器人能移动但路径不显示”
  是 RViz 配置与运行架构不同步，不足以说明 Nav2 没有规划。
- **已实施：** Phase 5A RViz 已加入 `/plan`（Nav2 当前路径）和
  `/semantic_search/phase4/planned_path`（Phase 4A 参考路径）两条只读显示；没有
  加入手动 Nav2 goal 工具，也没有改变运动接口。

### ISSUE-2026-08-06-03：靠近角落或近距离目标时导航长期停滞

- **状态：** OPEN，根因未确认
- **实机现象：** 机器人已经开始接近目标，但在靠近角落或目标较近的位置停止，
  随后长时间没有可见动作。
- **代码相关事实：** Phase 4A 在目标周围 0.8 m 半径上生成 16 个候选点，目标
  朝向固定为面对物体。Navfn 负责二维位置路径，Regulated Pure Pursuit 负责
  跟踪和最终转向；原近目标和代价调速最低线速度为 0.03 m/s，低于 Bunker
  已确认可克服静摩擦的约 0.10 m/s。目标容差为 0.10 m/0.15 rad，
  progress checker 在 30 秒内移动不足 0.10 m 才会 abort。静态任务遇到安全
  `BLOCKED` 时保持授权和目标，Nav2 abort 后还会有界重试并在周期耗尽后继续保留
  授权。
- **候选原因：**
  1. 控制器原先在近目标或高代价区域降至 0.03 m/s，产生非零指令但底盘不能
     克服静摩擦；
  2. Navfn 找到位置可达路径，但“面对目标”的最终朝向在角落对 0.88 m × 0.80 m
     矩形底盘不可执行，控制器的转向碰撞检查因扫掠体积而停止；
  3. 真实或孤立残留栅格使 safety supervisor 长期处于 `BLOCKED`，任务保持但
     最终速度被压为零；
  4. 上述状态触发 30 秒 progress timeout 与重试冷却，界面表现为“什么也不做”。
- **待采证：** 在停滞事件中同步记录 `/plan`、`/nav2/cmd_vel_raw`、
  `/nav2/cmd_vel_safe`、`/cmd_vel`、`/safety/state`、
  `/semantic_navigation/diagnostics`、local/global costmap、footprint、odom 和 Nav2
  action result。原始命令非零但 `/cmd_vel` 为零指向安全门控；所有命令约
  0.03 m/s 而 odom 不动指向静摩擦；旋转命令伴随碰撞状态指向最终朝向不可行。
- **已实施的单项缓解：** 两套 Nav2 配置的
  `min_approach_linear_velocity` 和 `regulated_linear_scaling_min_speed` 已提高
  到 `0.10 m/s`，最高速度仍为 `0.15 m/s`。这只排除“指令低于静摩擦”这一项，
  不把角落几何问题标记为已修复。

### ISSUE-2026-08-06-04：Phase 5A 每次旋转后的静止观察时间不足

- **状态：** 已实施简单固定等待，待实机复测
- **实机现象：** 机器人完成一次旋转后很快进入下一次旋转，视觉模型可能来不及
  在新视角完成可靠推理。
- **已实施：** 不扩展状态机，也不增加额外授权条件；仅把每次 Spin 完成后的
  固定 `settle_duration_sec` 从 0.75 秒提高到 2.5 秒。按当前约 2 Hz 的视觉目标
  频率，为每个新视角预留约五个处理周期。
- **待采证：** 记录每次 `SPIN_COMPLETED`、odom 角速度、图像时间戳、
  `frame_processed` 模型诊断、ranking source stamp、selected target source stamp
  和下一次 motion intent，量化每个视角实际获得的新推理次数。

### ISSUE-2026-08-06-05：`green bottle` 查询出现颜色或形状相近的误框

- **状态：** OPEN，需要模型与数据评估
- **实机现象：** YOLO-World 偶尔框选黄色圆柱体，或仅颜色接近但不像瓶子的物体。
- **代码相关事实：** 当前 YOLO-World 每次只设置一个正类 `[query]`，推理的底层
  confidence floor 为 0.05；下游 Phase 3 新目标门槛为 0.26。输出标签等于查询
  只证明该检测来自当前唯一词表，不是独立的类别复核。DINOv3 只为最多三个
  YOLO crop 生成外观 descriptor，用于相机 track/对象关联；任务相关性计算中的
  appearance weight 当前固定为 0.0，所以 DINOv3 不会拒绝“不是瓶子”的首次
  YOLO 候选。
- **候选原因：** 单正类开放词汇检测缺少负类对照；YOLO-World/CLIP 特征可能对
  颜色和大体轮廓权重过高；零样本 score 未针对当前相机、目标尺度和场景校准；
  首次误检进入后，时序稳定性可能提高其下游排名。当前没有逐类别标注数据，
  不能据此断言更换模型一定优于调整现有架构。
- **待研究问题：** 先评估在保留 YOLO-World 文本定位的前提下，DINOv3 是否能
  作为“已确认目标身份一致性”证据；再评估正/负提示词、crop 二阶段语义复核、
  多视角确认与零样本阈值校准。任何模型替换比较均应使用同一小型实机场景集，
  分开统计召回、误检、身份保持、延迟和显存。

## 当前边界

- ISSUE-01、ISSUE-02、ISSUE-04 已完成对应的小范围配置/RViz 修改，静态回归
  通过，但仍需下一轮实机确认现场现象是否消失。
- ISSUE-03 只提高了最低有效线速度；角落几何、碰撞检查与安全门控原因仍保持
  OPEN，等待操作者复测时结合可见路径判断。
- ISSUE-05 按操作者决定保持不变，等待进一步模型研究。
