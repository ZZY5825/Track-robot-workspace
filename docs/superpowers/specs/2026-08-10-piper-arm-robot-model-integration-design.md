# PiPER 机械臂与 Bunker Pro 2 整机模型集成设计

**日期：** 2026-08-10  
**目标分支：** `feature/phase1-5-urdf-tf-integration`  
**范围：** 仅集成 PiPER 机械臂模型、URDF 和 TF；不修改 Nav2、Point-LIO、ZED、LiDAR、机械臂控制或手眼标定逻辑。

## 1. 目标

将下载目录中的 PiPER 机械臂完整模型加入现有 Bunker Pro 2 权威 `robot_description`，形成一棵连续且无重复所有权的 TF 树：

```text
robot_bottom
└── base_link
    ├── sensor_station_link
    │   ├── camera_mount_link
    │   │   └── zed_camera_link
    │   └── lidar_link
    └── arm_base_link
        └── link1 ... link6
            └── gripper_base
                ├── camera_holder
                │   └── l515_visual
                ├── link7
                └── link8
```

RViz 必须能够显示底盘、传感器站、PiPER 本体、夹爪、相机支架和 L515 可视模型。机械臂的动态关节 TF 必须能够由现有 `joint1` 至 `joint8` JointState 名称驱动。

## 2. 权威来源与不可修改内容

机械臂权威模型为：

`/home/track-robot/Downloads/Piper_Jetson_Model_Handoff_2026-08-09/piper_description/urdf/piper_description.xacro`

交付包中的整个 `piper_description` package 必须进入工作区，保留 `package://piper_description/meshes/...` 路径。模型、mesh 和运行文件保持原样；若交付测试与哈希锁定的权威 Xacro 矛盾，只允许修正测试期望并记录差异。以下内容不允许为适配底盘而修改：

- `joint1` 至 `joint8` 的名称、轴、限制和内部变换；
- `joint6_to_gripper_base` 的固定旋转；
- `gripper_base_to_camera_holder`；
- `camera_holder_to_l515_visual`；
- 所有机械臂、夹爪、支架和 L515 mesh 的 visual/collision/inertial 定义。

集成前后校验以下权威哈希：

- `piper_description.xacro`: `e32d340b72389d237fb367ad700af9de34970fe987aa7eb8bb795c6b2e2f35e1`
- `camera_holder.STL`: `a68851c67c3c631b3176d1038478a37acd5b2ebc6aa6189793df4b6ee68478b2`
- `Intel_RealSense_L515_CAD_external.STL`: `8da72869225af4826ed8b059361109b9e18df5295624d629109e32a906f02d6f`

实施时确认 ZIP 内测试早于最终 Xacro，仍检查旧版 L515 visual 位姿；最终 Xacro、ZIP 内 Xacro 和上述哈希彼此一致。因此本集成以 Xacro 为准，仅同步导入副本中的过期测试期望，不改变任何模型或 mesh。

机械臂交付包中的 operational `camera_link`、手眼标定发布器和 L515 驱动 TF 不属于本次范围。`l515_visual` 只用于显示，不替代现有 ZED 相机 TF，也不创建新的运行相机 TF。

## 3. 根连接与安装位姿

机械臂原模型的独立根连接 `world -> base_link` 不进入整机模型。机械臂根 link 仅重命名为 `arm_base_link`，再由单个固定关节连接到机器人 `base_link`：

```xml
<joint name="base_to_arm_base_joint" type="fixed">
  <origin xyz="0.39 0 0.016" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="arm_base_link"/>
</joint>
```

安装坐标依据：

- Bunker CAD 沿 x 的范围为 `-0.6192 m` 至 `+0.4481 m`，正 x 为机器人前方；
- PiPER 底座相对根坐标的 x 范围为 `-0.0585 m` 至 `+0.0400 m`；
- 根放置在 `x=0.39 m` 后，机械臂底座最前缘约为 `x=0.43 m`，仍在 Bunker 前缘内侧约 18 mm；
- `y=0` 表示安装在两条纵向导轨的横向中心；
- `z=0.016 m` 与现有 `sensor_station_joint` 的导轨安装高度一致；
- `rpy=0 0 0` 保持 PiPER 本体竖直，且不改变内部关节坐标约定。

因此 `sensor_station_link` 与 `arm_base_link` 都是 `base_link` 的直接子节点，但各自保留自己的安装朝向。

## 4. 模型组合与 TF 所有权

采用一个组合后的 Bunker 权威 URDF 和一个 `robot_state_publisher`：

- `bunker_pro2` 继续拥有 `robot_bottom -> base_link`、传感器站、ZED 根和 LiDAR 固定 TF；
- 同一个 `robot_description` 增加 PiPER link/joint 定义；
- 删除 PiPER 独立模型中的 `world` link 和 `fixed_base_joint`；
- 仅将 PiPER 的根 `base_link` 重命名为 `arm_base_link`；
- PiPER 内部 `link1...link8`、`joint1...joint8` 保持原名，兼容现有驱动反馈；
- 不启动第二个 PiPER `robot_state_publisher`，避免重复或分裂 TF 所有权。

为了避免人工复制时改变机械臂内部定义，集成文件必须明确标注上游来源和哈希，并通过结构测试确认除根替换外的内部 link/joint 内容保持一致。

## 5. JointState 与 RViz 行为

组合模型的动态机械臂 TF 由同一个 `robot_state_publisher` 发布。兼容策略为：

- 保持机械臂驱动反馈 topic `/joint_states_single`；
- 使用 `joint_state_publisher` 作为汇聚入口，订阅 `/joint_states_single` 并向组合模型提供 `/joint_states`；
- 当机械臂驱动未运行时，显示启动方式允许发布零位预览，使 RViz 仍能显示完整机械臂；
- 实际 Phase 1–5 启动中不得同时存在两个发布相同关节名称的预览/真实反馈源；真实反馈接入后以 `/joint_states_single` 为数据来源。

此集成只生成 TF 和模型显示，不启动机械臂电机、不发送关节命令，也不引入手眼标定发布器。

## 6. 文件范围

预计新增或修改：

- 新增完整 `src/piper_description/` package；
- 修改 `src/bunker_pro2/urdf/bunker_pro2.urdf`，加入机械臂子树；
- 修改 `src/bunker_pro2/package.xml`，声明模型/JointState 所需依赖；
- 修改 `src/bunker_pro2/launch/description.launch.py`，接入机械臂关节状态；
- 必要时修改 `src/bunker_pro2/launch/display.launch.py`，提供零位预览参数；
- 更新 `bunker_pro2` 结构测试和说明文档；
- 只在现有 Phase 1–5 RViz RobotModel 无法自动显示组合模型时，做最小 RViz 配置调整。

不会修改：

- Nav2 footprint、costmap、控制器或规划器；
- Point-LIO；
- ZED/LiDAR 现有 TF；
- Phase 1–5 感知、语义记忆或运动逻辑；
- PiPER 控制、手眼标定和 L515 operational TF。

## 7. 验证与验收

实施必须通过以下门禁：

1. 三个机械臂权威文件 SHA256 与交付记录一致。
2. `check_urdf` 成功，组合模型只有 `robot_bottom` 一个根。
3. `base_link` 的直接子节点同时包含 `sensor_station_link` 和 `arm_base_link`。
4. `base_to_arm_base_joint` 严格为 `xyz=0.39 0 0.016`、`rpy=0 0 0`。
5. 机械臂内部所有关节、限制和固定相机支架/L515 变换与权威源一致。
6. `piper_description`、`bunker_pro2` 和受影响 bringup package 成功构建。
7. 现有 Bunker/Phase 1–5 launch contract 测试无回归。
8. RViz 中底盘、传感器站、机械臂、夹爪、支架和 L515 完整显示。
9. TF 树中不存在第二个 `base_link`、第二棵机械臂树或重复 ZED/LiDAR 边。
10. 本次验证保持只读显示；不启用底盘和机械臂运动。

## 8. 回滚

本变更保持为独立提交。若构建、TF 唯一性、原模型哈希或 Phase 1–5 回归测试失败，则撤销机械臂集成提交即可恢复当前已验证的 Bunker/传感器 TF 模型，不需要修改任何感知或导航配置。
