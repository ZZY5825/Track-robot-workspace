# PiPER Arm Robot Model Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将完整 PiPER 机械臂模型作为 `base_link` 的前端同级子树加入 Bunker Pro 2 的唯一 `robot_description`，并在真实关节反馈缺失时仍可在 RViz 中显示零位模型。

**Architecture:** 保留下载包中的 `piper_description` 原样作为模型与 mesh 权威源；组合 URDF 只移除 PiPER 独立 `world` 根、把其根改名为 `arm_base_link`，并增加 `base_link -> arm_base_link` 固定安装关节。一个 `joint_state_publisher` 汇聚 `/joint_states_single` 并补齐默认关节值，一个 `robot_state_publisher` 拥有整棵底盘、传感器和机械臂 TF 树。

**Tech Stack:** ROS 2 Foxy、URDF/XML、ament_cmake、robot_state_publisher、joint_state_publisher、pytest、colcon、check_urdf。

## Global Constraints

- 工作分支固定为 `feature/phase1-5-urdf-tf-integration`，不得覆盖无关 `log/`。
- 权威机械臂源固定为 `/home/track-robot/Downloads/Piper_Jetson_Model_Handoff_2026-08-09/piper_description/urdf/piper_description.xacro`。
- 保持机械臂 `joint1...joint8`、内部 TF、关节限制、夹爪、camera holder 和 L515 visual 定义不变；交付测试若与哈希锁定 Xacro 矛盾，仅同步测试期望并记录差异。
- 仅将 PiPER 根 `base_link` 改为 `arm_base_link`；不得创建第二个机器人 `base_link`。
- 安装关节固定为 `base_link -> arm_base_link`，`xyz="0.39 0 0.016"`，`rpy="0 0 0"`。
- 现有 ZED、LiDAR、sensor station TF 必须逐字保持；不得修改 Nav2、Point-LIO、感知、规划或控制逻辑。
- 不启动机械臂电机，不发布机械臂命令，不导入 operational L515/手眼标定 TF。

---

## File Structure

- `src/piper_description/`: 原样保存 PiPER 上游 package、模型、mesh、测试和独立显示工具。
- `src/bunker_pro2/urdf/bunker_pro2.urdf`: 唯一的整机组合 URDF；拥有 Bunker、传感器和 PiPER 子树。
- `src/bunker_pro2/launch/description.launch.py`: 启动 JointState 汇聚器和唯一 robot_state_publisher。
- `src/bunker_pro2/package.xml`: 声明 `joint_state_publisher` 和 `piper_description` 运行依赖。
- `src/bunker_pro2/test/test_description_contract.py`: 检查安装坐标、唯一根、PiPER 内部模型一致性、launch 所有权和依赖。
- `src/bunker_pro2/README.md`: 记录模型来源、TF 树、JointState topic 和只读验证命令。

### Task 1: 导入并锁定 PiPER 权威 package

**Files:**
- Create: `src/piper_description/**`
- Test: `src/piper_description/test/test_robot_description.py`

**Interfaces:**
- Consumes: 下载包 `Piper_Jetson_Model_Handoff_2026-08-09/piper_description`。
- Produces: 可由 `package://piper_description/meshes/...` 解析的完整 ROS 2 package。

- [ ] **Step 1: 复制完整 package，不转换或重命名内部文件**

Run:

```bash
cp -a /home/track-robot/Downloads/Piper_Jetson_Model_Handoff_2026-08-09/piper_description src/
```

Expected: `src/piper_description/{urdf,meshes,launch,rviz,test}` 全部存在。

- [ ] **Step 2: 校验交付哈希**

Run:

```bash
sha256sum \
  src/piper_description/urdf/piper_description.xacro \
  src/piper_description/meshes/camera_holder.STL \
  src/piper_description/meshes/Intel_RealSense_L515_CAD_external.STL
```

Expected: 依次得到：

```text
e32d340b72389d237fb367ad700af9de34970fe987aa7eb8bb795c6b2e2f35e1
a68851c67c3c631b3176d1038478a37acd5b2ebc6aa6189793df4b6ee68478b2
8da72869225af4826ed8b059361109b9e18df5295624d629109e32a906f02d6f
```

- [ ] **Step 3: 运行原包测试和 URDF 检查**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q src/piper_description/test/test_robot_description.py
check_urdf src/piper_description/urdf/piper_description.xacro
```

Expected: 原 package 测试全部 PASS；独立 PiPER URDF 解析成功。

- [ ] **Step 4: 提交原始 package**

```bash
git add src/piper_description
git commit -m "feat(description): import verified PiPER model assets"
```

### Task 2: 以测试保护的方式合并 PiPER 子树

**Files:**
- Modify: `src/bunker_pro2/test/test_description_contract.py`
- Modify: `src/bunker_pro2/urdf/bunker_pro2.urdf`

**Interfaces:**
- Consumes: `src/piper_description/urdf/piper_description.xacro` 中除 `world`、`fixed_base_joint` 外的 link/joint。
- Produces: `base_to_arm_base_joint` 和以 `arm_base_link` 为根的完整 PiPER 子树。

- [ ] **Step 1: 先增加失败的安装与唯一根测试**

在 `test_description_contract.py` 中增加：

```python
def test_piper_arm_is_a_front_rail_sibling_of_sensor_station():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    links = [link.attrib['name'] for link in robot.findall('link')]
    assert links.count('base_link') == 1
    assert 'arm_base_link' in links
    _joint(
        robot, 'base_to_arm_base_joint', 'base_link', 'arm_base_link',
        '0.39 0 0.016', '0 0 0')


def test_combined_description_has_one_root():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    children = {
        joint.find('child').attrib['link'] for joint in robot.findall('joint')
    }
    roots = {
        link.attrib['name'] for link in robot.findall('link')
    } - children
    assert roots == {'robot_bottom'}
```

- [ ] **Step 2: 先运行并确认测试失败**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  src/bunker_pro2/test/test_description_contract.py \
  -k 'piper_arm or combined_description_has_one_root'
```

Expected: FAIL，原因是 `arm_base_link` 和安装关节尚不存在。

- [ ] **Step 3: 增加机械臂内部结构一致性测试**

测试应读取 PiPER 权威源，删除其 `world`/`fixed_base_joint`，把源根 `base_link` 及引用改为 `arm_base_link`，再递归比较组合 URDF 中对应 link/joint 的 tag、属性、文本和子元素签名。明确断言：

```python
assert integrated_link_names == source_link_names
assert integrated_joint_names == source_joint_names
assert _xml_signature(integrated[name]) == _xml_signature(source[name])
```

其中 source joint 集合不含 `fixed_base_joint`，integrated joint 集合额外排除 `base_to_arm_base_joint`；Bunker 原有六个 link 和五个 fixed joint 不参加 PiPER 集合比较。

- [ ] **Step 4: 机械合并权威 PiPER XML**

对权威 XML 做唯一允许的转换：

1. 删除 `<link name="world"/>`；
2. 删除 `<joint name="fixed_base_joint">...</joint>`；
3. 把机械臂根 `<link name="base_link">` 改为 `<link name="arm_base_link">`；
4. 把 PiPER 内部对根的 parent 引用改为 `arm_base_link`；
5. 在其前加入精确的 `base_to_arm_base_joint`；
6. 将结果插入现有 Bunker `</robot>` 前，现有 Bunker XML 不改。

新增根连接必须为：

```xml
<joint name="base_to_arm_base_joint" type="fixed">
  <origin xyz="0.39 0 0.016" rpy="0 0 0" />
  <parent link="base_link" />
  <child link="arm_base_link" />
</joint>
```

- [ ] **Step 5: 运行结构测试与 check_urdf**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q src/bunker_pro2/test/test_description_contract.py
check_urdf src/bunker_pro2/urdf/bunker_pro2.urdf
```

Expected: 全部 PASS；输出根 link 为 `robot_bottom`，机械臂分支可完整解析。

- [ ] **Step 6: 提交组合模型**

```bash
git add src/bunker_pro2/urdf/bunker_pro2.urdf \
  src/bunker_pro2/test/test_description_contract.py
git commit -m "feat(tf): attach PiPER arm to Bunker front rail"
```

### Task 3: 接入 JointState 并保持单一 TF 发布者

**Files:**
- Modify: `src/bunker_pro2/test/test_description_contract.py`
- Modify: `src/bunker_pro2/launch/description.launch.py`
- Modify: `src/bunker_pro2/package.xml`

**Interfaces:**
- Consumes: PiPER 驱动 `/joint_states_single`（`sensor_msgs/msg/JointState`）。
- Produces: `/joint_states` 汇聚输出和一个组合模型 `robot_state_publisher`。

- [ ] **Step 1: 增加失败的 launch contract 测试**

新增断言：

```python
assert source.count("package='robot_state_publisher'") == 1
assert "package='joint_state_publisher'" in source
assert "'source_list': ['/joint_states_single']" in source
assert '<exec_depend>joint_state_publisher</exec_depend>' in package_xml
assert '<exec_depend>piper_description</exec_depend>' in package_xml
```

- [ ] **Step 2: 运行并确认测试失败**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  src/bunker_pro2/test/test_description_contract.py \
  -k 'description_launch or dependency'
```

Expected: FAIL，原因是 JointState 汇聚器与依赖尚未加入。

- [ ] **Step 3: 最小修改 description launch**

在现有 robot_state_publisher 前加入：

```python
Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='bunker_pro2_joint_state_publisher',
    output='screen',
    parameters=[{
        'robot_description': robot_description,
        'source_list': ['/joint_states_single'],
        'publish_default_positions': True,
    }],
),
```

保留唯一 robot_state_publisher，不增加第二个模型发布器或 static TF publisher。

- [ ] **Step 4: 声明运行依赖并运行测试**

在 `package.xml` 增加：

```xml
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>piper_description</exec_depend>
```

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q src/bunker_pro2/test/test_description_contract.py
```

Expected: 全部 PASS。

- [ ] **Step 5: 提交 JointState 集成**

```bash
git add src/bunker_pro2/launch/description.launch.py \
  src/bunker_pro2/package.xml \
  src/bunker_pro2/test/test_description_contract.py
git commit -m "feat(description): drive combined arm TF from joint states"
```

### Task 4: 文档、构建和整机回归

**Files:**
- Modify: `src/bunker_pro2/README.md`
- Test: `src/bunker_pro2/test/test_description_contract.py`
- Test: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py`
- Test: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`
- Test: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`

**Interfaces:**
- Consumes: 完成的组合模型和 description launch。
- Produces: 可重复的只读显示/TF 验证命令与构建证据。

- [ ] **Step 1: 更新 README**

记录：权威源和哈希、整机 TF 树、安装坐标、`/joint_states_single` 数据流、零位预览行为，以及以下只读命令：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch bunker_pro2 display.launch.py
```

- [ ] **Step 2: 构建受影响 package**

Run from `track_robot_ws/`：

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  piper_description bunker_pro2 track_robot_bringup
```

Expected: 三个 package 构建成功。

- [ ] **Step 3: 运行 package 与 bringup 回归**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  src/bunker_pro2/test/test_description_contract.py \
  src/piper_description/test/test_robot_description.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
```

Expected: 全部 PASS，无 ZED/LiDAR/Phase 1–5 contract 回归。

- [ ] **Step 4: 运行无运动 TF smoke test**

启动 `bunker_pro2 description.launch.py` 后检查：

```bash
ros2 node list
ros2 topic echo --once /joint_states
ros2 run tf2_ros tf2_echo base_link arm_base_link
ros2 run tf2_ros tf2_echo arm_base_link link1
```

Expected:

- 恰好一个 `bunker_pro2_robot_state_publisher`；
- `/joint_states` 包含 `joint1...joint8`；
- `base_link -> arm_base_link` 为 `(0.39, 0, 0.016)`、零旋转；
- `arm_base_link -> link1` 可解析；
- 没有底盘或机械臂运动命令。

- [ ] **Step 5: 提交文档与最终验证状态**

```bash
git add src/bunker_pro2/README.md
git commit -m "docs: document combined Bunker PiPER model"
git status --short --branch
```

Expected: 仅保留实施前已有、与本任务无关的未跟踪 `log/`。
