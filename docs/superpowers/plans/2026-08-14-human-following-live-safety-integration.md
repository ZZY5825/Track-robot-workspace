# Human-Following Live Safety Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a separate, one-command human-following feature that defaults to motionless shadow operation, uses a gesture-gated authorization session in active mode, and preserves the existing Bunker RC takeover and fail-closed command chain.

**Architecture:** Keep perception, fusion, decision, control, avoidance, safety, and Bunker command conversion as separate existing layers. Add a typed human-following session supervisor in `track_robot_decision`, compose those layers through a dedicated `human_following_live.launch.py`, and expose them through an independent `human_following_ctl` process lifecycle. Extract only the shared physical sensor/platform launch layer; semantic-search and human-following retain independent feature state and commands.

**Tech Stack:** ROS2 Foxy, C++17 `rclcpp`, Python 3.8 launch/CLI code, `ament_cmake`, `ament_cmake_pytest`, `launch_testing`, Bunker ROS2 SDK messages, RViz2.

## Global Constraints

- Work only in `/home/track-robot/track_robot_ws/.worktrees/main-test` on canonical branch `main`.
- Do not modify or delete `track_robot_ws/bunker_pro2_joint_state_publisher.yaml`; it is unrelated user work.
- Human-following and semantic-search remain separate features with separate CLI state files and feature launches.
- `SHADOW` is the default and must not start `cmd_vel_gate` or own a `/cmd_vel` publisher.
- `ACTIVE` requires both `--runtime-mode active` and `--confirm-motion`.
- A valid start wave may authorize motion only after camera-LiDAR confirmation and all readiness conditions pass.
- `/safety/arm` remains the only final motion authorization; `cmd_vel_gate` remains the only active `/cmd_vel` publisher.
- RC control mode `3` immediately disarms and clears authorization; returning to CAN control mode `1` never resumes the prior target.
- The first supervised profile is capped at `0.05 m/s` linear speed and `0.15 rad/s` angular speed across decision, controller, planner, safety, and gate.
- Initial authorization requires camera-visible `camera_lidar`; LiDAR-only evidence cannot arm and cannot command forward motion.
- Stop gesture, target loss, hard fault, E-stop, stale required input, and target ID mismatch disarm and clear authorization.
- Short obstacle blocking may retain authorization for `10.0 s`; uncertainty may retain it for `1.0 s`; `SEARCH_ROTATE` clears authorization in this milestone.
- Existing perception topics, follow topics, semantic-search launch arguments, and legacy controller standalone interfaces remain compatible.
- Do not claim hardware readiness until supervised Gates A-D have been run on the Bunker and reviewed.

---

## File Map

**Interfaces**

- Create `track_robot_ws/src/track_robot/track_robot_interfaces/msg/HumanFollowingSession.msg`: typed runtime/session state.
- Modify `track_robot_ws/src/track_robot/track_robot_interfaces/CMakeLists.txt`: generate the new message.
- Modify `track_robot_ws/src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py`: verify constants and fields construct correctly.

**Decision and authorization**

- Create `track_robot_ws/src/track_robot/track_robot_decision/include/track_robot_decision/human_following_session_policy.hpp`: ROS-independent authorization state machine.
- Create `track_robot_ws/src/track_robot/track_robot_decision/src/human_following_session_policy.cpp`: deterministic state transitions and requested actions.
- Create `track_robot_ws/src/track_robot/track_robot_decision/src/human_following_supervisor_node.cpp`: subscriptions, freshness, service calls, typed/debug/marker publication.
- Create `track_robot_ws/src/track_robot/track_robot_decision/test/test_human_following_session_policy.cpp`: transition tests.
- Create `track_robot_ws/src/track_robot/track_robot_decision/test/test_human_following_supervisor_launch.py`: ROS service and topic integration tests.
- Modify `track_robot_ws/src/track_robot/track_robot_decision/CMakeLists.txt` and `package.xml`: build/install/test dependencies.
- Modify `track_robot_ws/src/track_robot/track_robot_decision/launch/outdoor_follow_decision.launch.py`: accept a feature profile override without changing defaults.

**Existing RC safety work**

- Finish and commit the existing changes in `follow_behavior_tree_node.cpp`, `motion_safety_supervisor_node.cpp`, their tests, package metadata, and related docs.
- Preserve the already implemented rule that Bunker RC mode is authoritative even with centered sticks.

**Shared hardware and feature bringup**

- Create `track_robot_ws/src/track_robot/track_robot_bringup/launch/track_robot_hardware.launch.py`: neutral description, ZED, RoboSense, Bunker, and IMU composition.
- Modify `semantic_search_sensors.launch.py` and `semantic_search_platform.launch.py`: compatibility wrappers over the neutral hardware launch.
- Create `track_robot_ws/src/track_robot/track_robot_bringup/launch/human_following_live.launch.py`: complete human-following composition.
- Create `track_robot_ws/src/track_robot/track_robot_bringup/config/human_following_shadow.yaml` and `human_following_supervised_test.yaml`: node-scoped feature profiles.
- Create `track_robot_ws/src/track_robot/track_robot_bringup/rviz/human_following_live.rviz`: live session, target, obstacle, command, and safety displays.
- Modify `safe_human_following.launch.py`, controller/safety nested launches, `CMakeLists.txt`, and `package.xml` to accept the profile and preserve old entry points.

**One-command lifecycle**

- Create `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/human_following_cli.py`: independent parser and lifecycle.
- Create `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/human_following_readiness.py`: feature-specific readiness checks.
- Create `track_robot_ws/src/track_robot/track_robot_bringup/scripts/human_following_ctl`: installed executable.
- Create `test/test_human_following_cli.py`, `test/test_human_following_readiness.py`, and `test/test_human_following_launch_contract.py`.
- Modify `process_control.py` only to expose a neutral state-path helper if tests show one is needed; do not share state files between features.
- Modify `track_robot_ws/src/track_robot_core/src/cmd_vel_gate.cpp`, `CMakeLists.txt`, and `package.xml`: provide a bounded zero-then-shutdown service for ordered feature shutdown.
- Create `track_robot_ws/src/track_robot_core/test/test_cmd_vel_gate_shutdown_launch.py`: prove the gate publishes zero and removes its `/cmd_vel` publisher before feature cleanup.

**Documentation**

- Update `track_robot_ws/src/track_robot_perception/docs/human_tracking_reinforcement.md` with the integration boundary and current status.
- Update `track_robot_ws/src/track_robot/track_robot_decision/docs/outdoor_decision.md` and `track_robot_ws/src/track_robot/track_robot_safety/docs/obstacle_safety.md` with authorization/RC contracts.
- Create `docs/guides/human-following/live-supervised-test.md` and `docs/guides/human-following/gate-report-template.md`.

---

### Task 1: Close and Commit the Existing RC Takeover Safety Change

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/src/motion_safety_supervisor_node.cpp`
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/test/test_rc_control_mode_launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/CMakeLists.txt`
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/package.xml`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/src/follow_behavior_tree_node.cpp`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/test/test_follow_decision_launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/docs/obstacle_safety.md`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/docs/outdoor_decision.md`

**Interfaces:**
- Consumes: `/bunker_status`, `/bunker_rc_state`, `/safety/state`, `/human_tracking/reset_target`.
- Produces: immediate `SafetyState.STATE_RC_OVERRIDE`, zero `/follow/cmd_vel_safe`, disarmed safety, and one target reset on decision transition.

- [ ] **Step 1: Review the existing dirty diff and preserve only in-scope changes**

Run:

```bash
git diff -- track_robot_ws/src/track_robot/track_robot_safety \
  track_robot_ws/src/track_robot/track_robot_decision
```

Expected: the safety diff separates `rc_control_mode_active_` from stick override, and the decision diff requests target reset on `BEHAVIOR_RC_OVERRIDE`. No unrelated file is staged.

- [ ] **Step 2: Build the two changed packages**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --packages-select track_robot_safety track_robot_decision
source track_robot_ws/install/setup.bash
```

Expected: both packages build successfully with the new launch-test dependency.

- [ ] **Step 3: Run the focused RC launch test**

Run:

```bash
source /opt/ros/foxy/setup.bash
source track_robot_ws/install/setup.bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_safety \
  --ctest-args -R test_rc_control_mode_launch --output-on-failure
```

Expected: the test proves CAN -> RC -> CAN results in `RC_OVERRIDE -> DISARMED`, never armed restoration.

- [ ] **Step 4: Run the focused decision launch test**

Run:

```bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_decision \
  --ctest-args -R test_follow_decision_launch --output-on-failure
```

Expected: `BEHAVIOR_RC_OVERRIDE` is motionless and invokes `/human_tracking/reset_target`.

- [ ] **Step 5: Commit the isolated RC behavior**

```bash
git add \
  track_robot_ws/src/track_robot/track_robot_safety \
  track_robot_ws/src/track_robot/track_robot_decision/src/follow_behavior_tree_node.cpp \
  track_robot_ws/src/track_robot/track_robot_decision/test/test_follow_decision_launch.py \
  track_robot_ws/src/track_robot/track_robot_decision/docs/outdoor_decision.md
git commit -m "fix: make Bunker RC mode revoke follow control"
```

Expected: the unrelated joint-state YAML and perception progress document remain unstaged.

---

### Task 2: Add the Typed Human-Following Session Interface

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_interfaces/msg/HumanFollowingSession.msg`
- Modify: `track_robot_ws/src/track_robot/track_robot_interfaces/CMakeLists.txt`
- Modify: `track_robot_ws/src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py`

**Interfaces:**
- Produces: `track_robot_interfaces/msg/HumanFollowingSession` used by the supervisor, RViz/debug tools, CLI diagnostics, and tests.

- [ ] **Step 1: Write the failing interface construction test**

Append this complete test:

```python
from track_robot_interfaces.msg import HumanFollowingSession


def test_human_following_session_exposes_motion_authorization_state():
    session = HumanFollowingSession(
        runtime_mode=HumanFollowingSession.MODE_ACTIVE,
        state=HumanFollowingSession.STATE_FOLLOWING,
        logical_target_id=17,
        motion_session_enabled=True,
        target_authorized=True,
        arm_request_pending=False,
        safety_armed=True,
        rc_override_active=False,
        target_confidence=0.82,
        reason='confirmed_camera_lidar',
    )

    assert HumanFollowingSession.MODE_SHADOW == 0
    assert HumanFollowingSession.MODE_ACTIVE == 1
    assert HumanFollowingSession.STATE_WAITING_FOR_GESTURE == 1
    assert HumanFollowingSession.STATE_RC_OVERRIDE == 6
    assert session.logical_target_id == 17
    assert session.target_authorized is True
```

- [ ] **Step 2: Run the test and verify the message is missing**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_interfaces --event-handlers console_direct+
```

Expected: failure importing `HumanFollowingSession`.

- [ ] **Step 3: Create the exact message**

```text
uint8 MODE_SHADOW=0
uint8 MODE_ACTIVE=1

uint8 STATE_STARTING=0
uint8 STATE_WAITING_FOR_GESTURE=1
uint8 STATE_VALIDATING_TARGET=2
uint8 STATE_ARMING=3
uint8 STATE_FOLLOWING=4
uint8 STATE_BLOCKED=5
uint8 STATE_RC_OVERRIDE=6
uint8 STATE_FAULT=7
uint8 STATE_DISARMED=8

std_msgs/Header header
uint8 runtime_mode
uint8 state
int32 logical_target_id
bool motion_session_enabled
bool target_authorized
bool arm_request_pending
bool safety_armed
bool rc_override_active
float32 target_confidence
string reason
```

Add `"msg/HumanFollowingSession.msg"` to `msg_files` in `CMakeLists.txt`.

- [ ] **Step 4: Build and run the interface test**

Run:

```bash
colcon build --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --packages-select track_robot_interfaces
source track_robot_ws/install/setup.bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_interfaces --event-handlers console_direct+
```

Expected: build and interface tests pass.

- [ ] **Step 5: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_interfaces
git commit -m "feat: add typed human-following session state"
```

---

### Task 3: Build a Deterministic Authorization Policy

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_decision/include/track_robot_decision/human_following_session_policy.hpp`
- Create: `track_robot_ws/src/track_robot/track_robot_decision/src/human_following_session_policy.cpp`
- Create: `track_robot_ws/src/track_robot/track_robot_decision/test/test_human_following_session_policy.cpp`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/CMakeLists.txt`

**Interfaces:**
- Consumes: immutable `SessionInputs` snapshots and explicit arm-service results.
- Produces: `SessionDecision` containing state, authorization, and one-shot actions `request_arm`, `request_disarm`, and `request_target_reset`.

- [ ] **Step 1: Define the policy interface and write failing transition tests**

Use this public contract:

```cpp
namespace track_robot_decision {

enum class RuntimeMode : uint8_t { Shadow = 0, Active = 1 };
enum class SessionState : uint8_t {
  Starting = 0, WaitingForGesture = 1, ValidatingTarget = 2,
  Arming = 3, Following = 4, Blocked = 5, RcOverride = 6,
  Fault = 7, Disarmed = 8
};

struct SessionInputs {
  double now_sec{0.0};
  bool start_gesture_event{false};
  bool stop_gesture_event{false};
  int32_t gesture_visual_track_id{-1};
  bool camera_confirmed{false};
  int32_t camera_visual_track_id{-1};
  int32_t camera_logical_target_id{-1};
  bool decision_confirmed_camera_lidar{false};
  bool decision_lidar_limited{false};
  bool decision_uncertain{false};
  bool decision_search_rotate{false};
  bool decision_target_lost{false};
  int32_t decision_logical_target_id{-1};
  float decision_confidence{0.0F};
  bool health_healthy{false};
  bool health_hard_fault{false};
  bool planner_ready{false};
  bool planner_blocked{false};
  bool safety_disarmed_ready{false};
  bool safety_armed{false};
  bool safety_hard_fault{false};
  bool rc_override{false};
  bool bunker_can_healthy{false};
  bool required_inputs_fresh{false};
  bool arm_service_ready{false};
};

struct SessionDecision {
  SessionState state{SessionState::Starting};
  int32_t logical_target_id{-1};
  bool target_authorized{false};
  bool arm_request_pending{false};
  bool request_arm{false};
  bool request_disarm{false};
  bool request_target_reset{false};
  std::string reason;
};

class HumanFollowingSessionPolicy {
public:
  HumanFollowingSessionPolicy(RuntimeMode mode, bool motion_confirmed,
    double blocked_timeout_sec, double uncertain_timeout_sec);
  SessionDecision update(const SessionInputs & inputs);
  SessionDecision acceptArmResult(bool success, const std::string & message);
};

}  // namespace track_robot_decision
```

Create one explicit GTest case for each of these exact sequences:

```cpp
TEST(HumanFollowingSessionPolicy, ShadowNeverRequestsArm)
TEST(HumanFollowingSessionPolicy, ConfirmedWaveRequestsArmOnce)
TEST(HumanFollowingSessionPolicy, LidarOnlyCannotInitiallyArm)
TEST(HumanFollowingSessionPolicy, StopGestureDisarmsAndResets)
TEST(HumanFollowingSessionPolicy, RcTakeoverRevokesAndCanReturnDoesNotResume)
TEST(HumanFollowingSessionPolicy, TargetMismatchDisarms)
TEST(HumanFollowingSessionPolicy, ShortBlockRetainsAuthorization)
TEST(HumanFollowingSessionPolicy, BlockTimeoutRevokesAuthorization)
TEST(HumanFollowingSessionPolicy, UncertaintyTimeoutRevokesAuthorization)
TEST(HumanFollowingSessionPolicy, SearchRotateRevokesAuthorization)
TEST(HumanFollowingSessionPolicy, ArmRejectionRequiresANewGesture)
```

- [ ] **Step 2: Run the policy test and verify it fails before implementation**

Run:

```bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_decision \
  --ctest-args -R test_human_following_session_policy --output-on-failure
```

Expected: target/test is absent or does not compile.

- [ ] **Step 3: Implement the state machine without ROS time or ROS messages**

Implementation rules inside `update()`:

```cpp
const bool initial_target_valid =
  inputs.camera_confirmed &&
  inputs.camera_visual_track_id == pending_visual_track_id_ &&
  inputs.decision_confirmed_camera_lidar &&
  inputs.decision_logical_target_id == inputs.camera_logical_target_id;

const bool initial_system_valid =
  inputs.health_healthy && inputs.planner_ready &&
  inputs.safety_disarmed_ready && inputs.bunker_can_healthy &&
  inputs.required_inputs_fresh && inputs.arm_service_ready;

if (inputs.rc_override) {
  return revoke(SessionState::RcOverride, "rc_override", true);
}
if (inputs.health_hard_fault || inputs.safety_hard_fault) {
  return revoke(SessionState::Fault, "hard_fault", true);
}
if (authorized_ && inputs.decision_logical_target_id != authorized_target_id_) {
  return revoke(SessionState::Fault, "logical_target_mismatch", true);
}
```

`revoke()` clears pending gesture, authorization, and arm request; emits disarm/reset only once on entry. `start_gesture_event` is consumed once. Returning from RC to CAN transitions to `WaitingForGesture`, not `ValidatingTarget`. An arm rejection enters `Fault` and cannot retry until a new start event is received.

- [ ] **Step 4: Run all policy tests**

Expected: all eleven transition tests pass deterministically without starting ROS.

- [ ] **Step 5: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_decision/include \
  track_robot_ws/src/track_robot/track_robot_decision/src/human_following_session_policy.cpp \
  track_robot_ws/src/track_robot/track_robot_decision/test/test_human_following_session_policy.cpp \
  track_robot_ws/src/track_robot/track_robot_decision/CMakeLists.txt
git commit -m "feat: add human-following authorization policy"
```

---

### Task 4: Connect the Policy to ROS Services and Typed State

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_decision/src/human_following_supervisor_node.cpp`
- Create: `track_robot_ws/src/track_robot/track_robot_decision/test/test_human_following_supervisor_launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/CMakeLists.txt`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/package.xml`

**Interfaces:**
- Consumes: `GestureState`, `CameraTarget`, `FollowDecision`, `PerceptionHealth`, `AvoidanceState`, `SafetyState`, and `BunkerStatus` on the design-specified topics.
- Calls: `/safety/arm`, `/safety/disarm`, `/human_tracking/reset_target` using `std_srvs/Trigger`.
- Produces: `/human_following/session_state`, `/human_following/supervisor_debug`, and `/human_following/supervisor_markers`.

- [ ] **Step 1: Write the failing ROS launch tests**

The test process supplies all seven input topics and three fake services. Cover:

```python
def test_shadow_wave_never_calls_arm(self):
    self.publish_confirmed_start_sequence()
    self.spin_for(0.5)
    self.assertEqual(self.arm_calls, 0)

def test_active_confirmed_sequence_calls_arm_once(self):
    self.publish_confirmed_start_sequence()
    self.spin_until(lambda: self.arm_calls == 1)
    self.spin_for(0.5)
    self.assertEqual(self.arm_calls, 1)

def test_stop_disarms_and_resets(self):
    self.authorize_active_session()
    self.publish_stop_gesture_for_authorized_target()
    self.spin_until(lambda: self.disarm_calls == 1 and self.reset_calls == 1)

def test_rc_mode_three_disarms_even_with_no_stick_message(self):
    self.authorize_active_session()
    self.publish_bunker_status(control_mode=3)
    self.spin_until(lambda: self.last_session.state ==
                    HumanFollowingSession.STATE_RC_OVERRIDE)
    self.assertFalse(self.last_session.target_authorized)
```

Use an isolated `ROS_DOMAIN_ID` that does not collide with existing test domains 226 and 227.

- [ ] **Step 2: Run the test and verify the node is missing**

Expected: launch cannot resolve `human_following_supervisor_node`.

- [ ] **Step 3: Implement the node adapter**

Declare these parameters with exact defaults:

```cpp
runtime_mode = "shadow"
motion_confirmed = false
tick_rate = 20.0
gesture_timeout_sec = 0.50
camera_timeout_sec = 0.35
decision_timeout_sec = 0.30
health_timeout_sec = 0.30
avoidance_timeout_sec = 0.30
safety_timeout_sec = 0.20
bunker_timeout_sec = 0.20
blocked_disarm_timeout_sec = 10.0
uncertain_authorization_timeout_sec = 1.0
```

Deduplicate gestures with `(header.stamp.sec, header.stamp.nanosec, track_id, command)`. A start event is only `command == "start_tracking" && trigger_active`. A stop event is only `command == "stop_tracking" && trigger_active` and must match the authorized visual target.

Build `SessionInputs` from fresh typed messages on every 20 Hz tick. Do not use JSON as a control input. Call each service asynchronously and keep one pending future per service. Feed the arm result exactly once into `acceptArmResult()`.

Publish a session marker array containing:

```text
namespace human_following_session/status: TEXT_VIEW_FACING state/reason
namespace human_following_session/authorization: SPHERE at target position when authorized
namespace human_following_session/mode: TEXT_VIEW_FACING SHADOW or ACTIVE
```

Map policy state values directly to `HumanFollowingSession` constants and publish `target_confidence` from the matching `FollowDecision.decision_confidence`.

- [ ] **Step 4: Add build metadata**

Add `bunker_msgs` to decision dependencies, install the new executable, add `ament_cmake_gtest`, and register both the policy GTest and launch test.

- [ ] **Step 5: Run policy and launch integration tests**

Expected: shadow never arms; active arms once; stop and RC each revoke authorization; CAN return remains waiting for a wave.

- [ ] **Step 6: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_decision
git commit -m "feat: supervise human-following motion sessions"
```

---

### Task 5: Add Consistent Shadow and Supervised Motion Profiles

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/config/human_following_shadow.yaml`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/config/human_following_supervised_test.yaml`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/launch/outdoor_follow_decision.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_control/launch/target_follow_controller.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/launch/motion_safety.launch.py`

**Interfaces:**
- Produces: one node-scoped YAML profile loaded after existing package defaults, so only explicit human-following overrides change.

- [ ] **Step 1: Write a failing profile consistency test**

```python
LIMIT_FIELDS = {
    'follow_behavior_tree_node': ('confirmed_max_linear', 'confirmed_max_angular'),
    'target_follow_controller_node': ('max_linear_x', 'max_angular_z'),
    'local_trajectory_planner_node': ('max_linear_x', 'max_angular_z'),
    'motion_safety_supervisor_node': ('max_linear_x', 'max_angular_z'),
    'cmd_vel_gate': ('max_linear_x', 'max_angular_z'),
}


def test_supervised_profile_has_one_consistent_limit_pair():
    profile = load_profile('human_following_supervised_test.yaml')
    pairs = {
        tuple(profile[node]['ros__parameters'][key] for key in keys)
        for node, keys in LIMIT_FIELDS.items()
    }
    assert pairs == {(0.05, 0.15)}
    controller = profile['target_follow_controller_node']['ros__parameters']
    supervisor = profile['motion_safety_supervisor_node']['ros__parameters']
    assert controller['follow_distance'] == 2.0
    assert controller['allow_lidar_only_forward_motion'] is False
    assert supervisor['require_odom'] is True
    assert supervisor['odom_timeout_sec'] == 0.25
```

- [ ] **Step 2: Run the test and verify profiles are missing**

Expected: file-not-found failure.

- [ ] **Step 3: Add complete node-scoped profiles**

The supervised profile must include:

```yaml
follow_behavior_tree_node:
  ros__parameters:
    confirmed_max_linear: 0.05
    confirmed_max_angular: 0.15
    lidar_max_linear: 0.0
    lidar_max_angular: 0.15
    search_max_angular: 0.0
target_follow_controller_node:
  ros__parameters:
    follow_distance: 2.0
    max_linear_x: 0.05
    max_angular_z: 0.15
    linear_accel_limit: 0.05
    angular_accel_limit: 0.15
    allow_lidar_only_forward_motion: false
local_trajectory_planner_node:
  ros__parameters:
    max_linear_x: 0.05
    max_angular_z: 0.15
motion_safety_supervisor_node:
  ros__parameters:
    max_linear_x: 0.05
    max_angular_z: 0.15
    require_odom: true
    odom_timeout_sec: 0.25
cmd_vel_gate:
  ros__parameters:
    max_linear_x: 0.05
    max_angular_z: 0.15
human_following_supervisor_node:
  ros__parameters:
    blocked_disarm_timeout_sec: 10.0
    uncertain_authorization_timeout_sec: 1.0
```

The shadow profile uses the same feature topology and conservative limits but sets supervisor `runtime_mode: shadow` and `motion_confirmed: false`. It is not a route to motion.

- [ ] **Step 4: Forward `profile_config` through nested launches**

Each nested launch declares `profile_config` and adds it after its existing defaults:

```python
parameters=[base_config, LaunchConfiguration('profile_config'), overrides]
```

The existing defaults and public arguments remain unchanged when `profile_config` is empty. Use an `OpaqueFunction` to omit an empty profile path rather than passing an invalid filename to ROS2 Foxy.

- [ ] **Step 5: Run profile and existing launch-contract tests**

Expected: profile test passes and existing standalone launch contracts remain valid.

- [ ] **Step 6: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_bringup/config \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py \
  track_robot_ws/src/track_robot/track_robot_decision/launch \
  track_robot_ws/src/track_robot/track_robot_control/launch \
  track_robot_ws/src/track_robot/track_robot_safety/launch
git commit -m "feat: add supervised human-following motion profiles"
```

---

### Task 6: Extract a Neutral Shared Hardware Launch

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/launch/track_robot_hardware.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_sensors.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_platform.launch.py`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py`

**Interfaces:**
- Consumes launch flags: `start_description`, `start_camera`, `start_lidar`, `start_base`, `start_imu` and existing camera/LiDAR/network/TF arguments.
- Produces the same physical ROS topics and TF as existing semantic-search sensor launches.

- [ ] **Step 1: Write failing AST launch-contract tests**

Assert that the neutral launch includes exactly one conditional instance of:

```text
bunker_pro2/description.launch.py
track_robot_bringup/semantic_search_camera.launch.py
track_robot_bringup/rslidar_with_tf.launch.py
bunker_base/bunker_base.launch.py
track_robot_perception/phidget_imu.launch.py
```

Assert that the neutral launch contains no semantic-search, human-tracking, navigation, controller, planner, safety, or `/cmd_vel` feature node.

- [ ] **Step 2: Run the launch-contract test and verify the neutral file is missing**

- [ ] **Step 3: Implement `track_robot_hardware.launch.py`**

Use one `IncludeLaunchDescription` per physical module with `IfCondition` on its start flag. Preserve these defaults:

```text
base_frame=base_link
camera_depth_mode=NONE
configure_network=true
network_interface=eth0
host_ip=192.168.1.102
host_cidr=24
driver_start_delay=1.0
publish_base_lidar_tf=false
extrinsic_mode=robot_description
```

The robot description owns canonical sensor transforms when `start_description=true`; duplicate static TF publication remains disabled.

- [ ] **Step 4: Convert semantic launch files into compatibility wrappers**

`semantic_search_sensors.launch.py` forwards its existing arguments into the neutral launch with `start_description=false`. `semantic_search_platform.launch.py` forwards only base/IMU flags with camera/LiDAR/description false. `semantic_search_live.launch.py` keeps its existing description ownership and public argument behavior.

- [ ] **Step 5: Run all bringup launch-contract tests**

Expected: old semantic-search contracts and new neutral hardware contract pass; no physical driver is duplicated in the composed AST.

- [ ] **Step 6: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_bringup/launch \
  track_robot_ws/src/track_robot/track_robot_bringup/test
git commit -m "refactor: share a neutral robot hardware launch"
```

---

### Task 7: Compose the Dedicated Human-Following Live Launch

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/launch/human_following_live.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/safe_human_following.launch.py`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/rviz/human_following_live.rviz`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py`

**Interfaces:**
- Consumes: hardware start booleans, `runtime_mode`, `motion_confirmed`, profile path, sensor topics/frames, and `start_rviz`.
- Produces: complete perception-to-safe-command graph; `/cmd_vel` exists only in confirmed active launch mode.

- [ ] **Step 1: Write failing launch topology tests**

The tests use the existing AST helpers and make these complete assertions:

```python
def test_shadow_is_the_fail_closed_default():
    defaults = _argument_defaults(HUMAN_FOLLOWING_LIVE)
    assert defaults['runtime_mode'] == 'shadow'
    assert defaults['motion_confirmed'] == 'false'


def test_gate_has_an_active_and_confirmed_condition():
    source = _source(HUMAN_FOLLOWING_LIVE)
    assert source.count("executable='cmd_vel_gate'") == 1
    assert "runtime_mode == 'active'" in source
    assert 'motion_confirmed' in source


def test_launch_composes_only_the_human_following_feature():
    source = _source(HUMAN_FOLLOWING_LIVE)
    assert 'human_tracking_simplified.launch.py' in source
    assert 'human_following_supervisor_node' in source
    assert 'enable_cmd_vel' in source and "'false'" in source
    assert 'semantic_search_phase' not in source


def test_one_profile_path_is_forwarded_to_all_command_layers():
    source = _source(HUMAN_FOLLOWING_LIVE)
    assert source.count("LaunchConfiguration('profile_config')") >= 4
    assert 'outdoor_follow_decision.launch.py' in source
    assert 'target_follow_controller.launch.py' in source
    assert 'motion_safety.launch.py' in source
    assert "executable='cmd_vel_gate'" in source
```

- [ ] **Step 2: Run tests and verify the live launch is missing**

- [ ] **Step 3: Implement fail-closed mode validation**

At launch expansion, accept only `shadow` and `active`. Raise `RuntimeError` when active lacks confirmation:

```python
if runtime_mode == 'active' and not motion_confirmed:
    raise RuntimeError(
        'active human following requires motion_confirmed:=true')
```

The gate condition is exactly `runtime_mode == 'active' and motion_confirmed`. The supervisor receives both values. Shadow never constructs a gate action.

- [ ] **Step 4: Compose the feature nodes in the existing safe path**

Include:

```text
track_robot_hardware.launch.py
human_tracking_simplified.launch.py
outdoor_follow_decision.launch.py
target_follow_controller.launch.py with enable_cmd_vel=false
motion_safety.launch.py
human_following_supervisor_node
cmd_vel_gate only in active mode
rviz2 when start_rviz=true
```

The command topics stay:

```text
/follow/cmd_vel_planned -> /follow/cmd_vel_avoiding
-> /follow/cmd_vel_safe -> /cmd_vel
```

`safe_human_following.launch.py` becomes a compatibility wrapper over the feature-only portion and does not silently enable motion.

- [ ] **Step 5: Add RViz displays**

Configure fixed frame `base_link` and displays for target/fusion markers, LiDAR tracklets, follow decision, controller arrow, avoidance trajectories, safety envelope, and `/human_following/supervisor_markers`. Display names clearly separate observed physical tracklet, filtered logical target, planned command, safe command, and authorization session.

- [ ] **Step 6: Run launch and profile contract tests**

Expected: shadow has zero gate nodes; active has one; both include the full decision path; semantic-search nodes are absent.

- [ ] **Step 7: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_bringup/launch \
  track_robot_ws/src/track_robot/track_robot_bringup/rviz/human_following_live.rviz \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py
git commit -m "feat: compose live human-following bringup"
```

---

### Task 8: Add Independent One-Command Control and Readiness

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/human_following_cli.py`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/human_following_readiness.py`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/scripts/human_following_ctl`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_cli.py`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_readiness.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/CMakeLists.txt`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/package.xml`

**Interfaces:**
- Produces commands `start`, `status`, `doctor`, and `stop` under `human_following_ctl`.
- Reuses `HardwareSelection`, `managed_environment`, hardware publisher probing, `ProcessManager`, and verified process-group cleanup.
- Uses state path `~/.ros/track_robot_human_following/managed_process.json`, never semantic-search's state path.

- [ ] **Step 1: Write parser and launch-vector tests**

```python
def test_shadow_is_default():
    args = build_parser().parse_args(['start'])
    assert args.runtime_mode == 'shadow'
    assert args.confirm_motion is False


def test_active_requires_confirmation():
    with pytest.raises(ValueError):
        validate_motion_request(build_parser().parse_args([
            'start', '--runtime-mode', 'active']))


def test_active_launch_vector_is_explicit(tmp_path):
    args = build_parser().parse_args([
        'start', '--runtime-mode', 'active', '--confirm-motion'])
    command = build_launch_argv(args, HardwareSelection(True, True, True, True))
    assert 'runtime_mode:=active' in command
    assert 'motion_confirmed:=true' in command
    profile_args = [value for value in command
                    if value.startswith('profile_config:=')]
    assert len(profile_args) == 1
    assert profile_args[0].endswith(
        '/config/human_following_supervised_test.yaml')
```

- [ ] **Step 2: Write readiness tests using an injected subprocess runner**

The readiness result checks exact publishers/freshness for image, camera info, IMU, cloud, Bunker status, RC state, odometry, target state, perception health, avoidance, and safety. It checks TF pairs, three services, and `/cmd_vel` publisher count:

```text
shadow expected /cmd_vel publishers: 0
active expected /cmd_vel publishers: 1
```

Any duplicate sensor publisher, missing service, non-CAN Bunker mode, base error, stale topic, missing TF, or wrong `/cmd_vel` count is `FAIL` for active start.

- [ ] **Step 3: Run tests and verify modules are missing**

- [ ] **Step 4: Implement the independent CLI**

Supported commands:

```text
human_following_ctl doctor [--runtime-mode shadow|active] [--hardware auto|external]
human_following_ctl start [--runtime-mode shadow|active] [--hardware auto|external] [--confirm-motion]
human_following_ctl status
human_following_ctl stop
```

Use ROS Domain 20 and the existing bounded subprocess rules. `--hardware auto` reuses any required topic with a publisher and starts only missing camera, LiDAR, base, or IMU modules. `--hardware external` starts none and fails readiness when dependencies are absent.

Build the launch vector explicitly:

```python
[
    'ros2', 'launch', 'track_robot_bringup',
    'human_following_live.launch.py',
    'runtime_mode:={}'.format(args.runtime_mode),
    'motion_confirmed:={}'.format(str(args.confirm_motion).lower()),
    'start_camera:={}'.format(str(selection.camera).lower()),
    'start_lidar:={}'.format(str(selection.lidar).lower()),
    'start_base:={}'.format(str(selection.base).lower()),
    'start_imu:={}'.format(str(selection.imu).lower()),
    'profile_config:={}'.format(paths['profile_config']),
]
```

- [ ] **Step 5: Add an ordered zero-then-shutdown service to `cmd_vel_gate`**

Declare `shutdown_service` with default `/cmd_vel_gate/shutdown`. The Trigger callback publishes one zero command, cancels command input and watchdog activity, responds successfully, and schedules `rclcpp::shutdown()` after `50 ms` so the service response can leave the process. Register a launch test that verifies a prior nonzero input is followed by zero and the `/cmd_vel` publisher disappears.

Add `std_srvs`, `launch_testing_ament_cmake`, `launch_testing_ros`, and `rclpy` only to the corresponding runtime/test dependencies.

- [ ] **Step 6: Implement bounded safe stop before process cleanup**

`stop` performs:

```python
disarmed = call_trigger('/safety/disarm', timeout=2.0)
zero = observe_zero_twist('/follow/cmd_vel_safe', timeout=1.0)
if not disarmed or not zero:
    call_trigger('/safety/emergency_stop', timeout=2.0)
if active_run:
    gate_stopped = call_trigger('/cmd_vel_gate/shutdown', timeout=2.0)
    if not gate_stopped:
        call_trigger('/safety/emergency_stop', timeout=2.0)
stopped = process_manager.stop_owned()
```

The CLI verifies that `/cmd_vel` has no publisher after the gate shutdown request and before signalling the owned feature process group. It sends no signal when ownership identity cannot be verified and never terminates externally owned hardware.

- [ ] **Step 7: Install the script and register tests**

`scripts/human_following_ctl` contains:

```python
#!/usr/bin/env python3
from track_robot_bringup.human_following_cli import main

raise SystemExit(main())
```

- [ ] **Step 8: Run all gate, CLI, readiness, process-control, and semantic-search CLI tests**

Expected: new feature tests pass and existing `semantic_search_ctl` behavior is unchanged.

- [ ] **Step 9: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_bringup \
  track_robot_ws/src/track_robot_core
git commit -m "feat: add one-command human-following operation"
```

---

### Task 9: Complete Cross-Package Safety Acceptance Tests

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/test/test_human_following_supervisor_launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_cli.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/test/test_rc_control_mode_launch.py`

**Interfaces:**
- Validates the design's eighteen automated guarantees without physical robot movement.

- [ ] **Step 1: Add the remaining supervisor scenario matrix**

Add explicit launch-test cases for this disarm-cause matrix:

```text
target_lost       -> disarm=1, reset=1, authorized=false
health_stale      -> disarm=1, reset=1, state=FAULT
base_fault        -> disarm=1, reset=1, state=FAULT
emergency_stop    -> disarm=1, reset=1, state=FAULT
input_stale       -> disarm=1, reset=1, state=FAULT
target_id_mismatch -> disarm=1, reset=1, state=FAULT
```

Also verify short blocking retains authorization, blocking beyond `10.0 s` disarms, LiDAR-only cannot initially arm, and a new wave after CAN return can create one new arm request.

- [ ] **Step 2: Add graph ownership assertions**

Run launch tests with isolated ROS domains and query `/cmd_vel`:

```text
shadow: Publisher count 0
active: Publisher count 1, node name /cmd_vel_gate
```

The active test uses fake hardware/topic publishers and does not connect to the physical CAN interface.

- [ ] **Step 3: Add effective parameter equality assertions**

For an expanded active launch, query all five nodes and assert exact values `0.05` and `0.15`. Also assert `require_odom=true`, `odom_timeout_sec=0.25`, and `allow_lidar_only_forward_motion=false`.

- [ ] **Step 4: Run affected packages together to expose ROS-domain leakage**

Run:

```bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select \
    track_robot_interfaces \
    track_robot_core \
    track_robot_decision \
    track_robot_control \
    track_robot_safety \
    track_robot_bringup \
  --event-handlers console_direct+
colcon test-result --test-result-base track_robot_ws/test_results --verbose
```

Expected: zero failed tests and no test receives another test's safety messages.

- [ ] **Step 5: Commit**

```bash
git add track_robot_ws/src/track_robot/track_robot_decision/test \
  track_robot_ws/src/track_robot/track_robot_safety/test \
  track_robot_ws/src/track_robot/track_robot_bringup/test
git commit -m "test: cover live human-following safety lifecycle"
```

---

### Task 10: Document One-Command Use and Hardware Gates

**Files:**
- Modify: `track_robot_ws/src/track_robot_perception/docs/human_tracking_reinforcement.md`
- Modify: `track_robot_ws/src/track_robot/track_robot_decision/docs/outdoor_decision.md`
- Modify: `track_robot_ws/src/track_robot/track_robot_safety/docs/obstacle_safety.md`
- Create: `docs/guides/human-following/live-supervised-test.md`
- Create: `docs/guides/human-following/gate-report-template.md`
- Create: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_docs.py`

**Interfaces:**
- Produces executable operator guidance, explicit limitations, emergency procedures, and evidence templates.

- [ ] **Step 1: Write documentation contract tests**

Assert the guide contains exact commands for shadow start, active start, status, stop, emergency stop, session-state echo, and `/cmd_vel` publisher inspection. Assert it contains Gates A-D and the warning that CAN return requires a new wave.

- [ ] **Step 2: Run the docs test and verify the guide is missing**

- [ ] **Step 3: Write the live operation guide**

Include these exact operator commands:

```bash
ros2 run track_robot_bringup human_following_ctl start \
  --runtime-mode shadow --hardware auto

ros2 run track_robot_bringup human_following_ctl start \
  --runtime-mode active --hardware auto --confirm-motion

ros2 run track_robot_bringup human_following_ctl status
ros2 run track_robot_bringup human_following_ctl stop
ros2 service call /safety/emergency_stop std_srvs/srv/Trigger '{}'
```

Document the expected session FSM, wave/stop behavior, RC mode takeover, no auto-resume, topic/marker meanings, and low obstacle/drop-off/terrain/weather/LiDAR-only limitations.

- [ ] **Step 4: Write Gate A-D procedures and report template**

Each gate records date, commit, operator, runtime mode, effective limits, topic rates, state transitions, observed stops, failures, and pass/fail decision. Gate B explicitly requires tracks lifted; Gate C uses `0.05 m/s`; Gate D uses foam or boxes and never a person as a collision obstacle.

- [ ] **Step 5: Update component documents**

The perception document ends its ownership at trusted fused target state. The decision document owns target usability and session intent. The safety document owns final arm/disarm, RC/E-stop, obstacle, freshness, and zero-command enforcement.

- [ ] **Step 6: Run docs tests and commit**

```bash
git add docs/guides/human-following \
  track_robot_ws/src/track_robot/track_robot_perception/docs/human_tracking_reinforcement.md \
  track_robot_ws/src/track_robot/track_robot_decision/docs/outdoor_decision.md \
  track_robot_ws/src/track_robot/track_robot_safety/docs/obstacle_safety.md \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_docs.py
git commit -m "docs: add supervised human-following test procedure"
```

---

### Task 11: Final Software Verification and Hardware Handoff

**Files:**
- Verify only; do not alter hardware calibration or acceptance results.

**Interfaces:**
- Produces a clean software verification record and commands for the operator-run Gates A-D.

- [ ] **Step 1: Build the complete affected dependency closure**

```bash
source /opt/ros/foxy/setup.bash
colcon build --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --packages-up-to track_robot_bringup \
  --event-handlers console_direct+
```

Expected: all affected packages compile on ROS2 Foxy/Jetson without adding a new neural runtime.

- [ ] **Step 2: Run the full affected test closure**

```bash
source track_robot_ws/install/setup.bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select \
    track_robot_interfaces \
    track_robot_core \
    track_robot_decision \
    track_robot_control \
    track_robot_safety \
    track_robot_bringup \
  --event-handlers console_direct+
colcon test-result --test-result-base track_robot_ws/test_results --verbose
```

Expected: zero errors and zero failures.

- [ ] **Step 3: Check repository scope and formatting**

```bash
git diff --check
git status --short
git log --oneline -12
```

Expected: only intentional feature changes remain; `bunker_pro2_joint_state_publisher.yaml` is untouched; implementation history is separated into reviewable commits.

- [ ] **Step 4: Run Gate A in shadow mode only**

Use the guide to verify the complete live graph and RViz while confirming `/cmd_vel` has no publisher. Record results in a dated copy of the report template.

- [ ] **Step 5: Hand off Gates B-D to supervised physical testing**

Do not mark the feature hardware-ready in software documentation. The operator advances only after each previous gate has a reviewed passing report.
