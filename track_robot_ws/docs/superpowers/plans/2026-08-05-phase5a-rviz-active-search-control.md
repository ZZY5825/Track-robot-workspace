# Phase 5A RViz Active Search Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one Start Finding / Stop Finding control to the existing RViz semantic-search panel so an operator can launch, authorize, observe, and cancel bounded Phase 5A rotation search without a second terminal.

**Architecture:** The RViz plugin becomes a `SearchForObject` action client but never a velocity publisher. A small pure C++ session state machine rejects duplicate/stale callbacks and emits a one-shot authorization decision only after `WAITING_FOR_AUTHORIZATION`; the panel translates those decisions into the existing Trigger services and queued Qt updates.

**Tech Stack:** ROS 2 Foxy, `rclcpp_action`, RViz2/Qt5, `track_robot_interfaces/action/SearchForObject`, `std_srvs/srv/Trigger`, C++17, GTest, pytest contract tests, colcon.

## Global Constraints

- Keep ROS domain policy at `ROS_DOMAIN_ID=20` for runtime validation.
- Preserve `/semantic_search/query`, all Phase 1–4B interfaces, target IDs, and existing default behavior.
- Do not publish `Twist`, `SearchMotionIntent`, Nav2 goals, or `/cmd_vel` from the RViz plugin.
- A Start Finding request permits bounded in-place rotation only; it never starts Phase 4B translation.
- RC override, E-stop, motion safety supervisor, velocity gate, and the existing Phase 5A rotation adapter remain authoritative.
- Send rotation authorization once, only after feedback reason equals `WAITING_FOR_AUTHORIZATION` for the current action generation.
- Use a 60-second goal timeout and `1.5708` radians maximum search angle.
- Keep Start Approach and Cancel & Disarm as separate existing controls.
- Add no network download and make no ROS interface-layout change.

---

## File structure

- Create `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/active_search_session.hpp`: ROS- and Qt-independent local action/UI state contract.
- Create `src/track_robot/track_robot_semantic_search_rviz_plugins/src/active_search_session.cpp`: deterministic state transitions and one-shot authorization decision.
- Create `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_active_search_session.cpp`: pure GTest coverage for duplicate, stale, stop, and authorization behavior.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/query_session.hpp`: declare adoption of an externally allocated query.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/src/query_session.cpp`: adopt manager query ID/version without publication.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_query_session.cpp`: prove external adoption and validation.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/semantic_search_panel.hpp`: action/service clients, current goal handle, callbacks, button, and label.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/src/semantic_search_panel.cpp`: feedback-driven Start/Stop implementation and UI rendering.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`: assert bounded interfaces and preserve the no-velocity boundary.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/CMakeLists.txt`: build the state helper/tests and link `rclcpp_action`.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/package.xml`: declare `rclcpp_action`.
- Modify `src/track_robot/track_robot_semantic_search_rviz_plugins/README.md`: document active-search behavior and safety boundary.
- Modify `docs/guides/semantic-search/phase5a-bounded-active-search-test.md`: add the RViz one-button test flow.

---

### Task 1: Add externally owned query correlation

**Files:**
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/query_session.hpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/query_session.cpp`
- Test: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_query_session.cpp`

**Interfaces:**
- Consumes: non-zero query ID/version allocated by the Phase 5A action manager.
- Produces: `QueryCommand QuerySession::adopt_query(const QString &, std::uint64_t, std::uint64_t)`; it updates correlation state and returns a canonical local record without publishing.

- [ ] **Step 1: Write the failing adoption tests**

Add these tests:

```cpp
TEST(QuerySession, AdoptsExternallyAllocatedQueryForCorrelation)
{
  plugin::QuerySession session;

  const auto adopted = session.adopt_query("  green   bottle  ", 901U, 1U);

  EXPECT_EQ(adopted.normalized_text, "green bottle");
  EXPECT_EQ(adopted.query_id, 901U);
  EXPECT_EQ(adopted.query_version, 1U);
  ASSERT_TRUE(session.correlate_diagnostic(
    R"({"state":"query_accepted","query_id":901,"query_version":1})")
    .has_value());
}

TEST(QuerySession, RejectsInvalidExternalQueryIdentity)
{
  plugin::QuerySession session;

  EXPECT_THROW((void)session.adopt_query("bottle", 0U, 1U),
    std::invalid_argument);
  EXPECT_THROW((void)session.adopt_query("bottle", 1U, 0U),
    std::invalid_argument);
  EXPECT_THROW((void)session.adopt_query("   ", 1U, 1U),
    std::invalid_argument);
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_semantic_search_rviz_plugins --cmake-args -DBUILD_TESTING=ON
```

Expected: compilation fails because `QuerySession::adopt_query` is not declared.

- [ ] **Step 3: Add the adoption API**

Declare:

```cpp
QueryCommand adopt_query(
  const QString & text,
  std::uint64_t query_id,
  std::uint64_t query_version);
```

Implement:

```cpp
QueryCommand QuerySession::adopt_query(
  const QString & text,
  std::uint64_t query_id,
  std::uint64_t query_version)
{
  if (query_id == 0U || query_version == 0U) {
    throw std::invalid_argument(
            "external query ID and version must be positive");
  }
  const auto normalized = normalize(text);
  current_ = QueryCommand{
    query_id,
    query_version,
    normalized,
    payload_for(normalized, query_id, query_version)};
  last_query_id_ = std::max(last_query_id_, query_id);
  return *current_;
}
```

- [ ] **Step 4: Build and run query-session tests**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_semantic_search_rviz_plugins --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select track_robot_semantic_search_rviz_plugins --ctest-args -R test_query_session --output-on-failure
colcon test-result --verbose
```

Expected: `test_query_session` passes with zero failures.

- [ ] **Step 5: Commit the independent correlation change**

```bash
git add src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/query_session.hpp src/track_robot/track_robot_semantic_search_rviz_plugins/src/query_session.cpp src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_query_session.cpp
git commit -m "feat(rviz): adopt active-search query identity"
```

---

### Task 2: Add the deterministic active-search UI state machine

**Files:**
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/active_search_session.hpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/active_search_session.cpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_active_search_session.cpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/CMakeLists.txt`

**Interfaces:**
- Consumes: start/stop clicks, action goal response, feedback query ID/reason, authorization result, terminal result, and a monotonically increasing local generation.
- Produces: `ActiveSearchSession::begin()`, `request_stop()`, `on_goal_response()`, `on_feedback()`, `on_authorization_result()`, `finish()`, `state()`, `generation()`, and `active()`.

- [ ] **Step 1: Write failing state-machine tests**

Create tests covering the exact contract:

```cpp
TEST(ActiveSearchSession, BeginsOnceAndRejectsDuplicateStart)
{
  plugin::ActiveSearchSession session;
  const auto generation = session.begin();
  ASSERT_TRUE(generation.has_value());
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::GOAL_PENDING);
  EXPECT_FALSE(session.begin().has_value());
}

TEST(ActiveSearchSession, AuthorizesOnlyOnceAfterWaitingFeedback)
{
  plugin::ActiveSearchSession session;
  const auto generation = *session.begin();
  ASSERT_TRUE(session.on_goal_response(generation, true));

  const auto observing = session.on_feedback(
    generation, 44U, "PASSIVE_OBSERVATION");
  EXPECT_TRUE(observing.adopt_query);
  EXPECT_FALSE(observing.authorize_rotation);

  const auto waiting = session.on_feedback(
    generation, 44U, "WAITING_FOR_AUTHORIZATION");
  EXPECT_TRUE(waiting.authorize_rotation);
  const auto repeated = session.on_feedback(
    generation, 44U, "WAITING_FOR_AUTHORIZATION");
  EXPECT_FALSE(repeated.authorize_rotation);
}

TEST(ActiveSearchSession, StopBeforeGoalResponseRemainsCancelled)
{
  plugin::ActiveSearchSession session;
  const auto generation = *session.begin();
  ASSERT_TRUE(session.request_stop());
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::CANCEL_PENDING);
  ASSERT_TRUE(session.on_goal_response(generation, true));
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::CANCEL_PENDING);
}

TEST(ActiveSearchSession, IgnoresStaleCallbacksAndResetsOnFinish)
{
  plugin::ActiveSearchSession session;
  const auto first = *session.begin();
  ASSERT_TRUE(session.finish(first));
  const auto second = *session.begin();
  EXPECT_FALSE(session.on_goal_response(first, true));
  EXPECT_FALSE(session.finish(first));
  EXPECT_TRUE(session.finish(second));
  EXPECT_EQ(session.state(), plugin::ActiveSearchState::IDLE);
}
```

- [ ] **Step 2: Register the new test and verify RED**

Add an `active_search_session` static library and
`ament_add_gtest(test_active_search_session ...)` target to `CMakeLists.txt`,
then run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_semantic_search_rviz_plugins --cmake-args -DBUILD_TESTING=ON
```

Expected: compilation fails because the new header and implementation do not
exist yet.

- [ ] **Step 3: Implement the state contract**

Use these public types:

```cpp
enum class ActiveSearchState
{
  IDLE,
  GOAL_PENDING,
  SEARCHING,
  AUTHORIZATION_PENDING,
  AUTHORIZED,
  CANCEL_PENDING,
};

struct ActiveSearchFeedbackDecision
{
  bool accepted{false};
  bool adopt_query{false};
  bool authorize_rotation{false};
  std::uint64_t query_id{0U};
};

class ActiveSearchSession
{
public:
  std::optional<std::uint64_t> begin();
  bool request_stop();
  bool on_goal_response(std::uint64_t generation, bool accepted);
  ActiveSearchFeedbackDecision on_feedback(
    std::uint64_t generation,
    std::uint64_t query_id,
    const std::string & reason);
  bool on_authorization_result(std::uint64_t generation, bool accepted);
  bool finish(std::uint64_t generation);
  [[nodiscard]] ActiveSearchState state() const;
  [[nodiscard]] std::uint64_t generation() const;
  [[nodiscard]] bool active() const;

private:
  ActiveSearchState state_{ActiveSearchState::IDLE};
  std::uint64_t generation_{0U};
  std::uint64_t adopted_query_id_{0U};
  bool authorization_requested_{false};
};
```

The implementation must increment `generation_` only from idle, reject zero
feedback query IDs, set `adopt_query` only for the first query ID, set
`authorize_rotation` only for the exact waiting reason and only once, retain
`CANCEL_PENDING` across a late accepted goal response, and accept callbacks
only when their generation equals `generation_`.

- [ ] **Step 4: Run state and query unit tests**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_semantic_search_rviz_plugins --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select track_robot_semantic_search_rviz_plugins --ctest-args -R "test_(active_search_session|query_session)" --output-on-failure
colcon test-result --verbose
```

Expected: both GTest executables pass with zero failures.

- [ ] **Step 5: Commit the independent state-machine change**

```bash
git add src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/active_search_session.hpp src/track_robot/track_robot_semantic_search_rviz_plugins/src/active_search_session.cpp src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_active_search_session.cpp src/track_robot/track_robot_semantic_search_rviz_plugins/CMakeLists.txt
git commit -m "feat(rviz): add bounded finding session state"
```

---

### Task 3: Wire Start Finding / Stop Finding into the RViz panel

**Files:**
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/semantic_search_panel.hpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/semantic_search_panel.cpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/CMakeLists.txt`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/package.xml`

**Interfaces:**
- Consumes: `/semantic_search/search_for_object`, `/semantic_search/active_search/authorize_rotation`, `/semantic_search/active_search/cancel`, action feedback/result, and the query line edit.
- Produces: one toggle button, one Active search label, manager-query correlation, and no velocity or approach authorization.

- [ ] **Step 1: Change the contract test first**

Require these strings and dependencies:

```python
for interface in (
        '/semantic_search/search_for_object',
        '/semantic_search/active_search/authorize_rotation',
        '/semantic_search/active_search/cancel'):
    assert interface in source

assert 'Start Finding' in source
assert 'Stop Finding' in source
assert 'WAITING_FOR_AUTHORIZATION' in source
assert 'SearchForObject' in source
assert 'rclcpp_action' in source

for forbidden in ('cmd_vel', 'SearchMotionIntent', 'geometry_msgs'):
    assert forbidden not in source
```

Also assert `rclcpp_action` appears in both `CMakeLists.txt` and `package.xml`,
and retain every existing Start Approach / Cancel & Disarm assertion.

- [ ] **Step 2: Run the contract test and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py
```

Expected: failure because action interfaces, button strings, and dependency do
not yet exist.

- [ ] **Step 3: Declare the action client and UI callbacks**

Add these core declarations to the panel header:

```cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "track_robot_interfaces/action/search_for_object.hpp"
#include "track_robot_semantic_search_rviz_plugins/active_search_session.hpp"

using SearchForObject = track_robot_interfaces::action::SearchForObject;
using SearchGoalHandle = rclcpp_action::ClientGoalHandle<SearchForObject>;

void toggle_finding();
void start_finding();
void stop_finding();
void authorize_rotation(std::uint64_t generation);
void render_finding_state(const QString & status);
void finish_finding(std::uint64_t generation, const QString & status);

rclcpp_action::Client<SearchForObject>::SharedPtr search_client_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
  authorize_rotation_client_;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_search_client_;
SearchGoalHandle::SharedPtr search_goal_handle_;
ActiveSearchSession finding_session_;
std::mutex finding_mutex_;
QPushButton * finding_button_{nullptr};
QLabel * finding_status_{nullptr};
std::string search_action_;
std::string authorize_rotation_service_;
std::string cancel_search_service_;
```

- [ ] **Step 4: Add the panel widgets and ROS clients**

Use these exact defaults:

```cpp
constexpr const char * kSearchAction =
  "/semantic_search/search_for_object";
constexpr const char * kAuthorizeRotationService =
  "/semantic_search/active_search/authorize_rotation";
constexpr const char * kCancelSearchService =
  "/semantic_search/active_search/cancel";
constexpr std::int32_t kSearchTimeoutSec = 60;
constexpr float kMaximumSearchAngleRad = 1.5708F;
```

Construct `finding_button_` with `tr("Start Finding")`, connect it to
`toggle_finding`, add it above the Phase 4B motion buttons, and add
`finding_status_` to the form under `Active search`. In `onInitialize`, create
the action client plus both Trigger clients. Save/load all three interface
names alongside the existing panel configuration.

- [ ] **Step 5: Implement feedback-driven goal submission**

Build the goal exactly as follows:

```cpp
SearchForObject::Goal goal;
goal.query_text = normalized_text;
goal.timeout.sec = kSearchTimeoutSec;
goal.timeout.nanosec = 0U;
goal.allow_rotation = true;
goal.maximum_rotation_angle = kMaximumSearchAngleRad;
goal.client_request_id =
  "rviz-phase5a-" + std::to_string(generation);
```

Configure `SendGoalOptions` so:

- goal response calls `finding_session_.on_goal_response(generation, accepted)`;
- if Stop was requested before acceptance, an accepted handle is immediately
  cancelled;
- feedback calls `on_feedback(generation, feedback->query_id,
  feedback->current_reason)`;
- `decision.adopt_query` invokes
  `session_.adopt_query(query_text, decision.query_id, 1U)` and updates the
  current-query label;
- `decision.authorize_rotation` invokes `authorize_rotation(generation)`;
- result maps `CONFIRMED`, `NOT_FOUND`, `UNCERTAIN`, `CANCELLED`, `TIMEOUT`,
  and other statuses to bounded readable text, including the selected global
  object ID only when `selected_object_valid` is true;
- every action callback captures and checks `generation`.

Do not wait synchronously for the action server. If
`search_client_->action_server_is_ready()` is false, show
`active-search action server is unavailable` and return to idle.

- [ ] **Step 6: Implement one-shot authorization and toggle cancellation**

`authorize_rotation(generation)` must call the Trigger client only when ready.
The response must feed `on_authorization_result`; a false response or exception
must invoke `stop_finding()` and display the service reason. It must never retry
authorization automatically.

`stop_finding()` must:

```cpp
finding_session_.request_stop();
finding_button_->setEnabled(false);
finding_button_->setText(tr("Stop Finding"));
finding_status_->setText(tr("cancelling search"));
```

Then asynchronously cancel the stored action handle when present and call the
explicit Trigger cancel service when ready. The first terminal action result
calls `finish_finding`, which clears the handle, returns the session to idle,
sets button text to `Start Finding`, and enables it. Late callbacks from an old
generation must not change the new search.

- [ ] **Step 7: Add dependencies and run the focused tests**

Add `find_package(rclcpp_action REQUIRED)`, include it in
`ament_target_dependencies` and `ament_export_dependencies`, link the panel to
`active_search_session`, and add `<depend>rclcpp_action</depend>`.

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_interfaces track_robot_semantic_search_rviz_plugins --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select track_robot_semantic_search_rviz_plugins --event-handlers console_direct+
colcon test-result --verbose
```

Expected: contract, state, and query tests pass; package builds on Foxy.

- [ ] **Step 8: Commit the panel integration**

```bash
git add src/track_robot/track_robot_semantic_search_rviz_plugins
git commit -m "feat(rviz): control bounded active target finding"
```

---

### Task 4: Document and regression-test the operator flow

**Files:**
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/README.md`
- Modify: `docs/guides/semantic-search/phase5a-bounded-active-search-test.md`
- Add: `docs/architecture/semantic-search/2026-08-05-phase5a-rviz-active-search-control-design.md`
- Add: `docs/superpowers/plans/2026-08-05-phase5a-rviz-active-search-control.md`

**Interfaces:**
- Consumes: installed `semantic_search_ctl run phase5a --rotation-supervised` and the RViz Phase 5A panel.
- Produces: repeatable blind-target success, cancellation, and no-target validation instructions.

- [ ] **Step 1: Update package documentation**

Replace the obsolete claim that the panel owns no action client with an exact
statement that it owns only `SearchForObject` plus the two bounded Phase 5A
Trigger clients, and explicitly state that it publishes no velocity and does
not start approach automatically.

- [ ] **Step 2: Add the one-button test sequence**

Document this operator flow:

```bash
cd ~/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export TRACK_ROBOT_WS=~/track_robot_ws
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 run track_robot_bringup semantic_search_ctl run phase5a --rotation-supervised
```

In RViz, enter `green bottle`, place the bottle initially outside the camera
view but within a bounded rotation, click **Start Finding**, verify the button
changes to **Stop Finding**, verify only rotation occurs, and verify the result
shows a confirmed global object ID or a bounded terminal failure. Add a second
run that clicks **Stop Finding** while rotating and verifies zero commanded
rotation after cancellation. Add a no-target run that terminates within 60
seconds without translation.

- [ ] **Step 3: Run package and Phase 5A regression tests**

Run:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon test --packages-select track_robot_interfaces track_robot_semantic_search track_robot_navigation track_robot_semantic_search_rviz_plugins track_robot_bringup --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests. Hardware motion is not claimed by this command;
the documented supervised runtime test remains a separate operator-observed
acceptance test.

- [ ] **Step 4: Review the safety boundary mechanically**

Run:

```bash
rg -n "cmd_vel|SearchMotionIntent|geometry_msgs" src/track_robot/track_robot_semantic_search_rviz_plugins
rg -n "search_for_object|authorize_rotation|active_search/cancel" src/track_robot/track_robot_semantic_search_rviz_plugins
```

Expected: the first command finds no production API use; the second finds only
the intended action/service client and documentation references.

- [ ] **Step 5: Commit documentation and test instructions**

```bash
git add docs/architecture/semantic-search/2026-08-05-phase5a-rviz-active-search-control-design.md docs/superpowers/plans/2026-08-05-phase5a-rviz-active-search-control.md docs/guides/semantic-search/phase5a-bounded-active-search-test.md src/track_robot/track_robot_semantic_search_rviz_plugins/README.md
git commit -m "docs(phase5a): add RViz active-search operator flow"
```

---

## Final regression gate

- [ ] Build all affected packages from the current worktree.
- [ ] Run all affected unit and contract tests with zero failures.
- [ ] Confirm Start Finding sends exactly one action goal.
- [ ] Confirm authorization occurs only after `WAITING_FOR_AUTHORIZATION`.
- [ ] Confirm Stop Finding cancels both the action and pending rotation intent.
- [ ] Confirm no panel source publishes velocity or a Phase 4B motion goal.
- [ ] Confirm existing New Query, Revise, Start Approach, and Cancel & Disarm tests remain green.
- [ ] Record any unexecuted hardware acceptance item explicitly rather than claiming it passed.

## Rollback

Each implementation concern is isolated by commit. Revert the panel-integration
commit to remove operator motion authorization while retaining the harmless
query-correlation and state-helper code; revert all three feature commits to
restore the exact passive/current panel behavior. No ROS message migration,
stored-data conversion, or configuration cleanup is required.
