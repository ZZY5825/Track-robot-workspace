#include <gtest/gtest.h>

#include "track_robot_decision/human_following_session_policy.hpp"

namespace track_robot_decision
{
namespace
{

SessionInputs validInputs()
{
  SessionInputs inputs;
  inputs.now_sec = 1.0;
  inputs.start_gesture_event = true;
  inputs.gesture_visual_track_id = 7;
  inputs.camera_confirmed = true;
  inputs.camera_visual_track_id = 7;
  inputs.camera_logical_target_id = 42;
  inputs.decision_confirmed_camera_lidar = true;
  inputs.decision_logical_target_id = 42;
  inputs.decision_confidence = 0.9F;
  inputs.health_healthy = true;
  inputs.planner_ready = true;
  inputs.safety_disarmed_ready = true;
  inputs.bunker_can_healthy = true;
  inputs.required_inputs_fresh = true;
  inputs.arm_service_ready = true;
  return inputs;
}

TEST(HumanFollowingSessionPolicy, ShadowNeverRequestsArm)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Shadow, false, 10.0, 1.0);

  const auto first = policy.update(validInputs());
  const auto second = policy.update(validInputs());

  EXPECT_EQ(first.state, SessionState::WaitingForGesture);
  EXPECT_FALSE(first.target_authorized);
  EXPECT_FALSE(first.request_arm);
  EXPECT_FALSE(second.request_arm);
}

TEST(HumanFollowingSessionPolicy, ConfirmedWaveRequestsArmOnce)
{
  HumanFollowingSessionPolicy stale_policy(RuntimeMode::Active, true, 10.0, 1.0);
  auto stale_inputs = validInputs();
  stale_inputs.required_inputs_fresh = false;
  const auto stale = stale_policy.update(stale_inputs);

  EXPECT_EQ(stale.state, SessionState::ValidatingTarget);
  EXPECT_FALSE(stale.request_arm);

  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);

  const auto first = policy.update(validInputs());
  const auto second = policy.update(validInputs());

  EXPECT_EQ(first.state, SessionState::Arming);
  EXPECT_EQ(first.logical_target_id, 42);
  EXPECT_TRUE(first.target_authorized);
  EXPECT_TRUE(first.arm_request_pending);
  EXPECT_TRUE(first.request_arm);
  EXPECT_EQ(second.state, SessionState::Arming);
  EXPECT_TRUE(second.arm_request_pending);
  EXPECT_FALSE(second.request_arm);
}

TEST(HumanFollowingSessionPolicy, LidarOnlyCannotInitiallyArm)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  auto inputs = validInputs();
  inputs.decision_confirmed_camera_lidar = false;
  inputs.decision_lidar_limited = true;

  const auto decision = policy.update(inputs);

  EXPECT_EQ(decision.state, SessionState::ValidatingTarget);
  EXPECT_FALSE(decision.target_authorized);
  EXPECT_FALSE(decision.arm_request_pending);
  EXPECT_FALSE(decision.request_arm);
}

TEST(HumanFollowingSessionPolicy, StopGestureDisarmsAndResets)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  const auto armed = policy.acceptArmResult(true, "armed");
  ASSERT_EQ(armed.state, SessionState::Following);
  ASSERT_TRUE(armed.target_authorized);

  auto inputs = validInputs();
  inputs.now_sec = 2.0;
  inputs.start_gesture_event = false;
  inputs.stop_gesture_event = true;
  const auto stopped = policy.update(inputs);
  const auto repeated = policy.update(inputs);

  EXPECT_EQ(stopped.state, SessionState::Disarmed);
  EXPECT_FALSE(stopped.target_authorized);
  EXPECT_FALSE(stopped.arm_request_pending);
  EXPECT_TRUE(stopped.request_disarm);
  EXPECT_TRUE(stopped.request_target_reset);
  EXPECT_EQ(repeated.state, SessionState::Disarmed);
  EXPECT_FALSE(repeated.request_disarm);
  EXPECT_FALSE(repeated.request_target_reset);
}

TEST(HumanFollowingSessionPolicy, RcTakeoverRevokesAndCanReturnDoesNotResume)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  ASSERT_EQ(policy.acceptArmResult(true, "armed").state, SessionState::Following);

  auto inputs = validInputs();
  inputs.now_sec = 2.0;
  inputs.start_gesture_event = false;
  inputs.rc_override = true;
  const auto rc = policy.update(inputs);
  const auto repeated_rc = policy.update(inputs);

  EXPECT_EQ(rc.state, SessionState::RcOverride);
  EXPECT_FALSE(rc.target_authorized);
  EXPECT_TRUE(rc.request_disarm);
  EXPECT_TRUE(rc.request_target_reset);
  EXPECT_FALSE(repeated_rc.request_disarm);
  EXPECT_FALSE(repeated_rc.request_target_reset);

  inputs.now_sec = 3.0;
  inputs.rc_override = false;
  const auto can = policy.update(inputs);
  EXPECT_EQ(can.state, SessionState::WaitingForGesture);
  EXPECT_FALSE(can.target_authorized);
  EXPECT_FALSE(can.request_arm);

  inputs.now_sec = 4.0;
  inputs.start_gesture_event = true;
  const auto new_gesture = policy.update(inputs);
  EXPECT_EQ(new_gesture.state, SessionState::Arming);
  EXPECT_TRUE(new_gesture.request_arm);
}

TEST(HumanFollowingSessionPolicy, TargetMismatchDisarms)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  ASSERT_EQ(policy.acceptArmResult(true, "armed").state, SessionState::Following);

  auto inputs = validInputs();
  inputs.now_sec = 2.0;
  inputs.start_gesture_event = false;
  inputs.decision_logical_target_id = 99;
  const auto mismatch = policy.update(inputs);
  const auto repeated = policy.update(inputs);

  EXPECT_EQ(mismatch.state, SessionState::Fault);
  EXPECT_EQ(mismatch.reason, "logical_target_mismatch");
  EXPECT_FALSE(mismatch.target_authorized);
  EXPECT_TRUE(mismatch.request_disarm);
  EXPECT_TRUE(mismatch.request_target_reset);
  EXPECT_FALSE(repeated.request_disarm);
  EXPECT_FALSE(repeated.request_target_reset);

  inputs.now_sec = 3.0;
  inputs.decision_logical_target_id = 42;
  const auto cleared = policy.update(inputs);
  EXPECT_EQ(cleared.state, SessionState::WaitingForGesture);
  EXPECT_FALSE(cleared.request_arm);

  HumanFollowingSessionPolicy hard_fault_policy(
    RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(hard_fault_policy.update(validInputs()).request_arm);
  ASSERT_EQ(
    hard_fault_policy.acceptArmResult(true, "armed").state,
    SessionState::Following);
  inputs.health_hard_fault = true;
  const auto hard_fault = hard_fault_policy.update(inputs);
  EXPECT_EQ(hard_fault.state, SessionState::Fault);
  EXPECT_EQ(hard_fault.reason, "hard_fault");
  EXPECT_TRUE(hard_fault.request_disarm);
  EXPECT_TRUE(hard_fault.request_target_reset);

  HumanFollowingSessionPolicy stale_policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(stale_policy.update(validInputs()).request_arm);
  ASSERT_EQ(stale_policy.acceptArmResult(true, "armed").state, SessionState::Following);
  inputs.health_hard_fault = false;
  inputs.required_inputs_fresh = false;
  const auto stale = stale_policy.update(inputs);
  EXPECT_EQ(stale.state, SessionState::Fault);
  EXPECT_EQ(stale.reason, "required_inputs_stale");
  EXPECT_TRUE(stale.request_disarm);
  EXPECT_TRUE(stale.request_target_reset);

  HumanFollowingSessionPolicy can_policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(can_policy.update(validInputs()).request_arm);
  ASSERT_EQ(can_policy.acceptArmResult(true, "armed").state, SessionState::Following);
  inputs.required_inputs_fresh = true;
  inputs.bunker_can_healthy = false;
  const auto can_fault = can_policy.update(inputs);
  EXPECT_EQ(can_fault.state, SessionState::Fault);
  EXPECT_EQ(can_fault.reason, "bunker_can_unhealthy");
  EXPECT_TRUE(can_fault.request_disarm);
  EXPECT_TRUE(can_fault.request_target_reset);
}

TEST(HumanFollowingSessionPolicy, ShortBlockRetainsAuthorization)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  ASSERT_EQ(policy.acceptArmResult(true, "armed").state, SessionState::Following);

  auto inputs = validInputs();
  inputs.start_gesture_event = false;
  inputs.planner_blocked = true;
  inputs.now_sec = 2.0;
  const auto blocked = policy.update(inputs);
  inputs.now_sec = 5.0;
  const auto still_blocked = policy.update(inputs);

  EXPECT_EQ(blocked.state, SessionState::Blocked);
  EXPECT_TRUE(blocked.target_authorized);
  EXPECT_FALSE(blocked.request_disarm);
  EXPECT_EQ(still_blocked.state, SessionState::Blocked);
  EXPECT_TRUE(still_blocked.target_authorized);

  inputs.planner_blocked = false;
  inputs.now_sec = 6.0;
  const auto clear = policy.update(inputs);
  EXPECT_EQ(clear.state, SessionState::Following);
  EXPECT_TRUE(clear.target_authorized);
  EXPECT_FALSE(clear.request_arm);
}

TEST(HumanFollowingSessionPolicy, BlockTimeoutRevokesAuthorization)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 3.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  ASSERT_EQ(policy.acceptArmResult(true, "armed").state, SessionState::Following);

  auto inputs = validInputs();
  inputs.start_gesture_event = false;
  inputs.planner_blocked = true;
  inputs.now_sec = 2.0;
  ASSERT_EQ(policy.update(inputs).state, SessionState::Blocked);
  inputs.now_sec = 4.99;
  const auto before_timeout = policy.update(inputs);
  inputs.now_sec = 5.0;
  const auto timeout = policy.update(inputs);
  inputs.now_sec = 5.1;
  const auto repeated = policy.update(inputs);

  EXPECT_EQ(before_timeout.state, SessionState::Blocked);
  EXPECT_TRUE(before_timeout.target_authorized);
  EXPECT_EQ(timeout.state, SessionState::Fault);
  EXPECT_EQ(timeout.reason, "block_timeout");
  EXPECT_FALSE(timeout.target_authorized);
  EXPECT_TRUE(timeout.request_disarm);
  EXPECT_TRUE(timeout.request_target_reset);
  EXPECT_FALSE(repeated.request_disarm);
  EXPECT_FALSE(repeated.request_target_reset);

  inputs.planner_blocked = false;
  inputs.now_sec = 6.0;
  EXPECT_EQ(policy.update(inputs).state, SessionState::WaitingForGesture);
  inputs.start_gesture_event = true;
  inputs.now_sec = 7.0;
  EXPECT_TRUE(policy.update(inputs).request_arm);

  HumanFollowingSessionPolicy overlap_policy(RuntimeMode::Active, true, 3.0, 10.0);
  ASSERT_TRUE(overlap_policy.update(validInputs()).request_arm);
  ASSERT_EQ(
    overlap_policy.acceptArmResult(true, "armed").state,
    SessionState::Following);
  inputs.start_gesture_event = false;
  inputs.planner_blocked = true;
  inputs.now_sec = 2.0;
  ASSERT_EQ(overlap_policy.update(inputs).state, SessionState::Blocked);
  inputs.decision_uncertain = true;
  inputs.now_sec = 3.0;
  ASSERT_EQ(overlap_policy.update(inputs).reason, "uncertain_hold");
  inputs.decision_uncertain = false;
  inputs.now_sec = 3.5;
  ASSERT_EQ(overlap_policy.update(inputs).state, SessionState::Blocked);
  inputs.now_sec = 5.0;
  const auto overlap_timeout = overlap_policy.update(inputs);
  EXPECT_EQ(overlap_timeout.state, SessionState::Fault);
  EXPECT_EQ(overlap_timeout.reason, "block_timeout");
  EXPECT_TRUE(overlap_timeout.request_disarm);
}

TEST(HumanFollowingSessionPolicy, UncertaintyTimeoutRevokesAuthorization)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  ASSERT_EQ(policy.acceptArmResult(true, "armed").state, SessionState::Following);

  auto inputs = validInputs();
  inputs.start_gesture_event = false;
  inputs.decision_uncertain = true;
  inputs.now_sec = 2.0;
  const auto uncertain = policy.update(inputs);
  inputs.now_sec = 2.99;
  const auto before_timeout = policy.update(inputs);
  inputs.now_sec = 3.0;
  const auto timeout = policy.update(inputs);
  const auto repeated = policy.update(inputs);

  EXPECT_EQ(uncertain.state, SessionState::Following);
  EXPECT_EQ(uncertain.reason, "uncertain_hold");
  EXPECT_TRUE(uncertain.target_authorized);
  EXPECT_FALSE(uncertain.request_disarm);
  EXPECT_TRUE(before_timeout.target_authorized);
  EXPECT_EQ(timeout.state, SessionState::Fault);
  EXPECT_EQ(timeout.reason, "uncertainty_timeout");
  EXPECT_FALSE(timeout.target_authorized);
  EXPECT_TRUE(timeout.request_disarm);
  EXPECT_TRUE(timeout.request_target_reset);
  EXPECT_FALSE(repeated.request_disarm);
  EXPECT_FALSE(repeated.request_target_reset);

  inputs.decision_uncertain = false;
  inputs.now_sec = 4.0;
  EXPECT_EQ(policy.update(inputs).state, SessionState::WaitingForGesture);
  inputs.start_gesture_event = true;
  inputs.now_sec = 5.0;
  EXPECT_TRUE(policy.update(inputs).request_arm);
}

TEST(HumanFollowingSessionPolicy, SearchRotateRevokesAuthorization)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  ASSERT_EQ(policy.acceptArmResult(true, "armed").state, SessionState::Following);

  auto inputs = validInputs();
  inputs.start_gesture_event = false;
  inputs.decision_search_rotate = true;
  inputs.now_sec = 2.0;
  const auto search = policy.update(inputs);
  const auto repeated = policy.update(inputs);

  EXPECT_EQ(search.state, SessionState::Fault);
  EXPECT_EQ(search.reason, "search_rotate");
  EXPECT_FALSE(search.target_authorized);
  EXPECT_TRUE(search.request_disarm);
  EXPECT_TRUE(search.request_target_reset);
  EXPECT_FALSE(repeated.request_disarm);
  EXPECT_FALSE(repeated.request_target_reset);

  inputs.decision_search_rotate = false;
  inputs.now_sec = 3.0;
  EXPECT_EQ(policy.update(inputs).state, SessionState::WaitingForGesture);
  inputs.start_gesture_event = true;
  inputs.now_sec = 4.0;
  EXPECT_TRUE(policy.update(inputs).request_arm);

  HumanFollowingSessionPolicy lost_policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(lost_policy.update(validInputs()).request_arm);
  ASSERT_EQ(lost_policy.acceptArmResult(true, "armed").state, SessionState::Following);
  inputs.start_gesture_event = false;
  inputs.decision_target_lost = true;
  const auto lost = lost_policy.update(inputs);
  EXPECT_EQ(lost.state, SessionState::Fault);
  EXPECT_EQ(lost.reason, "target_lost");
  EXPECT_TRUE(lost.request_disarm);
  EXPECT_TRUE(lost.request_target_reset);
}

TEST(HumanFollowingSessionPolicy, ArmRejectionRequiresANewGesture)
{
  HumanFollowingSessionPolicy policy(RuntimeMode::Active, true, 10.0, 1.0);
  ASSERT_TRUE(policy.update(validInputs()).request_arm);
  auto inputs = validInputs();
  inputs.start_gesture_event = false;
  inputs.now_sec = 1.5;
  ASSERT_EQ(policy.update(inputs).state, SessionState::Arming);

  const auto rejected = policy.acceptArmResult(false, "arm denied");
  const auto repeated_result = policy.acceptArmResult(false, "arm denied");
  EXPECT_EQ(rejected.state, SessionState::Fault);
  EXPECT_EQ(rejected.reason, "arm denied");
  EXPECT_FALSE(rejected.target_authorized);
  EXPECT_FALSE(rejected.arm_request_pending);
  EXPECT_TRUE(rejected.request_disarm);
  EXPECT_TRUE(rejected.request_target_reset);
  EXPECT_FALSE(repeated_result.request_disarm);
  EXPECT_FALSE(repeated_result.request_target_reset);

  inputs.start_gesture_event = true;
  inputs.now_sec = 2.0;
  const auto new_gesture = policy.update(inputs);
  EXPECT_EQ(new_gesture.state, SessionState::Arming);
  EXPECT_TRUE(new_gesture.request_arm);
}

}  // namespace
}  // namespace track_robot_decision
