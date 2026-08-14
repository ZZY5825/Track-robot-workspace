#include "track_robot_decision/human_following_session_policy.hpp"

namespace track_robot_decision
{

HumanFollowingSessionPolicy::HumanFollowingSessionPolicy(
  RuntimeMode mode, bool motion_confirmed, double blocked_timeout_sec,
  double uncertain_timeout_sec)
: mode_(mode), motion_confirmed_(motion_confirmed),
  blocked_timeout_sec_(blocked_timeout_sec),
  uncertain_timeout_sec_(uncertain_timeout_sec)
{
}

SessionDecision HumanFollowingSessionPolicy::update(const SessionInputs & inputs)
{
  const bool new_start_event = inputs.start_gesture_event && !start_event_active_;
  start_event_active_ = inputs.start_gesture_event;

  if (inputs.rc_override) {
    return revoke(SessionState::RcOverride, "rc_override", true);
  }

  if (inputs.health_hard_fault || inputs.safety_hard_fault) {
    return revoke(SessionState::Fault, "hard_fault", true);
  }

  if (authorized_ && !inputs.required_inputs_fresh) {
    return revoke(SessionState::Fault, "required_inputs_stale", true);
  }

  if (authorized_ && !inputs.bunker_can_healthy) {
    return revoke(SessionState::Fault, "bunker_can_unhealthy", true);
  }

  if (authorized_ && inputs.decision_logical_target_id != authorized_target_id_) {
    return revoke(SessionState::Fault, "logical_target_mismatch", true);
  }

  if (inputs.stop_gesture_event) {
    return revoke(SessionState::Disarmed, "stop_gesture", true);
  }

  if (authorized_ && inputs.decision_target_lost) {
    return revoke(SessionState::Fault, "target_lost", true);
  }

  if (authorized_ && inputs.decision_search_rotate) {
    return revoke(SessionState::Fault, "search_rotate", true);
  }

  if (state_ == SessionState::RcOverride) {
    state_ = SessionState::WaitingForGesture;
    return decision("waiting_for_gesture");
  }

  if (
    (state_ == SessionState::Fault || state_ == SessionState::Disarmed) &&
    !new_start_event)
  {
    state_ = SessionState::WaitingForGesture;
    return decision("waiting_for_gesture");
  }

  if (mode_ == RuntimeMode::Shadow || !motion_confirmed_) {
    state_ = SessionState::WaitingForGesture;
    return decision(mode_ == RuntimeMode::Shadow ? "shadow_mode" : "motion_not_confirmed");
  }

  if (state_ == SessionState::Arming) {
    return decision("arm_request_pending");
  }

  if (authorized_) {
    if (inputs.planner_blocked) {
      if (!blocked_active_) {
        blocked_active_ = true;
        blocked_since_sec_ = inputs.now_sec;
      }
      if (inputs.now_sec - blocked_since_sec_ >= blocked_timeout_sec_) {
        return revoke(SessionState::Fault, "block_timeout", true);
      }
    } else {
      blocked_active_ = false;
    }

    if (inputs.decision_uncertain) {
      if (!uncertain_active_) {
        uncertain_active_ = true;
        uncertain_since_sec_ = inputs.now_sec;
      }
      if (inputs.now_sec - uncertain_since_sec_ >= uncertain_timeout_sec_) {
        return revoke(SessionState::Fault, "uncertainty_timeout", true);
      }
      state_ = SessionState::Following;
      return decision("uncertain_hold");
    }
    uncertain_active_ = false;

    if (inputs.planner_blocked) {
      state_ = SessionState::Blocked;
      return decision("blocked");
    }

    state_ = SessionState::Following;
    return decision("following");
  }

  if (new_start_event && inputs.gesture_visual_track_id >= 0) {
    pending_visual_track_id_ = inputs.gesture_visual_track_id;
    state_ = SessionState::ValidatingTarget;
  }

  const bool initial_target_valid =
    inputs.camera_confirmed &&
    inputs.camera_visual_track_id == pending_visual_track_id_ &&
    inputs.decision_confirmed_camera_lidar &&
    inputs.decision_logical_target_id == inputs.camera_logical_target_id;

  const bool initial_system_valid =
    inputs.health_healthy && inputs.planner_ready &&
    inputs.safety_disarmed_ready && inputs.bunker_can_healthy &&
    inputs.required_inputs_fresh && inputs.arm_service_ready;

  if (
    state_ == SessionState::ValidatingTarget &&
    inputs.camera_logical_target_id >= 0 &&
    initial_target_valid && initial_system_valid)
  {
    authorized_target_id_ = inputs.decision_logical_target_id;
    authorized_ = true;
    arm_request_pending_ = true;
    pending_arm_request_generation_ = ++next_arm_request_generation_;
    state_ = SessionState::Arming;
    return decision("arm_requested", true);
  }

  if (state_ == SessionState::ValidatingTarget) {
    return decision("validating_target");
  }

  state_ = SessionState::WaitingForGesture;
  return decision("waiting_for_gesture");
}

SessionDecision HumanFollowingSessionPolicy::decision(
  const std::string & reason, bool request_arm,
  bool request_disarm, bool request_target_reset) const
{
  SessionDecision decision;
  decision.state = state_;
  decision.logical_target_id = authorized_target_id_;
  decision.target_authorized = authorized_;
  decision.arm_request_pending = arm_request_pending_;
  decision.request_arm = request_arm;
  decision.request_disarm = request_disarm;
  decision.request_target_reset = request_target_reset;
  decision.arm_request_generation = pending_arm_request_generation_;
  decision.reason = reason;
  return decision;
}

SessionDecision HumanFollowingSessionPolicy::revoke(
  SessionState state, const std::string & reason, bool request_target_reset)
{
  const bool entering_state = state_ != state;
  state_ = state;
  pending_visual_track_id_ = -1;
  authorized_target_id_ = -1;
  authorized_ = false;
  arm_request_pending_ = false;
  pending_arm_request_generation_ = 0;
  blocked_active_ = false;
  uncertain_active_ = false;
  return decision(reason, false, entering_state, entering_state && request_target_reset);
}

SessionDecision HumanFollowingSessionPolicy::acceptArmResult(
  uint64_t generation, bool success, const std::string & message)
{
  if (
    !arm_request_pending_ || generation == 0 ||
    generation != pending_arm_request_generation_)
  {
    if (!success) {
      return decision("stale_arm_result");
    }

    // A stale success may have armed the safety supervisor after this session
    // was revoked or superseded. Invalidate any newer request and explicitly
    // disarm instead of allowing the callback to authorize the wrong target.
    state_ = SessionState::Fault;
    pending_visual_track_id_ = -1;
    authorized_target_id_ = -1;
    authorized_ = false;
    arm_request_pending_ = false;
    pending_arm_request_generation_ = 0;
    blocked_active_ = false;
    uncertain_active_ = false;
    return decision("stale_arm_success", false, true, true);
  }

  arm_request_pending_ = false;
  pending_arm_request_generation_ = 0;
  if (success) {
    pending_visual_track_id_ = -1;
    state_ = SessionState::Following;
    return decision(message.empty() ? "arm_accepted" : message);
  }

  return revoke(
    SessionState::Fault,
    message.empty() ? "arm_rejected" : message,
    true);
}

}  // namespace track_robot_decision
