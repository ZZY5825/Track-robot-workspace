#ifndef TRACK_ROBOT_DECISION__HUMAN_FOLLOWING_SESSION_POLICY_HPP_
#define TRACK_ROBOT_DECISION__HUMAN_FOLLOWING_SESSION_POLICY_HPP_

#include <cstdint>
#include <string>

namespace track_robot_decision
{

enum class RuntimeMode : uint8_t {Shadow = 0, Active = 1};
enum class SessionState : uint8_t {
  Starting = 0, WaitingForGesture = 1, ValidatingTarget = 2,
  Arming = 3, Following = 4, Blocked = 5, RcOverride = 6,
  Fault = 7, Disarmed = 8
};

struct SessionInputs
{
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

struct SessionDecision
{
  SessionState state{SessionState::Starting};
  int32_t logical_target_id{-1};
  bool target_authorized{false};
  bool arm_request_pending{false};
  bool request_arm{false};
  bool request_disarm{false};
  bool request_target_reset{false};
  uint64_t arm_request_generation{0};
  std::string reason;
};

class HumanFollowingSessionPolicy
{
public:
  HumanFollowingSessionPolicy(
    RuntimeMode mode, bool motion_confirmed,
    double blocked_timeout_sec, double uncertain_timeout_sec);
  SessionDecision update(const SessionInputs & inputs);
  SessionDecision acceptArmResult(
    uint64_t generation, bool success, const std::string & message);

private:
  SessionDecision decision(
    const std::string & reason, bool request_arm = false,
    bool request_disarm = false, bool request_target_reset = false) const;
  SessionDecision revoke(
    SessionState state, const std::string & reason, bool request_target_reset);

  RuntimeMode mode_;
  bool motion_confirmed_;
  double blocked_timeout_sec_;
  double uncertain_timeout_sec_;
  SessionState state_{SessionState::Starting};
  int32_t pending_visual_track_id_{-1};
  int32_t authorized_target_id_{-1};
  bool authorized_{false};
  bool arm_request_pending_{false};
  bool start_event_active_{false};
  bool blocked_active_{false};
  double blocked_since_sec_{0.0};
  bool uncertain_active_{false};
  double uncertain_since_sec_{0.0};
  uint64_t next_arm_request_generation_{0};
  uint64_t pending_arm_request_generation_{0};
};

}  // namespace track_robot_decision

#endif  // TRACK_ROBOT_DECISION__HUMAN_FOLLOWING_SESSION_POLICY_HPP_
