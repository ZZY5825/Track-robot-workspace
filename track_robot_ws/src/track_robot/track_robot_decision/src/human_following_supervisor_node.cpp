#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "bunker_msgs/msg/bunker_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "track_robot_decision/human_following_session_policy.hpp"
#include "track_robot_interfaces/msg/avoidance_state.hpp"
#include "track_robot_interfaces/msg/camera_target.hpp"
#include "track_robot_interfaces/msg/follow_decision.hpp"
#include "track_robot_interfaces/msg/gesture_state.hpp"
#include "track_robot_interfaces/msg/human_following_session.hpp"
#include "track_robot_interfaces/msg/perception_health.hpp"
#include "track_robot_interfaces/msg/safety_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace track_robot_decision
{

namespace
{

constexpr uint8_t BUNKER_CONTROL_MODE_CAN = 1U;
constexpr uint8_t BUNKER_CONTROL_MODE_RC = 3U;

double steadyNowSec()
{
  using SteadyClock = std::chrono::steady_clock;
  return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
}

std::string escapeJson(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character; break;
    }
  }
  return escaped;
}

std::string sessionStateName(const SessionState state)
{
  switch (state) {
    case SessionState::Starting: return "STARTING";
    case SessionState::WaitingForGesture: return "WAITING_FOR_GESTURE";
    case SessionState::ValidatingTarget: return "VALIDATING_TARGET";
    case SessionState::Arming: return "ARMING";
    case SessionState::Following: return "FOLLOWING";
    case SessionState::Blocked: return "BLOCKED";
    case SessionState::RcOverride: return "RC_OVERRIDE";
    case SessionState::Fault: return "FAULT";
    case SessionState::Disarmed: return "DISARMED";
  }
  return "UNKNOWN";
}

}  // namespace

class HumanFollowingSupervisorNode : public rclcpp::Node
{
public:
  HumanFollowingSupervisorNode()
  : Node("human_following_supervisor_node"),
    runtime_mode_(parseRuntimeMode(declare_parameter<std::string>("runtime_mode", "shadow"))),
    motion_confirmed_(declare_parameter<bool>("motion_confirmed", false)),
    tick_rate_(declare_parameter<double>("tick_rate", 20.0)),
    gesture_timeout_sec_(declare_parameter<double>("gesture_timeout_sec", 0.50)),
    camera_timeout_sec_(declare_parameter<double>("camera_timeout_sec", 0.35)),
    decision_timeout_sec_(declare_parameter<double>("decision_timeout_sec", 0.30)),
    health_timeout_sec_(declare_parameter<double>("health_timeout_sec", 0.30)),
    avoidance_timeout_sec_(declare_parameter<double>("avoidance_timeout_sec", 0.30)),
    safety_timeout_sec_(declare_parameter<double>("safety_timeout_sec", 0.20)),
    bunker_timeout_sec_(declare_parameter<double>("bunker_timeout_sec", 0.20)),
    policy_(
      runtime_mode_, motion_confirmed_,
      declare_parameter<double>("blocked_disarm_timeout_sec", 10.0),
      declare_parameter<double>("uncertain_authorization_timeout_sec", 1.0))
  {
    if (tick_rate_ <= 0.0) {
      throw std::invalid_argument("tick_rate must be positive");
    }

    gesture_sub_ = create_subscription<track_robot_interfaces::msg::GestureState>(
      "/human_tracking/gesture_state", 10,
      std::bind(&HumanFollowingSupervisorNode::onGesture, this, std::placeholders::_1));
    camera_sub_ = create_subscription<track_robot_interfaces::msg::CameraTarget>(
      "/human_tracking/camera_target", 10,
      [this](track_robot_interfaces::msg::CameraTarget::SharedPtr msg) {
        camera_ = std::move(msg);
        camera_received_sec_ = steadyNowSec();
      });
    decision_sub_ = create_subscription<track_robot_interfaces::msg::FollowDecision>(
      "/follow/decision", 10,
      [this](track_robot_interfaces::msg::FollowDecision::SharedPtr msg) {
        follow_decision_ = std::move(msg);
        decision_received_sec_ = steadyNowSec();
      });
    health_sub_ = create_subscription<track_robot_interfaces::msg::PerceptionHealth>(
      "/perception/health", 10,
      [this](track_robot_interfaces::msg::PerceptionHealth::SharedPtr msg) {
        health_ = std::move(msg);
        health_received_sec_ = steadyNowSec();
      });
    avoidance_sub_ = create_subscription<track_robot_interfaces::msg::AvoidanceState>(
      "/follow/avoidance_state", 10,
      [this](track_robot_interfaces::msg::AvoidanceState::SharedPtr msg) {
        avoidance_ = std::move(msg);
        avoidance_received_sec_ = steadyNowSec();
      });
    safety_sub_ = create_subscription<track_robot_interfaces::msg::SafetyState>(
      "/safety/state", 10,
      [this](track_robot_interfaces::msg::SafetyState::SharedPtr msg) {
        safety_ = std::move(msg);
        safety_received_sec_ = steadyNowSec();
      });
    bunker_sub_ = create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", rclcpp::SensorDataQoS(),
      [this](bunker_msgs::msg::BunkerStatus::SharedPtr msg) {
        bunker_ = std::move(msg);
        bunker_received_sec_ = steadyNowSec();
      });

    arm_client_ = create_client<std_srvs::srv::Trigger>("/safety/arm");
    disarm_client_ = create_client<std_srvs::srv::Trigger>("/safety/disarm");
    reset_client_ = create_client<std_srvs::srv::Trigger>("/human_tracking/reset_target");

    session_pub_ = create_publisher<track_robot_interfaces::msg::HumanFollowingSession>(
      "/human_following/session_state", 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(
      "/human_following/supervisor_debug", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/human_following/supervisor_markers", 10);

    latest_policy_decision_ = policy_.update(SessionInputs{});
    const auto period = std::chrono::duration<double>(1.0 / tick_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HumanFollowingSupervisorNode::onTick, this));

    RCLCPP_INFO(
      get_logger(), "Human-following supervisor started in %s mode (motion_confirmed=%s)",
      runtime_mode_ == RuntimeMode::Active ? "ACTIVE" : "SHADOW",
      motion_confirmed_ ? "true" : "false");
  }

private:
  struct GestureEvent
  {
    bool start{false};
    bool stop{false};
    int32_t visual_track_id{-1};
    double received_sec{0.0};
  };

  static RuntimeMode parseRuntimeMode(const std::string & value)
  {
    if (value == "active") {
      return RuntimeMode::Active;
    }
    if (value != "shadow") {
      throw std::invalid_argument("runtime_mode must be 'shadow' or 'active'");
    }
    return RuntimeMode::Shadow;
  }

  static std::string gestureKey(
    const track_robot_interfaces::msg::GestureState & message)
  {
    std::ostringstream stream;
    stream << message.header.stamp.sec << ':' << message.header.stamp.nanosec << ':' <<
      message.track_id << ':' << message.command;
    return stream.str();
  }

  void onGesture(const track_robot_interfaces::msg::GestureState::SharedPtr message)
  {
    const std::string key = gestureKey(*message);
    if (seen_gesture_keys_.count(key) != 0U) {
      return;
    }
    seen_gesture_keys_.insert(key);
    gesture_key_order_.push_back(key);
    while (gesture_key_order_.size() > 128U) {
      seen_gesture_keys_.erase(gesture_key_order_.front());
      gesture_key_order_.pop_front();
    }

    if (!message->trigger_active) {
      return;
    }
    const bool start = message->command == "start_tracking";
    const bool stop = message->command == "stop_tracking";
    if (!start && !stop) {
      return;
    }
    gesture_events_.push_back(
      GestureEvent{start, stop, message->track_id, steadyNowSec()});
  }

  bool fresh(
    const bool present, const double received_sec, const double timeout_sec,
    const double now_sec) const
  {
    return present && now_sec >= received_sec && now_sec - received_sec <= timeout_sec;
  }

  GestureEvent nextGestureEvent(const double now_sec)
  {
    while (
      !gesture_events_.empty() &&
      now_sec - gesture_events_.front().received_sec > gesture_timeout_sec_)
    {
      gesture_events_.pop_front();
    }
    if (gesture_events_.empty()) {
      return GestureEvent{};
    }
    const GestureEvent event = gesture_events_.front();
    gesture_events_.pop_front();
    return event;
  }

  void onTick()
  {
    const double now_sec = steadyNowSec();
    const bool camera_fresh = fresh(
      static_cast<bool>(camera_), camera_received_sec_, camera_timeout_sec_, now_sec);
    const bool decision_fresh = fresh(
      static_cast<bool>(follow_decision_), decision_received_sec_, decision_timeout_sec_, now_sec);
    const bool health_fresh = fresh(
      static_cast<bool>(health_), health_received_sec_, health_timeout_sec_, now_sec);
    const bool avoidance_fresh = fresh(
      static_cast<bool>(avoidance_), avoidance_received_sec_, avoidance_timeout_sec_, now_sec);
    const bool safety_fresh = fresh(
      static_cast<bool>(safety_), safety_received_sec_, safety_timeout_sec_, now_sec);
    const bool bunker_fresh = fresh(
      static_cast<bool>(bunker_), bunker_received_sec_, bunker_timeout_sec_, now_sec);

    const GestureEvent gesture = nextGestureEvent(now_sec);
    const bool camera_confirmed = camera_fresh && camera_->camera_visible &&
      camera_->lock_state == camera_->LOCK_TARGET_LOCKED &&
      camera_->identity_state == camera_->IDENTITY_CONFIRMED;
    const bool rc_override =
      (safety_fresh &&
      (safety_->rc_override_active || safety_->state == safety_->STATE_RC_OVERRIDE)) ||
      (bunker_fresh && bunker_->control_mode == BUNKER_CONTROL_MODE_RC);

    SessionInputs inputs;
    inputs.now_sec = now_sec;
    inputs.start_gesture_event = gesture.start;
    inputs.gesture_visual_track_id = gesture.visual_track_id;
    inputs.stop_gesture_event = gesture.stop &&
      gesture.visual_track_id >= 0 &&
      gesture.visual_track_id == authorized_visual_track_id_ &&
      (latest_policy_decision_.target_authorized ||
      latest_policy_decision_.arm_request_pending);
    inputs.camera_confirmed = camera_confirmed;
    inputs.camera_visual_track_id = camera_fresh ? camera_->visual_track_id : -1;
    inputs.camera_logical_target_id = camera_fresh ? camera_->logical_target_id : -1;

    if (decision_fresh) {
      inputs.decision_logical_target_id = follow_decision_->logical_target_id;
      inputs.decision_confidence = follow_decision_->decision_confidence;
      inputs.decision_confirmed_camera_lidar =
        follow_decision_->behavior == follow_decision_->BEHAVIOR_FOLLOW_CONFIRMED &&
        follow_decision_->target_source == "camera_lidar" &&
        follow_decision_->motion_permitted &&
        follow_decision_->decision_confidence > 0.0F;
      inputs.decision_lidar_limited =
        follow_decision_->behavior == follow_decision_->BEHAVIOR_FOLLOW_LIDAR_LIMITED;
      inputs.decision_uncertain =
        follow_decision_->behavior == follow_decision_->BEHAVIOR_UNCERTAIN_HOLD;
      inputs.decision_search_rotate =
        follow_decision_->behavior == follow_decision_->BEHAVIOR_SEARCH_ROTATE;
      inputs.decision_target_lost =
        follow_decision_->behavior == follow_decision_->BEHAVIOR_TARGET_LOST;
    }

    inputs.health_healthy = health_fresh && health_->state == health_->HEALTHY;
    inputs.health_hard_fault = health_fresh &&
      (health_->state == health_->UNSAFE || health_->state == health_->STALE);
    inputs.planner_ready = avoidance_fresh &&
      (avoidance_->state == avoidance_->STATE_DIRECT_CLEAR ||
      avoidance_->state == avoidance_->STATE_AVOIDING);
    inputs.planner_blocked = avoidance_fresh &&
      avoidance_->state == avoidance_->STATE_NO_SAFE_TRAJECTORY;
    inputs.safety_disarmed_ready = safety_fresh &&
      safety_->state == safety_->STATE_DISARMED && !safety_->armed &&
      !safety_->rc_override_active && !safety_->emergency_stop_latched;
    inputs.safety_armed = safety_fresh && safety_->armed;
    inputs.safety_hard_fault = safety_fresh &&
      (safety_->emergency_stop_latched ||
      safety_->state == safety_->STATE_EMERGENCY_STOP ||
      safety_->state == safety_->STATE_BASE_FAULT ||
      safety_->state == safety_->STATE_SENSOR_STALE);
    inputs.rc_override = rc_override;
    inputs.bunker_can_healthy = bunker_fresh &&
      bunker_->control_mode == BUNKER_CONTROL_MODE_CAN &&
      bunker_->vehicle_state == 0U && bunker_->error_code == 0U;
    inputs.required_inputs_fresh = camera_fresh && decision_fresh && health_fresh &&
      avoidance_fresh && safety_fresh && bunker_fresh;
    inputs.arm_service_ready = arm_client_->service_is_ready() && !arm_future_pending_ &&
      !disarm_requested_ && !disarm_future_pending_ &&
      !reset_requested_ && !reset_future_pending_;

    applyPolicyDecision(policy_.update(inputs));

    if (
      latest_policy_decision_.target_authorized && camera_confirmed &&
      camera_->logical_target_id == latest_policy_decision_.logical_target_id)
    {
      authorized_visual_track_id_ = camera_->visual_track_id;
    } else if (
      !latest_policy_decision_.target_authorized &&
      !latest_policy_decision_.arm_request_pending &&
      latest_policy_decision_.state != SessionState::ValidatingTarget)
    {
      authorized_visual_track_id_ = -1;
    }

    submitRetainedRequests();
    publishState(
      now_sec, camera_fresh, decision_fresh, health_fresh, avoidance_fresh,
      safety_fresh, bunker_fresh, rc_override);
  }

  void applyPolicyDecision(const SessionDecision & decision)
  {
    latest_policy_decision_ = decision;
    if (decision.request_disarm) {
      queueDisarm();
    }
    if (decision.request_target_reset) {
      queueReset();
    }
    if (decision.request_arm) {
      if (camera_) {
        authorized_visual_track_id_ = camera_->visual_track_id;
      }
      submitArm(decision.arm_request_generation);
    }
  }

  void submitArm(const uint64_t generation)
  {
    if (arm_future_pending_) {
      return;
    }
    if (!arm_client_->service_is_ready()) {
      applyPolicyDecision(policy_.acceptArmResult(generation, false, "arm_service_unavailable"));
      return;
    }

    arm_future_pending_ = true;
    arm_future_generation_ = generation;
    try {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      arm_client_->async_send_request(
        request,
        [this, generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          bool success = false;
          std::string message = "arm_service_exception";
          try {
            const auto response = future.get();
            success = response->success;
            message = response->message;
          } catch (const std::exception & exception) {
            message = exception.what();
          }
          if (arm_future_generation_ == generation) {
            arm_future_pending_ = false;
            arm_future_generation_ = 0;
          }
          applyPolicyDecision(policy_.acceptArmResult(generation, success, message));
          submitRetainedRequests();
        });
    } catch (const std::exception & exception) {
      arm_future_pending_ = false;
      arm_future_generation_ = 0;
      applyPolicyDecision(policy_.acceptArmResult(generation, false, exception.what()));
    }
  }

  void queueDisarm()
  {
    disarm_requested_ = true;
    ++disarm_request_generation_;
  }

  void queueReset()
  {
    reset_requested_ = true;
    ++reset_request_generation_;
  }

  void submitRetainedRequests()
  {
    if (disarm_requested_ && !disarm_future_pending_ && disarm_client_->service_is_ready()) {
      const uint64_t generation = disarm_request_generation_;
      disarm_future_pending_ = true;
      try {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        disarm_client_->async_send_request(
          request,
          [this, generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool success = false;
            try {
              success = future.get()->success;
            } catch (const std::exception &) {
              success = false;
            }
            disarm_future_pending_ = false;
            if (success && generation == disarm_request_generation_) {
              disarm_requested_ = false;
            }
          });
      } catch (const std::exception &) {
        disarm_future_pending_ = false;
      }
    }

    if (reset_requested_ && !reset_future_pending_ && reset_client_->service_is_ready()) {
      const uint64_t generation = reset_request_generation_;
      reset_future_pending_ = true;
      try {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        reset_client_->async_send_request(
          request,
          [this, generation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            bool success = false;
            try {
              success = future.get()->success;
            } catch (const std::exception &) {
              success = false;
            }
            reset_future_pending_ = false;
            if (success && generation == reset_request_generation_) {
              reset_requested_ = false;
            }
          });
      } catch (const std::exception &) {
        reset_future_pending_ = false;
      }
    }
  }

  void publishState(
    const double now_sec, const bool camera_fresh, const bool decision_fresh,
    const bool health_fresh, const bool avoidance_fresh, const bool safety_fresh,
    const bool bunker_fresh, const bool rc_override)
  {
    track_robot_interfaces::msg::HumanFollowingSession session;
    session.header.stamp = get_clock()->now();
    session.header.frame_id = "base_link";
    session.runtime_mode = runtime_mode_ == RuntimeMode::Active ?
      session.MODE_ACTIVE : session.MODE_SHADOW;
    session.state = static_cast<uint8_t>(latest_policy_decision_.state);
    session.logical_target_id = latest_policy_decision_.logical_target_id;
    session.motion_session_enabled =
      runtime_mode_ == RuntimeMode::Active && motion_confirmed_;
    session.target_authorized = latest_policy_decision_.target_authorized;
    session.arm_request_pending = latest_policy_decision_.arm_request_pending;
    session.safety_armed = safety_fresh && safety_ && safety_->armed;
    session.rc_override_active = rc_override;
    session.target_confidence = decision_fresh && follow_decision_ ?
      follow_decision_->decision_confidence : 0.0F;
    session.reason = latest_policy_decision_.reason;
    session_pub_->publish(session);

    std_msgs::msg::String debug;
    std::ostringstream json;
    json << std::fixed << std::setprecision(3) << '{'
         << "\"state\":\"" << sessionStateName(latest_policy_decision_.state) << "\","
         << "\"reason\":\"" << escapeJson(latest_policy_decision_.reason) << "\","
         << "\"logical_target_id\":" << latest_policy_decision_.logical_target_id << ','
         << "\"authorized_visual_track_id\":" << authorized_visual_track_id_ << ','
         << "\"target_authorized\":" <<
      (latest_policy_decision_.target_authorized ? "true" : "false") << ','
         << "\"arm_request_pending\":" <<
      (latest_policy_decision_.arm_request_pending ? "true" : "false") << ','
         << "\"arm_generation\":" << latest_policy_decision_.arm_request_generation << ','
         << "\"arm_future_pending\":" << (arm_future_pending_ ? "true" : "false") << ','
         << "\"disarm_retained\":" << (disarm_requested_ ? "true" : "false") << ','
         << "\"reset_retained\":" << (reset_requested_ ? "true" : "false") << ','
         << "\"camera_fresh\":" << (camera_fresh ? "true" : "false") << ','
         << "\"decision_fresh\":" << (decision_fresh ? "true" : "false") << ','
         << "\"health_fresh\":" << (health_fresh ? "true" : "false") << ','
         << "\"avoidance_fresh\":" << (avoidance_fresh ? "true" : "false") << ','
         << "\"safety_fresh\":" << (safety_fresh ? "true" : "false") << ','
         << "\"bunker_fresh\":" << (bunker_fresh ? "true" : "false") << ','
         << "\"rc_override\":" << (rc_override ? "true" : "false") << ','
         << "\"steady_now_sec\":" << now_sec << '}';
    debug.data = json.str();
    debug_pub_->publish(debug);

    publishMarkers(session, decision_fresh);
  }

  visualization_msgs::msg::Marker textMarker(
    const std::string & frame, const builtin_interfaces::msg::Time & stamp,
    const std::string & marker_namespace, const int id, const double z,
    const std::string & text) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.22;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.text = text;
    return marker;
  }

  void publishMarkers(
    const track_robot_interfaces::msg::HumanFollowingSession & session,
    const bool decision_fresh)
  {
    const std::string frame = decision_fresh && follow_decision_ &&
      !follow_decision_->header.frame_id.empty() ?
      follow_decision_->header.frame_id : "base_link";
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(textMarker(
      frame, session.header.stamp, "human_following_session/status", 0, 2.2,
      sessionStateName(latest_policy_decision_.state) + ": " + session.reason));
    array.markers.push_back(textMarker(
      frame, session.header.stamp, "human_following_session/mode", 0, 2.5,
      runtime_mode_ == RuntimeMode::Active ? "ACTIVE" : "SHADOW"));

    visualization_msgs::msg::Marker authorization;
    authorization.header.frame_id = frame;
    authorization.header.stamp = session.header.stamp;
    authorization.ns = "human_following_session/authorization";
    authorization.id = 0;
    authorization.pose.orientation.w = 1.0;
    if (latest_policy_decision_.target_authorized && decision_fresh && follow_decision_) {
      authorization.type = visualization_msgs::msg::Marker::SPHERE;
      authorization.action = visualization_msgs::msg::Marker::ADD;
      authorization.pose.position = follow_decision_->target_position;
      authorization.scale.x = 0.35;
      authorization.scale.y = 0.35;
      authorization.scale.z = 0.35;
      authorization.color.g = 1.0F;
      authorization.color.a = 0.85F;
    } else {
      authorization.action = visualization_msgs::msg::Marker::DELETE;
    }
    array.markers.push_back(authorization);
    marker_pub_->publish(array);
  }

  RuntimeMode runtime_mode_;
  bool motion_confirmed_;
  double tick_rate_;
  double gesture_timeout_sec_;
  double camera_timeout_sec_;
  double decision_timeout_sec_;
  double health_timeout_sec_;
  double avoidance_timeout_sec_;
  double safety_timeout_sec_;
  double bunker_timeout_sec_;
  HumanFollowingSessionPolicy policy_;
  SessionDecision latest_policy_decision_;

  track_robot_interfaces::msg::CameraTarget::SharedPtr camera_;
  track_robot_interfaces::msg::FollowDecision::SharedPtr follow_decision_;
  track_robot_interfaces::msg::PerceptionHealth::SharedPtr health_;
  track_robot_interfaces::msg::AvoidanceState::SharedPtr avoidance_;
  track_robot_interfaces::msg::SafetyState::SharedPtr safety_;
  bunker_msgs::msg::BunkerStatus::SharedPtr bunker_;
  double camera_received_sec_{0.0};
  double decision_received_sec_{0.0};
  double health_received_sec_{0.0};
  double avoidance_received_sec_{0.0};
  double safety_received_sec_{0.0};
  double bunker_received_sec_{0.0};

  std::deque<GestureEvent> gesture_events_;
  std::deque<std::string> gesture_key_order_;
  std::unordered_set<std::string> seen_gesture_keys_;
  int32_t authorized_visual_track_id_{-1};

  bool arm_future_pending_{false};
  uint64_t arm_future_generation_{0};
  bool disarm_requested_{false};
  bool disarm_future_pending_{false};
  uint64_t disarm_request_generation_{0};
  bool reset_requested_{false};
  bool reset_future_pending_{false};
  uint64_t reset_request_generation_{0};

  rclcpp::Subscription<track_robot_interfaces::msg::GestureState>::SharedPtr gesture_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::CameraTarget>::SharedPtr camera_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::FollowDecision>::SharedPtr decision_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::PerceptionHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::AvoidanceState>::SharedPtr avoidance_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr bunker_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disarm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Publisher<track_robot_interfaces::msg::HumanFollowingSession>::SharedPtr session_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace track_robot_decision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<track_robot_decision::HumanFollowingSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
