#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "bunker_msgs/msg/bunker_rc_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "track_robot_interfaces/msg/follow_decision.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

class TargetFollowController : public rclcpp::Node
{
public:
  TargetFollowController()
  : Node("target_follow_controller_node")
  {
    declare_parameter<std::string>("decision_topic", "/follow/decision");
    declare_parameter<std::string>("rc_state_topic", "/bunker_rc_state");
    declare_parameter<std::string>("debug_cmd_vel_topic", "/follow/cmd_vel_debug");
    declare_parameter<std::string>("planned_cmd_vel_topic", "/follow/cmd_vel_planned");
    declare_parameter<std::string>("debug_text_topic", "/follow/controller_debug");
    declare_parameter<std::string>("marker_topic", "/follow/controller_markers");
    declare_parameter<std::string>("marker_frame", "base_link");
    declare_parameter<std::string>("output_topic", "/follow/cmd_vel");
    declare_parameter<std::string>("enable_service", "/follow/enable_cmd_vel");
    declare_parameter<std::string>("disable_service", "/follow/disable_cmd_vel");
    declare_parameter<bool>("enable_cmd_vel", false);
    declare_parameter<double>("follow_distance", 1.8);
    declare_parameter<double>("deadband_distance", 0.25);
    declare_parameter<double>("max_linear_x", 0.15);
    declare_parameter<double>("max_angular_z", 0.35);
    declare_parameter<double>("linear_gain", 0.35);
    declare_parameter<double>("angular_gain", 0.9);
    declare_parameter<double>("min_confidence", 0.35);
    declare_parameter<double>("target_timeout_sec", 0.5);
    declare_parameter<double>("front_cone_rad", 0.26);
    declare_parameter<double>("publish_rate", 20.0);
    declare_parameter<double>("linear_accel_limit", 0.10);
    declare_parameter<double>("angular_accel_limit", 0.25);
    declare_parameter<double>("lidar_only_no_motion_distance", 1.8);
    declare_parameter<int>("rc_override_deadband", 10);
    declare_parameter<bool>("require_gesture_relock_after_rc_override", true);
    declare_parameter<bool>("allow_lidar_only_forward_motion", false);

    decision_topic_ = get_parameter("decision_topic").as_string();
    rc_state_topic_ = get_parameter("rc_state_topic").as_string();
    debug_cmd_vel_topic_ = get_parameter("debug_cmd_vel_topic").as_string();
    planned_cmd_vel_topic_ = get_parameter("planned_cmd_vel_topic").as_string();
    debug_text_topic_ = get_parameter("debug_text_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    marker_frame_ = get_parameter("marker_frame").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    enable_service_ = get_parameter("enable_service").as_string();
    disable_service_ = get_parameter("disable_service").as_string();
    enable_cmd_vel_ = get_parameter("enable_cmd_vel").as_bool();
    follow_distance_ = get_parameter("follow_distance").as_double();
    deadband_distance_ = get_parameter("deadband_distance").as_double();
    max_linear_x_ = get_parameter("max_linear_x").as_double();
    max_angular_z_ = get_parameter("max_angular_z").as_double();
    linear_gain_ = get_parameter("linear_gain").as_double();
    angular_gain_ = get_parameter("angular_gain").as_double();
    min_confidence_ = get_parameter("min_confidence").as_double();
    target_timeout_sec_ = get_parameter("target_timeout_sec").as_double();
    front_cone_rad_ = get_parameter("front_cone_rad").as_double();
    publish_rate_ = get_parameter("publish_rate").as_double();
    linear_accel_limit_ = get_parameter("linear_accel_limit").as_double();
    angular_accel_limit_ = get_parameter("angular_accel_limit").as_double();
    lidar_only_no_motion_distance_ = get_parameter("lidar_only_no_motion_distance").as_double();
    rc_override_deadband_ = get_parameter("rc_override_deadband").as_int();
    require_gesture_relock_after_rc_override_ =
      get_parameter("require_gesture_relock_after_rc_override").as_bool();
    allow_lidar_only_forward_motion_ = get_parameter("allow_lidar_only_forward_motion").as_bool();

    debug_pub_ = create_publisher<geometry_msgs::msg::Twist>(debug_cmd_vel_topic_, 10);
    planned_pub_ = create_publisher<geometry_msgs::msg::Twist>(planned_cmd_vel_topic_, 10);
    debug_text_pub_ = create_publisher<std_msgs::msg::String>(debug_text_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    enable_srv_ = create_service<std_srvs::srv::Trigger>(
      enable_service_,
      std::bind(
        &TargetFollowController::enableCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    disable_srv_ = create_service<std_srvs::srv::Trigger>(
      disable_service_,
      std::bind(
        &TargetFollowController::disableCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    decision_sub_ = create_subscription<track_robot_interfaces::msg::FollowDecision>(
      decision_topic_, 10,
      std::bind(&TargetFollowController::decisionCallback, this, std::placeholders::_1));
    rc_sub_ = create_subscription<bunker_msgs::msg::BunkerRCState>(
      rc_state_topic_, 10,
      std::bind(&TargetFollowController::rcStateCallback, this, std::placeholders::_1));
    const auto timer_period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&TargetFollowController::timerCallback, this));

    RCLCPP_WARN(
      get_logger(),
      "target_follow_controller_node started. enable_cmd_vel=%s, debug=%s, output=%s",
      enable_cmd_vel_ ? "true" : "false",
      debug_cmd_vel_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  struct CommandDecision
  {
    geometry_msgs::msg::Twist cmd;
    std::string behavior{"no_target"};
    std::string output_status{"cmd_vel_disabled"};
    bool target_valid{false};
    bool immediate_stop{false};
  };

  void decisionCallback(const track_robot_interfaces::msg::FollowDecision::SharedPtr msg)
  {
    last_decision_ = *msg;
    last_decision_time_ = now();
    have_decision_ = true;
    updateGestureRelockState(*msg);
    publishCommand(makeCommand(*msg));
  }

  void rcStateCallback(const bunker_msgs::msg::BunkerRCState::SharedPtr msg)
  {
    last_rc_state_ = *msg;
    have_rc_state_ = true;
    current_rc_override_active_ = rcOverrideActive(*msg);
    if (!current_rc_override_active_) {
      if (rc_override_latched_ && !gesture_relock_required_) {
        rc_override_latched_ = false;
      }
      return;
    }

    if (!rc_override_latched_) {
      restore_enable_after_relock_ = enable_cmd_vel_;
      rc_override_latched_ = true;
      gesture_relock_required_ = require_gesture_relock_after_rc_override_;
      relock_saw_unlocked_ = false;
      enable_cmd_vel_ = false;
      RCLCPP_WARN(
        get_logger(),
        "RC override detected. Follow output disabled; gesture_relock_required=%s",
        gesture_relock_required_ ? "true" : "false");
    }
    publishCommand(makeStopDecision("rc_override"));
  }

  void timerCallback()
  {
    if (rc_override_latched_) {
      publishCommand(makeStopDecision(current_rc_override_active_ ?
        "rc_override" : "gesture_relock_required"));
      return;
    }
    if (!have_decision_) {
      publishCommand(makeStopDecision("no_target"));
      return;
    }
    const double age = (now() - last_decision_time_).seconds();
    if (age > target_timeout_sec_) {
      publishCommand(makeStopDecision("stale_target"));
      return;
    }
    publishCommand(makeCommand(last_decision_));
  }

  CommandDecision makeCommand(
    const track_robot_interfaces::msg::FollowDecision & target) const
  {
    CommandDecision decision;
    if (rc_override_latched_) {
      decision.behavior = current_rc_override_active_ ?
        "rc_override" : "gesture_relock_required";
      decision.immediate_stop = true;
      return decision;
    }
    if (!target.motion_permitted) {
      decision.behavior = target.reason.empty() ? "motion_not_permitted" : target.reason;
      decision.immediate_stop = true;
      return decision;
    }
    if (!std::isfinite(static_cast<double>(target.target_distance)) ||
      !std::isfinite(static_cast<double>(target.target_bearing)))
    {
      decision.behavior = "invalid_target_geometry";
      decision.immediate_stop = true;
      return decision;
    }

    if (target.behavior == target.BEHAVIOR_SEARCH_ROTATE) {
      decision.target_valid = true;
      decision.behavior = "search_rotate";
      decision.cmd.angular.z = clampValue(
        angular_gain_ * static_cast<double>(target.search_target_bearing),
        -std::min(max_angular_z_, static_cast<double>(target.maximum_angular_speed)),
        std::min(max_angular_z_, static_cast<double>(target.maximum_angular_speed)));
      return decision;
    }

    decision.target_valid = true;
    const double angular_limit = std::min(
      max_angular_z_, static_cast<double>(target.maximum_angular_speed));
    decision.cmd.angular.z = clampValue(
      angular_gain_ * static_cast<double>(target.target_bearing),
      -angular_limit, angular_limit);

    const double decision_front_cone = target.behavior == target.BEHAVIOR_FOLLOW_LIDAR_LIMITED ?
      std::min(front_cone_rad_, 15.0 * M_PI / 180.0) :
      std::min(front_cone_rad_, 20.0 * M_PI / 180.0);
    if (!target.forward_permitted ||
      std::abs(static_cast<double>(target.target_bearing)) > decision_front_cone) {
      decision.behavior = "aligning_to_target";
      return decision;
    }

    const double distance_error = static_cast<double>(target.target_distance) - follow_distance_;
    if (distance_error <= deadband_distance_) {
      decision.behavior = "inside_follow_distance";
      decision.cmd.angular.z = 0.0;
      return decision;
    }

    decision.behavior = "following_target";
    decision.cmd.linear.x = clampValue(
      linear_gain_ * distance_error, 0.0,
      std::min(max_linear_x_, static_cast<double>(target.maximum_linear_speed)));
    return decision;
  }

  CommandDecision makeStopDecision(const std::string & behavior) const
  {
    CommandDecision decision;
    decision.behavior = behavior;
    decision.immediate_stop = true;
    return decision;
  }

  void enableCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    enable_cmd_vel_ = true;
    response->success = true;
    response->message = gesture_relock_required_ ?
      "target_follow_controller enable requested; waiting for stop/start gesture relock" :
      "target_follow_controller real cmd_vel output enabled";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void disableCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    last_ramped_cmd_ = geometry_msgs::msg::Twist();
    ramp_initialized_ = false;
    restore_enable_after_relock_ = false;
    enable_cmd_vel_ = false;
    response->success = true;
    response->message = "target_follow_controller real cmd_vel output disabled and zero sent";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void publishCommand(CommandDecision decision)
  {
    decision.cmd.linear.x = clampValue(decision.cmd.linear.x, 0.0, max_linear_x_);
    decision.cmd.angular.z = clampValue(decision.cmd.angular.z, -max_angular_z_, max_angular_z_);
    decision.cmd.linear.y = 0.0;
    decision.cmd.linear.z = 0.0;
    decision.cmd.angular.x = 0.0;
    decision.cmd.angular.y = 0.0;

    decision.cmd = applyCommandRamp(decision.cmd, decision.immediate_stop);
    debug_pub_->publish(decision.cmd);
    planned_pub_->publish(decision.cmd);
    if (enable_cmd_vel_ && !gesture_relock_required_ && !rc_override_latched_) {
      decision.output_status = "enabled_publishing";
      cmd_pub_->publish(decision.cmd);
    } else if (gesture_relock_required_ || rc_override_latched_) {
      decision.output_status = "gesture_relock_required";
    } else {
      decision.output_status = "cmd_vel_disabled";
    }
    publishDebugText(decision);
    publishMarkers(decision);
  }

  void publishDebugText(const CommandDecision & decision)
  {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    const double age = have_decision_ ? (now() - last_decision_time_).seconds() : -1.0;
    ss << "{"
       << "\"enabled\":" << (enable_cmd_vel_ ? "true" : "false") << ","
       << "\"behavior\":\"" << decision.behavior << "\","
       << "\"output_status\":\"" << decision.output_status << "\","
       << "\"target_valid\":" << (decision.target_valid ? "true" : "false") << ","
       << "\"target_age_sec\":" << age << ","
       << "\"rc_override_latched\":" << (rc_override_latched_ ? "true" : "false") << ","
       << "\"rc_override_active\":" << (current_rc_override_active_ ? "true" : "false") << ","
       << "\"gesture_relock_required\":" << (gesture_relock_required_ ? "true" : "false") << ","
       << "\"relock_saw_unlocked\":" << (relock_saw_unlocked_ ? "true" : "false") << ","
       << "\"rc_deadband\":" << rc_override_deadband_ << ","
       << "\"rc_sticks\":[" << rcStickValue(last_rc_state_.stick_left_h) << ","
       << rcStickValue(last_rc_state_.stick_left_v) << ","
       << rcStickValue(last_rc_state_.stick_right_h) << ","
       << rcStickValue(last_rc_state_.stick_right_v) << "],"
       << "\"target_id\":" << (have_decision_ ? last_decision_.logical_target_id : -1) << ","
       << "\"decision_behavior\":" << (have_decision_ ? static_cast<int>(last_decision_.behavior) : -1) << ","
       << "\"confidence\":" << (have_decision_ ? last_decision_.decision_confidence : 0.0F) << ","
       << "\"distance\":" << (have_decision_ ? last_decision_.target_distance : 0.0F) << ","
       << "\"bearing\":" << (have_decision_ ? last_decision_.target_bearing : 0.0F) << ","
       << "\"linear_x\":" << decision.cmd.linear.x << ","
       << "\"angular_z\":" << decision.cmd.angular.z
       << "}";
    msg.data = ss.str();
    debug_text_pub_->publish(msg);
  }

  std_msgs::msg::ColorRGBA markerColor(const CommandDecision & decision) const
  {
    std_msgs::msg::ColorRGBA color;
    color.a = 0.9F;
    if (decision.behavior == "following_target") {
      color.r = 0.1F;
      color.g = 0.9F;
      color.b = 0.2F;
    } else if (decision.behavior == "aligning_to_target") {
      color.r = 1.0F;
      color.g = 0.75F;
      color.b = 0.05F;
    } else if (decision.behavior == "lidar_only_rotate_only") {
      color.r = 0.1F;
      color.g = 0.45F;
      color.b = 1.0F;
    } else if (decision.behavior == "prediction_only_stop" ||
      decision.behavior == "stale_target" ||
      decision.behavior == "low_confidence" ||
      decision.behavior == "invalid_track_state" ||
      decision.behavior == "rc_override" ||
      decision.behavior == "gesture_relock_required" ||
      decision.behavior == "no_target")
    {
      color.r = 0.9F;
      color.g = 0.1F;
      color.b = 0.1F;
    } else {
      color.r = 0.55F;
      color.g = 0.55F;
      color.b = 0.55F;
    }
    return color;
  }

  void publishMarkers(const CommandDecision & decision)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = now();

    visualization_msgs::msg::Marker arrow;
    arrow.header.stamp = stamp;
    arrow.header.frame_id = marker_frame_;
    arrow.ns = "follow_controller";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.12;
    arrow.scale.z = 0.12;
    arrow.color = markerColor(decision);
    arrow.lifetime = rclcpp::Duration::from_seconds(0.25);

    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.18;
    geometry_msgs::msg::Point end = start;
    const double bearing = have_decision_ &&
      std::isfinite(static_cast<double>(last_decision_.target_bearing)) ?
      static_cast<double>(last_decision_.target_bearing) : 0.0;
    const double speed_fraction = max_linear_x_ > 1e-6 ?
      clampValue(std::abs(decision.cmd.linear.x) / max_linear_x_, 0.0, 1.0) : 0.0;
    const double angular_fraction = max_angular_z_ > 1e-6 ?
      clampValue(std::abs(decision.cmd.angular.z) / max_angular_z_, 0.0, 1.0) : 0.0;
    const double arrow_length = decision.behavior == "aligning_to_target" ?
      0.35 + 0.45 * angular_fraction :
      0.20 + 1.30 * speed_fraction;
    end.x = arrow_length * std::cos(bearing);
    end.y = arrow_length * std::sin(bearing);
    end.z = start.z;
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    markers.markers.push_back(arrow);

    visualization_msgs::msg::Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = marker_frame_;
    text.ns = "follow_controller";
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 0.0;
    text.pose.position.y = 0.0;
    text.pose.position.z = 0.75;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    text.color = markerColor(decision);
    text.lifetime = rclcpp::Duration::from_seconds(0.25);
    std::ostringstream label;
    label << decision.behavior
          << "\n" << decision.output_status
          << "\nv=" << decision.cmd.linear.x
          << " wz=" << decision.cmd.angular.z;
    if (have_decision_) {
      label << "\nd=" << last_decision_.target_distance
            << " b=" << last_decision_.target_bearing;
    }
    text.text = label.str();
    markers.markers.push_back(text);

    marker_pub_->publish(markers);
  }

  geometry_msgs::msg::Twist applyCommandRamp(
    const geometry_msgs::msg::Twist & target_cmd,
    const bool immediate_stop)
  {
    if (immediate_stop) {
      last_ramped_cmd_ = geometry_msgs::msg::Twist();
      last_ramp_time_ = now();
      ramp_initialized_ = true;
      return last_ramped_cmd_;
    }

    const auto stamp = now();
    double dt = 1.0 / std::max(1.0, publish_rate_);
    if (ramp_initialized_) {
      dt = std::max(0.0, (stamp - last_ramp_time_).seconds());
    } else {
      ramp_initialized_ = true;
    }
    last_ramp_time_ = stamp;

    geometry_msgs::msg::Twist ramped = target_cmd;
    ramped.linear.x = rampValue(
      last_ramped_cmd_.linear.x,
      target_cmd.linear.x,
      std::max(0.0, linear_accel_limit_) * dt);
    ramped.angular.z = rampValue(
      last_ramped_cmd_.angular.z,
      target_cmd.angular.z,
      std::max(0.0, angular_accel_limit_) * dt);
    ramped.linear.y = 0.0;
    ramped.linear.z = 0.0;
    ramped.angular.x = 0.0;
    ramped.angular.y = 0.0;
    last_ramped_cmd_ = ramped;
    return ramped;
  }

  static double rampValue(const double current, const double target, const double max_delta)
  {
    if (max_delta <= 0.0) {
      return target;
    }
    return current + clampValue(target - current, -max_delta, max_delta);
  }

  bool rcOverrideActive(const bunker_msgs::msg::BunkerRCState & msg) const
  {
    return std::abs(rcStickValue(msg.stick_left_h)) > rc_override_deadband_ ||
      std::abs(rcStickValue(msg.stick_left_v)) > rc_override_deadband_ ||
      std::abs(rcStickValue(msg.stick_right_h)) > rc_override_deadband_ ||
      std::abs(rcStickValue(msg.stick_right_v)) > rc_override_deadband_;
  }

  static int rcStickValue(const int8_t value)
  {
    return static_cast<int>(value);
  }

  void updateGestureRelockState(const track_robot_interfaces::msg::FollowDecision & target)
  {
    if (!gesture_relock_required_) {
      return;
    }

    const bool unlocked =
      target.logical_target_id < 0 ||
      target.behavior == track_robot_interfaces::msg::FollowDecision::BEHAVIOR_WAITING_FOR_TARGET;
    if (unlocked) {
      relock_saw_unlocked_ = true;
      return;
    }

    const bool relocked =
      relock_saw_unlocked_ &&
      target.logical_target_id >= 0 &&
      (target.behavior == track_robot_interfaces::msg::FollowDecision::BEHAVIOR_FOLLOW_CONFIRMED ||
      target.behavior == track_robot_interfaces::msg::FollowDecision::BEHAVIOR_FOLLOW_LIDAR_LIMITED);
    if (!relocked) {
      return;
    }

    if (current_rc_override_active_) {
      return;
    }

    gesture_relock_required_ = false;
    rc_override_latched_ = false;
    relock_saw_unlocked_ = false;
    if (restore_enable_after_relock_) {
      enable_cmd_vel_ = true;
      restore_enable_after_relock_ = false;
      RCLCPP_WARN(get_logger(), "Gesture relock complete. Restored follow output enable.");
    } else {
      RCLCPP_WARN(get_logger(), "Gesture relock complete. Follow output remains disabled.");
    }
  }

  std::string decision_topic_;
  std::string rc_state_topic_;
  std::string debug_cmd_vel_topic_;
  std::string planned_cmd_vel_topic_;
  std::string debug_text_topic_;
  std::string marker_topic_;
  std::string marker_frame_;
  std::string output_topic_;
  std::string enable_service_;
  std::string disable_service_;
  bool enable_cmd_vel_;
  double follow_distance_;
  double deadband_distance_;
  double max_linear_x_;
  double max_angular_z_;
  double linear_gain_;
  double angular_gain_;
  double min_confidence_;
  double target_timeout_sec_;
  double front_cone_rad_;
  double publish_rate_;
  double linear_accel_limit_;
  double angular_accel_limit_;
  double lidar_only_no_motion_distance_;
  int rc_override_deadband_;
  bool require_gesture_relock_after_rc_override_;
  bool allow_lidar_only_forward_motion_;

  bool have_decision_{false};
  rclcpp::Time last_decision_time_;
  track_robot_interfaces::msg::FollowDecision last_decision_;
  bool have_rc_state_{false};
  bunker_msgs::msg::BunkerRCState last_rc_state_;
  bool current_rc_override_active_{false};
  bool rc_override_latched_{false};
  bool gesture_relock_required_{false};
  bool relock_saw_unlocked_{false};
  bool restore_enable_after_relock_{false};
  bool ramp_initialized_{false};
  rclcpp::Time last_ramp_time_;
  geometry_msgs::msg::Twist last_ramped_cmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr planned_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_text_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Subscription<track_robot_interfaces::msg::FollowDecision>::SharedPtr decision_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerRCState>::SharedPtr rc_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetFollowController>());
  rclcpp::shutdown();
  return 0;
}
