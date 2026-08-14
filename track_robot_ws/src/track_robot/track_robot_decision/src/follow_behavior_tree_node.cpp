#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "track_robot_interfaces/msg/avoidance_state.hpp"
#include "track_robot_interfaces/msg/follow_decision.hpp"
#include "track_robot_interfaces/msg/perception_health.hpp"
#include "track_robot_interfaces/msg/safety_state.hpp"
#include "track_robot_interfaces/msg/target_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {

double clamp(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace

class FollowBehaviorTreeNode : public rclcpp::Node
{
public:
  FollowBehaviorTreeNode()
  : Node("follow_behavior_tree_node")
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/human_tracking/target_state");
    health_topic_ = declare_parameter<std::string>("health_topic", "/perception/health");
    avoidance_topic_ = declare_parameter<std::string>("avoidance_topic", "/follow/avoidance_state");
    safety_topic_ = declare_parameter<std::string>("safety_topic", "/safety/state");
    decision_topic_ = declare_parameter<std::string>("decision_topic", "/follow/decision");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/follow/decision_debug");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/follow/decision_markers");
    reset_target_service_ = declare_parameter<std::string>(
      "reset_target_service", "/human_tracking/reset_target");
    tree_xml_ = declare_parameter<std::string>("tree_xml", "");
    tick_rate_ = declare_parameter<double>("tick_rate", 20.0);
    require_health_ = declare_parameter<bool>("require_health", true);
    require_avoidance_feedback_ = declare_parameter<bool>("require_avoidance_feedback", false);
    require_safety_feedback_ = declare_parameter<bool>("require_safety_feedback", false);
    applyBooleanOverride("require_health_override", require_health_);
    applyBooleanOverride("require_avoidance_feedback_override", require_avoidance_feedback_);
    applyBooleanOverride("require_safety_feedback_override", require_safety_feedback_);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.50);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.50);
    confirmation_ticks_ = declare_parameter<int>("confirmation_ticks", 3);
    blocked_clear_ticks_required_ = declare_parameter<int>("blocked_clear_ticks", 10);
    confirmed_identity_confidence_ = declare_parameter<double>("confirmed_identity_confidence", 0.70);
    confirmed_geometry_confidence_ = declare_parameter<double>("confirmed_geometry_confidence", 0.60);
    confirmed_overall_confidence_ = declare_parameter<double>("confirmed_overall_confidence", 0.65);
    confirmed_max_sigma_ = declare_parameter<double>("confirmed_max_sigma", 0.45);
    confirmed_max_measurement_age_ = declare_parameter<double>("confirmed_max_measurement_age", 0.20);
    lidar_geometry_confidence_ = declare_parameter<double>("lidar_geometry_confidence", 0.65);
    lidar_overall_confidence_ = declare_parameter<double>("lidar_overall_confidence", 0.55);
    lidar_max_sigma_ = declare_parameter<double>("lidar_max_sigma", 0.60);
    lidar_max_age_ = declare_parameter<double>("lidar_max_age", 0.20);
    lidar_only_timeout_sec_ = declare_parameter<double>("lidar_only_timeout_sec", 3.0);
    uncertain_hold_sec_ = declare_parameter<double>("uncertain_hold_sec", 1.0);
    search_entry_max_age_sec_ = declare_parameter<double>("search_entry_max_age_sec", 1.5);
    search_timeout_sec_ = declare_parameter<double>("search_timeout_sec", 4.0);
    search_sector_rad_ = declare_parameter<double>("search_sector_deg", 120.0) * M_PI / 180.0;
    confirmed_max_linear_ = declare_parameter<double>("confirmed_max_linear", 0.30);
    confirmed_max_angular_ = declare_parameter<double>("confirmed_max_angular", 0.45);
    lidar_max_linear_ = declare_parameter<double>("lidar_max_linear", 0.15);
    lidar_max_angular_ = declare_parameter<double>("lidar_max_angular", 0.35);
    search_max_angular_ = declare_parameter<double>("search_max_angular", 0.20);

    target_sub_ = create_subscription<track_robot_interfaces::msg::TargetState>(
      target_topic_, 10,
      [this](const track_robot_interfaces::msg::TargetState::SharedPtr msg) {
        targetCallback(*msg);
      });
    health_sub_ = create_subscription<track_robot_interfaces::msg::PerceptionHealth>(
      health_topic_, 10,
      [this](const track_robot_interfaces::msg::PerceptionHealth::SharedPtr msg) {
        health_ = *msg; health_time_ = steadyNow(); have_health_ = true;
      });
    avoidance_sub_ = create_subscription<track_robot_interfaces::msg::AvoidanceState>(
      avoidance_topic_, 10,
      [this](const track_robot_interfaces::msg::AvoidanceState::SharedPtr msg) {
        avoidanceCallback(*msg);
      });
    safety_sub_ = create_subscription<track_robot_interfaces::msg::SafetyState>(
      safety_topic_, 10,
      [this](const track_robot_interfaces::msg::SafetyState::SharedPtr msg) {
        safety_ = *msg; safety_time_ = steadyNow(); have_safety_ = true;
      });
    decision_pub_ = create_publisher<track_robot_interfaces::msg::FollowDecision>(decision_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    reset_target_client_ = create_client<std_srvs::srv::Trigger>(reset_target_service_);

    registerTreeNodes();
    if (tree_xml_.empty()) {
      tree_xml_ = ament_index_cpp::get_package_share_directory("track_robot_decision") +
        "/config/follow_behavior_tree.xml";
    }
    tree_ = factory_.createTreeFromFile(tree_xml_);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tick_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FollowBehaviorTreeNode::tick, this));
    RCLCPP_WARN(get_logger(), "Outdoor follow decision tree started; motion decisions only");
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  static SteadyTime steadyNow() {return std::chrono::steady_clock::now();}
  void applyBooleanOverride(const std::string & name, bool & value)
  {
    const auto override = declare_parameter<std::string>(name, "");
    if (override == "true" || override == "1") value = true;
    if (override == "false" || override == "0") value = false;
  }
  double age(const SteadyTime & time, const bool have) const
  {
    if (!have) return std::numeric_limits<double>::infinity();
    return std::chrono::duration<double>(steadyNow() - time).count();
  }

  void targetCallback(const track_robot_interfaces::msg::TargetState & msg)
  {
    if (msg.target_id < 0 || msg.track_state == msg.TRACK_NO_TARGET) {
      target_ = msg;
      target_time_ = steadyNow();
      have_target_ = true;
      have_reliable_target_ = false;
      search_active_ = false;
      lidar_only_active_ = false;
      confirmed_ticks_ = 0;
      lidar_ticks_ = 0;
      last_candidate_id_ = -1;
      return;
    }
    if (have_reliable_target_ && msg.target_id != last_reliable_target_.target_id &&
      msg.lock_state == msg.LOCK_TARGET_LOCKED) {
      have_reliable_target_ = false;
      search_active_ = false;
    }
    target_ = msg;
    target_time_ = steadyNow();
    have_target_ = true;
    const bool same_id = msg.target_id >= 0 && msg.target_id == last_candidate_id_;
    last_candidate_id_ = msg.target_id;
    if (rawConfirmed(msg)) {
      confirmed_ticks_ = same_id ? confirmed_ticks_ + 1 : 1;
      lidar_ticks_ = 0;
      rememberReliableTarget(msg);
    } else if (rawLidar(msg)) {
      lidar_ticks_ = same_id ? lidar_ticks_ + 1 : 1;
      confirmed_ticks_ = 0;
      rememberReliableTarget(msg);
      if (!lidar_only_active_) {
        lidar_only_start_ = steadyNow();
        lidar_only_active_ = true;
      }
    } else {
      confirmed_ticks_ = 0;
      lidar_ticks_ = 0;
      lidar_only_active_ = false;
    }
  }

  void avoidanceCallback(const track_robot_interfaces::msg::AvoidanceState & msg)
  {
    avoidance_ = msg;
    avoidance_time_ = steadyNow();
    have_avoidance_ = true;
    const bool blocked = msg.state == msg.STATE_NO_SAFE_TRAJECTORY;
    if (blocked) {
      blocked_latched_ = true;
      blocked_clear_ticks_ = 0;
    } else if (blocked_latched_ &&
      (msg.state == msg.STATE_DIRECT_CLEAR || msg.state == msg.STATE_AVOIDING)) {
      if (++blocked_clear_ticks_ >= blocked_clear_ticks_required_) {
        blocked_latched_ = false;
        blocked_clear_ticks_ = 0;
      }
    }
  }

  double targetSigma(const track_robot_interfaces::msg::TargetState & target) const
  {
    return std::sqrt(std::max(0.0F,
      std::max(target.position_covariance[0], target.position_covariance[4])));
  }

  bool rawConfirmed(const track_robot_interfaces::msg::TargetState & target) const
  {
    return target.target_id >= 0 && target.position_base_valid &&
      target.track_state == target.TRACK_CAMERA_LIDAR_TRACKED &&
      target.association_state == target.ASSOCIATION_CONFIRMED &&
      target.identity_confidence >= confirmed_identity_confidence_ &&
      target.geometry_confidence >= confirmed_geometry_confidence_ &&
      target.overall_confidence >= confirmed_overall_confidence_ &&
      targetSigma(target) <= confirmed_max_sigma_ &&
      target.measurement_age <= confirmed_max_measurement_age_;
  }

  bool rawLidar(const track_robot_interfaces::msg::TargetState & target) const
  {
    return target.target_id >= 0 && target.position_base_valid &&
      target.track_state == target.TRACK_LIDAR_ONLY_TRACKING &&
      (target.association_state == target.ASSOCIATION_CONFIRMED ||
      target.association_state == target.ASSOCIATION_PREDICTED) &&
      target.geometry_confidence >= lidar_geometry_confidence_ &&
      target.overall_confidence >= lidar_overall_confidence_ &&
      targetSigma(target) <= lidar_max_sigma_ && target.time_since_lidar_seen <= lidar_max_age_;
  }

  void rememberReliableTarget(const track_robot_interfaces::msg::TargetState & target)
  {
    last_reliable_target_ = target;
    last_reliable_time_ = steadyNow();
    have_reliable_target_ = true;
    search_active_ = false;
  }

  bool feedbackFresh(const SteadyTime & time, const bool have) const
  {
    return have && age(time, have) <= feedback_timeout_sec_;
  }

  bool hardStopRequired()
  {
    hard_stop_reason_.clear();
    if (require_health_ && !feedbackFresh(health_time_, have_health_)) {
      hard_stop_reason_ = "perception_health_stale";
    } else if (require_health_ && have_health_ &&
      (health_.state == health_.UNSAFE || health_.state == health_.STALE)) {
      hard_stop_reason_ = health_.reason;
    } else if (require_safety_feedback_ && !feedbackFresh(safety_time_, have_safety_)) {
      hard_stop_reason_ = "safety_feedback_stale";
    } else if (have_safety_ && safetyCritical(safety_)) {
      hard_stop_reason_ = safety_.reason;
    }
    return !hard_stop_reason_.empty();
  }

  bool safetyCritical(const track_robot_interfaces::msg::SafetyState & msg) const
  {
    return msg.state == msg.STATE_SENSOR_STALE || msg.state == msg.STATE_RC_OVERRIDE ||
      msg.state == msg.STATE_BASE_FAULT || msg.state == msg.STATE_EMERGENCY_STOP;
  }

  bool blockedRequired()
  {
    if (require_avoidance_feedback_ && !feedbackFresh(avoidance_time_, have_avoidance_)) {
      blocked_latched_ = true;
      blocked_reason_ = "avoidance_feedback_stale";
    } else if (have_safety_ && safety_.state == safety_.STATE_BLOCKED) {
      blocked_latched_ = true;
      blocked_reason_ = safety_.reason;
    } else if (blocked_latched_ && blocked_reason_.empty()) {
      blocked_reason_ = "no_safe_trajectory";
    }
    return blocked_latched_;
  }

  bool confirmedReady() const
  {
    return age(target_time_, have_target_) <= target_timeout_sec_ && rawConfirmed(target_) &&
      confirmed_ticks_ >= confirmation_ticks_ &&
      (!require_health_ || (have_health_ && health_.lidar_usable));
  }

  bool lidarReady() const
  {
    return age(target_time_, have_target_) <= target_timeout_sec_ && rawLidar(target_) &&
      lidar_ticks_ >= confirmation_ticks_ && lidar_only_active_ &&
      age(lidar_only_start_, true) <= lidar_only_timeout_sec_ &&
      (!require_health_ || (have_health_ && health_.lidar_usable));
  }

  bool uncertainTarget() const
  {
    if (!hasLogicalTarget()) return false;
    if (confirmedReady() || lidarReady()) return false;
    return have_reliable_target_ && age(last_reliable_time_, true) <= uncertain_hold_sec_;
  }

  bool hasLogicalTarget() const
  {
    return (have_target_ && target_.target_id >= 0) || have_reliable_target_;
  }

  bool searchAllowed()
  {
    if (!have_reliable_target_) return false;
    if (!search_active_) {
      if (age(last_reliable_time_, true) > search_entry_max_age_sec_) return false;
      search_active_ = true;
      search_start_ = steadyNow();
      search_center_world_yaw_ = health_.odometry_yaw + last_reliable_target_.bearing;
    }
    if (age(search_start_, true) > search_timeout_sec_) {
      search_active_ = false;
      return false;
    }
    return !have_safety_ ||
      (safety_.state != safety_.STATE_BLOCKED && safety_.state != safety_.STATE_EMERGENCY_STOP);
  }

  void setBaseDecision(const uint8_t behavior, const std::string & reason)
  {
    decision_ = track_robot_interfaces::msg::FollowDecision();
    decision_.header.stamp = now();
    decision_.header.frame_id = "base_link";
    decision_.behavior = behavior;
    decision_.logical_target_id = hasLogicalTarget() ?
      (have_target_ && target_.target_id >= 0 ? target_.target_id : last_reliable_target_.target_id) : -1;
    const auto & source = have_target_ ? target_ : last_reliable_target_;
    decision_.selected_tracklet_id = source.selected_tracklet_id;
    decision_.target_distance = source.distance;
    decision_.target_bearing = source.bearing;
    decision_.target_position = source.position_base;
    decision_.target_velocity = source.velocity;
    decision_.identity_confidence = source.identity_confidence;
    decision_.geometry_confidence = source.geometry_confidence;
    decision_.overall_confidence = source.overall_confidence;
    decision_.horizontal_position_sigma = targetSigma(source);
    decision_.measurement_age = source.measurement_age;
    decision_.camera_age = source.time_since_camera_seen;
    decision_.lidar_age = source.time_since_lidar_seen;
    decision_.target_source = sourceName(source.source_state);
    decision_.reason = reason;
  }

  std::string sourceName(const uint8_t source) const
  {
    if (source == track_robot_interfaces::msg::TargetState::SOURCE_CAMERA_LIDAR) return "camera_lidar";
    if (source == track_robot_interfaces::msg::TargetState::SOURCE_LIDAR_ONLY) return "lidar_only";
    if (source == track_robot_interfaces::msg::TargetState::SOURCE_CAMERA_ONLY) return "camera_only";
    if (source == track_robot_interfaces::msg::TargetState::SOURCE_PREDICTION_ONLY) return "prediction_only";
    return "none";
  }

  BT::NodeStatus setHardStop()
  {
    const bool rc = have_safety_ && safety_.state == safety_.STATE_RC_OVERRIDE;
    setBaseDecision(rc ? decision_.BEHAVIOR_RC_OVERRIDE : decision_.BEHAVIOR_FAULT_HOLD,
      hard_stop_reason_);
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus setBlocked()
  {
    setBaseDecision(decision_.BEHAVIOR_BLOCKED_HOLD, blocked_reason_);
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus setConfirmed()
  {
    setBaseDecision(decision_.BEHAVIOR_FOLLOW_CONFIRMED, "confirmed_camera_lidar");
    decision_.motion_permitted = true;
    decision_.forward_permitted = true;
    decision_.rotation_permitted = true;
    decision_.automatic_resume_permitted = true;
    const double uncertainty_scale = clamp(
      1.0 - 0.5 * decision_.horizontal_position_sigma / confirmed_max_sigma_, 0.35, 1.0);
    const double health_scale = require_health_ && have_health_ &&
      health_.state == health_.DEGRADED ? 0.5 : 1.0;
    decision_.maximum_linear_speed = confirmed_max_linear_ * uncertainty_scale * health_scale;
    decision_.maximum_angular_speed = confirmed_max_angular_;
    decision_.decision_confidence = target_.overall_confidence;
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus setLidar()
  {
    setBaseDecision(decision_.BEHAVIOR_FOLLOW_LIDAR_LIMITED, "incumbent_lidar_tracklet");
    decision_.motion_permitted = true;
    decision_.forward_permitted = true;
    decision_.rotation_permitted = true;
    decision_.automatic_resume_permitted = true;
    decision_.maximum_linear_speed = lidar_max_linear_;
    decision_.maximum_angular_speed = lidar_max_angular_;
    decision_.decision_confidence = target_.overall_confidence;
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus setUncertain()
  {
    setBaseDecision(decision_.BEHAVIOR_UNCERTAIN_HOLD, "target_evidence_uncertain");
    decision_.decision_confidence = target_.overall_confidence;
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus setSearch()
  {
    setBaseDecision(decision_.BEHAVIOR_SEARCH_ROTATE, "bounded_last_bearing_search");
    decision_.motion_permitted = true;
    decision_.rotation_permitted = true;
    decision_.maximum_angular_speed = search_max_angular_;
    const double elapsed = age(search_start_, true);
    double offset = 0.0;
    if (elapsed >= 1.0) {
      const int phase = static_cast<int>((elapsed - 1.0) / 1.5);
      offset = (phase % 2 == 0 ? 0.5 : -0.5) * search_sector_rad_;
    }
    decision_.search_target_bearing = normalizeAngle(
      search_center_world_yaw_ + offset - health_.odometry_yaw);
    decision_.decision_confidence = clamp(1.0 - elapsed / search_timeout_sec_, 0.0, 1.0);
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus setLost()
  {
    setBaseDecision(decision_.BEHAVIOR_TARGET_LOST, "search_exhausted_or_identity_lost");
    return BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus setWaiting()
  {
    setBaseDecision(decision_.BEHAVIOR_WAITING_FOR_TARGET, "waiting_for_gesture_target");
    return BT::NodeStatus::SUCCESS;
  }

  void registerTreeNodes()
  {
    factory_.registerSimpleCondition("HardStopRequired", [this](BT::TreeNode &) {
      return hardStopRequired() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    factory_.registerSimpleCondition("BlockedRequired", [this](BT::TreeNode &) {
      return blockedRequired() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    factory_.registerSimpleCondition("ConfirmedFollowReady", [this](BT::TreeNode &) {
      return confirmedReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    factory_.registerSimpleCondition("LidarFollowReady", [this](BT::TreeNode &) {
      return lidarReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    factory_.registerSimpleCondition("UncertainTarget", [this](BT::TreeNode &) {
      return uncertainTarget() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    factory_.registerSimpleCondition("SearchAllowed", [this](BT::TreeNode &) {
      return searchAllowed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    factory_.registerSimpleCondition("HasLogicalTarget", [this](BT::TreeNode &) {
      return hasLogicalTarget() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;});
    factory_.registerSimpleAction("SetHardStop", [this](BT::TreeNode &) {return setHardStop();});
    factory_.registerSimpleAction("SetBlockedHold", [this](BT::TreeNode &) {return setBlocked();});
    factory_.registerSimpleAction("SetConfirmedFollow", [this](BT::TreeNode &) {return setConfirmed();});
    factory_.registerSimpleAction("SetLidarFollow", [this](BT::TreeNode &) {return setLidar();});
    factory_.registerSimpleAction("SetUncertainHold", [this](BT::TreeNode &) {return setUncertain();});
    factory_.registerSimpleAction("SetSearchRotate", [this](BT::TreeNode &) {return setSearch();});
    factory_.registerSimpleAction("SetTargetLost", [this](BT::TreeNode &) {return setLost();});
    factory_.registerSimpleAction("SetWaitingForTarget", [this](BT::TreeNode &) {return setWaiting();});
  }

  void tick()
  {
    blocked_reason_.clear();
    tree_.tickRoot();
    decision_pub_->publish(decision_);
    publishDebug();
    publishMarkers();
    if (decision_.behavior != last_behavior_) {
      if (decision_.behavior == decision_.BEHAVIOR_TARGET_LOST ||
        decision_.behavior == decision_.BEHAVIOR_RC_OVERRIDE)
      {
        queueTargetReset(
          decision_.behavior == decision_.BEHAVIOR_RC_OVERRIDE ? "rc_override" : "target_lost");
      }
      RCLCPP_INFO(get_logger(), "Decision %u -> %u: %s", last_behavior_, decision_.behavior,
        decision_.reason.c_str());
      last_behavior_ = decision_.behavior;
    }
    attemptPendingTargetReset();
  }

  void queueTargetReset(const std::string & reason)
  {
    reset_target_pending_ = true;
    reset_target_reason_ = reason;
    next_reset_target_attempt_time_ = steadyNow();
  }

  void attemptPendingTargetReset()
  {
    if (!reset_target_pending_ || reset_target_request_in_flight_ ||
      steadyNow() < next_reset_target_attempt_time_)
    {
      return;
    }
    if (!reset_target_client_->service_is_ready()) {
      next_reset_target_attempt_time_ = steadyNow() + std::chrono::milliseconds(200);
      return;
    }

    reset_target_request_in_flight_ = true;
    reset_target_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        reset_target_request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (response->success) {
            reset_target_pending_ = false;
            RCLCPP_WARN(get_logger(), "%s; reset logical target", reset_target_reason_.c_str());
            return;
          }
          RCLCPP_WARN(get_logger(), "%s; reset target failed; retrying",
            reset_target_reason_.c_str());
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "%s; reset target request failed: %s; retrying",
            reset_target_reason_.c_str(), error.what());
        }
        next_reset_target_attempt_time_ = steadyNow() + std::chrono::milliseconds(200);
      });
  }

  void publishDebug()
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"behavior\":" << static_cast<int>(decision_.behavior)
        << ",\"reason\":\"" << decision_.reason << "\""
        << ",\"target_id\":" << decision_.logical_target_id
        << ",\"confirmed_ticks\":" << confirmed_ticks_
        << ",\"lidar_ticks\":" << lidar_ticks_
        << ",\"blocked_clear_ticks\":" << blocked_clear_ticks_
        << ",\"target_age\":" << age(target_time_, have_target_)
        << ",\"health_age\":" << age(health_time_, have_health_)
        << ",\"sigma\":" << decision_.horizontal_position_sigma
        << ",\"max_linear\":" << decision_.maximum_linear_speed
        << ",\"max_angular\":" << decision_.maximum_angular_speed
        << ",\"search_elapsed\":" << (search_active_ ? age(search_start_, true) : -1.0)
        << "}";
    msg.data = out.str();
    debug_pub_->publish(msg);
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker arrow;
    arrow.header = decision_.header;
    arrow.ns = "follow_decision";
    arrow.id = 0;
    arrow.type = arrow.ARROW;
    arrow.action = arrow.ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.12;
    arrow.scale.z = 0.12;
    arrow.color = markerColor();
    arrow.lifetime = rclcpp::Duration::from_seconds(0.15);
    geometry_msgs::msg::Point start;
    start.z = 0.25;
    geometry_msgs::msg::Point end = start;
    const double angle = decision_.behavior == decision_.BEHAVIOR_SEARCH_ROTATE ?
      decision_.search_target_bearing : decision_.target_bearing;
    const double length = decision_.motion_permitted ?
      0.3 + 2.0 * decision_.maximum_linear_speed : 0.2;
    end.x = length * std::cos(angle);
    end.y = length * std::sin(angle);
    arrow.points = {start, end};
    array.markers.push_back(arrow);

    visualization_msgs::msg::Marker text;
    text.header = decision_.header;
    text.ns = "follow_decision";
    text.id = 1;
    text.type = text.TEXT_VIEW_FACING;
    text.action = text.ADD;
    text.pose.position.z = 0.9;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    text.color = markerColor();
    text.lifetime = rclcpp::Duration::from_seconds(0.15);
    std::ostringstream label;
    label << behaviorName(decision_.behavior) << "\n" << decision_.reason
          << "\nid=" << decision_.logical_target_id
          << " vlim=" << decision_.maximum_linear_speed;
    text.text = label.str();
    array.markers.push_back(text);

    visualization_msgs::msg::Marker uncertainty;
    uncertainty.header = decision_.header;
    uncertainty.ns = "follow_decision_uncertainty";
    uncertainty.id = 2;
    uncertainty.type = uncertainty.LINE_STRIP;
    uncertainty.action = uncertainty.ADD;
    uncertainty.pose.orientation.w = 1.0;
    uncertainty.scale.x = 0.035;
    uncertainty.color = markerColor();
    uncertainty.color.a = 0.55F;
    uncertainty.lifetime = rclcpp::Duration::from_seconds(0.15);
    const double sigma_radius = clamp(
      2.0 * static_cast<double>(decision_.horizontal_position_sigma), 0.10, 1.20);
    for (int index = 0; index <= 48; ++index) {
      const double phase = 2.0 * M_PI * index / 48.0;
      geometry_msgs::msg::Point point;
      point.x = decision_.target_position.x + sigma_radius * std::cos(phase);
      point.y = decision_.target_position.y + sigma_radius * std::sin(phase);
      point.z = 0.08;
      uncertainty.points.push_back(point);
    }
    if (decision_.logical_target_id >= 0) array.markers.push_back(uncertainty);

    visualization_msgs::msg::Marker sector;
    sector.header = decision_.header;
    sector.ns = "follow_decision_search_sector";
    sector.id = 3;
    sector.type = sector.LINE_STRIP;
    sector.action = sector.ADD;
    sector.pose.orientation.w = 1.0;
    sector.scale.x = 0.04;
    sector.color.r = 1.0F;
    sector.color.g = 0.75F;
    sector.color.a = 0.75F;
    sector.lifetime = rclcpp::Duration::from_seconds(0.15);
    geometry_msgs::msg::Point origin;
    origin.z = 0.12;
    sector.points.push_back(origin);
    const double center_bearing = normalizeAngle(
      search_center_world_yaw_ - static_cast<double>(health_.odometry_yaw));
    for (int index = 0; index <= 32; ++index) {
      const double phase = center_bearing - 0.5 * search_sector_rad_ +
        search_sector_rad_ * index / 32.0;
      geometry_msgs::msg::Point point;
      point.x = 1.5 * std::cos(phase);
      point.y = 1.5 * std::sin(phase);
      point.z = 0.12;
      sector.points.push_back(point);
    }
    sector.points.push_back(origin);
    if (decision_.behavior == decision_.BEHAVIOR_SEARCH_ROTATE) array.markers.push_back(sector);
    marker_pub_->publish(array);
  }

  std_msgs::msg::ColorRGBA markerColor() const
  {
    std_msgs::msg::ColorRGBA color;
    color.a = 0.9F;
    if (decision_.behavior == decision_.BEHAVIOR_FOLLOW_CONFIRMED) {color.g = 1.0F;}
    else if (decision_.behavior == decision_.BEHAVIOR_FOLLOW_LIDAR_LIMITED) {color.b = 1.0F; color.g = 0.4F;}
    else if (decision_.behavior == decision_.BEHAVIOR_UNCERTAIN_HOLD ||
      decision_.behavior == decision_.BEHAVIOR_SEARCH_ROTATE) {color.r = 1.0F; color.g = 0.75F;}
    else if (decision_.behavior == decision_.BEHAVIOR_BLOCKED_HOLD) {color.r = 1.0F; color.g = 0.35F;}
    else {color.r = 1.0F; color.g = 0.1F; color.b = 0.1F;}
    return color;
  }

  std::string behaviorName(const uint8_t behavior) const
  {
    if (behavior == decision_.BEHAVIOR_FOLLOW_CONFIRMED) return "FOLLOW_CONFIRMED";
    if (behavior == decision_.BEHAVIOR_FOLLOW_LIDAR_LIMITED) return "FOLLOW_LIDAR_LIMITED";
    if (behavior == decision_.BEHAVIOR_UNCERTAIN_HOLD) return "UNCERTAIN_HOLD";
    if (behavior == decision_.BEHAVIOR_SEARCH_ROTATE) return "SEARCH_ROTATE";
    if (behavior == decision_.BEHAVIOR_BLOCKED_HOLD) return "BLOCKED_HOLD";
    if (behavior == decision_.BEHAVIOR_FAULT_HOLD) return "FAULT_HOLD";
    if (behavior == decision_.BEHAVIOR_TARGET_LOST) return "TARGET_LOST";
    if (behavior == decision_.BEHAVIOR_RC_OVERRIDE) return "RC_OVERRIDE";
    return "WAITING_FOR_TARGET";
  }

  std::string target_topic_, health_topic_, avoidance_topic_, safety_topic_;
  std::string decision_topic_, debug_topic_, marker_topic_, tree_xml_, reset_target_service_;
  double tick_rate_, target_timeout_sec_, feedback_timeout_sec_;
  double confirmed_identity_confidence_, confirmed_geometry_confidence_;
  double confirmed_overall_confidence_, confirmed_max_sigma_, confirmed_max_measurement_age_;
  double lidar_geometry_confidence_, lidar_overall_confidence_, lidar_max_sigma_, lidar_max_age_;
  double lidar_only_timeout_sec_, uncertain_hold_sec_, search_entry_max_age_sec_;
  double search_timeout_sec_, search_sector_rad_, confirmed_max_linear_, confirmed_max_angular_;
  double lidar_max_linear_, lidar_max_angular_, search_max_angular_;
  int confirmation_ticks_, blocked_clear_ticks_required_;
  bool require_health_, require_avoidance_feedback_, require_safety_feedback_;
  bool have_target_{false}, have_health_{false}, have_avoidance_{false}, have_safety_{false};
  bool have_reliable_target_{false}, lidar_only_active_{false}, search_active_{false};
  bool blocked_latched_{false};
  bool reset_target_pending_{false}, reset_target_request_in_flight_{false};
  int confirmed_ticks_{0}, lidar_ticks_{0}, blocked_clear_ticks_{0}, last_candidate_id_{-1};
  uint8_t last_behavior_{255};
  double search_center_world_yaw_{0.0};
  std::string hard_stop_reason_, blocked_reason_, reset_target_reason_;
  SteadyTime target_time_, health_time_, avoidance_time_, safety_time_;
  SteadyTime last_reliable_time_, lidar_only_start_, search_start_;
  SteadyTime next_reset_target_attempt_time_;
  track_robot_interfaces::msg::TargetState target_, last_reliable_target_;
  track_robot_interfaces::msg::PerceptionHealth health_;
  track_robot_interfaces::msg::AvoidanceState avoidance_;
  track_robot_interfaces::msg::SafetyState safety_;
  track_robot_interfaces::msg::FollowDecision decision_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::Subscription<track_robot_interfaces::msg::TargetState>::SharedPtr target_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::PerceptionHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::AvoidanceState>::SharedPtr avoidance_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Publisher<track_robot_interfaces::msg::FollowDecision>::SharedPtr decision_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_target_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FollowBehaviorTreeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("follow_behavior_tree_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
