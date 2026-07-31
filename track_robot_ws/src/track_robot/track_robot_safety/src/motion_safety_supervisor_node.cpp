#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "bunker_msgs/msg/bunker_rc_state.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "track_robot_safety/command_freshness.hpp"
#include "track_robot_safety/rotation_collision.hpp"
#include "track_robot_interfaces/msg/avoidance_state.hpp"
#include "track_robot_interfaces/msg/safety_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {

double clampValue(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

struct Point2f
{
  float x{0.0F};
  float y{0.0F};
};

struct CollisionResult
{
  double closest_obstacle{std::numeric_limits<double>::infinity()};
  double collision_path_distance{std::numeric_limits<double>::infinity()};
  double time_to_collision{std::numeric_limits<double>::infinity()};
  bool collision_predicted{false};
};

struct SafetyDecision
{
  uint8_t state{track_robot_interfaces::msg::SafetyState::STATE_DISARMED};
  std::string reason{"disarmed"};
  geometry_msgs::msg::Twist planned;
  geometry_msgs::msg::Twist safe;
  CollisionResult collision;
  double stopping_distance{0.0};
};

}  // namespace

class MotionSafetySupervisorNode : public rclcpp::Node
{
public:
  MotionSafetySupervisorNode()
  : Node("motion_safety_supervisor_node")
  {
    planned_cmd_topic_ = declare_parameter<std::string>(
      "planned_cmd_topic", "/follow/cmd_vel_avoiding");
    planner_state_topic_ = declare_parameter<std::string>(
      "planner_state_topic", "/follow/avoidance_state");
    obstacle_cloud_topic_ = declare_parameter<std::string>(
      "obstacle_cloud_topic", "/safety/filtered_obstacle_points");
    bunker_status_topic_ = declare_parameter<std::string>(
      "bunker_status_topic", "/bunker_status");
    rc_state_topic_ = declare_parameter<std::string>("rc_state_topic", "/bunker_rc_state");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    safe_cmd_topic_ = declare_parameter<std::string>(
      "safe_cmd_topic", "/follow/cmd_vel_safe");
    safety_state_topic_ = declare_parameter<std::string>(
      "safety_state_topic", "/safety/state");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/safety/controller_debug");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/safety/collision_envelope_markers");
    marker_frame_ = declare_parameter<std::string>("marker_frame", "base_link");

    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    marker_publish_rate_ = declare_parameter<double>("marker_publish_rate", 10.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.15);
    cloud_timeout_sec_ = declare_parameter<double>("cloud_timeout_sec", 0.25);
    base_status_timeout_sec_ = declare_parameter<double>("base_status_timeout_sec", 0.20);
    rc_timeout_sec_ = declare_parameter<double>("rc_timeout_sec", 0.25);
    require_bunker_status_ = declare_parameter<bool>("require_bunker_status", true);
    require_rc_state_ = declare_parameter<bool>("require_rc_state", true);
    require_can_control_mode_ = declare_parameter<bool>("require_can_control_mode", true);
    require_planner_state_ = declare_parameter<bool>("require_planner_state", true);
    planner_state_timeout_sec_ = declare_parameter<double>("planner_state_timeout_sec", 0.25);
    require_odom_ = declare_parameter<bool>("require_odom", false);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.25);
    allow_arm_without_command_ = declare_parameter<bool>(
      "allow_arm_without_command", false);

    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.15);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.35);
    braking_deceleration_ = declare_parameter<double>("braking_deceleration", 0.25);
    angular_braking_deceleration_ = declare_parameter<double>(
      "angular_braking_deceleration", 0.80);
    response_latency_sec_ = declare_parameter<double>("response_latency_sec", 0.25);
    fixed_stop_margin_ = declare_parameter<double>("fixed_stop_margin", 0.45);
    fixed_rotation_margin_ = declare_parameter<double>("fixed_rotation_margin", 0.05);
    bounded_rotation_collision_enabled_ = declare_parameter<bool>(
      "bounded_rotation_collision_enabled", false);
    slowdown_path_distance_ = declare_parameter<double>("slowdown_path_distance", 1.0);
    trajectory_step_sec_ = declare_parameter<double>("trajectory_step_sec", 0.05);
    max_lookahead_distance_ = declare_parameter<double>("max_lookahead_distance", 1.5);
    max_lookahead_time_sec_ = declare_parameter<double>("max_lookahead_time_sec", 10.0);
    footprint_length_ = declare_parameter<double>("footprint_length", 1.20);
    footprint_width_ = declare_parameter<double>("footprint_width", 1.00);
    safety_inflation_ = declare_parameter<double>("safety_inflation", 0.20);
    rc_override_deadband_ = declare_parameter<int>("rc_override_deadband", 10);

    arm_service_name_ = declare_parameter<std::string>("arm_service", "/safety/arm");
    disarm_service_name_ = declare_parameter<std::string>("disarm_service", "/safety/disarm");
    emergency_stop_service_name_ = declare_parameter<std::string>(
      "emergency_stop_service", "/safety/emergency_stop");
    reset_emergency_stop_service_name_ = declare_parameter<std::string>(
      "reset_emergency_stop_service", "/safety/reset_emergency_stop");

    planned_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      planned_cmd_topic_, 10,
      std::bind(&MotionSafetySupervisorNode::plannedCmdCallback, this, std::placeholders::_1));
    planner_state_sub_ = create_subscription<track_robot_interfaces::msg::AvoidanceState>(
      planner_state_topic_, 10,
      std::bind(&MotionSafetySupervisorNode::plannerStateCallback, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      std::bind(&MotionSafetySupervisorNode::obstacleCloudCallback, this, std::placeholders::_1));
    bunker_status_sub_ = create_subscription<bunker_msgs::msg::BunkerStatus>(
      bunker_status_topic_, 10,
      std::bind(&MotionSafetySupervisorNode::bunkerStatusCallback, this, std::placeholders::_1));
    rc_state_sub_ = create_subscription<bunker_msgs::msg::BunkerRCState>(
      rc_state_topic_, 10,
      std::bind(&MotionSafetySupervisorNode::rcStateCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&MotionSafetySupervisorNode::odomCallback, this, std::placeholders::_1));

    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, 10);
    safety_state_pub_ = create_publisher<track_robot_interfaces::msg::SafetyState>(
      safety_state_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    arm_srv_ = create_service<std_srvs::srv::Trigger>(
      arm_service_name_, std::bind(
        &MotionSafetySupervisorNode::armCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
      disarm_service_name_, std::bind(
        &MotionSafetySupervisorNode::disarmCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    emergency_stop_srv_ = create_service<std_srvs::srv::Trigger>(
      emergency_stop_service_name_, std::bind(
        &MotionSafetySupervisorNode::emergencyStopCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_emergency_stop_srv_ = create_service<std_srvs::srv::Trigger>(
      reset_emergency_stop_service_name_, std::bind(
        &MotionSafetySupervisorNode::resetEmergencyStopCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MotionSafetySupervisorNode::timerCallback, this));

    RCLCPP_WARN(
      get_logger(),
      "Motion safety supervisor started DISARMED. planned=%s safe=%s rate=%.1fHz",
      planned_cmd_topic_.c_str(), safe_cmd_topic_.c_str(), publish_rate_);
  }

  ~MotionSafetySupervisorNode() override
  {
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < 3; ++i) {
      safe_cmd_pub_->publish(zero);
    }
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void plannedCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_planned_cmd_ = *msg;
    last_command_time_ = std::chrono::steady_clock::now();
    have_command_ = true;
    if (armed_) {
      waiting_for_first_command_after_arm_ = false;
    }
  }

  void obstacleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<Point2f> parsed;
    if (!readObstacleCloud(*msg, parsed)) {
      return;
    }
    obstacle_points_ = std::move(parsed);
    last_cloud_time_ = std::chrono::steady_clock::now();
    have_cloud_ = true;
  }

  void plannerStateCallback(const track_robot_interfaces::msg::AvoidanceState::SharedPtr msg)
  {
    latest_planner_state_ = *msg;
    last_planner_state_time_ = std::chrono::steady_clock::now();
    have_planner_state_ = true;
  }

  void bunkerStatusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    latest_bunker_status_ = *msg;
    last_base_status_time_ = std::chrono::steady_clock::now();
    have_base_status_ = true;
    if (msg->vehicle_state != 0U || msg->error_code != 0U) {
      armed_ = false;
    }
  }

  void rcStateCallback(const bunker_msgs::msg::BunkerRCState::SharedPtr msg)
  {
    latest_rc_state_ = *msg;
    last_rc_time_ = std::chrono::steady_clock::now();
    have_rc_state_ = true;
    rc_override_active_ = rcOverrideActive(*msg);
    if (rc_override_active_) {
      armed_ = false;
      rc_override_latched_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr)
  {
    last_odom_time_ = std::chrono::steady_clock::now();
    have_odom_ = true;
  }

  void timerCallback()
  {
    const SafetyDecision decision = makeDecision();
    safe_cmd_pub_->publish(decision.safe);
    publishSafetyState(decision);
    publishDebug(decision);
    ++timer_count_;
    const int marker_divisor = std::max(
      1, static_cast<int>(std::round(publish_rate_ / std::max(1.0, marker_publish_rate_))));
    if (timer_count_ % marker_divisor == 0) {
      publishMarkers(decision);
    }
  }

  SafetyDecision makeDecision() const
  {
    SafetyDecision decision;
    decision.planned = limitedCommand(latest_planned_cmd_);
    decision.stopping_distance = stoppingDistance(std::abs(decision.planned.linear.x));

    if (emergency_stop_latched_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_EMERGENCY_STOP;
      decision.reason = "software_emergency_stop_latched";
      return decision;
    }
    if (rc_override_active_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_RC_OVERRIDE;
      decision.reason = "rc_stick_override";
      return decision;
    }
    if (!armed_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_DISARMED;
      decision.reason = rc_override_latched_ ? "disarmed_after_rc_override" : "disarmed";
      return decision;
    }

    const double command_age = ageSeconds(last_command_time_, have_command_);
    const double cloud_age = ageSeconds(last_cloud_time_, have_cloud_);
    const double base_age = ageSeconds(last_base_status_time_, have_base_status_);
    const double rc_age = ageSeconds(last_rc_time_, have_rc_state_);
    const double planner_age = ageSeconds(last_planner_state_time_, have_planner_state_);
    const double odom_age = ageSeconds(last_odom_time_, have_odom_);
    const auto command_freshness = track_robot_safety::classifyCommandFreshness(
      waiting_for_first_command_after_arm_, have_command_, command_age, command_timeout_sec_,
      allow_arm_without_command_ &&
      std::abs(decision.planned.linear.x) < 1e-4 &&
      std::abs(decision.planned.angular.z) < 1e-4);
    if (cloud_age > cloud_timeout_sec_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SENSOR_STALE;
      decision.reason = "obstacle_cloud_stale";
      return decision;
    }
    if (require_bunker_status_ && base_age > base_status_timeout_sec_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SENSOR_STALE;
      decision.reason = "bunker_status_stale";
      return decision;
    }
    if (require_rc_state_ && rc_age > rc_timeout_sec_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SENSOR_STALE;
      decision.reason = "rc_state_stale";
      return decision;
    }
    if (require_planner_state_ && planner_age > planner_state_timeout_sec_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SENSOR_STALE;
      decision.reason = "avoidance_planner_state_stale";
      return decision;
    }
    if (require_odom_ && odom_age > odom_timeout_sec_) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SENSOR_STALE;
      decision.reason = "odometry_stale";
      return decision;
    }
    if (require_planner_state_ &&
      (latest_planner_state_.state ==
      track_robot_interfaces::msg::AvoidanceState::STATE_WAITING_FOR_DATA ||
      latest_planner_state_.state == track_robot_interfaces::msg::AvoidanceState::STATE_STALE))
    {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SENSOR_STALE;
      decision.reason = "avoidance_planner_not_ready";
      return decision;
    }
    if (require_planner_state_ && latest_planner_state_.state ==
      track_robot_interfaces::msg::AvoidanceState::STATE_NO_SAFE_TRAJECTORY)
    {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_BLOCKED;
      decision.reason = "avoidance_planner_no_safe_trajectory";
      return decision;
    }
    if (require_bunker_status_ && !baseStatusOk()) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_BASE_FAULT;
      decision.reason = baseFaultReason();
      return decision;
    }
    if (command_freshness == track_robot_safety::CommandFreshness::STALE) {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SENSOR_STALE;
      decision.reason = "planned_command_stale";
      return decision;
    }

    decision.collision = evaluateCollision(decision.planned);
    if (decision.collision.collision_predicted &&
      decision.collision.collision_path_distance <= decision.stopping_distance)
    {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_BLOCKED;
      decision.reason = "obstacle_inside_stopping_envelope";
      return decision;
    }
    if (
      command_freshness ==
      track_robot_safety::CommandFreshness::WAITING_FOR_FIRST_COMMAND)
    {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_CLEAR;
      decision.reason = "armed_idle_zero_command";
      return decision;
    }

    decision.safe = decision.planned;
    if (decision.collision.collision_predicted &&
      decision.collision.collision_path_distance < slowdown_path_distance_)
    {
      const double denominator = std::max(
        0.05, slowdown_path_distance_ - decision.stopping_distance);
      const double scale = clampValue(
        (decision.collision.collision_path_distance - decision.stopping_distance) /
        denominator, 0.0, 1.0);
      decision.safe.linear.x *= scale;
      decision.safe.angular.z *= scale;
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_SLOWDOWN;
      decision.reason = "obstacle_in_slowdown_envelope";
      return decision;
    }

    if (require_planner_state_ && latest_planner_state_.state ==
      track_robot_interfaces::msg::AvoidanceState::STATE_AVOIDING)
    {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_AVOIDING;
      decision.reason = "avoidance_trajectory_clear";
    } else {
      decision.state = track_robot_interfaces::msg::SafetyState::STATE_CLEAR;
      decision.reason = "clear";
    }
    return decision;
  }

  CollisionResult evaluateCollision(const geometry_msgs::msg::Twist & command) const
  {
    CollisionResult result;
    const double physical_half_length = footprint_length_ * 0.5;
    const double physical_half_width = footprint_width_ * 0.5;
    const double half_length = physical_half_length + safety_inflation_;
    const double half_width = physical_half_width + safety_inflation_;
    for (const auto & point : obstacle_points_) {
      const double dx = std::max(std::abs(static_cast<double>(point.x)) -
        physical_half_length, 0.0);
      const double dy = std::max(std::abs(static_cast<double>(point.y)) -
        physical_half_width, 0.0);
      result.closest_obstacle = std::min(result.closest_obstacle, std::hypot(dx, dy));
      if (insideRectangle(point.x, point.y, half_length, half_width)) {
        result.collision_predicted = true;
        result.collision_path_distance = 0.0;
        result.time_to_collision = 0.0;
        return result;
      }
    }

    const double linear = command.linear.x;
    const double angular = command.angular.z;
    if (std::abs(linear) < 1e-4 && std::abs(angular) < 1e-4) {
      return result;
    }

    if (std::abs(linear) < 0.02 && std::abs(angular) >= 1e-4) {
      if (bounded_rotation_collision_enabled_) {
        const double sample_angle = std::max(
          std::abs(angular) * std::max(trajectory_step_sec_, 0.02), 0.005);
        const auto rotation = track_robot_safety::evaluateRotationCollision(
          obstacle_points_, angular, half_length, half_width,
          angular_braking_deceleration_, response_latency_sec_,
          fixed_rotation_margin_, sample_angle);
        if (rotation.collision) {
          result.collision_predicted = true;
          result.collision_path_distance = 0.0;
          result.time_to_collision = rotation.time_to_collision;
        }
        return result;
      }
      const double swept_radius = std::hypot(half_length, half_width);
      for (const auto & point : obstacle_points_) {
        if (std::hypot(static_cast<double>(point.x), static_cast<double>(point.y)) <=
          swept_radius)
        {
          result.collision_predicted = true;
          result.collision_path_distance = 0.0;
          result.time_to_collision = 0.0;
          return result;
        }
      }
      return result;
    }

    const double speed = std::max(std::abs(linear), 0.02);
    const double horizon = std::min(
      max_lookahead_time_sec_, std::max(1.0, max_lookahead_distance_ / speed));
    const double dt = std::max(trajectory_step_sec_, 0.02);
    for (double time = dt; time <= horizon; time += dt) {
      double robot_x = linear * time;
      double robot_y = 0.0;
      double yaw = angular * time;
      if (std::abs(angular) >= 1e-5) {
        const double radius = linear / angular;
        robot_x = radius * std::sin(yaw);
        robot_y = radius * (1.0 - std::cos(yaw));
      }
      const double cosine = std::cos(yaw);
      const double sine = std::sin(yaw);
      for (const auto & point : obstacle_points_) {
        const double dx = static_cast<double>(point.x) - robot_x;
        const double dy = static_cast<double>(point.y) - robot_y;
        const double local_x = cosine * dx + sine * dy;
        const double local_y = -sine * dx + cosine * dy;
        if (insideRectangle(local_x, local_y, half_length, half_width)) {
          result.collision_predicted = true;
          result.collision_path_distance = std::abs(linear) * time;
          result.time_to_collision = time;
          return result;
        }
      }
    }
    return result;
  }

  static bool insideRectangle(
    const double x, const double y, const double half_length, const double half_width)
  {
    return std::abs(x) <= half_length && std::abs(y) <= half_width;
  }

  double stoppingDistance(const double speed) const
  {
    const double deceleration = std::max(braking_deceleration_, 0.05);
    return speed * speed / (2.0 * deceleration) +
      speed * std::max(0.0, response_latency_sec_) + std::max(0.0, fixed_stop_margin_);
  }

  geometry_msgs::msg::Twist limitedCommand(const geometry_msgs::msg::Twist & input) const
  {
    geometry_msgs::msg::Twist output;
    output.linear.x = clampValue(input.linear.x, 0.0, max_linear_x_);
    output.angular.z = clampValue(input.angular.z, -max_angular_z_, max_angular_z_);
    return output;
  }

  bool readObstacleCloud(
    const sensor_msgs::msg::PointCloud2 & cloud, std::vector<Point2f> & points)
  {
    if (!cloud.header.frame_id.empty() && cloud.header.frame_id != marker_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting obstacle cloud in frame %s; expected %s",
        cloud.header.frame_id.c_str(), marker_frame_.c_str());
      return false;
    }
    int x_offset = -1;
    int y_offset = -1;
    for (const auto & field : cloud.fields) {
      if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        y_offset = static_cast<int>(field.offset);
      }
    }
    if (x_offset < 0 || y_offset < 0 || cloud.is_bigendian) {
      return false;
    }
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    for (uint32_t row = 0; row < cloud.height; ++row) {
      for (uint32_t col = 0; col < cloud.width; ++col) {
        const std::size_t offset = static_cast<std::size_t>(row) * cloud.row_step +
          static_cast<std::size_t>(col) * cloud.point_step;
        if (offset + cloud.point_step > cloud.data.size()) {
          continue;
        }
        Point2f point;
        std::memcpy(&point.x, cloud.data.data() + offset + x_offset, sizeof(float));
        std::memcpy(&point.y, cloud.data.data() + offset + y_offset, sizeof(float));
        if (std::isfinite(point.x) && std::isfinite(point.y)) {
          points.push_back(point);
        }
      }
    }
    return true;
  }

  bool baseStatusOk() const
  {
    if (!have_base_status_) {
      return false;
    }
    if (latest_bunker_status_.vehicle_state != 0U || latest_bunker_status_.error_code != 0U) {
      return false;
    }
    return !require_can_control_mode_ || latest_bunker_status_.control_mode == 1U;
  }

  std::string baseFaultReason() const
  {
    if (latest_bunker_status_.vehicle_state != 0U) {
      return "bunker_vehicle_not_normal";
    }
    if (latest_bunker_status_.error_code != 0U) {
      return "bunker_error_code_nonzero";
    }
    if (require_can_control_mode_ && latest_bunker_status_.control_mode != 1U) {
      return "bunker_not_in_can_mode";
    }
    return "bunker_status_unavailable";
  }

  bool rcOverrideActive(const bunker_msgs::msg::BunkerRCState & msg) const
  {
    return std::abs(static_cast<int>(msg.stick_left_h)) > rc_override_deadband_ ||
      std::abs(static_cast<int>(msg.stick_left_v)) > rc_override_deadband_ ||
      std::abs(static_cast<int>(msg.stick_right_h)) > rc_override_deadband_ ||
      std::abs(static_cast<int>(msg.stick_right_v)) > rc_override_deadband_;
  }

  static double ageSeconds(const SteadyTime & stamp, const bool valid)
  {
    if (!valid) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
  }

  void armCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (emergency_stop_latched_) {
      response->success = false;
      response->message = "Cannot arm: software emergency stop is latched";
      return;
    }
    if (require_odom_ && ageSeconds(last_odom_time_, have_odom_) > odom_timeout_sec_) {
      response->success = false;
      response->message = "Cannot arm: odometry is stale";
      return;
    }
    if (rc_override_active_) {
      response->success = false;
      response->message = "Cannot arm: RC sticks are active";
      return;
    }
    if (!allow_arm_without_command_ &&
      ageSeconds(last_command_time_, have_command_) > command_timeout_sec_)
    {
      response->success = false;
      response->message = "Cannot arm: planned command is stale";
      return;
    }
    if (ageSeconds(last_cloud_time_, have_cloud_) > cloud_timeout_sec_) {
      response->success = false;
      response->message = "Cannot arm: obstacle cloud is stale";
      return;
    }
    if (require_bunker_status_ &&
      (ageSeconds(last_base_status_time_, have_base_status_) > base_status_timeout_sec_ ||
      !baseStatusOk()))
    {
      response->success = false;
      response->message = "Cannot arm: Bunker status is stale or unhealthy";
      return;
    }
    if (require_rc_state_ && ageSeconds(last_rc_time_, have_rc_state_) > rc_timeout_sec_) {
      response->success = false;
      response->message = "Cannot arm: RC state is stale";
      return;
    }
    if (require_planner_state_ &&
      (ageSeconds(last_planner_state_time_, have_planner_state_) > planner_state_timeout_sec_ ||
      latest_planner_state_.state ==
      track_robot_interfaces::msg::AvoidanceState::STATE_WAITING_FOR_DATA ||
      latest_planner_state_.state == track_robot_interfaces::msg::AvoidanceState::STATE_STALE ||
      latest_planner_state_.state ==
      track_robot_interfaces::msg::AvoidanceState::STATE_NO_SAFE_TRAJECTORY))
    {
      response->success = false;
      response->message = "Cannot arm: avoidance planner is stale, waiting, or blocked";
      return;
    }
    armed_ = true;
    waiting_for_first_command_after_arm_ = allow_arm_without_command_;
    rc_override_latched_ = false;
    response->success = true;
    response->message = "Motion safety supervisor armed";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void disarmCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    armed_ = false;
    waiting_for_first_command_after_arm_ = false;
    safe_cmd_pub_->publish(geometry_msgs::msg::Twist());
    response->success = true;
    response->message = "Motion safety supervisor disarmed and zero command sent";
  }

  void emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    emergency_stop_latched_ = true;
    armed_ = false;
    waiting_for_first_command_after_arm_ = false;
    safe_cmd_pub_->publish(geometry_msgs::msg::Twist());
    response->success = true;
    response->message = "Software emergency stop latched";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  void resetEmergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (rc_override_active_) {
      response->success = false;
      response->message = "Cannot reset emergency stop while RC sticks are active";
      return;
    }
    if (require_bunker_status_ && have_base_status_ && !baseStatusOk()) {
      response->success = false;
      response->message = "Cannot reset emergency stop while Bunker reports a fault";
      return;
    }
    emergency_stop_latched_ = false;
    armed_ = false;
    waiting_for_first_command_after_arm_ = false;
    response->success = true;
    response->message = "Software emergency stop reset; supervisor remains disarmed";
  }

  void publishSafetyState(const SafetyDecision & decision)
  {
    track_robot_interfaces::msg::SafetyState msg;
    msg.header.stamp = now();
    msg.header.frame_id = marker_frame_;
    msg.state = decision.state;
    msg.armed = armed_;
    msg.emergency_stop_latched = emergency_stop_latched_;
    msg.cloud_fresh = ageSeconds(last_cloud_time_, have_cloud_) <= cloud_timeout_sec_;
    msg.command_fresh = ageSeconds(last_command_time_, have_command_) <= command_timeout_sec_;
    msg.base_status_fresh = !require_bunker_status_ ||
      ageSeconds(last_base_status_time_, have_base_status_) <= base_status_timeout_sec_;
    msg.base_status_ok = !require_bunker_status_ || baseStatusOk();
    msg.rc_override_active = rc_override_active_;
    msg.planner_state = have_planner_state_ ? latest_planner_state_.state : 0U;
    msg.closest_obstacle_distance = finiteOrNegative(decision.collision.closest_obstacle);
    msg.stopping_distance = static_cast<float>(decision.stopping_distance);
    msg.collision_path_distance = finiteOrNegative(
      decision.collision.collision_path_distance);
    msg.time_to_collision = finiteOrNegative(decision.collision.time_to_collision);
    msg.planned_cmd = decision.planned;
    msg.safe_cmd = decision.safe;
    msg.reason = decision.reason;
    safety_state_pub_->publish(msg);
  }

  void publishDebug(const SafetyDecision & decision)
  {
    std_msgs::msg::String msg;
    std::ostringstream data;
    data << "{\"state\":" << static_cast<int>(decision.state) << ","
         << "\"reason\":\"" << decision.reason << "\","
         << "\"armed\":" << (armed_ ? "true" : "false") << ","
         << "\"emergency_stop\":" << (emergency_stop_latched_ ? "true" : "false") << ","
         << "\"command_age\":" << finiteOrNegative(ageSeconds(last_command_time_, have_command_)) << ","
         << "\"cloud_age\":" << finiteOrNegative(ageSeconds(last_cloud_time_, have_cloud_)) << ","
         << "\"base_status_age\":" << finiteOrNegative(ageSeconds(last_base_status_time_, have_base_status_)) << ","
         << "\"planner_state_age\":" << finiteOrNegative(
      ageSeconds(last_planner_state_time_, have_planner_state_)) << ","
         << "\"odom_age\":" << finiteOrNegative(
      ageSeconds(last_odom_time_, have_odom_)) << ","
         << "\"planner_state\":" << (have_planner_state_ ?
      static_cast<int>(latest_planner_state_.state) : -1) << ","
         << "\"obstacle_points\":" << obstacle_points_.size() << ","
         << "\"stopping_distance\":" << decision.stopping_distance << ","
         << "\"collision_path_distance\":" <<
      finiteOrNegative(decision.collision.collision_path_distance) << ","
         << "\"planned_v\":" << decision.planned.linear.x << ","
         << "\"planned_w\":" << decision.planned.angular.z << ","
         << "\"safe_v\":" << decision.safe.linear.x << ","
         << "\"safe_w\":" << decision.safe.angular.z << "}";
    msg.data = data.str();
    debug_pub_->publish(msg);
  }

  void publishMarkers(const SafetyDecision & decision)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker trajectory;
    trajectory.header.stamp = now();
    trajectory.header.frame_id = marker_frame_;
    trajectory.ns = "safety_trajectory";
    trajectory.id = 0;
    trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::msg::Marker::ADD;
    trajectory.pose.orientation.w = 1.0;
    trajectory.scale.x = 0.05;
    setStateColor(decision.state, trajectory);
    trajectory.lifetime = rclcpp::Duration::from_seconds(0.25);
    const double linear = decision.safe.linear.x;
    const double angular = decision.safe.angular.z;
    for (double time = 0.0; time <= 3.0; time += 0.10) {
      geometry_msgs::msg::Point point;
      if (std::abs(angular) < 1e-5) {
        point.x = linear * time;
      } else {
        const double yaw = angular * time;
        const double radius = linear / angular;
        point.x = radius * std::sin(yaw);
        point.y = radius * (1.0 - std::cos(yaw));
      }
      point.z = 0.12;
      trajectory.points.push_back(point);
    }
    markers.markers.push_back(trajectory);

    visualization_msgs::msg::Marker text;
    text.header = trajectory.header;
    text.ns = "safety_state";
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.z = 1.45;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    setStateColor(decision.state, text);
    text.lifetime = rclcpp::Duration::from_seconds(0.25);
    std::ostringstream label;
    label << decision.reason << "\narmed=" << (armed_ ? "true" : "false")
          << " v=" << decision.safe.linear.x << " w=" << decision.safe.angular.z
          << "\nstop=" << decision.stopping_distance << " collision="
          << finiteOrNegative(decision.collision.collision_path_distance);
    text.text = label.str();
    markers.markers.push_back(text);
    marker_pub_->publish(markers);
  }

  static void setStateColor(
    const uint8_t state, visualization_msgs::msg::Marker & marker)
  {
    marker.color.a = 0.95F;
    if (state == track_robot_interfaces::msg::SafetyState::STATE_CLEAR) {
      marker.color.g = 1.0F;
      marker.color.r = 0.1F;
    } else if (state == track_robot_interfaces::msg::SafetyState::STATE_AVOIDING) {
      marker.color.g = 1.0F;
      marker.color.b = 0.9F;
      marker.color.r = 0.05F;
    } else if (state == track_robot_interfaces::msg::SafetyState::STATE_SLOWDOWN) {
      marker.color.r = 1.0F;
      marker.color.g = 0.7F;
    } else {
      marker.color.r = 1.0F;
      marker.color.g = 0.1F;
    }
  }

  static float finiteOrNegative(const double value)
  {
    return std::isfinite(value) ? static_cast<float>(value) : -1.0F;
  }

  std::string planned_cmd_topic_;
  std::string planner_state_topic_;
  std::string obstacle_cloud_topic_;
  std::string bunker_status_topic_;
  std::string rc_state_topic_;
  std::string odom_topic_;
  std::string safe_cmd_topic_;
  std::string safety_state_topic_;
  std::string debug_topic_;
  std::string marker_topic_;
  std::string marker_frame_;
  std::string arm_service_name_;
  std::string disarm_service_name_;
  std::string emergency_stop_service_name_;
  std::string reset_emergency_stop_service_name_;

  double publish_rate_{50.0};
  double marker_publish_rate_{10.0};
  double command_timeout_sec_{0.15};
  double cloud_timeout_sec_{0.25};
  double base_status_timeout_sec_{0.20};
  double rc_timeout_sec_{0.25};
  bool require_bunker_status_{true};
  bool require_rc_state_{true};
  bool require_can_control_mode_{true};
  bool require_planner_state_{true};
  double planner_state_timeout_sec_{0.25};
  bool require_odom_{false};
  double odom_timeout_sec_{0.25};
  bool allow_arm_without_command_{false};
  bool waiting_for_first_command_after_arm_{false};
  double max_linear_x_{0.15};
  double max_angular_z_{0.35};
  double braking_deceleration_{0.25};
  double angular_braking_deceleration_{0.80};
  double response_latency_sec_{0.25};
  double fixed_stop_margin_{0.45};
  double fixed_rotation_margin_{0.05};
  bool bounded_rotation_collision_enabled_{false};
  double slowdown_path_distance_{1.0};
  double trajectory_step_sec_{0.05};
  double max_lookahead_distance_{1.5};
  double max_lookahead_time_sec_{10.0};
  double footprint_length_{1.20};
  double footprint_width_{1.00};
  double safety_inflation_{0.20};
  int rc_override_deadband_{10};

  bool armed_{false};
  bool emergency_stop_latched_{false};
  bool rc_override_active_{false};
  bool rc_override_latched_{false};
  bool have_command_{false};
  bool have_cloud_{false};
  bool have_base_status_{false};
  bool have_rc_state_{false};
  bool have_planner_state_{false};
  bool have_odom_{false};
  geometry_msgs::msg::Twist latest_planned_cmd_;
  bunker_msgs::msg::BunkerStatus latest_bunker_status_;
  bunker_msgs::msg::BunkerRCState latest_rc_state_;
  track_robot_interfaces::msg::AvoidanceState latest_planner_state_;
  std::vector<Point2f> obstacle_points_;
  SteadyTime last_command_time_;
  SteadyTime last_cloud_time_;
  SteadyTime last_base_status_time_;
  SteadyTime last_rc_time_;
  SteadyTime last_planner_state_time_;
  SteadyTime last_odom_time_;
  int timer_count_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planned_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::AvoidanceState>::SharedPtr
  planner_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr bunker_status_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerRCState>::SharedPtr rc_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<track_robot_interfaces::msg::SafetyState>::SharedPtr safety_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_emergency_stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionSafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
