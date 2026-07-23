#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "track_robot_interfaces/msg/avoidance_state.hpp"
#include "track_robot_interfaces/msg/target_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {

double clampValue(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

int turnSign(const double angular)
{
  if (angular > 0.03) {
    return 1;
  }
  if (angular < -0.03) {
    return -1;
  }
  return 0;
}

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct FootprintPoint
{
  double x{0.0};
  double y{0.0};
};

struct Trajectory
{
  geometry_msgs::msg::Twist command;
  std::vector<Pose2d> poses;
  bool feasible{false};
  bool direct{false};
  double minimum_clearance{0.0};
  double score{std::numeric_limits<double>::infinity()};
};

struct PlannerDecision
{
  uint8_t state{track_robot_interfaces::msg::AvoidanceState::STATE_WAITING_FOR_DATA};
  std::string reason{"waiting_for_data"};
  geometry_msgs::msg::Twist desired;
  geometry_msgs::msg::Twist selected;
  int candidate_count{0};
  int feasible_count{0};
  int selected_index{-1};
  double selected_score{std::numeric_limits<double>::infinity()};
  double selected_clearance{0.0};
};

struct QueueCell
{
  float distance;
  int index;

  bool operator>(const QueueCell & other) const
  {
    return distance > other.distance;
  }
};

}  // namespace

class LocalTrajectoryPlannerNode : public rclcpp::Node
{
public:
  LocalTrajectoryPlannerNode()
  : Node("local_trajectory_planner_node")
  {
    desired_cmd_topic_ = declare_parameter<std::string>(
      "desired_cmd_topic", "/follow/cmd_vel_planned");
    target_topic_ = declare_parameter<std::string>(
      "target_topic", "/human_tracking/target_state");
    obstacle_grid_topic_ = declare_parameter<std::string>(
      "obstacle_grid_topic", "/safety/local_obstacle_grid");
    output_cmd_topic_ = declare_parameter<std::string>(
      "output_cmd_topic", "/follow/cmd_vel_avoiding");
    planner_state_topic_ = declare_parameter<std::string>(
      "planner_state_topic", "/follow/avoidance_state");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/follow/avoidance_trajectory_markers");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/follow/avoidance_debug");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    enable_avoidance_ = declare_parameter<bool>("enable_avoidance", true);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    marker_publish_rate_ = declare_parameter<double>("marker_publish_rate", 10.0);
    desired_timeout_sec_ = declare_parameter<double>("desired_timeout_sec", 0.20);
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.50);
    grid_timeout_sec_ = declare_parameter<double>("grid_timeout_sec", 0.30);
    require_target_state_ = declare_parameter<bool>("require_target_state", true);
    unknown_is_obstacle_ = declare_parameter<bool>("unknown_is_obstacle", true);

    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.15);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.35);
    linear_samples_ = declare_parameter<int>("linear_samples", 4);
    angular_samples_ = declare_parameter<int>("angular_samples", 15);
    simulation_horizon_sec_ = declare_parameter<double>("simulation_horizon_sec", 6.0);
    simulation_step_sec_ = declare_parameter<double>("simulation_step_sec", 0.10);

    footprint_length_ = declare_parameter<double>("footprint_length", 1.20);
    footprint_width_ = declare_parameter<double>("footprint_width", 1.00);
    safety_inflation_ = declare_parameter<double>("safety_inflation", 0.20);
    footprint_sample_spacing_ = declare_parameter<double>("footprint_sample_spacing", 0.10);
    collision_padding_ = declare_parameter<double>("collision_padding", 0.08);
    direct_preference_clearance_ = declare_parameter<double>(
      "direct_preference_clearance", 0.35);

    weight_target_heading_ = declare_parameter<double>("weight_target_heading", 2.5);
    weight_target_progress_ = declare_parameter<double>("weight_target_progress", 2.0);
    weight_clearance_ = declare_parameter<double>("weight_clearance", 0.8);
    weight_command_change_ = declare_parameter<double>("weight_command_change", 0.8);
    weight_desired_command_ = declare_parameter<double>("weight_desired_command", 1.0);
    weight_forward_speed_ = declare_parameter<double>("weight_forward_speed", 0.5);
    stationary_penalty_ = declare_parameter<double>("stationary_penalty", 2.5);
    side_switch_penalty_ = declare_parameter<double>("side_switch_penalty", 3.0);
    side_commitment_sec_ = declare_parameter<double>("side_commitment_sec", 1.5);
    direct_command_bonus_ = declare_parameter<double>("direct_command_bonus", 1.0);
    max_marker_candidates_ = declare_parameter<int>("max_marker_candidates", 30);

    buildFootprintSamples();

    desired_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      desired_cmd_topic_, 10,
      std::bind(&LocalTrajectoryPlannerNode::desiredCallback, this, std::placeholders::_1));
    target_sub_ = create_subscription<track_robot_interfaces::msg::TargetState>(
      target_topic_, 10,
      std::bind(&LocalTrajectoryPlannerNode::targetCallback, this, std::placeholders::_1));
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      obstacle_grid_topic_, 5,
      std::bind(&LocalTrajectoryPlannerNode::gridCallback, this, std::placeholders::_1));

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);
    state_pub_ = create_publisher<track_robot_interfaces::msg::AvoidanceState>(
      planner_state_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LocalTrajectoryPlannerNode::timerCallback, this));

    RCLCPP_WARN(
      get_logger(),
      "Local trajectory planner: enabled=%s desired=%s output=%s samples=%dx%d",
      enable_avoidance_ ? "true" : "false", desired_cmd_topic_.c_str(),
      output_cmd_topic_.c_str(), linear_samples_, angular_samples_);
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void desiredCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_desired_ = limitCommand(*msg);
    last_desired_time_ = std::chrono::steady_clock::now();
    have_desired_ = true;
  }

  void targetCallback(const track_robot_interfaces::msg::TargetState::SharedPtr msg)
  {
    latest_target_ = *msg;
    last_target_time_ = std::chrono::steady_clock::now();
    have_target_ = true;
  }

  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting local grid in frame %s; expected %s",
        msg->header.frame_id.c_str(), base_frame_.c_str());
      return;
    }
    if (msg->info.width == 0U || msg->info.height == 0U || msg->info.resolution <= 0.0F ||
      msg->data.size() != static_cast<std::size_t>(msg->info.width) * msg->info.height)
    {
      return;
    }
    latest_grid_ = *msg;
    buildDistanceField();
    last_grid_time_ = std::chrono::steady_clock::now();
    have_grid_ = true;
  }

  void timerCallback()
  {
    PlannerDecision decision = makeDecision();
    output_pub_->publish(decision.selected);
    publishState(decision);
    publishDebug(decision);
    ++timer_count_;
    const int marker_divisor = std::max(
      1, static_cast<int>(std::round(publish_rate_ / std::max(1.0, marker_publish_rate_))));
    if (timer_count_ % marker_divisor == 0) {
      publishMarkers(decision);
    }
  }

  PlannerDecision makeDecision()
  {
    PlannerDecision decision;
    decision.desired = latest_desired_;
    last_trajectories_.clear();

    const double desired_age = ageSeconds(last_desired_time_, have_desired_);
    const double target_age = ageSeconds(last_target_time_, have_target_);
    const double grid_age = ageSeconds(last_grid_time_, have_grid_);
    if (desired_age > desired_timeout_sec_ || grid_age > grid_timeout_sec_ ||
      (require_target_state_ && target_age > target_timeout_sec_))
    {
      decision.state = have_desired_ || have_grid_ ?
        track_robot_interfaces::msg::AvoidanceState::STATE_STALE :
        track_robot_interfaces::msg::AvoidanceState::STATE_WAITING_FOR_DATA;
      decision.reason = desired_age > desired_timeout_sec_ ? "desired_command_stale" :
        (grid_age > grid_timeout_sec_ ? "obstacle_grid_stale" : "target_state_stale");
      return decision;
    }

    if (require_target_state_ && !targetValid(latest_target_)) {
      decision.state = track_robot_interfaces::msg::AvoidanceState::STATE_STALE;
      decision.reason = "target_not_motion_valid";
      return decision;
    }

    if (!enable_avoidance_) {
      decision.state = track_robot_interfaces::msg::AvoidanceState::STATE_DISABLED;
      decision.reason = "avoidance_disabled_passthrough";
      decision.selected = decision.desired;
      return decision;
    }

    const bool requested_motion = std::abs(decision.desired.linear.x) > 1e-4 ||
      std::abs(decision.desired.angular.z) > 1e-4;
    if (!requested_motion) {
      decision.state = track_robot_interfaces::msg::AvoidanceState::STATE_DIRECT_CLEAR;
      decision.reason = "zero_command";
      decision.selected = decision.desired;
      return decision;
    }

    const auto commands = candidateCommands(decision.desired);
    last_trajectories_.reserve(commands.size());
    int direct_index = -1;
    for (std::size_t index = 0; index < commands.size(); ++index) {
      Trajectory trajectory = evaluateTrajectory(commands[index], decision.desired);
      trajectory.direct = commandsEquivalent(commands[index], decision.desired);
      if (trajectory.direct) {
        direct_index = static_cast<int>(index);
      }
      if (trajectory.feasible) {
        trajectory.score = scoreTrajectory(trajectory, decision.desired);
        ++decision.feasible_count;
      }
      last_trajectories_.push_back(std::move(trajectory));
    }
    decision.candidate_count = static_cast<int>(last_trajectories_.size());

    if (direct_index >= 0 && last_trajectories_[static_cast<std::size_t>(direct_index)].feasible &&
      last_trajectories_[static_cast<std::size_t>(direct_index)].minimum_clearance >=
      direct_preference_clearance_)
    {
      decision.selected_index = direct_index;
    } else {
      double best_score = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < last_trajectories_.size(); ++index) {
        if (!last_trajectories_[index].feasible || last_trajectories_[index].score >= best_score) {
          continue;
        }
        best_score = last_trajectories_[index].score;
        decision.selected_index = static_cast<int>(index);
      }
    }

    if (decision.selected_index < 0) {
      decision.state = track_robot_interfaces::msg::AvoidanceState::STATE_NO_SAFE_TRAJECTORY;
      decision.reason = "no_safe_trajectory";
      committed_turn_sign_ = 0;
      return decision;
    }

    const auto & selected = last_trajectories_[
      static_cast<std::size_t>(decision.selected_index)];
    decision.selected = selected.command;
    decision.selected_score = selected.score;
    decision.selected_clearance = selected.minimum_clearance;
    const bool changed = !commandsEquivalent(decision.selected, decision.desired);
    if (changed) {
      decision.state = track_robot_interfaces::msg::AvoidanceState::STATE_AVOIDING;
      decision.reason = "avoidance_trajectory_selected";
      const int selected_sign = turnSign(decision.selected.angular.z);
      if (selected_sign != 0) {
        committed_turn_sign_ = selected_sign;
        side_commit_until_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(side_commitment_sec_));
      }
    } else {
      decision.state = track_robot_interfaces::msg::AvoidanceState::STATE_DIRECT_CLEAR;
      decision.reason = "direct_trajectory_clear";
      if (std::chrono::steady_clock::now() >= side_commit_until_) {
        committed_turn_sign_ = 0;
      }
    }
    return decision;
  }

  std::vector<geometry_msgs::msg::Twist> candidateCommands(
    const geometry_msgs::msg::Twist & desired) const
  {
    std::vector<double> linear_values;
    const int linear_count = std::max(1, linear_samples_);
    if (desired.linear.x <= 1e-4) {
      linear_values.push_back(0.0);
    } else {
      for (int i = 0; i < linear_count; ++i) {
        const double ratio = linear_count == 1 ? 1.0 :
          static_cast<double>(i) / static_cast<double>(linear_count - 1);
        linear_values.push_back(desired.linear.x * ratio);
      }
    }

    std::vector<double> angular_values;
    const int angular_count = std::max(3, angular_samples_);
    for (int i = 0; i < angular_count; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(angular_count - 1);
      angular_values.push_back(-max_angular_z_ + 2.0 * max_angular_z_ * ratio);
    }
    angular_values.push_back(desired.angular.z);
    angular_values.push_back(0.0);
    std::sort(angular_values.begin(), angular_values.end());
    angular_values.erase(
      std::unique(angular_values.begin(), angular_values.end(),
        [](const double lhs, const double rhs) {return std::abs(lhs - rhs) < 1e-4;}),
      angular_values.end());

    std::vector<geometry_msgs::msg::Twist> commands;
    for (const double linear : linear_values) {
      for (const double angular : angular_values) {
        geometry_msgs::msg::Twist command;
        command.linear.x = linear;
        command.angular.z = angular;
        commands.push_back(command);
      }
    }
    if (std::none_of(commands.begin(), commands.end(),
      [&desired](const auto & command) {return commandsEquivalent(command, desired);}))
    {
      commands.push_back(desired);
    }
    return commands;
  }

  Trajectory evaluateTrajectory(
    const geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Twist & desired) const
  {
    Trajectory trajectory;
    trajectory.command = command;
    trajectory.minimum_clearance = std::numeric_limits<double>::infinity();
    Pose2d pose;
    const double dt = std::max(0.04, simulation_step_sec_);
    const double desired_speed_fraction = max_linear_x_ > 1e-4 ?
      desired.linear.x / max_linear_x_ : 0.0;
    const double horizon = std::max(
      2.0, simulation_horizon_sec_ * clampValue(0.5 + desired_speed_fraction, 0.5, 1.0));

    for (double time = 0.0; time <= horizon + 1e-6; time += dt) {
      if (time > 0.0) {
        integratePose(pose, command.linear.x, command.angular.z, dt);
      }
      trajectory.poses.push_back(pose);
      const double clearance = footprintClearance(pose);
      trajectory.minimum_clearance = std::min(trajectory.minimum_clearance, clearance);
      if (std::isnan(clearance) || clearance < 0.0 || clearance <= collision_padding_) {
        trajectory.feasible = false;
        return trajectory;
      }
    }
    trajectory.feasible = true;
    return trajectory;
  }

  double scoreTrajectory(
    const Trajectory & trajectory, const geometry_msgs::msg::Twist & desired) const
  {
    const Pose2d endpoint = trajectory.poses.empty() ? Pose2d() : trajectory.poses.back();
    double target_x = latest_target_.position_base.x;
    double target_y = latest_target_.position_base.y;
    if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
      std::hypot(target_x, target_y) < 1e-4)
    {
      target_x = latest_target_.distance * std::cos(latest_target_.bearing);
      target_y = latest_target_.distance * std::sin(latest_target_.bearing);
    }
    const double initial_distance = std::hypot(target_x, target_y);
    const double endpoint_dx = target_x - endpoint.x;
    const double endpoint_dy = target_y - endpoint.y;
    const double endpoint_distance = std::hypot(endpoint_dx, endpoint_dy);
    const double heading_error = std::abs(normalizeAngle(
      std::atan2(endpoint_dy, endpoint_dx) - endpoint.yaw));
    const double progress = initial_distance - endpoint_distance;
    const double clearance = std::max(trajectory.minimum_clearance, 0.02);
    double score = weight_target_heading_ * heading_error -
      weight_target_progress_ * progress + weight_clearance_ / clearance +
      weight_command_change_ * (
      std::abs(trajectory.command.linear.x - last_selected_cmd_.linear.x) +
      std::abs(trajectory.command.angular.z - last_selected_cmd_.angular.z)) +
      weight_desired_command_ * (
      std::abs(trajectory.command.linear.x - desired.linear.x) +
      std::abs(trajectory.command.angular.z - desired.angular.z)) -
      weight_forward_speed_ * trajectory.command.linear.x;

    if (std::abs(desired.linear.x) > 0.02 && trajectory.command.linear.x < 0.01 &&
      std::abs(trajectory.command.angular.z) < 0.03)
    {
      score += stationary_penalty_;
    }
    if (trajectory.direct) {
      score -= direct_command_bonus_;
    }
    const int candidate_sign = turnSign(trajectory.command.angular.z);
    if (committed_turn_sign_ != 0 && std::chrono::steady_clock::now() < side_commit_until_ &&
      candidate_sign != 0 && candidate_sign != committed_turn_sign_)
    {
      score += side_switch_penalty_;
    }
    return score;
  }

  void integratePose(Pose2d & pose, const double linear, const double angular, const double dt) const
  {
    if (std::abs(angular) < 1e-6) {
      pose.x += linear * std::cos(pose.yaw) * dt;
      pose.y += linear * std::sin(pose.yaw) * dt;
    } else {
      const double next_yaw = pose.yaw + angular * dt;
      const double radius = linear / angular;
      pose.x += radius * (std::sin(next_yaw) - std::sin(pose.yaw));
      pose.y -= radius * (std::cos(next_yaw) - std::cos(pose.yaw));
      pose.yaw = next_yaw;
    }
    pose.yaw = normalizeAngle(pose.yaw);
  }

  double footprintClearance(const Pose2d & pose) const
  {
    double minimum = std::numeric_limits<double>::infinity();
    const double cosine = std::cos(pose.yaw);
    const double sine = std::sin(pose.yaw);
    for (const auto & sample : footprint_samples_) {
      const double world_x = pose.x + cosine * sample.x - sine * sample.y;
      const double world_y = pose.y + sine * sample.x + cosine * sample.y;
      const double clearance = distanceAt(world_x, world_y);
      if (std::isnan(clearance) || clearance < 0.0) {
        return -1.0;
      }
      minimum = std::min(minimum, clearance);
    }
    return minimum;
  }

  double distanceAt(const double x, const double y) const
  {
    int cell_x = 0;
    int cell_y = 0;
    if (!worldToCell(x, y, cell_x, cell_y)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const std::size_t index = static_cast<std::size_t>(cell_y) * latest_grid_.info.width +
      static_cast<std::size_t>(cell_x);
    if (index >= distance_field_.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return distance_field_[index];
  }

  bool worldToCell(const double x, const double y, int & cell_x, int & cell_y) const
  {
    cell_x = static_cast<int>(std::floor(
      (x - latest_grid_.info.origin.position.x) / latest_grid_.info.resolution));
    cell_y = static_cast<int>(std::floor(
      (y - latest_grid_.info.origin.position.y) / latest_grid_.info.resolution));
    return cell_x >= 0 && cell_y >= 0 &&
      cell_x < static_cast<int>(latest_grid_.info.width) &&
      cell_y < static_cast<int>(latest_grid_.info.height);
  }

  void buildDistanceField()
  {
    const std::size_t cell_count = static_cast<std::size_t>(latest_grid_.info.width) *
      latest_grid_.info.height;
    distance_field_.assign(cell_count, std::numeric_limits<float>::infinity());
    std::priority_queue<QueueCell, std::vector<QueueCell>, std::greater<QueueCell>> queue;
    for (std::size_t index = 0; index < cell_count; ++index) {
      const int8_t occupancy = latest_grid_.data[index];
      if (occupancy >= 50 || (unknown_is_obstacle_ && occupancy < 0)) {
        distance_field_[index] = 0.0F;
        queue.push(QueueCell{0.0F, static_cast<int>(index)});
      }
    }

    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const float resolution = latest_grid_.info.resolution;
    while (!queue.empty()) {
      const QueueCell current = queue.top();
      queue.pop();
      if (current.distance > distance_field_[static_cast<std::size_t>(current.index)] + 1e-6F) {
        continue;
      }
      const int x = current.index % static_cast<int>(latest_grid_.info.width);
      const int y = current.index / static_cast<int>(latest_grid_.info.width);
      for (int neighbor = 0; neighbor < 8; ++neighbor) {
        const int nx = x + dx[neighbor];
        const int ny = y + dy[neighbor];
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(latest_grid_.info.width) ||
          ny >= static_cast<int>(latest_grid_.info.height))
        {
          continue;
        }
        const int next_index = ny * static_cast<int>(latest_grid_.info.width) + nx;
        const float step = resolution * (neighbor < 4 ? 1.0F : 1.41421356F);
        const float candidate = current.distance + step;
        if (candidate < distance_field_[static_cast<std::size_t>(next_index)]) {
          distance_field_[static_cast<std::size_t>(next_index)] = candidate;
          queue.push(QueueCell{candidate, next_index});
        }
      }
    }
  }

  void buildFootprintSamples()
  {
    footprint_samples_.clear();
    const double half_length = footprint_length_ * 0.5 + safety_inflation_;
    const double half_width = footprint_width_ * 0.5 + safety_inflation_;
    const double spacing = std::max(0.05, footprint_sample_spacing_);
    const int x_count = std::max(2, static_cast<int>(std::ceil(2.0 * half_length / spacing)));
    const int y_count = std::max(2, static_cast<int>(std::ceil(2.0 * half_width / spacing)));
    for (int ix = 0; ix <= x_count; ++ix) {
      const double x = -half_length + 2.0 * half_length *
        static_cast<double>(ix) / static_cast<double>(x_count);
      for (int iy = 0; iy <= y_count; ++iy) {
        const double y = -half_width + 2.0 * half_width *
          static_cast<double>(iy) / static_cast<double>(y_count);
        footprint_samples_.push_back(FootprintPoint{x, y});
      }
    }
  }

  bool targetValid(const track_robot_interfaces::msg::TargetState & target) const
  {
    return target.target_id >= 0 &&
      (target.track_state == track_robot_interfaces::msg::TargetState::TRACK_CAMERA_LIDAR_TRACKED ||
      target.track_state == track_robot_interfaces::msg::TargetState::TRACK_LIDAR_ONLY_TRACKING) &&
      std::isfinite(static_cast<double>(target.distance)) &&
      std::isfinite(static_cast<double>(target.bearing));
  }

  geometry_msgs::msg::Twist limitCommand(const geometry_msgs::msg::Twist & input) const
  {
    geometry_msgs::msg::Twist output;
    output.linear.x = clampValue(input.linear.x, 0.0, max_linear_x_);
    output.angular.z = clampValue(input.angular.z, -max_angular_z_, max_angular_z_);
    return output;
  }

  static bool commandsEquivalent(
    const geometry_msgs::msg::Twist & lhs, const geometry_msgs::msg::Twist & rhs)
  {
    return std::abs(lhs.linear.x - rhs.linear.x) < 0.005 &&
      std::abs(lhs.angular.z - rhs.angular.z) < 0.01;
  }

  static double ageSeconds(const SteadyTime & stamp, const bool valid)
  {
    if (!valid) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
  }

  void publishState(const PlannerDecision & decision)
  {
    track_robot_interfaces::msg::AvoidanceState msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.state = decision.state;
    msg.desired_command_fresh = ageSeconds(last_desired_time_, have_desired_) <=
      desired_timeout_sec_;
    msg.obstacle_grid_fresh = ageSeconds(last_grid_time_, have_grid_) <= grid_timeout_sec_;
    msg.target_fresh = !require_target_state_ ||
      ageSeconds(last_target_time_, have_target_) <= target_timeout_sec_;
    msg.candidate_count = decision.candidate_count;
    msg.feasible_candidate_count = decision.feasible_count;
    msg.selected_score = std::isfinite(decision.selected_score) ?
      static_cast<float>(decision.selected_score) : -1.0F;
    msg.selected_clearance = std::isfinite(decision.selected_clearance) ?
      static_cast<float>(decision.selected_clearance) : -1.0F;
    msg.desired_cmd = decision.desired;
    msg.selected_cmd = decision.selected;
    msg.reason = decision.reason;
    state_pub_->publish(msg);
    last_selected_cmd_ = decision.selected;
  }

  void publishDebug(const PlannerDecision & decision)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "{\"state\":" << static_cast<int>(decision.state) << ","
           << "\"reason\":\"" << decision.reason << "\","
           << "\"candidate_count\":" << decision.candidate_count << ","
           << "\"feasible_count\":" << decision.feasible_count << ","
           << "\"selected_score\":" <<
      (std::isfinite(decision.selected_score) ? decision.selected_score : -1.0) << ","
           << "\"selected_clearance\":" << decision.selected_clearance << ","
           << "\"desired_v\":" << decision.desired.linear.x << ","
           << "\"desired_w\":" << decision.desired.angular.z << ","
           << "\"selected_v\":" << decision.selected.linear.x << ","
           << "\"selected_w\":" << decision.selected.angular.z << ","
           << "\"committed_turn_sign\":" << committed_turn_sign_ << "}";
    msg.data = stream.str();
    debug_pub_->publish(msg);
  }

  void publishMarkers(const PlannerDecision & decision)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    std::vector<int> feasible_indices;
    for (std::size_t index = 0; index < last_trajectories_.size(); ++index) {
      if (last_trajectories_[index].feasible) {
        feasible_indices.push_back(static_cast<int>(index));
      }
    }
    std::sort(feasible_indices.begin(), feasible_indices.end(),
      [this](const int lhs, const int rhs) {
        return last_trajectories_[static_cast<std::size_t>(lhs)].score <
               last_trajectories_[static_cast<std::size_t>(rhs)].score;
      });
    if (static_cast<int>(feasible_indices.size()) > max_marker_candidates_) {
      feasible_indices.resize(static_cast<std::size_t>(max_marker_candidates_));
    }

    int marker_id = 1;
    for (const int index : feasible_indices) {
      const auto & candidate = last_trajectories_[static_cast<std::size_t>(index)];
      visualization_msgs::msg::Marker line;
      line.header.stamp = now();
      line.header.frame_id = base_frame_;
      line.ns = index == decision.selected_index ? "selected_trajectory" : "candidate_trajectories";
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = index == decision.selected_index ? 0.07 : 0.018;
      line.color.a = index == decision.selected_index ? 1.0F : 0.25F;
      if (index == decision.selected_index) {
        line.color.r = 0.1F;
        line.color.g = 1.0F;
        line.color.b = 0.8F;
      } else {
        line.color.r = 0.5F;
        line.color.g = 0.7F;
        line.color.b = 1.0F;
      }
      line.lifetime = rclcpp::Duration::from_seconds(0.25);
      for (const auto & pose : candidate.poses) {
        geometry_msgs::msg::Point point;
        point.x = pose.x;
        point.y = pose.y;
        point.z = 0.14;
        line.points.push_back(point);
      }
      markers.markers.push_back(line);
    }

    visualization_msgs::msg::Marker text;
    text.header.stamp = now();
    text.header.frame_id = base_frame_;
    text.ns = "avoidance_state";
    text.id = marker_id;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.z = 1.75;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.17;
    text.color.a = 1.0F;
    text.color.r = decision.state == track_robot_interfaces::msg::AvoidanceState::STATE_AVOIDING ?
      0.1F : 1.0F;
    text.color.g = decision.state == track_robot_interfaces::msg::AvoidanceState::STATE_NO_SAFE_TRAJECTORY ?
      0.1F : 0.9F;
    text.color.b = 0.8F;
    text.lifetime = rclcpp::Duration::from_seconds(0.25);
    std::ostringstream label;
    label << decision.reason << "\ncandidates=" << decision.feasible_count << "/"
          << decision.candidate_count << " v=" << decision.selected.linear.x
          << " w=" << decision.selected.angular.z;
    text.text = label.str();
    markers.markers.push_back(text);
    marker_pub_->publish(markers);
  }

  std::string desired_cmd_topic_;
  std::string target_topic_;
  std::string obstacle_grid_topic_;
  std::string output_cmd_topic_;
  std::string planner_state_topic_;
  std::string marker_topic_;
  std::string debug_topic_;
  std::string base_frame_;
  bool enable_avoidance_{true};
  double publish_rate_{20.0};
  double marker_publish_rate_{10.0};
  double desired_timeout_sec_{0.20};
  double target_timeout_sec_{0.50};
  double grid_timeout_sec_{0.30};
  bool require_target_state_{true};
  bool unknown_is_obstacle_{true};
  double max_linear_x_{0.15};
  double max_angular_z_{0.35};
  int linear_samples_{4};
  int angular_samples_{15};
  double simulation_horizon_sec_{6.0};
  double simulation_step_sec_{0.10};
  double footprint_length_{1.20};
  double footprint_width_{1.00};
  double safety_inflation_{0.20};
  double footprint_sample_spacing_{0.10};
  double collision_padding_{0.08};
  double direct_preference_clearance_{0.35};
  double weight_target_heading_{2.5};
  double weight_target_progress_{2.0};
  double weight_clearance_{0.8};
  double weight_command_change_{0.8};
  double weight_desired_command_{1.0};
  double weight_forward_speed_{0.5};
  double stationary_penalty_{2.5};
  double side_switch_penalty_{3.0};
  double side_commitment_sec_{1.5};
  double direct_command_bonus_{1.0};
  int max_marker_candidates_{30};

  bool have_desired_{false};
  bool have_target_{false};
  bool have_grid_{false};
  geometry_msgs::msg::Twist latest_desired_;
  geometry_msgs::msg::Twist last_selected_cmd_;
  track_robot_interfaces::msg::TargetState latest_target_;
  nav_msgs::msg::OccupancyGrid latest_grid_;
  std::vector<float> distance_field_;
  std::vector<FootprintPoint> footprint_samples_;
  std::vector<Trajectory> last_trajectories_;
  SteadyTime last_desired_time_;
  SteadyTime last_target_time_;
  SteadyTime last_grid_time_;
  SteadyTime side_commit_until_;
  int committed_turn_sign_{0};
  int timer_count_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_sub_;
  rclcpp::Subscription<track_robot_interfaces::msg::TargetState>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<track_robot_interfaces::msg::AvoidanceState>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalTrajectoryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
