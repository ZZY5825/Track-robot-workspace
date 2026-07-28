#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "track_robot_interfaces/msg/perception_health.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

double clamp(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void rollPitchFromQuaternion(
  const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch)
{
  roll = std::atan2(
    2.0 * (q.w * q.x + q.y * q.z),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  pitch = std::asin(clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}

}  // namespace

class PerceptionHealthMonitorNode : public rclcpp::Node
{
public:
  PerceptionHealthMonitorNode()
  : Node("perception_health_monitor_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/zed/zed_node/left/image_rect_color");
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/rslidar_points");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/zed/zed_node/imu/data");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/odom");
    command_topic_ = declare_parameter<std::string>("command_topic", "/follow/cmd_vel_safe");
    health_topic_ = declare_parameter<std::string>("health_topic", "/perception/health");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "rslidar");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "zed_left_camera_optical_frame");
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    camera_timeout_sec_ = declare_parameter<double>("camera_timeout_sec", 0.35);
    lidar_timeout_sec_ = declare_parameter<double>("lidar_timeout_sec", 0.25);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.25);
    odometry_timeout_sec_ = declare_parameter<double>("odometry_timeout_sec", 0.30);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.30);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.03);
    minimum_brightness_ = declare_parameter<double>("minimum_brightness", 25.0);
    maximum_brightness_ = declare_parameter<double>("maximum_brightness", 235.0);
    minimum_contrast_ = declare_parameter<double>("minimum_contrast", 18.0);
    minimum_blur_score_ = declare_parameter<double>("minimum_blur_score", 20.0);
    minimum_lidar_points_ = declare_parameter<int>("minimum_lidar_points", 1000);
    minimum_lidar_coverage_ = declare_parameter<double>("minimum_lidar_coverage", 0.60);
    maximum_near_noise_ratio_ = declare_parameter<double>("maximum_near_noise_ratio", 0.20);
    maximum_roll_ = declare_parameter<double>("maximum_roll_deg", 12.0) * M_PI / 180.0;
    maximum_pitch_ = declare_parameter<double>("maximum_pitch_deg", 15.0) * M_PI / 180.0;
    stuck_command_speed_ = declare_parameter<double>("stuck_command_speed", 0.05);
    stuck_measured_speed_ = declare_parameter<double>("stuck_measured_speed", 0.02);
    stuck_timeout_sec_ = declare_parameter<double>("stuck_timeout_sec", 1.0);
    unexpected_motion_speed_ = declare_parameter<double>("unexpected_motion_speed", 0.05);

    auto sensor_qos = rclcpp::SensorDataQoS();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos,
      std::bind(&PerceptionHealthMonitorNode::imageCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::QoS(5).reliable(),
      std::bind(&PerceptionHealthMonitorNode::cloudCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&PerceptionHealthMonitorNode::imuCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 10,
      std::bind(&PerceptionHealthMonitorNode::odomCallback, this, std::placeholders::_1));
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      std::bind(&PerceptionHealthMonitorNode::commandCallback, this, std::placeholders::_1));
    health_pub_ = create_publisher<track_robot_interfaces::msg::PerceptionHealth>(
      health_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PerceptionHealthMonitorNode::publishHealth, this));
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  double age(const SteadyTime & stamp, const bool available) const
  {
    if (!available) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const int channels = msg->encoding == "mono8" ? 1 :
      (msg->encoding == "rgba8" || msg->encoding == "bgra8" ? 4 : 3);
    if (msg->data.empty() || msg->width < 3 || msg->height < 3 ||
      msg->step < msg->width * static_cast<unsigned>(channels)) {
      return;
    }

    const unsigned stride = std::max(1U, std::min(msg->width, msg->height) / 160U);
    std::vector<double> previous_row;
    std::vector<double> current_row;
    std::vector<double> next_row;
    double sum = 0.0;
    double sum_sq = 0.0;
    double lap_sum = 0.0;
    double lap_sq = 0.0;
    std::size_t count = 0;
    std::size_t lap_count = 0;

    auto gray = [&](const unsigned x, const unsigned y) {
        const auto offset = static_cast<std::size_t>(y) * msg->step +
          static_cast<std::size_t>(x) * channels;
        if (channels == 1) {
          return static_cast<double>(msg->data[offset]);
        }
        return 0.299 * msg->data[offset + 2] + 0.587 * msg->data[offset + 1] +
               0.114 * msg->data[offset];
      };

    for (unsigned y = stride; y + stride < msg->height; y += stride) {
      for (unsigned x = stride; x + stride < msg->width; x += stride) {
        const double center = gray(x, y);
        sum += center;
        sum_sq += center * center;
        ++count;
        const double lap = gray(x - stride, y) + gray(x + stride, y) +
          gray(x, y - stride) + gray(x, y + stride) - 4.0 * center;
        lap_sum += lap;
        lap_sq += lap * lap;
        ++lap_count;
      }
    }
    if (count == 0) {
      return;
    }
    camera_brightness_ = sum / count;
    camera_contrast_ = std::sqrt(std::max(0.0, sum_sq / count - camera_brightness_ * camera_brightness_));
    const double lap_mean = lap_count ? lap_sum / lap_count : 0.0;
    camera_blur_score_ = lap_count ? std::max(0.0, lap_sq / lap_count - lap_mean * lap_mean) : 0.0;
    last_camera_time_ = std::chrono::steady_clock::now();
    have_camera_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    for (const auto & field : msg->fields) {
      if (field.name == "x") x_offset = static_cast<int>(field.offset);
      if (field.name == "y") y_offset = static_cast<int>(field.offset);
      if (field.name == "z") z_offset = static_cast<int>(field.offset);
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || msg->point_step == 0) {
      return;
    }

    std::array<bool, 72> sectors{};
    std::size_t valid = 0;
    std::size_t near = 0;
    const std::size_t points = msg->data.size() / msg->point_step;
    for (std::size_t index = 0; index < points; ++index) {
      const auto * data = msg->data.data() + index * msg->point_step;
      float x;
      float y;
      float z;
      std::memcpy(&x, data + x_offset, sizeof(float));
      std::memcpy(&y, data + y_offset, sizeof(float));
      std::memcpy(&z, data + z_offset, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      const double range = std::hypot(x, y);
      if (range < 0.2 || range > 10.0) continue;
      ++valid;
      if (range < 1.0 && (z < -0.3 || z > 2.2)) ++near;
      const double normalized = (std::atan2(y, x) + M_PI) / (2.0 * M_PI);
      const auto sector = std::min<std::size_t>(71, static_cast<std::size_t>(normalized * 72.0));
      sectors[sector] = true;
    }
    lidar_valid_points_ = static_cast<double>(valid);
    lidar_near_noise_ratio_ = valid ? static_cast<double>(near) / valid : 1.0;
    lidar_coverage_ = static_cast<double>(std::count(sectors.begin(), sectors.end(), true)) /
      sectors.size();
    last_lidar_time_ = std::chrono::steady_clock::now();
    have_lidar_ = true;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rollPitchFromQuaternion(msg->orientation, roll_, pitch_);
    angular_rate_ = std::sqrt(
      msg->angular_velocity.x * msg->angular_velocity.x +
      msg->angular_velocity.y * msg->angular_velocity.y +
      msg->angular_velocity.z * msg->angular_velocity.z);
    last_imu_time_ = std::chrono::steady_clock::now();
    have_imu_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    measured_linear_speed_ = msg->twist.twist.linear.x;
    measured_angular_speed_ = msg->twist.twist.angular.z;
    odometry_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    last_odom_time_ = std::chrono::steady_clock::now();
    have_odom_ = true;
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    commanded_linear_speed_ = msg->linear.x;
    commanded_angular_speed_ = msg->angular.z;
    last_command_time_ = std::chrono::steady_clock::now();
    have_command_ = true;
  }

  bool transformsAvailable()
  {
    try {
      const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(tf_timeout_sec_));
      return tf_buffer_.canTransform(base_frame_, lidar_frame_, tf2::TimePointZero, timeout) &&
        tf_buffer_.canTransform(base_frame_, camera_frame_, tf2::TimePointZero, timeout);
    } catch (const std::exception &) {
      return false;
    }
  }

  void publishHealth()
  {
    track_robot_interfaces::msg::PerceptionHealth msg;
    msg.header.stamp = now();
    msg.camera_fresh = age(last_camera_time_, have_camera_) <= camera_timeout_sec_;
    msg.lidar_fresh = age(last_lidar_time_, have_lidar_) <= lidar_timeout_sec_;
    msg.imu_fresh = age(last_imu_time_, have_imu_) <= imu_timeout_sec_;
    msg.odometry_fresh = age(last_odom_time_, have_odom_) <= odometry_timeout_sec_;
    msg.command_fresh = age(last_command_time_, have_command_) <= command_timeout_sec_;
    msg.tf_available = transformsAvailable();
    msg.camera_brightness = camera_brightness_;
    msg.camera_contrast = camera_contrast_;
    msg.camera_blur_score = camera_blur_score_;
    msg.lidar_valid_point_count = lidar_valid_points_;
    msg.lidar_near_noise_ratio = lidar_near_noise_ratio_;
    msg.lidar_coverage = lidar_coverage_;
    msg.roll = roll_;
    msg.pitch = pitch_;
    msg.angular_rate = angular_rate_;
    msg.measured_linear_speed = measured_linear_speed_;
    msg.measured_angular_speed = measured_angular_speed_;
    msg.odometry_yaw = odometry_yaw_;
    const bool motion_requested = msg.command_fresh &&
      (std::abs(commanded_linear_speed_) >= stuck_command_speed_ ||
      std::abs(commanded_angular_speed_) >= stuck_command_speed_);
    const bool robot_moving = std::abs(measured_linear_speed_) >= stuck_measured_speed_ ||
      std::abs(measured_angular_speed_) >= stuck_measured_speed_;
    if (motion_requested && !robot_moving) {
      if (!stuck_timer_active_) {
        stuck_timer_active_ = true;
        stuck_start_time_ = std::chrono::steady_clock::now();
      }
    } else {
      stuck_timer_active_ = false;
    }
    const bool stuck = stuck_timer_active_ && age(stuck_start_time_, true) >= stuck_timeout_sec_;
    const bool unexpected_motion = msg.command_fresh && !motion_requested &&
      (std::abs(measured_linear_speed_) >= unexpected_motion_speed_ ||
      std::abs(measured_angular_speed_) >= unexpected_motion_speed_);
    msg.motion_consistent = !stuck && !unexpected_motion;
    msg.attitude_safe = std::isfinite(roll_) && std::isfinite(pitch_) &&
      std::abs(roll_) <= maximum_roll_ && std::abs(pitch_) <= maximum_pitch_;
    msg.camera_usable = msg.camera_fresh && camera_brightness_ >= minimum_brightness_ &&
      camera_brightness_ <= maximum_brightness_ && camera_contrast_ >= minimum_contrast_ &&
      camera_blur_score_ >= minimum_blur_score_;
    msg.lidar_usable = msg.lidar_fresh && lidar_valid_points_ >= minimum_lidar_points_ &&
      lidar_coverage_ >= minimum_lidar_coverage_ &&
      lidar_near_noise_ratio_ <= maximum_near_noise_ratio_;

    if (!msg.attitude_safe) {
      msg.state = msg.UNSAFE;
      msg.reason = "unsafe_attitude";
    } else if (!msg.tf_available) {
      msg.state = msg.STALE;
      msg.reason = "required_tf_unavailable";
    } else if (!msg.motion_consistent) {
      msg.state = msg.UNSAFE;
      msg.reason = stuck ? "blocked_or_stuck" : "unexpected_base_motion";
    } else if (!msg.lidar_fresh || !msg.imu_fresh || !msg.odometry_fresh) {
      msg.state = msg.STALE;
      msg.reason = !msg.lidar_fresh ? "lidar_stale" :
        (!msg.imu_fresh ? "imu_stale" : "odometry_stale");
    } else if (!msg.camera_usable && !msg.lidar_usable) {
      msg.state = msg.UNSAFE;
      msg.reason = "camera_and_lidar_unusable";
    } else if (!msg.camera_usable || !msg.lidar_usable) {
      msg.state = msg.DEGRADED;
      msg.reason = !msg.camera_usable ? "camera_degraded" : "lidar_degraded";
    } else {
      msg.state = msg.HEALTHY;
      msg.reason = "healthy";
    }
    health_pub_->publish(msg);
  }

  std::string image_topic_, lidar_topic_, imu_topic_, odometry_topic_, command_topic_, health_topic_;
  std::string base_frame_, lidar_frame_, camera_frame_;
  double publish_rate_, camera_timeout_sec_, lidar_timeout_sec_, imu_timeout_sec_;
  double odometry_timeout_sec_, minimum_brightness_, maximum_brightness_, minimum_contrast_;
  double command_timeout_sec_, tf_timeout_sec_;
  double minimum_blur_score_, minimum_lidar_coverage_, maximum_near_noise_ratio_;
  double maximum_roll_, maximum_pitch_;
  double stuck_command_speed_, stuck_measured_speed_, stuck_timeout_sec_, unexpected_motion_speed_;
  int minimum_lidar_points_;
  bool have_camera_{false}, have_lidar_{false}, have_imu_{false}, have_odom_{false};
  bool have_command_{false}, stuck_timer_active_{false};
  SteadyTime last_camera_time_, last_lidar_time_, last_imu_time_, last_odom_time_;
  SteadyTime last_command_time_, stuck_start_time_;
  double camera_brightness_{0.0}, camera_contrast_{0.0}, camera_blur_score_{0.0};
  double lidar_valid_points_{0.0}, lidar_near_noise_ratio_{1.0}, lidar_coverage_{0.0};
  double roll_{0.0}, pitch_{0.0}, angular_rate_{0.0};
  double measured_linear_speed_{0.0}, measured_angular_speed_{0.0}, odometry_yaw_{0.0};
  double commanded_linear_speed_{0.0}, commanded_angular_speed_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<track_robot_interfaces::msg::PerceptionHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionHealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
