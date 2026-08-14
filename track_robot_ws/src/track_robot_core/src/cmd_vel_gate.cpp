#include <chrono>
#include <memory>
#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class CmdVelGate : public rclcpp::Node
{
public:
  CmdVelGate()
  : Node("cmd_vel_gate"),
    timeout_active_(false)
  {
    // Parameters
    this->declare_parameter<std::string>("input_topic", "/teleop/cmd_vel");
    this->declare_parameter<std::string>("output_topic", "/cmd_vel");
    this->declare_parameter<double>("timeout_sec", 0.2);
    this->declare_parameter<double>("publish_rate", 20.0);
    this->declare_parameter<double>("max_linear_x", 0.6);
    this->declare_parameter<double>("max_angular_z", 1.0);
    this->declare_parameter<std::string>(
      "shutdown_service", "/cmd_vel_gate/shutdown");

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    timeout_sec_ = this->get_parameter("timeout_sec").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    max_linear_x_ = this->get_parameter("max_linear_x").as_double();
    max_angular_z_ = this->get_parameter("max_angular_z").as_double();
    shutdown_service_ = this->get_parameter("shutdown_service").as_string();

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

    // Subscriber
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic_,
      10,
      std::bind(&CmdVelGate::cmdCallback, this, std::placeholders::_1)
    );

    // Initialize last command time
    last_cmd_time_ = this->now();

    // Timer for watchdog
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CmdVelGate::timerCallback, this)
    );

    shutdown_srv_ = this->create_service<std_srvs::srv::Trigger>(
      shutdown_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        shutdownCallback(response);
      });

    RCLCPP_INFO(this->get_logger(), "cmd_vel_gate started.");
    RCLCPP_INFO(this->get_logger(), "Input topic  : %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic : %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Timeout      : %.3f s", timeout_sec_);
    RCLCPP_INFO(this->get_logger(), "Publish rate : %.1f Hz", publish_rate_);
    RCLCPP_INFO(this->get_logger(), "Max linear.x : %.3f", max_linear_x_);
    RCLCPP_INFO(this->get_logger(), "Max angular.z: %.3f", max_angular_z_);
    RCLCPP_INFO(this->get_logger(), "Shutdown svc : %s", shutdown_service_.c_str());
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (shutting_down_) {
      return;
    }
    geometry_msgs::msg::Twist limited_msg = *msg;

    // Clamp linear.x
    limited_msg.linear.x = std::clamp(
      limited_msg.linear.x,
      -max_linear_x_,
      max_linear_x_
    );

    // Clamp angular.z
    limited_msg.angular.z = std::clamp(
      limited_msg.angular.z,
      -max_angular_z_,
      max_angular_z_
    );

    // Force unused fields to zero
    limited_msg.linear.y = 0.0;
    limited_msg.linear.z = 0.0;
    limited_msg.angular.x = 0.0;
    limited_msg.angular.y = 0.0;

    last_cmd_ = limited_msg;
    last_cmd_time_ = this->now();
    timeout_active_ = false;

    cmd_pub_->publish(last_cmd_);
  }

  void timerCallback()
  {
    if (shutting_down_) {
      return;
    }
    const double dt = (this->now() - last_cmd_time_).seconds();

    if (dt > timeout_sec_) {
      if (!timeout_active_) {
        geometry_msgs::msg::Twist zero_msg;
        cmd_pub_->publish(zero_msg);
        timeout_active_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Teleop timeout (%.3f s > %.3f s). Publishing zero cmd_vel.",
          dt, timeout_sec_
        );
      }
    }
  }

  void shutdownCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (!shutting_down_) {
      shutting_down_ = true;
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      cmd_sub_.reset();
      if (timer_) {
        timer_->cancel();
        timer_.reset();
      }
      shutdown_timer_ = this->create_wall_timer(50ms, [this]() {
          if (shutdown_timer_) {
            shutdown_timer_->cancel();
          }
          rclcpp::shutdown();
        });
    }
    response->success = true;
    response->message = "Zero command published; cmd_vel gate shutting down";
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string shutdown_service_;
  double timeout_sec_;
  double publish_rate_;
  double max_linear_x_;
  double max_angular_z_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_srv_;

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  bool timeout_active_;
  bool shutting_down_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelGate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
