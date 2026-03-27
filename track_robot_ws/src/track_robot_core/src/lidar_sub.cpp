#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class LidarSub : public rclcpp::Node
{
public:
  LidarSub() : Node("lidar_sub")
  {
    // 参数：默认直接订阅你的 /rslidar_points
    this->declare_parameter<std::string>("input_topic", "/rslidar_points");
    this->declare_parameter<bool>("republish", false);
    this->declare_parameter<std::string>("output_topic", "/points");

    input_topic_  = this->get_parameter("input_topic").as_string();
    republish_    = this->get_parameter("republish").as_bool();
    output_topic_ = this->get_parameter("output_topic").as_string();

    // QoS：点云/传感器通常用 SensorDataQoS
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarSub::on_cloud, this, std::placeholders::_1)
    );

    if (republish_) {
      pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    }

    last_print_time_ = this->now();
    count_ = 0;

    RCLCPP_INFO(this->get_logger(),
      "lidar_sub started. subscribe: %s | republish: %s -> %s",
      input_topic_.c_str(),
      republish_ ? "true" : "false",
      output_topic_.c_str());
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    count_++;

    // 每 1 秒打印一次接收频率和 frame_id
    const auto now = this->now();
    const double dt = (now - last_print_time_).seconds();
    if (dt >= 1.0) {
      const double hz = static_cast<double>(count_) / dt;
      RCLCPP_INFO(this->get_logger(), "Rx PointCloud2: ~%.1f Hz | frame_id: %s | size: %ux%u",
                  hz, msg->header.frame_id.c_str(), msg->width, msg->height);
      count_ = 0;
      last_print_time_ = now;
    }

    if (republish_ && pub_) {
      pub_->publish(*msg);
    }
  }

  std::string input_topic_;
  bool republish_;
  std::string output_topic_;

  rclcpp::Time last_print_time_;
  int count_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSub>());
  rclcpp::shutdown();
  return 0;
}
