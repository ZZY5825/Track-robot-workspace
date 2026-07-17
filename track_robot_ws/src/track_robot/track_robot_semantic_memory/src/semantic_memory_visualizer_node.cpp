#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "track_robot_interfaces/msg/semantic_object_array.hpp"
#include "track_robot_semantic_memory/visualization.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace track_robot_semantic_memory
{

class SemanticMemoryVisualizerNode final : public rclcpp::Node
{
public:
  SemanticMemoryVisualizerNode()
  : Node("semantic_memory_visualizer"),
    registry_(read_max_objects())
  {
    const bool enabled = declare_parameter<bool>("enabled", true);
    const auto input_topic = declare_parameter<std::string>(
      "active_objects_topic", "/semantic_memory/active_objects");
    const auto output_topic = declare_parameter<std::string>(
      "markers_topic", "/semantic_memory/markers");
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      output_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    if (enabled) {
      snapshot_subscription_ = create_subscription<
        track_robot_interfaces::msg::SemanticObjectArray>(
        input_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        std::bind(
          &SemanticMemoryVisualizerNode::on_snapshot, this,
          std::placeholders::_1));
    }
  }

private:
  std::size_t read_max_objects()
  {
    const auto value = declare_parameter<std::int64_t>("max_objects", 256);
    if (value < 1 || value > 256) {
      throw std::invalid_argument("visualizer max_objects must be in [1,256]");
    }
    return static_cast<std::size_t>(value);
  }

  void on_snapshot(
    track_robot_interfaces::msg::SemanticObjectArray::ConstSharedPtr snapshot)
  {
    try {
      marker_publisher_->publish(registry_.update(*snapshot));
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected semantic-memory snapshot: %s", error.what());
    }
  }

  MarkerRegistry registry_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_publisher_;
  rclcpp::Subscription<
    track_robot_interfaces::msg::SemanticObjectArray>::SharedPtr
    snapshot_subscription_;
};

}  // namespace track_robot_semantic_memory

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
      track_robot_semantic_memory::SemanticMemoryVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
