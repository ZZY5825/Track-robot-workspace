#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros
{

inline std::shared_ptr<rclcpp::Node> & node()
{
  static std::shared_ptr<rclcpp::Node> node_handle;
  return node_handle;
}

inline std::string to_ros2_param_name(std::string name)
{
  std::replace(name.begin(), name.end(), '/', '.');
  return name;
}

class Time
{
public:
  Time() = default;
  explicit Time(const builtin_interfaces::msg::Time & stamp) : stamp_(stamp) {}

  Time fromSec(double seconds) const
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(seconds);
    stamp.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(stamp.sec)) * 1e9);
    return Time(stamp);
  }

  double toSec() const
  {
    return static_cast<double>(stamp_.sec) + static_cast<double>(stamp_.nanosec) * 1e-9;
  }

  operator builtin_interfaces::msg::Time() const { return stamp_; }

private:
  builtin_interfaces::msg::Time stamp_{};
};

inline double toSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline Time fromSec(double seconds)
{
  return Time().fromSec(seconds);
}

inline void init(int argc, char ** argv, const std::string & name)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  node() = std::make_shared<rclcpp::Node>(name, options);
}

inline bool ok()
{
  return rclcpp::ok();
}

inline void spinOnce()
{
  if (node()) {
    rclcpp::spin_some(node());
  }
}

inline void shutdown()
{
  rclcpp::shutdown();
}

class Rate
{
public:
  explicit Rate(double hz) : rate_(hz) {}
  void sleep() { rate_.sleep(); }

private:
  rclcpp::Rate rate_;
};

class AsyncSpinner
{
public:
  explicit AsyncSpinner(int) {}
  void start() {}
};

class Publisher
{
public:
  Publisher() = default;

  template <typename MsgT>
  static Publisher make(const typename rclcpp::Publisher<MsgT>::SharedPtr & publisher)
  {
    Publisher out;
    out.publish_fn_ = [publisher](const void * msg) {
      publisher->publish(*static_cast<const MsgT *>(msg));
    };
    return out;
  }

  template <typename MsgT>
  void publish(const MsgT & msg) const
  {
    if (publish_fn_) {
      publish_fn_(&msg);
    }
  }

private:
  std::function<void(const void *)> publish_fn_;
};

class Subscriber
{
public:
  Subscriber() = default;
  explicit Subscriber(const rclcpp::SubscriptionBase::SharedPtr & subscription)
  : subscription_(subscription) {}

private:
  rclcpp::SubscriptionBase::SharedPtr subscription_;
};

class NodeHandle
{
public:
  explicit NodeHandle(const std::string & = "") {}

  template <typename T>
  void param(const std::string & name, T & value, const T & default_value)
  {
    const std::string ros2_name = to_ros2_param_name(name);
    if (node()->has_parameter(ros2_name)) {
      node()->get_parameter(ros2_name, value);
    } else {
      value = node()->declare_parameter<T>(ros2_name, default_value);
    }
  }

  template <typename MsgT, typename CallbackT>
  Subscriber subscribe(const std::string & topic, size_t queue_size, CallbackT && callback)
  {
    const size_t depth = std::min<size_t>(queue_size, 1000);
    auto wrapped_callback = [callback](typename MsgT::ConstSharedPtr msg) {
      callback(msg);
    };
    auto subscription = node()->create_subscription<MsgT>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort(),
      wrapped_callback);
    return Subscriber(subscription);
  }

  template <typename MsgT>
  Publisher advertise(const std::string & topic, size_t queue_size)
  {
    const size_t depth = std::min<size_t>(queue_size, 1000);
    auto publisher = node()->create_publisher<MsgT>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(depth)).reliable());
    return Publisher::make<MsgT>(publisher);
  }
};

}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("point_lio"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("point_lio"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("point_lio"), __VA_ARGS__)
