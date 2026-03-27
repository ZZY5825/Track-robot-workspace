// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__BUILDER_HPP_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__BUILDER_HPP_

#include "bunker_msgs/msg/detail/bunker_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace bunker_msgs
{

namespace msg
{

namespace builder
{

class Init_BunkerStatus_actuator_states
{
public:
  explicit Init_BunkerStatus_actuator_states(::bunker_msgs::msg::BunkerStatus & msg)
  : msg_(msg)
  {}
  ::bunker_msgs::msg::BunkerStatus actuator_states(::bunker_msgs::msg::BunkerStatus::_actuator_states_type arg)
  {
    msg_.actuator_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

class Init_BunkerStatus_battery_voltage
{
public:
  explicit Init_BunkerStatus_battery_voltage(::bunker_msgs::msg::BunkerStatus & msg)
  : msg_(msg)
  {}
  Init_BunkerStatus_actuator_states battery_voltage(::bunker_msgs::msg::BunkerStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_BunkerStatus_actuator_states(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

class Init_BunkerStatus_error_code
{
public:
  explicit Init_BunkerStatus_error_code(::bunker_msgs::msg::BunkerStatus & msg)
  : msg_(msg)
  {}
  Init_BunkerStatus_battery_voltage error_code(::bunker_msgs::msg::BunkerStatus::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_BunkerStatus_battery_voltage(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

class Init_BunkerStatus_control_mode
{
public:
  explicit Init_BunkerStatus_control_mode(::bunker_msgs::msg::BunkerStatus & msg)
  : msg_(msg)
  {}
  Init_BunkerStatus_error_code control_mode(::bunker_msgs::msg::BunkerStatus::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_BunkerStatus_error_code(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

class Init_BunkerStatus_vehicle_state
{
public:
  explicit Init_BunkerStatus_vehicle_state(::bunker_msgs::msg::BunkerStatus & msg)
  : msg_(msg)
  {}
  Init_BunkerStatus_control_mode vehicle_state(::bunker_msgs::msg::BunkerStatus::_vehicle_state_type arg)
  {
    msg_.vehicle_state = std::move(arg);
    return Init_BunkerStatus_control_mode(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

class Init_BunkerStatus_angular_velocity
{
public:
  explicit Init_BunkerStatus_angular_velocity(::bunker_msgs::msg::BunkerStatus & msg)
  : msg_(msg)
  {}
  Init_BunkerStatus_vehicle_state angular_velocity(::bunker_msgs::msg::BunkerStatus::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_BunkerStatus_vehicle_state(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

class Init_BunkerStatus_linear_velocity
{
public:
  explicit Init_BunkerStatus_linear_velocity(::bunker_msgs::msg::BunkerStatus & msg)
  : msg_(msg)
  {}
  Init_BunkerStatus_angular_velocity linear_velocity(::bunker_msgs::msg::BunkerStatus::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_BunkerStatus_angular_velocity(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

class Init_BunkerStatus_header
{
public:
  Init_BunkerStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BunkerStatus_linear_velocity header(::bunker_msgs::msg::BunkerStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BunkerStatus_linear_velocity(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bunker_msgs::msg::BunkerStatus>()
{
  return bunker_msgs::msg::builder::Init_BunkerStatus_header();
}

}  // namespace bunker_msgs

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__BUILDER_HPP_
