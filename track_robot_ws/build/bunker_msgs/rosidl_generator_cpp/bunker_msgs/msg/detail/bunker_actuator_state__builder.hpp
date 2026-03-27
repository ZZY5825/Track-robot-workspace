// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__BUILDER_HPP_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__BUILDER_HPP_

#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace bunker_msgs
{

namespace msg
{

namespace builder
{

class Init_BunkerActuatorState_driver_state
{
public:
  explicit Init_BunkerActuatorState_driver_state(::bunker_msgs::msg::BunkerActuatorState & msg)
  : msg_(msg)
  {}
  ::bunker_msgs::msg::BunkerActuatorState driver_state(::bunker_msgs::msg::BunkerActuatorState::_driver_state_type arg)
  {
    msg_.driver_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

class Init_BunkerActuatorState_motor_temperature
{
public:
  explicit Init_BunkerActuatorState_motor_temperature(::bunker_msgs::msg::BunkerActuatorState & msg)
  : msg_(msg)
  {}
  Init_BunkerActuatorState_driver_state motor_temperature(::bunker_msgs::msg::BunkerActuatorState::_motor_temperature_type arg)
  {
    msg_.motor_temperature = std::move(arg);
    return Init_BunkerActuatorState_driver_state(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

class Init_BunkerActuatorState_driver_temperature
{
public:
  explicit Init_BunkerActuatorState_driver_temperature(::bunker_msgs::msg::BunkerActuatorState & msg)
  : msg_(msg)
  {}
  Init_BunkerActuatorState_motor_temperature driver_temperature(::bunker_msgs::msg::BunkerActuatorState::_driver_temperature_type arg)
  {
    msg_.driver_temperature = std::move(arg);
    return Init_BunkerActuatorState_motor_temperature(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

class Init_BunkerActuatorState_driver_voltage
{
public:
  explicit Init_BunkerActuatorState_driver_voltage(::bunker_msgs::msg::BunkerActuatorState & msg)
  : msg_(msg)
  {}
  Init_BunkerActuatorState_driver_temperature driver_voltage(::bunker_msgs::msg::BunkerActuatorState::_driver_voltage_type arg)
  {
    msg_.driver_voltage = std::move(arg);
    return Init_BunkerActuatorState_driver_temperature(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

class Init_BunkerActuatorState_pulse_count
{
public:
  explicit Init_BunkerActuatorState_pulse_count(::bunker_msgs::msg::BunkerActuatorState & msg)
  : msg_(msg)
  {}
  Init_BunkerActuatorState_driver_voltage pulse_count(::bunker_msgs::msg::BunkerActuatorState::_pulse_count_type arg)
  {
    msg_.pulse_count = std::move(arg);
    return Init_BunkerActuatorState_driver_voltage(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

class Init_BunkerActuatorState_current
{
public:
  explicit Init_BunkerActuatorState_current(::bunker_msgs::msg::BunkerActuatorState & msg)
  : msg_(msg)
  {}
  Init_BunkerActuatorState_pulse_count current(::bunker_msgs::msg::BunkerActuatorState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BunkerActuatorState_pulse_count(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

class Init_BunkerActuatorState_rpm
{
public:
  explicit Init_BunkerActuatorState_rpm(::bunker_msgs::msg::BunkerActuatorState & msg)
  : msg_(msg)
  {}
  Init_BunkerActuatorState_current rpm(::bunker_msgs::msg::BunkerActuatorState::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_BunkerActuatorState_current(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

class Init_BunkerActuatorState_motor_id
{
public:
  Init_BunkerActuatorState_motor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BunkerActuatorState_rpm motor_id(::bunker_msgs::msg::BunkerActuatorState::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_BunkerActuatorState_rpm(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerActuatorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bunker_msgs::msg::BunkerActuatorState>()
{
  return bunker_msgs::msg::builder::Init_BunkerActuatorState_motor_id();
}

}  // namespace bunker_msgs

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__BUILDER_HPP_
