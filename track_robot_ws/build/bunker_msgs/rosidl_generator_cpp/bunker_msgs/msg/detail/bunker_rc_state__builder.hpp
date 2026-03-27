// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bunker_msgs:msg/BunkerRCState.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_RC_STATE__BUILDER_HPP_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_RC_STATE__BUILDER_HPP_

#include "bunker_msgs/msg/detail/bunker_rc_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace bunker_msgs
{

namespace msg
{

namespace builder
{

class Init_BunkerRCState_var_a
{
public:
  explicit Init_BunkerRCState_var_a(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  ::bunker_msgs::msg::BunkerRCState var_a(::bunker_msgs::msg::BunkerRCState::_var_a_type arg)
  {
    msg_.var_a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_stick_left_h
{
public:
  explicit Init_BunkerRCState_stick_left_h(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  Init_BunkerRCState_var_a stick_left_h(::bunker_msgs::msg::BunkerRCState::_stick_left_h_type arg)
  {
    msg_.stick_left_h = std::move(arg);
    return Init_BunkerRCState_var_a(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_stick_left_v
{
public:
  explicit Init_BunkerRCState_stick_left_v(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  Init_BunkerRCState_stick_left_h stick_left_v(::bunker_msgs::msg::BunkerRCState::_stick_left_v_type arg)
  {
    msg_.stick_left_v = std::move(arg);
    return Init_BunkerRCState_stick_left_h(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_stick_right_h
{
public:
  explicit Init_BunkerRCState_stick_right_h(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  Init_BunkerRCState_stick_left_v stick_right_h(::bunker_msgs::msg::BunkerRCState::_stick_right_h_type arg)
  {
    msg_.stick_right_h = std::move(arg);
    return Init_BunkerRCState_stick_left_v(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_stick_right_v
{
public:
  explicit Init_BunkerRCState_stick_right_v(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  Init_BunkerRCState_stick_right_h stick_right_v(::bunker_msgs::msg::BunkerRCState::_stick_right_v_type arg)
  {
    msg_.stick_right_v = std::move(arg);
    return Init_BunkerRCState_stick_right_h(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_swd
{
public:
  explicit Init_BunkerRCState_swd(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  Init_BunkerRCState_stick_right_v swd(::bunker_msgs::msg::BunkerRCState::_swd_type arg)
  {
    msg_.swd = std::move(arg);
    return Init_BunkerRCState_stick_right_v(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_swc
{
public:
  explicit Init_BunkerRCState_swc(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  Init_BunkerRCState_swd swc(::bunker_msgs::msg::BunkerRCState::_swc_type arg)
  {
    msg_.swc = std::move(arg);
    return Init_BunkerRCState_swd(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_swb
{
public:
  explicit Init_BunkerRCState_swb(::bunker_msgs::msg::BunkerRCState & msg)
  : msg_(msg)
  {}
  Init_BunkerRCState_swc swb(::bunker_msgs::msg::BunkerRCState::_swb_type arg)
  {
    msg_.swb = std::move(arg);
    return Init_BunkerRCState_swc(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

class Init_BunkerRCState_swa
{
public:
  Init_BunkerRCState_swa()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BunkerRCState_swb swa(::bunker_msgs::msg::BunkerRCState::_swa_type arg)
  {
    msg_.swa = std::move(arg);
    return Init_BunkerRCState_swb(msg_);
  }

private:
  ::bunker_msgs::msg::BunkerRCState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bunker_msgs::msg::BunkerRCState>()
{
  return bunker_msgs::msg::builder::Init_BunkerRCState_swa();
}

}  // namespace bunker_msgs

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_RC_STATE__BUILDER_HPP_
