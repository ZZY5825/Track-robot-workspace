// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__TRAITS_HPP_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__TRAITS_HPP_

#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bunker_msgs::msg::BunkerActuatorState>()
{
  return "bunker_msgs::msg::BunkerActuatorState";
}

template<>
inline const char * name<bunker_msgs::msg::BunkerActuatorState>()
{
  return "bunker_msgs/msg/BunkerActuatorState";
}

template<>
struct has_fixed_size<bunker_msgs::msg::BunkerActuatorState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<bunker_msgs::msg::BunkerActuatorState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<bunker_msgs::msg::BunkerActuatorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__TRAITS_HPP_
