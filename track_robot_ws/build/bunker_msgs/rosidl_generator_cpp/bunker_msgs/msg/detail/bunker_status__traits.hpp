// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__TRAITS_HPP_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__TRAITS_HPP_

#include "bunker_msgs/msg/detail/bunker_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'actuator_states'
#include "bunker_msgs/msg/detail/bunker_actuator_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bunker_msgs::msg::BunkerStatus>()
{
  return "bunker_msgs::msg::BunkerStatus";
}

template<>
inline const char * name<bunker_msgs::msg::BunkerStatus>()
{
  return "bunker_msgs/msg/BunkerStatus";
}

template<>
struct has_fixed_size<bunker_msgs::msg::BunkerStatus>
  : std::integral_constant<bool, has_fixed_size<bunker_msgs::msg::BunkerActuatorState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<bunker_msgs::msg::BunkerStatus>
  : std::integral_constant<bool, has_bounded_size<bunker_msgs::msg::BunkerActuatorState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<bunker_msgs::msg::BunkerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__TRAITS_HPP_
