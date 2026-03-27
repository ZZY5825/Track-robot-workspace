// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "bunker_msgs/msg/detail/bunker_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace bunker_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BunkerStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) bunker_msgs::msg::BunkerStatus(_init);
}

void BunkerStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<bunker_msgs::msg::BunkerStatus *>(message_memory);
  typed_message->~BunkerStatus();
}

size_t size_function__BunkerStatus__actuator_states(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BunkerStatus__actuator_states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bunker_msgs::msg::BunkerActuatorState, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BunkerStatus__actuator_states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bunker_msgs::msg::BunkerActuatorState, 3> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BunkerStatus_message_member_array[8] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, linear_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, angular_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vehicle_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, vehicle_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "control_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, control_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error_code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, error_code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, battery_voltage),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "actuator_states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<bunker_msgs::msg::BunkerActuatorState>(),  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs::msg::BunkerStatus, actuator_states),  // bytes offset in struct
    nullptr,  // default value
    size_function__BunkerStatus__actuator_states,  // size() function pointer
    get_const_function__BunkerStatus__actuator_states,  // get_const(index) function pointer
    get_function__BunkerStatus__actuator_states,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BunkerStatus_message_members = {
  "bunker_msgs::msg",  // message namespace
  "BunkerStatus",  // message name
  8,  // number of fields
  sizeof(bunker_msgs::msg::BunkerStatus),
  BunkerStatus_message_member_array,  // message members
  BunkerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  BunkerStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BunkerStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BunkerStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace bunker_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bunker_msgs::msg::BunkerStatus>()
{
  return &::bunker_msgs::msg::rosidl_typesupport_introspection_cpp::BunkerStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bunker_msgs, msg, BunkerStatus)() {
  return &::bunker_msgs::msg::rosidl_typesupport_introspection_cpp::BunkerStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
