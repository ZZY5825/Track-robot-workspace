// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "bunker_msgs/msg/detail/bunker_status__rosidl_typesupport_introspection_c.h"
#include "bunker_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "bunker_msgs/msg/detail/bunker_status__functions.h"
#include "bunker_msgs/msg/detail/bunker_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `actuator_states`
#include "bunker_msgs/msg/bunker_actuator_state.h"
// Member `actuator_states`
#include "bunker_msgs/msg/detail/bunker_actuator_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  bunker_msgs__msg__BunkerStatus__init(message_memory);
}

void BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_fini_function(void * message_memory)
{
  bunker_msgs__msg__BunkerStatus__fini(message_memory);
}

size_t BunkerStatus__rosidl_typesupport_introspection_c__size_function__BunkerActuatorState__actuator_states(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * BunkerStatus__rosidl_typesupport_introspection_c__get_const_function__BunkerActuatorState__actuator_states(
  const void * untyped_member, size_t index)
{
  const bunker_msgs__msg__BunkerActuatorState ** member =
    (const bunker_msgs__msg__BunkerActuatorState **)(untyped_member);
  return &(*member)[index];
}

void * BunkerStatus__rosidl_typesupport_introspection_c__get_function__BunkerActuatorState__actuator_states(
  void * untyped_member, size_t index)
{
  bunker_msgs__msg__BunkerActuatorState ** member =
    (bunker_msgs__msg__BunkerActuatorState **)(untyped_member);
  return &(*member)[index];
}

static rosidl_typesupport_introspection_c__MessageMember BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vehicle_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, vehicle_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, control_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, battery_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actuator_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerStatus, actuator_states),  // bytes offset in struct
    NULL,  // default value
    BunkerStatus__rosidl_typesupport_introspection_c__size_function__BunkerActuatorState__actuator_states,  // size() function pointer
    BunkerStatus__rosidl_typesupport_introspection_c__get_const_function__BunkerActuatorState__actuator_states,  // get_const(index) function pointer
    BunkerStatus__rosidl_typesupport_introspection_c__get_function__BunkerActuatorState__actuator_states,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_members = {
  "bunker_msgs__msg",  // message namespace
  "BunkerStatus",  // message name
  8,  // number of fields
  sizeof(bunker_msgs__msg__BunkerStatus),
  BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_member_array,  // message members
  BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_type_support_handle = {
  0,
  &BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_bunker_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bunker_msgs, msg, BunkerStatus)() {
  BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bunker_msgs, msg, BunkerActuatorState)();
  if (!BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_type_support_handle.typesupport_identifier) {
    BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BunkerStatus__rosidl_typesupport_introspection_c__BunkerStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
