// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "bunker_msgs/msg/detail/bunker_actuator_state__rosidl_typesupport_introspection_c.h"
#include "bunker_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "bunker_msgs/msg/detail/bunker_actuator_state__functions.h"
#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  bunker_msgs__msg__BunkerActuatorState__init(message_memory);
}

void BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_fini_function(void * message_memory)
{
  bunker_msgs__msg__BunkerActuatorState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_member_array[8] = {
  {
    "motor_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, motor_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rpm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, rpm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pulse_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, pulse_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, driver_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, driver_temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, motor_temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerActuatorState, driver_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_members = {
  "bunker_msgs__msg",  // message namespace
  "BunkerActuatorState",  // message name
  8,  // number of fields
  sizeof(bunker_msgs__msg__BunkerActuatorState),
  BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_member_array,  // message members
  BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_init_function,  // function to initialize message memory (memory has to be allocated)
  BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_type_support_handle = {
  0,
  &BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_bunker_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bunker_msgs, msg, BunkerActuatorState)() {
  if (!BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_type_support_handle.typesupport_identifier) {
    BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BunkerActuatorState__rosidl_typesupport_introspection_c__BunkerActuatorState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
