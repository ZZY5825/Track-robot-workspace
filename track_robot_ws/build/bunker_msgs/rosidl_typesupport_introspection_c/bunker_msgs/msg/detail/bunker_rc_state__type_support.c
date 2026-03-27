// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from bunker_msgs:msg/BunkerRCState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "bunker_msgs/msg/detail/bunker_rc_state__rosidl_typesupport_introspection_c.h"
#include "bunker_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "bunker_msgs/msg/detail/bunker_rc_state__functions.h"
#include "bunker_msgs/msg/detail/bunker_rc_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  bunker_msgs__msg__BunkerRCState__init(message_memory);
}

void BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_fini_function(void * message_memory)
{
  bunker_msgs__msg__BunkerRCState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_member_array[9] = {
  {
    "swa",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, swa),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "swb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, swb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "swc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, swc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "swd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, swd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stick_right_v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, stick_right_v),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stick_right_h",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, stick_right_h),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stick_left_v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, stick_left_v),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stick_left_h",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, stick_left_h),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "var_a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bunker_msgs__msg__BunkerRCState, var_a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_members = {
  "bunker_msgs__msg",  // message namespace
  "BunkerRCState",  // message name
  9,  // number of fields
  sizeof(bunker_msgs__msg__BunkerRCState),
  BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_member_array,  // message members
  BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_init_function,  // function to initialize message memory (memory has to be allocated)
  BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_type_support_handle = {
  0,
  &BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_bunker_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bunker_msgs, msg, BunkerRCState)() {
  if (!BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_type_support_handle.typesupport_identifier) {
    BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BunkerRCState__rosidl_typesupport_introspection_c__BunkerRCState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
