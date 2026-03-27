// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rslidar_msg:msg/RslidarPacket.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rslidar_msg/msg/detail/rslidar_packet__rosidl_typesupport_introspection_c.h"
#include "rslidar_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rslidar_msg/msg/detail/rslidar_packet__functions.h"
#include "rslidar_msg/msg/detail/rslidar_packet__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rslidar_msg__msg__RslidarPacket__init(message_memory);
}

void RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_fini_function(void * message_memory)
{
  rslidar_msg__msg__RslidarPacket__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rslidar_msg__msg__RslidarPacket, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_difop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rslidar_msg__msg__RslidarPacket, is_difop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_frame_begin",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rslidar_msg__msg__RslidarPacket, is_frame_begin),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rslidar_msg__msg__RslidarPacket, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_members = {
  "rslidar_msg__msg",  // message namespace
  "RslidarPacket",  // message name
  4,  // number of fields
  sizeof(rslidar_msg__msg__RslidarPacket),
  RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_member_array,  // message members
  RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_init_function,  // function to initialize message memory (memory has to be allocated)
  RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_type_support_handle = {
  0,
  &RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rslidar_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rslidar_msg, msg, RslidarPacket)() {
  RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_type_support_handle.typesupport_identifier) {
    RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RslidarPacket__rosidl_typesupport_introspection_c__RslidarPacket_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
