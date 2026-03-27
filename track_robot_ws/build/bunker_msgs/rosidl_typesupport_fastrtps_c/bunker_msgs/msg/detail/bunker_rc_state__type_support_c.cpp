// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from bunker_msgs:msg/BunkerRCState.idl
// generated code does not contain a copyright notice
#include "bunker_msgs/msg/detail/bunker_rc_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "bunker_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bunker_msgs/msg/detail/bunker_rc_state__struct.h"
#include "bunker_msgs/msg/detail/bunker_rc_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _BunkerRCState__ros_msg_type = bunker_msgs__msg__BunkerRCState;

static bool _BunkerRCState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BunkerRCState__ros_msg_type * ros_message = static_cast<const _BunkerRCState__ros_msg_type *>(untyped_ros_message);
  // Field name: swa
  {
    cdr << ros_message->swa;
  }

  // Field name: swb
  {
    cdr << ros_message->swb;
  }

  // Field name: swc
  {
    cdr << ros_message->swc;
  }

  // Field name: swd
  {
    cdr << ros_message->swd;
  }

  // Field name: stick_right_v
  {
    cdr << ros_message->stick_right_v;
  }

  // Field name: stick_right_h
  {
    cdr << ros_message->stick_right_h;
  }

  // Field name: stick_left_v
  {
    cdr << ros_message->stick_left_v;
  }

  // Field name: stick_left_h
  {
    cdr << ros_message->stick_left_h;
  }

  // Field name: var_a
  {
    cdr << ros_message->var_a;
  }

  return true;
}

static bool _BunkerRCState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BunkerRCState__ros_msg_type * ros_message = static_cast<_BunkerRCState__ros_msg_type *>(untyped_ros_message);
  // Field name: swa
  {
    cdr >> ros_message->swa;
  }

  // Field name: swb
  {
    cdr >> ros_message->swb;
  }

  // Field name: swc
  {
    cdr >> ros_message->swc;
  }

  // Field name: swd
  {
    cdr >> ros_message->swd;
  }

  // Field name: stick_right_v
  {
    cdr >> ros_message->stick_right_v;
  }

  // Field name: stick_right_h
  {
    cdr >> ros_message->stick_right_h;
  }

  // Field name: stick_left_v
  {
    cdr >> ros_message->stick_left_v;
  }

  // Field name: stick_left_h
  {
    cdr >> ros_message->stick_left_h;
  }

  // Field name: var_a
  {
    cdr >> ros_message->var_a;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bunker_msgs
size_t get_serialized_size_bunker_msgs__msg__BunkerRCState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BunkerRCState__ros_msg_type * ros_message = static_cast<const _BunkerRCState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name swa
  {
    size_t item_size = sizeof(ros_message->swa);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swb
  {
    size_t item_size = sizeof(ros_message->swb);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swc
  {
    size_t item_size = sizeof(ros_message->swc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swd
  {
    size_t item_size = sizeof(ros_message->swd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stick_right_v
  {
    size_t item_size = sizeof(ros_message->stick_right_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stick_right_h
  {
    size_t item_size = sizeof(ros_message->stick_right_h);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stick_left_v
  {
    size_t item_size = sizeof(ros_message->stick_left_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stick_left_h
  {
    size_t item_size = sizeof(ros_message->stick_left_h);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name var_a
  {
    size_t item_size = sizeof(ros_message->var_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BunkerRCState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_bunker_msgs__msg__BunkerRCState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bunker_msgs
size_t max_serialized_size_bunker_msgs__msg__BunkerRCState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: swa
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: swb
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: swc
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: swd
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: stick_right_v
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: stick_right_h
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: stick_left_v
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: stick_left_h
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: var_a
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _BunkerRCState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_bunker_msgs__msg__BunkerRCState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BunkerRCState = {
  "bunker_msgs::msg",
  "BunkerRCState",
  _BunkerRCState__cdr_serialize,
  _BunkerRCState__cdr_deserialize,
  _BunkerRCState__get_serialized_size,
  _BunkerRCState__max_serialized_size
};

static rosidl_message_type_support_t _BunkerRCState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BunkerRCState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bunker_msgs, msg, BunkerRCState)() {
  return &_BunkerRCState__type_support;
}

#if defined(__cplusplus)
}
#endif
