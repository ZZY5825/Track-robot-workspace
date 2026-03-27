// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
// generated code does not contain a copyright notice
#include "bunker_msgs/msg/detail/bunker_actuator_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "bunker_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.h"
#include "bunker_msgs/msg/detail/bunker_actuator_state__functions.h"
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


using _BunkerActuatorState__ros_msg_type = bunker_msgs__msg__BunkerActuatorState;

static bool _BunkerActuatorState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BunkerActuatorState__ros_msg_type * ros_message = static_cast<const _BunkerActuatorState__ros_msg_type *>(untyped_ros_message);
  // Field name: motor_id
  {
    cdr << ros_message->motor_id;
  }

  // Field name: rpm
  {
    cdr << ros_message->rpm;
  }

  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: pulse_count
  {
    cdr << ros_message->pulse_count;
  }

  // Field name: driver_voltage
  {
    cdr << ros_message->driver_voltage;
  }

  // Field name: driver_temperature
  {
    cdr << ros_message->driver_temperature;
  }

  // Field name: motor_temperature
  {
    cdr << ros_message->motor_temperature;
  }

  // Field name: driver_state
  {
    cdr << ros_message->driver_state;
  }

  return true;
}

static bool _BunkerActuatorState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BunkerActuatorState__ros_msg_type * ros_message = static_cast<_BunkerActuatorState__ros_msg_type *>(untyped_ros_message);
  // Field name: motor_id
  {
    cdr >> ros_message->motor_id;
  }

  // Field name: rpm
  {
    cdr >> ros_message->rpm;
  }

  // Field name: current
  {
    cdr >> ros_message->current;
  }

  // Field name: pulse_count
  {
    cdr >> ros_message->pulse_count;
  }

  // Field name: driver_voltage
  {
    cdr >> ros_message->driver_voltage;
  }

  // Field name: driver_temperature
  {
    cdr >> ros_message->driver_temperature;
  }

  // Field name: motor_temperature
  {
    cdr >> ros_message->motor_temperature;
  }

  // Field name: driver_state
  {
    cdr >> ros_message->driver_state;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bunker_msgs
size_t get_serialized_size_bunker_msgs__msg__BunkerActuatorState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BunkerActuatorState__ros_msg_type * ros_message = static_cast<const _BunkerActuatorState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name motor_id
  {
    size_t item_size = sizeof(ros_message->motor_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rpm
  {
    size_t item_size = sizeof(ros_message->rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pulse_count
  {
    size_t item_size = sizeof(ros_message->pulse_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name driver_voltage
  {
    size_t item_size = sizeof(ros_message->driver_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name driver_temperature
  {
    size_t item_size = sizeof(ros_message->driver_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_temperature
  {
    size_t item_size = sizeof(ros_message->motor_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name driver_state
  {
    size_t item_size = sizeof(ros_message->driver_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BunkerActuatorState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_bunker_msgs__msg__BunkerActuatorState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bunker_msgs
size_t max_serialized_size_bunker_msgs__msg__BunkerActuatorState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: motor_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: pulse_count
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: driver_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: driver_temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motor_temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: driver_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _BunkerActuatorState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_bunker_msgs__msg__BunkerActuatorState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BunkerActuatorState = {
  "bunker_msgs::msg",
  "BunkerActuatorState",
  _BunkerActuatorState__cdr_serialize,
  _BunkerActuatorState__cdr_deserialize,
  _BunkerActuatorState__get_serialized_size,
  _BunkerActuatorState__max_serialized_size
};

static rosidl_message_type_support_t _BunkerActuatorState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BunkerActuatorState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bunker_msgs, msg, BunkerActuatorState)() {
  return &_BunkerActuatorState__type_support;
}

#if defined(__cplusplus)
}
#endif
