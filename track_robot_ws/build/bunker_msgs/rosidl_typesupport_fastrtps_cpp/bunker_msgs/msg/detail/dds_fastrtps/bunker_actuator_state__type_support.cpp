// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
// generated code does not contain a copyright notice
#include "bunker_msgs/msg/detail/bunker_actuator_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace bunker_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bunker_msgs
cdr_serialize(
  const bunker_msgs::msg::BunkerActuatorState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: motor_id
  cdr << ros_message.motor_id;
  // Member: rpm
  cdr << ros_message.rpm;
  // Member: current
  cdr << ros_message.current;
  // Member: pulse_count
  cdr << ros_message.pulse_count;
  // Member: driver_voltage
  cdr << ros_message.driver_voltage;
  // Member: driver_temperature
  cdr << ros_message.driver_temperature;
  // Member: motor_temperature
  cdr << ros_message.motor_temperature;
  // Member: driver_state
  cdr << ros_message.driver_state;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bunker_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  bunker_msgs::msg::BunkerActuatorState & ros_message)
{
  // Member: motor_id
  cdr >> ros_message.motor_id;

  // Member: rpm
  cdr >> ros_message.rpm;

  // Member: current
  cdr >> ros_message.current;

  // Member: pulse_count
  cdr >> ros_message.pulse_count;

  // Member: driver_voltage
  cdr >> ros_message.driver_voltage;

  // Member: driver_temperature
  cdr >> ros_message.driver_temperature;

  // Member: motor_temperature
  cdr >> ros_message.motor_temperature;

  // Member: driver_state
  cdr >> ros_message.driver_state;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bunker_msgs
get_serialized_size(
  const bunker_msgs::msg::BunkerActuatorState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: motor_id
  {
    size_t item_size = sizeof(ros_message.motor_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rpm
  {
    size_t item_size = sizeof(ros_message.rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current
  {
    size_t item_size = sizeof(ros_message.current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pulse_count
  {
    size_t item_size = sizeof(ros_message.pulse_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: driver_voltage
  {
    size_t item_size = sizeof(ros_message.driver_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: driver_temperature
  {
    size_t item_size = sizeof(ros_message.driver_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor_temperature
  {
    size_t item_size = sizeof(ros_message.motor_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: driver_state
  {
    size_t item_size = sizeof(ros_message.driver_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bunker_msgs
max_serialized_size_BunkerActuatorState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: motor_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: rpm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: pulse_count
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: driver_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: driver_temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: motor_temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: driver_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _BunkerActuatorState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const bunker_msgs::msg::BunkerActuatorState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BunkerActuatorState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<bunker_msgs::msg::BunkerActuatorState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BunkerActuatorState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const bunker_msgs::msg::BunkerActuatorState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BunkerActuatorState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_BunkerActuatorState(full_bounded, 0);
}

static message_type_support_callbacks_t _BunkerActuatorState__callbacks = {
  "bunker_msgs::msg",
  "BunkerActuatorState",
  _BunkerActuatorState__cdr_serialize,
  _BunkerActuatorState__cdr_deserialize,
  _BunkerActuatorState__get_serialized_size,
  _BunkerActuatorState__max_serialized_size
};

static rosidl_message_type_support_t _BunkerActuatorState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BunkerActuatorState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace bunker_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_bunker_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<bunker_msgs::msg::BunkerActuatorState>()
{
  return &bunker_msgs::msg::typesupport_fastrtps_cpp::_BunkerActuatorState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bunker_msgs, msg, BunkerActuatorState)() {
  return &bunker_msgs::msg::typesupport_fastrtps_cpp::_BunkerActuatorState__handle;
}

#ifdef __cplusplus
}
#endif
