// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__STRUCT_H_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MOTOR_ID_FRONT_RIGHT'.
enum
{
  bunker_msgs__msg__BunkerStatus__MOTOR_ID_FRONT_RIGHT = 0
};

/// Constant 'MOTOR_ID_FRONT_LEFT'.
enum
{
  bunker_msgs__msg__BunkerStatus__MOTOR_ID_FRONT_LEFT = 1
};

/// Constant 'MOTOR_ID_REAR_RIGHT'.
enum
{
  bunker_msgs__msg__BunkerStatus__MOTOR_ID_REAR_RIGHT = 2
};

/// Constant 'MOTOR_ID_REAR_LEFT'.
enum
{
  bunker_msgs__msg__BunkerStatus__MOTOR_ID_REAR_LEFT = 3
};

/// Constant 'LIGHT_ID_FRONT'.
enum
{
  bunker_msgs__msg__BunkerStatus__LIGHT_ID_FRONT = 0
};

/// Constant 'LIGHT_ID_REAR'.
enum
{
  bunker_msgs__msg__BunkerStatus__LIGHT_ID_REAR = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'actuator_states'
#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.h"

// Struct defined in msg/BunkerStatus in the package bunker_msgs.
typedef struct bunker_msgs__msg__BunkerStatus
{
  std_msgs__msg__Header header;
  double linear_velocity;
  double angular_velocity;
  uint8_t vehicle_state;
  uint8_t control_mode;
  uint16_t error_code;
  double battery_voltage;
  bunker_msgs__msg__BunkerActuatorState actuator_states[3];
} bunker_msgs__msg__BunkerStatus;

// Struct for a sequence of bunker_msgs__msg__BunkerStatus.
typedef struct bunker_msgs__msg__BunkerStatus__Sequence
{
  bunker_msgs__msg__BunkerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bunker_msgs__msg__BunkerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__STRUCT_H_
