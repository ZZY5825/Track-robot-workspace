// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bunker_msgs:msg/BunkerRCState.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_RC_STATE__STRUCT_H_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_RC_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/BunkerRCState in the package bunker_msgs.
typedef struct bunker_msgs__msg__BunkerRCState
{
  uint8_t swa;
  uint8_t swb;
  uint8_t swc;
  uint8_t swd;
  int8_t stick_right_v;
  int8_t stick_right_h;
  int8_t stick_left_v;
  int8_t stick_left_h;
  int8_t var_a;
} bunker_msgs__msg__BunkerRCState;

// Struct for a sequence of bunker_msgs__msg__BunkerRCState.
typedef struct bunker_msgs__msg__BunkerRCState__Sequence
{
  bunker_msgs__msg__BunkerRCState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bunker_msgs__msg__BunkerRCState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_RC_STATE__STRUCT_H_
