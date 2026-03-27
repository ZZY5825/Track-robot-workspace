// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__FUNCTIONS_H_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "bunker_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "bunker_msgs/msg/detail/bunker_actuator_state__struct.h"

/// Initialize msg/BunkerActuatorState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * bunker_msgs__msg__BunkerActuatorState
 * )) before or use
 * bunker_msgs__msg__BunkerActuatorState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerActuatorState__init(bunker_msgs__msg__BunkerActuatorState * msg);

/// Finalize msg/BunkerActuatorState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerActuatorState__fini(bunker_msgs__msg__BunkerActuatorState * msg);

/// Create msg/BunkerActuatorState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * bunker_msgs__msg__BunkerActuatorState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bunker_msgs__msg__BunkerActuatorState *
bunker_msgs__msg__BunkerActuatorState__create();

/// Destroy msg/BunkerActuatorState message.
/**
 * It calls
 * bunker_msgs__msg__BunkerActuatorState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerActuatorState__destroy(bunker_msgs__msg__BunkerActuatorState * msg);

/// Check for msg/BunkerActuatorState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerActuatorState__are_equal(const bunker_msgs__msg__BunkerActuatorState * lhs, const bunker_msgs__msg__BunkerActuatorState * rhs);

/// Copy a msg/BunkerActuatorState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerActuatorState__copy(
  const bunker_msgs__msg__BunkerActuatorState * input,
  bunker_msgs__msg__BunkerActuatorState * output);

/// Initialize array of msg/BunkerActuatorState messages.
/**
 * It allocates the memory for the number of elements and calls
 * bunker_msgs__msg__BunkerActuatorState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerActuatorState__Sequence__init(bunker_msgs__msg__BunkerActuatorState__Sequence * array, size_t size);

/// Finalize array of msg/BunkerActuatorState messages.
/**
 * It calls
 * bunker_msgs__msg__BunkerActuatorState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerActuatorState__Sequence__fini(bunker_msgs__msg__BunkerActuatorState__Sequence * array);

/// Create array of msg/BunkerActuatorState messages.
/**
 * It allocates the memory for the array and calls
 * bunker_msgs__msg__BunkerActuatorState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bunker_msgs__msg__BunkerActuatorState__Sequence *
bunker_msgs__msg__BunkerActuatorState__Sequence__create(size_t size);

/// Destroy array of msg/BunkerActuatorState messages.
/**
 * It calls
 * bunker_msgs__msg__BunkerActuatorState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerActuatorState__Sequence__destroy(bunker_msgs__msg__BunkerActuatorState__Sequence * array);

/// Check for msg/BunkerActuatorState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerActuatorState__Sequence__are_equal(const bunker_msgs__msg__BunkerActuatorState__Sequence * lhs, const bunker_msgs__msg__BunkerActuatorState__Sequence * rhs);

/// Copy an array of msg/BunkerActuatorState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerActuatorState__Sequence__copy(
  const bunker_msgs__msg__BunkerActuatorState__Sequence * input,
  bunker_msgs__msg__BunkerActuatorState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_ACTUATOR_STATE__FUNCTIONS_H_
