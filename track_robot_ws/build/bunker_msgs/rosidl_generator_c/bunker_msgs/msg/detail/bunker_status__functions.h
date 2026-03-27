// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice

#ifndef BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__FUNCTIONS_H_
#define BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "bunker_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "bunker_msgs/msg/detail/bunker_status__struct.h"

/// Initialize msg/BunkerStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * bunker_msgs__msg__BunkerStatus
 * )) before or use
 * bunker_msgs__msg__BunkerStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerStatus__init(bunker_msgs__msg__BunkerStatus * msg);

/// Finalize msg/BunkerStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerStatus__fini(bunker_msgs__msg__BunkerStatus * msg);

/// Create msg/BunkerStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * bunker_msgs__msg__BunkerStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bunker_msgs__msg__BunkerStatus *
bunker_msgs__msg__BunkerStatus__create();

/// Destroy msg/BunkerStatus message.
/**
 * It calls
 * bunker_msgs__msg__BunkerStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerStatus__destroy(bunker_msgs__msg__BunkerStatus * msg);

/// Check for msg/BunkerStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerStatus__are_equal(const bunker_msgs__msg__BunkerStatus * lhs, const bunker_msgs__msg__BunkerStatus * rhs);

/// Copy a msg/BunkerStatus message.
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
bunker_msgs__msg__BunkerStatus__copy(
  const bunker_msgs__msg__BunkerStatus * input,
  bunker_msgs__msg__BunkerStatus * output);

/// Initialize array of msg/BunkerStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * bunker_msgs__msg__BunkerStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerStatus__Sequence__init(bunker_msgs__msg__BunkerStatus__Sequence * array, size_t size);

/// Finalize array of msg/BunkerStatus messages.
/**
 * It calls
 * bunker_msgs__msg__BunkerStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerStatus__Sequence__fini(bunker_msgs__msg__BunkerStatus__Sequence * array);

/// Create array of msg/BunkerStatus messages.
/**
 * It allocates the memory for the array and calls
 * bunker_msgs__msg__BunkerStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bunker_msgs__msg__BunkerStatus__Sequence *
bunker_msgs__msg__BunkerStatus__Sequence__create(size_t size);

/// Destroy array of msg/BunkerStatus messages.
/**
 * It calls
 * bunker_msgs__msg__BunkerStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
void
bunker_msgs__msg__BunkerStatus__Sequence__destroy(bunker_msgs__msg__BunkerStatus__Sequence * array);

/// Check for msg/BunkerStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bunker_msgs
bool
bunker_msgs__msg__BunkerStatus__Sequence__are_equal(const bunker_msgs__msg__BunkerStatus__Sequence * lhs, const bunker_msgs__msg__BunkerStatus__Sequence * rhs);

/// Copy an array of msg/BunkerStatus messages.
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
bunker_msgs__msg__BunkerStatus__Sequence__copy(
  const bunker_msgs__msg__BunkerStatus__Sequence * input,
  bunker_msgs__msg__BunkerStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BUNKER_MSGS__MSG__DETAIL__BUNKER_STATUS__FUNCTIONS_H_
