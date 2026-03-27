// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bunker_msgs:msg/BunkerStatus.idl
// generated code does not contain a copyright notice
#include "bunker_msgs/msg/detail/bunker_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `actuator_states`
#include "bunker_msgs/msg/detail/bunker_actuator_state__functions.h"

bool
bunker_msgs__msg__BunkerStatus__init(bunker_msgs__msg__BunkerStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    bunker_msgs__msg__BunkerStatus__fini(msg);
    return false;
  }
  // linear_velocity
  // angular_velocity
  // vehicle_state
  // control_mode
  // error_code
  // battery_voltage
  // actuator_states
  for (size_t i = 0; i < 3; ++i) {
    if (!bunker_msgs__msg__BunkerActuatorState__init(&msg->actuator_states[i])) {
      bunker_msgs__msg__BunkerStatus__fini(msg);
      return false;
    }
  }
  return true;
}

void
bunker_msgs__msg__BunkerStatus__fini(bunker_msgs__msg__BunkerStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // linear_velocity
  // angular_velocity
  // vehicle_state
  // control_mode
  // error_code
  // battery_voltage
  // actuator_states
  for (size_t i = 0; i < 3; ++i) {
    bunker_msgs__msg__BunkerActuatorState__fini(&msg->actuator_states[i]);
  }
}

bool
bunker_msgs__msg__BunkerStatus__are_equal(const bunker_msgs__msg__BunkerStatus * lhs, const bunker_msgs__msg__BunkerStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // vehicle_state
  if (lhs->vehicle_state != rhs->vehicle_state) {
    return false;
  }
  // control_mode
  if (lhs->control_mode != rhs->control_mode) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // actuator_states
  for (size_t i = 0; i < 3; ++i) {
    if (!bunker_msgs__msg__BunkerActuatorState__are_equal(
        &(lhs->actuator_states[i]), &(rhs->actuator_states[i])))
    {
      return false;
    }
  }
  return true;
}

bool
bunker_msgs__msg__BunkerStatus__copy(
  const bunker_msgs__msg__BunkerStatus * input,
  bunker_msgs__msg__BunkerStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // vehicle_state
  output->vehicle_state = input->vehicle_state;
  // control_mode
  output->control_mode = input->control_mode;
  // error_code
  output->error_code = input->error_code;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // actuator_states
  for (size_t i = 0; i < 3; ++i) {
    if (!bunker_msgs__msg__BunkerActuatorState__copy(
        &(input->actuator_states[i]), &(output->actuator_states[i])))
    {
      return false;
    }
  }
  return true;
}

bunker_msgs__msg__BunkerStatus *
bunker_msgs__msg__BunkerStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerStatus * msg = (bunker_msgs__msg__BunkerStatus *)allocator.allocate(sizeof(bunker_msgs__msg__BunkerStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bunker_msgs__msg__BunkerStatus));
  bool success = bunker_msgs__msg__BunkerStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bunker_msgs__msg__BunkerStatus__destroy(bunker_msgs__msg__BunkerStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bunker_msgs__msg__BunkerStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bunker_msgs__msg__BunkerStatus__Sequence__init(bunker_msgs__msg__BunkerStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerStatus * data = NULL;

  if (size) {
    data = (bunker_msgs__msg__BunkerStatus *)allocator.zero_allocate(size, sizeof(bunker_msgs__msg__BunkerStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bunker_msgs__msg__BunkerStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bunker_msgs__msg__BunkerStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
bunker_msgs__msg__BunkerStatus__Sequence__fini(bunker_msgs__msg__BunkerStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      bunker_msgs__msg__BunkerStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

bunker_msgs__msg__BunkerStatus__Sequence *
bunker_msgs__msg__BunkerStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerStatus__Sequence * array = (bunker_msgs__msg__BunkerStatus__Sequence *)allocator.allocate(sizeof(bunker_msgs__msg__BunkerStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bunker_msgs__msg__BunkerStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bunker_msgs__msg__BunkerStatus__Sequence__destroy(bunker_msgs__msg__BunkerStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bunker_msgs__msg__BunkerStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bunker_msgs__msg__BunkerStatus__Sequence__are_equal(const bunker_msgs__msg__BunkerStatus__Sequence * lhs, const bunker_msgs__msg__BunkerStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bunker_msgs__msg__BunkerStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bunker_msgs__msg__BunkerStatus__Sequence__copy(
  const bunker_msgs__msg__BunkerStatus__Sequence * input,
  bunker_msgs__msg__BunkerStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bunker_msgs__msg__BunkerStatus);
    bunker_msgs__msg__BunkerStatus * data =
      (bunker_msgs__msg__BunkerStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bunker_msgs__msg__BunkerStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bunker_msgs__msg__BunkerStatus__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bunker_msgs__msg__BunkerStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
