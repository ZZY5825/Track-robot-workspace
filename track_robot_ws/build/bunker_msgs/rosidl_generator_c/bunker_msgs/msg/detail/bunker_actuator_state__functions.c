// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bunker_msgs:msg/BunkerActuatorState.idl
// generated code does not contain a copyright notice
#include "bunker_msgs/msg/detail/bunker_actuator_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
bunker_msgs__msg__BunkerActuatorState__init(bunker_msgs__msg__BunkerActuatorState * msg)
{
  if (!msg) {
    return false;
  }
  // motor_id
  // rpm
  // current
  // pulse_count
  // driver_voltage
  // driver_temperature
  // motor_temperature
  // driver_state
  return true;
}

void
bunker_msgs__msg__BunkerActuatorState__fini(bunker_msgs__msg__BunkerActuatorState * msg)
{
  if (!msg) {
    return;
  }
  // motor_id
  // rpm
  // current
  // pulse_count
  // driver_voltage
  // driver_temperature
  // motor_temperature
  // driver_state
}

bool
bunker_msgs__msg__BunkerActuatorState__are_equal(const bunker_msgs__msg__BunkerActuatorState * lhs, const bunker_msgs__msg__BunkerActuatorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_id
  if (lhs->motor_id != rhs->motor_id) {
    return false;
  }
  // rpm
  if (lhs->rpm != rhs->rpm) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // pulse_count
  if (lhs->pulse_count != rhs->pulse_count) {
    return false;
  }
  // driver_voltage
  if (lhs->driver_voltage != rhs->driver_voltage) {
    return false;
  }
  // driver_temperature
  if (lhs->driver_temperature != rhs->driver_temperature) {
    return false;
  }
  // motor_temperature
  if (lhs->motor_temperature != rhs->motor_temperature) {
    return false;
  }
  // driver_state
  if (lhs->driver_state != rhs->driver_state) {
    return false;
  }
  return true;
}

bool
bunker_msgs__msg__BunkerActuatorState__copy(
  const bunker_msgs__msg__BunkerActuatorState * input,
  bunker_msgs__msg__BunkerActuatorState * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_id
  output->motor_id = input->motor_id;
  // rpm
  output->rpm = input->rpm;
  // current
  output->current = input->current;
  // pulse_count
  output->pulse_count = input->pulse_count;
  // driver_voltage
  output->driver_voltage = input->driver_voltage;
  // driver_temperature
  output->driver_temperature = input->driver_temperature;
  // motor_temperature
  output->motor_temperature = input->motor_temperature;
  // driver_state
  output->driver_state = input->driver_state;
  return true;
}

bunker_msgs__msg__BunkerActuatorState *
bunker_msgs__msg__BunkerActuatorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerActuatorState * msg = (bunker_msgs__msg__BunkerActuatorState *)allocator.allocate(sizeof(bunker_msgs__msg__BunkerActuatorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bunker_msgs__msg__BunkerActuatorState));
  bool success = bunker_msgs__msg__BunkerActuatorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bunker_msgs__msg__BunkerActuatorState__destroy(bunker_msgs__msg__BunkerActuatorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bunker_msgs__msg__BunkerActuatorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bunker_msgs__msg__BunkerActuatorState__Sequence__init(bunker_msgs__msg__BunkerActuatorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerActuatorState * data = NULL;

  if (size) {
    data = (bunker_msgs__msg__BunkerActuatorState *)allocator.zero_allocate(size, sizeof(bunker_msgs__msg__BunkerActuatorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bunker_msgs__msg__BunkerActuatorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bunker_msgs__msg__BunkerActuatorState__fini(&data[i - 1]);
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
bunker_msgs__msg__BunkerActuatorState__Sequence__fini(bunker_msgs__msg__BunkerActuatorState__Sequence * array)
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
      bunker_msgs__msg__BunkerActuatorState__fini(&array->data[i]);
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

bunker_msgs__msg__BunkerActuatorState__Sequence *
bunker_msgs__msg__BunkerActuatorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerActuatorState__Sequence * array = (bunker_msgs__msg__BunkerActuatorState__Sequence *)allocator.allocate(sizeof(bunker_msgs__msg__BunkerActuatorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bunker_msgs__msg__BunkerActuatorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bunker_msgs__msg__BunkerActuatorState__Sequence__destroy(bunker_msgs__msg__BunkerActuatorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bunker_msgs__msg__BunkerActuatorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bunker_msgs__msg__BunkerActuatorState__Sequence__are_equal(const bunker_msgs__msg__BunkerActuatorState__Sequence * lhs, const bunker_msgs__msg__BunkerActuatorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bunker_msgs__msg__BunkerActuatorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bunker_msgs__msg__BunkerActuatorState__Sequence__copy(
  const bunker_msgs__msg__BunkerActuatorState__Sequence * input,
  bunker_msgs__msg__BunkerActuatorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bunker_msgs__msg__BunkerActuatorState);
    bunker_msgs__msg__BunkerActuatorState * data =
      (bunker_msgs__msg__BunkerActuatorState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bunker_msgs__msg__BunkerActuatorState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bunker_msgs__msg__BunkerActuatorState__fini(&data[i]);
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
    if (!bunker_msgs__msg__BunkerActuatorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
