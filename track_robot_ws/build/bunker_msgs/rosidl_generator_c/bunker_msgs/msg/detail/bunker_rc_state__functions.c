// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bunker_msgs:msg/BunkerRCState.idl
// generated code does not contain a copyright notice
#include "bunker_msgs/msg/detail/bunker_rc_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
bunker_msgs__msg__BunkerRCState__init(bunker_msgs__msg__BunkerRCState * msg)
{
  if (!msg) {
    return false;
  }
  // swa
  // swb
  // swc
  // swd
  // stick_right_v
  // stick_right_h
  // stick_left_v
  // stick_left_h
  // var_a
  return true;
}

void
bunker_msgs__msg__BunkerRCState__fini(bunker_msgs__msg__BunkerRCState * msg)
{
  if (!msg) {
    return;
  }
  // swa
  // swb
  // swc
  // swd
  // stick_right_v
  // stick_right_h
  // stick_left_v
  // stick_left_h
  // var_a
}

bool
bunker_msgs__msg__BunkerRCState__are_equal(const bunker_msgs__msg__BunkerRCState * lhs, const bunker_msgs__msg__BunkerRCState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // swa
  if (lhs->swa != rhs->swa) {
    return false;
  }
  // swb
  if (lhs->swb != rhs->swb) {
    return false;
  }
  // swc
  if (lhs->swc != rhs->swc) {
    return false;
  }
  // swd
  if (lhs->swd != rhs->swd) {
    return false;
  }
  // stick_right_v
  if (lhs->stick_right_v != rhs->stick_right_v) {
    return false;
  }
  // stick_right_h
  if (lhs->stick_right_h != rhs->stick_right_h) {
    return false;
  }
  // stick_left_v
  if (lhs->stick_left_v != rhs->stick_left_v) {
    return false;
  }
  // stick_left_h
  if (lhs->stick_left_h != rhs->stick_left_h) {
    return false;
  }
  // var_a
  if (lhs->var_a != rhs->var_a) {
    return false;
  }
  return true;
}

bool
bunker_msgs__msg__BunkerRCState__copy(
  const bunker_msgs__msg__BunkerRCState * input,
  bunker_msgs__msg__BunkerRCState * output)
{
  if (!input || !output) {
    return false;
  }
  // swa
  output->swa = input->swa;
  // swb
  output->swb = input->swb;
  // swc
  output->swc = input->swc;
  // swd
  output->swd = input->swd;
  // stick_right_v
  output->stick_right_v = input->stick_right_v;
  // stick_right_h
  output->stick_right_h = input->stick_right_h;
  // stick_left_v
  output->stick_left_v = input->stick_left_v;
  // stick_left_h
  output->stick_left_h = input->stick_left_h;
  // var_a
  output->var_a = input->var_a;
  return true;
}

bunker_msgs__msg__BunkerRCState *
bunker_msgs__msg__BunkerRCState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerRCState * msg = (bunker_msgs__msg__BunkerRCState *)allocator.allocate(sizeof(bunker_msgs__msg__BunkerRCState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bunker_msgs__msg__BunkerRCState));
  bool success = bunker_msgs__msg__BunkerRCState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bunker_msgs__msg__BunkerRCState__destroy(bunker_msgs__msg__BunkerRCState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bunker_msgs__msg__BunkerRCState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bunker_msgs__msg__BunkerRCState__Sequence__init(bunker_msgs__msg__BunkerRCState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerRCState * data = NULL;

  if (size) {
    data = (bunker_msgs__msg__BunkerRCState *)allocator.zero_allocate(size, sizeof(bunker_msgs__msg__BunkerRCState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bunker_msgs__msg__BunkerRCState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bunker_msgs__msg__BunkerRCState__fini(&data[i - 1]);
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
bunker_msgs__msg__BunkerRCState__Sequence__fini(bunker_msgs__msg__BunkerRCState__Sequence * array)
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
      bunker_msgs__msg__BunkerRCState__fini(&array->data[i]);
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

bunker_msgs__msg__BunkerRCState__Sequence *
bunker_msgs__msg__BunkerRCState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bunker_msgs__msg__BunkerRCState__Sequence * array = (bunker_msgs__msg__BunkerRCState__Sequence *)allocator.allocate(sizeof(bunker_msgs__msg__BunkerRCState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bunker_msgs__msg__BunkerRCState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bunker_msgs__msg__BunkerRCState__Sequence__destroy(bunker_msgs__msg__BunkerRCState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bunker_msgs__msg__BunkerRCState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bunker_msgs__msg__BunkerRCState__Sequence__are_equal(const bunker_msgs__msg__BunkerRCState__Sequence * lhs, const bunker_msgs__msg__BunkerRCState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bunker_msgs__msg__BunkerRCState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bunker_msgs__msg__BunkerRCState__Sequence__copy(
  const bunker_msgs__msg__BunkerRCState__Sequence * input,
  bunker_msgs__msg__BunkerRCState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bunker_msgs__msg__BunkerRCState);
    bunker_msgs__msg__BunkerRCState * data =
      (bunker_msgs__msg__BunkerRCState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bunker_msgs__msg__BunkerRCState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bunker_msgs__msg__BunkerRCState__fini(&data[i]);
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
    if (!bunker_msgs__msg__BunkerRCState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
