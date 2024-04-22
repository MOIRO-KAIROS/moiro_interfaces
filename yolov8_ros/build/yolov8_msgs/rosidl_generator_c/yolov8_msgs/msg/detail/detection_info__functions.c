// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolov8_msgs:msg/DetectionInfo.idl
// generated code does not contain a copyright notice
#include "yolov8_msgs/msg/detail/detection_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yolov8_msgs__msg__DetectionInfo__init(yolov8_msgs__msg__DetectionInfo * msg)
{
  if (!msg) {
    return false;
  }
  // center_x
  // center_y
  // average_depth
  // id
  return true;
}

void
yolov8_msgs__msg__DetectionInfo__fini(yolov8_msgs__msg__DetectionInfo * msg)
{
  if (!msg) {
    return;
  }
  // center_x
  // center_y
  // average_depth
  // id
}

bool
yolov8_msgs__msg__DetectionInfo__are_equal(const yolov8_msgs__msg__DetectionInfo * lhs, const yolov8_msgs__msg__DetectionInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // center_x
  if (lhs->center_x != rhs->center_x) {
    return false;
  }
  // center_y
  if (lhs->center_y != rhs->center_y) {
    return false;
  }
  // average_depth
  if (lhs->average_depth != rhs->average_depth) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  return true;
}

bool
yolov8_msgs__msg__DetectionInfo__copy(
  const yolov8_msgs__msg__DetectionInfo * input,
  yolov8_msgs__msg__DetectionInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // center_x
  output->center_x = input->center_x;
  // center_y
  output->center_y = input->center_y;
  // average_depth
  output->average_depth = input->average_depth;
  // id
  output->id = input->id;
  return true;
}

yolov8_msgs__msg__DetectionInfo *
yolov8_msgs__msg__DetectionInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov8_msgs__msg__DetectionInfo * msg = (yolov8_msgs__msg__DetectionInfo *)allocator.allocate(sizeof(yolov8_msgs__msg__DetectionInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolov8_msgs__msg__DetectionInfo));
  bool success = yolov8_msgs__msg__DetectionInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolov8_msgs__msg__DetectionInfo__destroy(yolov8_msgs__msg__DetectionInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolov8_msgs__msg__DetectionInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolov8_msgs__msg__DetectionInfo__Sequence__init(yolov8_msgs__msg__DetectionInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov8_msgs__msg__DetectionInfo * data = NULL;

  if (size) {
    data = (yolov8_msgs__msg__DetectionInfo *)allocator.zero_allocate(size, sizeof(yolov8_msgs__msg__DetectionInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolov8_msgs__msg__DetectionInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolov8_msgs__msg__DetectionInfo__fini(&data[i - 1]);
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
yolov8_msgs__msg__DetectionInfo__Sequence__fini(yolov8_msgs__msg__DetectionInfo__Sequence * array)
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
      yolov8_msgs__msg__DetectionInfo__fini(&array->data[i]);
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

yolov8_msgs__msg__DetectionInfo__Sequence *
yolov8_msgs__msg__DetectionInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolov8_msgs__msg__DetectionInfo__Sequence * array = (yolov8_msgs__msg__DetectionInfo__Sequence *)allocator.allocate(sizeof(yolov8_msgs__msg__DetectionInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolov8_msgs__msg__DetectionInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolov8_msgs__msg__DetectionInfo__Sequence__destroy(yolov8_msgs__msg__DetectionInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolov8_msgs__msg__DetectionInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolov8_msgs__msg__DetectionInfo__Sequence__are_equal(const yolov8_msgs__msg__DetectionInfo__Sequence * lhs, const yolov8_msgs__msg__DetectionInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolov8_msgs__msg__DetectionInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolov8_msgs__msg__DetectionInfo__Sequence__copy(
  const yolov8_msgs__msg__DetectionInfo__Sequence * input,
  yolov8_msgs__msg__DetectionInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolov8_msgs__msg__DetectionInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolov8_msgs__msg__DetectionInfo * data =
      (yolov8_msgs__msg__DetectionInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolov8_msgs__msg__DetectionInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolov8_msgs__msg__DetectionInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolov8_msgs__msg__DetectionInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
