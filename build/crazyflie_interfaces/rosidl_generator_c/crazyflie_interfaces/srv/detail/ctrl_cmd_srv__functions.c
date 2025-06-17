// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crazyflie_interfaces:srv/CTRLCmdSrv.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/srv/detail/ctrl_cmd_srv__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `cmd_vals`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Request__init(crazyflie_interfaces__srv__CTRLCmdSrv_Request * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_type
  // cmd_vals
  if (!geometry_msgs__msg__Vector3__init(&msg->cmd_vals)) {
    crazyflie_interfaces__srv__CTRLCmdSrv_Request__fini(msg);
    return false;
  }
  // cmd_flag
  // cmd_rx
  return true;
}

void
crazyflie_interfaces__srv__CTRLCmdSrv_Request__fini(crazyflie_interfaces__srv__CTRLCmdSrv_Request * msg)
{
  if (!msg) {
    return;
  }
  // cmd_type
  // cmd_vals
  geometry_msgs__msg__Vector3__fini(&msg->cmd_vals);
  // cmd_flag
  // cmd_rx
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Request__are_equal(const crazyflie_interfaces__srv__CTRLCmdSrv_Request * lhs, const crazyflie_interfaces__srv__CTRLCmdSrv_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_type
  if (lhs->cmd_type != rhs->cmd_type) {
    return false;
  }
  // cmd_vals
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->cmd_vals), &(rhs->cmd_vals)))
  {
    return false;
  }
  // cmd_flag
  if (lhs->cmd_flag != rhs->cmd_flag) {
    return false;
  }
  // cmd_rx
  if (lhs->cmd_rx != rhs->cmd_rx) {
    return false;
  }
  return true;
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Request__copy(
  const crazyflie_interfaces__srv__CTRLCmdSrv_Request * input,
  crazyflie_interfaces__srv__CTRLCmdSrv_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_type
  output->cmd_type = input->cmd_type;
  // cmd_vals
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->cmd_vals), &(output->cmd_vals)))
  {
    return false;
  }
  // cmd_flag
  output->cmd_flag = input->cmd_flag;
  // cmd_rx
  output->cmd_rx = input->cmd_rx;
  return true;
}

crazyflie_interfaces__srv__CTRLCmdSrv_Request *
crazyflie_interfaces__srv__CTRLCmdSrv_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__srv__CTRLCmdSrv_Request * msg = (crazyflie_interfaces__srv__CTRLCmdSrv_Request *)allocator.allocate(sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Request));
  bool success = crazyflie_interfaces__srv__CTRLCmdSrv_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_interfaces__srv__CTRLCmdSrv_Request__destroy(crazyflie_interfaces__srv__CTRLCmdSrv_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_interfaces__srv__CTRLCmdSrv_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__init(crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__srv__CTRLCmdSrv_Request * data = NULL;

  if (size) {
    data = (crazyflie_interfaces__srv__CTRLCmdSrv_Request *)allocator.zero_allocate(size, sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_interfaces__srv__CTRLCmdSrv_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_interfaces__srv__CTRLCmdSrv_Request__fini(&data[i - 1]);
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
crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__fini(crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * array)
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
      crazyflie_interfaces__srv__CTRLCmdSrv_Request__fini(&array->data[i]);
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

crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence *
crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * array = (crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence *)allocator.allocate(sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__destroy(crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__are_equal(const crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * lhs, const crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_interfaces__srv__CTRLCmdSrv_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence__copy(
  const crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * input,
  crazyflie_interfaces__srv__CTRLCmdSrv_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_interfaces__srv__CTRLCmdSrv_Request * data =
      (crazyflie_interfaces__srv__CTRLCmdSrv_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_interfaces__srv__CTRLCmdSrv_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_interfaces__srv__CTRLCmdSrv_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_interfaces__srv__CTRLCmdSrv_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
crazyflie_interfaces__srv__CTRLCmdSrv_Response__init(crazyflie_interfaces__srv__CTRLCmdSrv_Response * msg)
{
  if (!msg) {
    return false;
  }
  // srv_success
  return true;
}

void
crazyflie_interfaces__srv__CTRLCmdSrv_Response__fini(crazyflie_interfaces__srv__CTRLCmdSrv_Response * msg)
{
  if (!msg) {
    return;
  }
  // srv_success
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Response__are_equal(const crazyflie_interfaces__srv__CTRLCmdSrv_Response * lhs, const crazyflie_interfaces__srv__CTRLCmdSrv_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // srv_success
  if (lhs->srv_success != rhs->srv_success) {
    return false;
  }
  return true;
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Response__copy(
  const crazyflie_interfaces__srv__CTRLCmdSrv_Response * input,
  crazyflie_interfaces__srv__CTRLCmdSrv_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // srv_success
  output->srv_success = input->srv_success;
  return true;
}

crazyflie_interfaces__srv__CTRLCmdSrv_Response *
crazyflie_interfaces__srv__CTRLCmdSrv_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__srv__CTRLCmdSrv_Response * msg = (crazyflie_interfaces__srv__CTRLCmdSrv_Response *)allocator.allocate(sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Response));
  bool success = crazyflie_interfaces__srv__CTRLCmdSrv_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_interfaces__srv__CTRLCmdSrv_Response__destroy(crazyflie_interfaces__srv__CTRLCmdSrv_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_interfaces__srv__CTRLCmdSrv_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__init(crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__srv__CTRLCmdSrv_Response * data = NULL;

  if (size) {
    data = (crazyflie_interfaces__srv__CTRLCmdSrv_Response *)allocator.zero_allocate(size, sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_interfaces__srv__CTRLCmdSrv_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_interfaces__srv__CTRLCmdSrv_Response__fini(&data[i - 1]);
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
crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__fini(crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * array)
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
      crazyflie_interfaces__srv__CTRLCmdSrv_Response__fini(&array->data[i]);
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

crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence *
crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * array = (crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence *)allocator.allocate(sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__destroy(crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__are_equal(const crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * lhs, const crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_interfaces__srv__CTRLCmdSrv_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence__copy(
  const crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * input,
  crazyflie_interfaces__srv__CTRLCmdSrv_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_interfaces__srv__CTRLCmdSrv_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_interfaces__srv__CTRLCmdSrv_Response * data =
      (crazyflie_interfaces__srv__CTRLCmdSrv_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_interfaces__srv__CTRLCmdSrv_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_interfaces__srv__CTRLCmdSrv_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_interfaces__srv__CTRLCmdSrv_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
