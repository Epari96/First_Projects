// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from order_interface:srv/Order.idl
// generated code does not contain a copyright notice
#include "order_interface/srv/detail/order__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `item_name`
#include "rosidl_runtime_c/string_functions.h"

bool
order_interface__srv__Order_Request__init(order_interface__srv__Order_Request * msg)
{
  if (!msg) {
    return false;
  }
  // item_name
  if (!rosidl_runtime_c__String__init(&msg->item_name)) {
    order_interface__srv__Order_Request__fini(msg);
    return false;
  }
  // quantity
  // is_receipt
  return true;
}

void
order_interface__srv__Order_Request__fini(order_interface__srv__Order_Request * msg)
{
  if (!msg) {
    return;
  }
  // item_name
  rosidl_runtime_c__String__fini(&msg->item_name);
  // quantity
  // is_receipt
}

bool
order_interface__srv__Order_Request__are_equal(const order_interface__srv__Order_Request * lhs, const order_interface__srv__Order_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // item_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->item_name), &(rhs->item_name)))
  {
    return false;
  }
  // quantity
  if (lhs->quantity != rhs->quantity) {
    return false;
  }
  // is_receipt
  if (lhs->is_receipt != rhs->is_receipt) {
    return false;
  }
  return true;
}

bool
order_interface__srv__Order_Request__copy(
  const order_interface__srv__Order_Request * input,
  order_interface__srv__Order_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // item_name
  if (!rosidl_runtime_c__String__copy(
      &(input->item_name), &(output->item_name)))
  {
    return false;
  }
  // quantity
  output->quantity = input->quantity;
  // is_receipt
  output->is_receipt = input->is_receipt;
  return true;
}

order_interface__srv__Order_Request *
order_interface__srv__Order_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  order_interface__srv__Order_Request * msg = (order_interface__srv__Order_Request *)allocator.allocate(sizeof(order_interface__srv__Order_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(order_interface__srv__Order_Request));
  bool success = order_interface__srv__Order_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
order_interface__srv__Order_Request__destroy(order_interface__srv__Order_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    order_interface__srv__Order_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
order_interface__srv__Order_Request__Sequence__init(order_interface__srv__Order_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  order_interface__srv__Order_Request * data = NULL;

  if (size) {
    data = (order_interface__srv__Order_Request *)allocator.zero_allocate(size, sizeof(order_interface__srv__Order_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = order_interface__srv__Order_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        order_interface__srv__Order_Request__fini(&data[i - 1]);
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
order_interface__srv__Order_Request__Sequence__fini(order_interface__srv__Order_Request__Sequence * array)
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
      order_interface__srv__Order_Request__fini(&array->data[i]);
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

order_interface__srv__Order_Request__Sequence *
order_interface__srv__Order_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  order_interface__srv__Order_Request__Sequence * array = (order_interface__srv__Order_Request__Sequence *)allocator.allocate(sizeof(order_interface__srv__Order_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = order_interface__srv__Order_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
order_interface__srv__Order_Request__Sequence__destroy(order_interface__srv__Order_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    order_interface__srv__Order_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
order_interface__srv__Order_Request__Sequence__are_equal(const order_interface__srv__Order_Request__Sequence * lhs, const order_interface__srv__Order_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!order_interface__srv__Order_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
order_interface__srv__Order_Request__Sequence__copy(
  const order_interface__srv__Order_Request__Sequence * input,
  order_interface__srv__Order_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(order_interface__srv__Order_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    order_interface__srv__Order_Request * data =
      (order_interface__srv__Order_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!order_interface__srv__Order_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          order_interface__srv__Order_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!order_interface__srv__Order_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
order_interface__srv__Order_Response__init(order_interface__srv__Order_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    order_interface__srv__Order_Response__fini(msg);
    return false;
  }
  return true;
}

void
order_interface__srv__Order_Response__fini(order_interface__srv__Order_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
order_interface__srv__Order_Response__are_equal(const order_interface__srv__Order_Response * lhs, const order_interface__srv__Order_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
order_interface__srv__Order_Response__copy(
  const order_interface__srv__Order_Response * input,
  order_interface__srv__Order_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

order_interface__srv__Order_Response *
order_interface__srv__Order_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  order_interface__srv__Order_Response * msg = (order_interface__srv__Order_Response *)allocator.allocate(sizeof(order_interface__srv__Order_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(order_interface__srv__Order_Response));
  bool success = order_interface__srv__Order_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
order_interface__srv__Order_Response__destroy(order_interface__srv__Order_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    order_interface__srv__Order_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
order_interface__srv__Order_Response__Sequence__init(order_interface__srv__Order_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  order_interface__srv__Order_Response * data = NULL;

  if (size) {
    data = (order_interface__srv__Order_Response *)allocator.zero_allocate(size, sizeof(order_interface__srv__Order_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = order_interface__srv__Order_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        order_interface__srv__Order_Response__fini(&data[i - 1]);
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
order_interface__srv__Order_Response__Sequence__fini(order_interface__srv__Order_Response__Sequence * array)
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
      order_interface__srv__Order_Response__fini(&array->data[i]);
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

order_interface__srv__Order_Response__Sequence *
order_interface__srv__Order_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  order_interface__srv__Order_Response__Sequence * array = (order_interface__srv__Order_Response__Sequence *)allocator.allocate(sizeof(order_interface__srv__Order_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = order_interface__srv__Order_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
order_interface__srv__Order_Response__Sequence__destroy(order_interface__srv__Order_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    order_interface__srv__Order_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
order_interface__srv__Order_Response__Sequence__are_equal(const order_interface__srv__Order_Response__Sequence * lhs, const order_interface__srv__Order_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!order_interface__srv__Order_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
order_interface__srv__Order_Response__Sequence__copy(
  const order_interface__srv__Order_Response__Sequence * input,
  order_interface__srv__Order_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(order_interface__srv__Order_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    order_interface__srv__Order_Response * data =
      (order_interface__srv__Order_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!order_interface__srv__Order_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          order_interface__srv__Order_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!order_interface__srv__Order_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
