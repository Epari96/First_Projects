// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from order_interface:srv/Order.idl
// generated code does not contain a copyright notice

#ifndef ORDER_INTERFACE__SRV__DETAIL__ORDER__STRUCT_H_
#define ORDER_INTERFACE__SRV__DETAIL__ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'item_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Order in the package order_interface.
typedef struct order_interface__srv__Order_Request
{
  rosidl_runtime_c__String item_name;
  int32_t quantity;
  int32_t is_receipt;
} order_interface__srv__Order_Request;

// Struct for a sequence of order_interface__srv__Order_Request.
typedef struct order_interface__srv__Order_Request__Sequence
{
  order_interface__srv__Order_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} order_interface__srv__Order_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Order in the package order_interface.
typedef struct order_interface__srv__Order_Response
{
  bool success;
  rosidl_runtime_c__String message;
} order_interface__srv__Order_Response;

// Struct for a sequence of order_interface__srv__Order_Response.
typedef struct order_interface__srv__Order_Response__Sequence
{
  order_interface__srv__Order_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} order_interface__srv__Order_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORDER_INTERFACE__SRV__DETAIL__ORDER__STRUCT_H_
