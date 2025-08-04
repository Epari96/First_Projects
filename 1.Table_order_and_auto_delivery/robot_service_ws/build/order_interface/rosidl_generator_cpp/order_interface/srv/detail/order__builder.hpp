// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from order_interface:srv/Order.idl
// generated code does not contain a copyright notice

#ifndef ORDER_INTERFACE__SRV__DETAIL__ORDER__BUILDER_HPP_
#define ORDER_INTERFACE__SRV__DETAIL__ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "order_interface/srv/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace order_interface
{

namespace srv
{

namespace builder
{

class Init_Order_Request_is_receipt
{
public:
  explicit Init_Order_Request_is_receipt(::order_interface::srv::Order_Request & msg)
  : msg_(msg)
  {}
  ::order_interface::srv::Order_Request is_receipt(::order_interface::srv::Order_Request::_is_receipt_type arg)
  {
    msg_.is_receipt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::order_interface::srv::Order_Request msg_;
};

class Init_Order_Request_quantity
{
public:
  explicit Init_Order_Request_quantity(::order_interface::srv::Order_Request & msg)
  : msg_(msg)
  {}
  Init_Order_Request_is_receipt quantity(::order_interface::srv::Order_Request::_quantity_type arg)
  {
    msg_.quantity = std::move(arg);
    return Init_Order_Request_is_receipt(msg_);
  }

private:
  ::order_interface::srv::Order_Request msg_;
};

class Init_Order_Request_item_name
{
public:
  Init_Order_Request_item_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Order_Request_quantity item_name(::order_interface::srv::Order_Request::_item_name_type arg)
  {
    msg_.item_name = std::move(arg);
    return Init_Order_Request_quantity(msg_);
  }

private:
  ::order_interface::srv::Order_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::order_interface::srv::Order_Request>()
{
  return order_interface::srv::builder::Init_Order_Request_item_name();
}

}  // namespace order_interface


namespace order_interface
{

namespace srv
{

namespace builder
{

class Init_Order_Response_message
{
public:
  explicit Init_Order_Response_message(::order_interface::srv::Order_Response & msg)
  : msg_(msg)
  {}
  ::order_interface::srv::Order_Response message(::order_interface::srv::Order_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::order_interface::srv::Order_Response msg_;
};

class Init_Order_Response_success
{
public:
  Init_Order_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Order_Response_message success(::order_interface::srv::Order_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Order_Response_message(msg_);
  }

private:
  ::order_interface::srv::Order_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::order_interface::srv::Order_Response>()
{
  return order_interface::srv::builder::Init_Order_Response_success();
}

}  // namespace order_interface

#endif  // ORDER_INTERFACE__SRV__DETAIL__ORDER__BUILDER_HPP_
