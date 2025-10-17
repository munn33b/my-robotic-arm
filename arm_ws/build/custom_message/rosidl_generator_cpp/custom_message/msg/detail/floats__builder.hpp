// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/floats.hpp"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__BUILDER_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_message/msg/detail/floats__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_message
{

namespace msg
{

namespace builder
{

class Init_Floats_data
{
public:
  Init_Floats_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_message::msg::Floats data(::custom_message::msg::Floats::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_message::msg::Floats msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_message::msg::Floats>()
{
  return custom_message::msg::builder::Init_Floats_data();
}

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__BUILDER_HPP_
