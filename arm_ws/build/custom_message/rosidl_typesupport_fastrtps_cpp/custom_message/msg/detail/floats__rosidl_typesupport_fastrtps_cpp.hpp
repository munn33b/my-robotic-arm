// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "custom_message/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "custom_message/msg/detail/floats__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace custom_message
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
cdr_serialize(
  const custom_message::msg::Floats & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  custom_message::msg::Floats & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
get_serialized_size(
  const custom_message::msg::Floats & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
max_serialized_size_Floats(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
cdr_serialize_key(
  const custom_message::msg::Floats & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
get_serialized_size_key(
  const custom_message::msg::Floats & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
max_serialized_size_key_Floats(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace custom_message

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_message
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_message, msg, Floats)();

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
