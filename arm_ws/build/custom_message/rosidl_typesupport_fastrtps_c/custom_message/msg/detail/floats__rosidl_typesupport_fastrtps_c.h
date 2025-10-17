// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice
#ifndef CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "custom_message/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "custom_message/msg/detail/floats__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
bool cdr_serialize_custom_message__msg__Floats(
  const custom_message__msg__Floats * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
bool cdr_deserialize_custom_message__msg__Floats(
  eprosima::fastcdr::Cdr &,
  custom_message__msg__Floats * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
size_t get_serialized_size_custom_message__msg__Floats(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
size_t max_serialized_size_custom_message__msg__Floats(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
bool cdr_serialize_key_custom_message__msg__Floats(
  const custom_message__msg__Floats * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
size_t get_serialized_size_key_custom_message__msg__Floats(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
size_t max_serialized_size_key_custom_message__msg__Floats(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_message
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_message, msg, Floats)();

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
