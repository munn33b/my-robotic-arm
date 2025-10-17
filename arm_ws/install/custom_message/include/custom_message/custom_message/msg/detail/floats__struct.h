// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/floats.h"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__STRUCT_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Floats in the package custom_message.
typedef struct custom_message__msg__Floats
{
  rosidl_runtime_c__float__Sequence data;
} custom_message__msg__Floats;

// Struct for a sequence of custom_message__msg__Floats.
typedef struct custom_message__msg__Floats__Sequence
{
  custom_message__msg__Floats * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__msg__Floats__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__STRUCT_H_
