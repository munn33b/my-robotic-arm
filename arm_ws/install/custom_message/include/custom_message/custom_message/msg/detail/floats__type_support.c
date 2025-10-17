// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_message/msg/detail/floats__rosidl_typesupport_introspection_c.h"
#include "custom_message/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_message/msg/detail/floats__functions.h"
#include "custom_message/msg/detail/floats__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_message__msg__Floats__init(message_memory);
}

void custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_fini_function(void * message_memory)
{
  custom_message__msg__Floats__fini(message_memory);
}

size_t custom_message__msg__Floats__rosidl_typesupport_introspection_c__size_function__Floats__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_message__msg__Floats__rosidl_typesupport_introspection_c__get_const_function__Floats__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_message__msg__Floats__rosidl_typesupport_introspection_c__get_function__Floats__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_message__msg__Floats__rosidl_typesupport_introspection_c__fetch_function__Floats__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__get_const_function__Floats__data(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void custom_message__msg__Floats__rosidl_typesupport_introspection_c__assign_function__Floats__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__get_function__Floats__data(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool custom_message__msg__Floats__rosidl_typesupport_introspection_c__resize_function__Floats__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_message__msg__Floats, data),  // bytes offset in struct
    NULL,  // default value
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__size_function__Floats__data,  // size() function pointer
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__get_const_function__Floats__data,  // get_const(index) function pointer
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__get_function__Floats__data,  // get(index) function pointer
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__fetch_function__Floats__data,  // fetch(index, &value) function pointer
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__assign_function__Floats__data,  // assign(index, value) function pointer
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__resize_function__Floats__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_members = {
  "custom_message__msg",  // message namespace
  "Floats",  // message name
  1,  // number of fields
  sizeof(custom_message__msg__Floats),
  false,  // has_any_key_member_
  custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_member_array,  // message members
  custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_type_support_handle = {
  0,
  &custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_members,
  get_message_typesupport_handle_function,
  &custom_message__msg__Floats__get_type_hash,
  &custom_message__msg__Floats__get_type_description,
  &custom_message__msg__Floats__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_message
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_message, msg, Floats)() {
  if (!custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_type_support_handle.typesupport_identifier) {
    custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_message__msg__Floats__rosidl_typesupport_introspection_c__Floats_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
