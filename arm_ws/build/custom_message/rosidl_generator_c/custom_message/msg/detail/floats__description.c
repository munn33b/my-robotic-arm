// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice

#include "custom_message/msg/detail/floats__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_type_hash_t *
custom_message__msg__Floats__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x12, 0x0f, 0x21, 0xfd, 0x1b, 0xdb, 0xb8, 0xf1,
      0x5d, 0xa7, 0x8e, 0x31, 0xbf, 0x90, 0x5d, 0x40,
      0x53, 0x4e, 0x88, 0x8a, 0xdd, 0x7a, 0x7b, 0xfc,
      0xaf, 0x34, 0x1d, 0xa5, 0xa7, 0x59, 0xab, 0x05,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char custom_message__msg__Floats__TYPE_NAME[] = "custom_message/msg/Floats";

// Define type names, field names, and default values
static char custom_message__msg__Floats__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field custom_message__msg__Floats__FIELDS[] = {
  {
    {custom_message__msg__Floats__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_message__msg__Floats__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_message__msg__Floats__TYPE_NAME, 25, 25},
      {custom_message__msg__Floats__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32[] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
custom_message__msg__Floats__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_message__msg__Floats__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 15, 15},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_message__msg__Floats__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_message__msg__Floats__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
