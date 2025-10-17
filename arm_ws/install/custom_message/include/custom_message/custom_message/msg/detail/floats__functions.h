// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/floats.h"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__FUNCTIONS_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "custom_message/msg/rosidl_generator_c__visibility_control.h"

#include "custom_message/msg/detail/floats__struct.h"

/// Initialize msg/Floats message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_message__msg__Floats
 * )) before or use
 * custom_message__msg__Floats__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Floats__init(custom_message__msg__Floats * msg);

/// Finalize msg/Floats message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Floats__fini(custom_message__msg__Floats * msg);

/// Create msg/Floats message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_message__msg__Floats__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
custom_message__msg__Floats *
custom_message__msg__Floats__create(void);

/// Destroy msg/Floats message.
/**
 * It calls
 * custom_message__msg__Floats__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Floats__destroy(custom_message__msg__Floats * msg);

/// Check for msg/Floats message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Floats__are_equal(const custom_message__msg__Floats * lhs, const custom_message__msg__Floats * rhs);

/// Copy a msg/Floats message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Floats__copy(
  const custom_message__msg__Floats * input,
  custom_message__msg__Floats * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_type_hash_t *
custom_message__msg__Floats__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_runtime_c__type_description__TypeDescription *
custom_message__msg__Floats__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_runtime_c__type_description__TypeSource *
custom_message__msg__Floats__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_custom_message
const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_message__msg__Floats__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/Floats messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_message__msg__Floats__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Floats__Sequence__init(custom_message__msg__Floats__Sequence * array, size_t size);

/// Finalize array of msg/Floats messages.
/**
 * It calls
 * custom_message__msg__Floats__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Floats__Sequence__fini(custom_message__msg__Floats__Sequence * array);

/// Create array of msg/Floats messages.
/**
 * It allocates the memory for the array and calls
 * custom_message__msg__Floats__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
custom_message__msg__Floats__Sequence *
custom_message__msg__Floats__Sequence__create(size_t size);

/// Destroy array of msg/Floats messages.
/**
 * It calls
 * custom_message__msg__Floats__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Floats__Sequence__destroy(custom_message__msg__Floats__Sequence * array);

/// Check for msg/Floats message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Floats__Sequence__are_equal(const custom_message__msg__Floats__Sequence * lhs, const custom_message__msg__Floats__Sequence * rhs);

/// Copy an array of msg/Floats messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Floats__Sequence__copy(
  const custom_message__msg__Floats__Sequence * input,
  custom_message__msg__Floats__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__FUNCTIONS_H_
