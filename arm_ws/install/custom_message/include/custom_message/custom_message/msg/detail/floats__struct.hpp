// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_message:msg/Floats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_message/msg/floats.hpp"


#ifndef CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__STRUCT_HPP_
#define CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_message__msg__Floats __attribute__((deprecated))
#else
# define DEPRECATED__custom_message__msg__Floats __declspec(deprecated)
#endif

namespace custom_message
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Floats_
{
  using Type = Floats_<ContainerAllocator>;

  explicit Floats_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Floats_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_message::msg::Floats_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_message::msg::Floats_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_message::msg::Floats_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_message::msg::Floats_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_message::msg::Floats_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_message::msg::Floats_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_message::msg::Floats_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_message::msg::Floats_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_message::msg::Floats_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_message::msg::Floats_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_message__msg__Floats
    std::shared_ptr<custom_message::msg::Floats_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_message__msg__Floats
    std::shared_ptr<custom_message::msg::Floats_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Floats_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Floats_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Floats_

// alias to use template instance with default allocator
using Floats =
  custom_message::msg::Floats_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_message

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__FLOATS__STRUCT_HPP_
