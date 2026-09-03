// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/MobEDCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__MobEDCommand __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__MobEDCommand __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MobEDCommand_
{
  using Type = MobEDCommand_<ContainerAllocator>;

  explicit MobEDCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->body_height = 0.0;
      this->body_roll = 0.0;
      this->body_pitch = 0.0;
    }
  }

  explicit MobEDCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->body_height = 0.0;
      this->body_roll = 0.0;
      this->body_pitch = 0.0;
    }
  }

  // field types and members
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _body_height_type =
    double;
  _body_height_type body_height;
  using _body_roll_type =
    double;
  _body_roll_type body_roll;
  using _body_pitch_type =
    double;
  _body_pitch_type body_pitch;

  // setters for named parameter idiom
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__body_height(
    const double & _arg)
  {
    this->body_height = _arg;
    return *this;
  }
  Type & set__body_roll(
    const double & _arg)
  {
    this->body_roll = _arg;
    return *this;
  }
  Type & set__body_pitch(
    const double & _arg)
  {
    this->body_pitch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::MobEDCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::MobEDCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::MobEDCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::MobEDCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__MobEDCommand
    std::shared_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__MobEDCommand
    std::shared_ptr<robot_interfaces::msg::MobEDCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MobEDCommand_ & other) const
  {
    if (this->twist != other.twist) {
      return false;
    }
    if (this->body_height != other.body_height) {
      return false;
    }
    if (this->body_roll != other.body_roll) {
      return false;
    }
    if (this->body_pitch != other.body_pitch) {
      return false;
    }
    return true;
  }
  bool operator!=(const MobEDCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MobEDCommand_

// alias to use template instance with default allocator
using MobEDCommand =
  robot_interfaces::msg::MobEDCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__STRUCT_HPP_
