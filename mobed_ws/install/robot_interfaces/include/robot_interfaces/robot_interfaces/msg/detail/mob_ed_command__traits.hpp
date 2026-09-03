// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/MobEDCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/mob_ed_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MobEDCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: body_height
  {
    out << "body_height: ";
    rosidl_generator_traits::value_to_yaml(msg.body_height, out);
    out << ", ";
  }

  // member: body_roll
  {
    out << "body_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.body_roll, out);
    out << ", ";
  }

  // member: body_pitch
  {
    out << "body_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.body_pitch, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MobEDCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: body_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_height: ";
    rosidl_generator_traits::value_to_yaml(msg.body_height, out);
    out << "\n";
  }

  // member: body_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.body_roll, out);
    out << "\n";
  }

  // member: body_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.body_pitch, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MobEDCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::msg::MobEDCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::MobEDCommand & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::MobEDCommand>()
{
  return "robot_interfaces::msg::MobEDCommand";
}

template<>
inline const char * name<robot_interfaces::msg::MobEDCommand>()
{
  return "robot_interfaces/msg/MobEDCommand";
}

template<>
struct has_fixed_size<robot_interfaces::msg::MobEDCommand>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<robot_interfaces::msg::MobEDCommand>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<robot_interfaces::msg::MobEDCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__TRAITS_HPP_
