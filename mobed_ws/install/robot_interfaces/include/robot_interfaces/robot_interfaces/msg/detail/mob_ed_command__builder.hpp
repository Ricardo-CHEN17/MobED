// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/MobEDCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/mob_ed_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MobEDCommand_body_pitch
{
public:
  explicit Init_MobEDCommand_body_pitch(::robot_interfaces::msg::MobEDCommand & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::MobEDCommand body_pitch(::robot_interfaces::msg::MobEDCommand::_body_pitch_type arg)
  {
    msg_.body_pitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::MobEDCommand msg_;
};

class Init_MobEDCommand_body_roll
{
public:
  explicit Init_MobEDCommand_body_roll(::robot_interfaces::msg::MobEDCommand & msg)
  : msg_(msg)
  {}
  Init_MobEDCommand_body_pitch body_roll(::robot_interfaces::msg::MobEDCommand::_body_roll_type arg)
  {
    msg_.body_roll = std::move(arg);
    return Init_MobEDCommand_body_pitch(msg_);
  }

private:
  ::robot_interfaces::msg::MobEDCommand msg_;
};

class Init_MobEDCommand_body_height
{
public:
  explicit Init_MobEDCommand_body_height(::robot_interfaces::msg::MobEDCommand & msg)
  : msg_(msg)
  {}
  Init_MobEDCommand_body_roll body_height(::robot_interfaces::msg::MobEDCommand::_body_height_type arg)
  {
    msg_.body_height = std::move(arg);
    return Init_MobEDCommand_body_roll(msg_);
  }

private:
  ::robot_interfaces::msg::MobEDCommand msg_;
};

class Init_MobEDCommand_twist
{
public:
  Init_MobEDCommand_twist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MobEDCommand_body_height twist(::robot_interfaces::msg::MobEDCommand::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_MobEDCommand_body_height(msg_);
  }

private:
  ::robot_interfaces::msg::MobEDCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::MobEDCommand>()
{
  return robot_interfaces::msg::builder::Init_MobEDCommand_twist();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__BUILDER_HPP_
