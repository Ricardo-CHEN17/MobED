// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/MobEDCommand.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/MobEDCommand in the package robot_interfaces.
/**
  * 1. Driving Command
  * Target linear velocity (x, y) and angular velocity (z) for the robot body
 */
typedef struct robot_interfaces__msg__MobEDCommand
{
  geometry_msgs__msg__Twist twist;
  /// 2. Posture Command
  /// Target posture variables for the balance controller
  /// Target body height from the ground (m)
  double body_height;
  /// Target roll angle (radians)
  double body_roll;
  /// Target pitch angle (radians)
  double body_pitch;
} robot_interfaces__msg__MobEDCommand;

// Struct for a sequence of robot_interfaces__msg__MobEDCommand.
typedef struct robot_interfaces__msg__MobEDCommand__Sequence
{
  robot_interfaces__msg__MobEDCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__MobEDCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOB_ED_COMMAND__STRUCT_H_
