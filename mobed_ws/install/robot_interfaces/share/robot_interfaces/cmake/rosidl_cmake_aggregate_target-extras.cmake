# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target robot_interfaces::robot_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${robot_interfaces_TARGETS}.
if(robot_interfaces_TARGETS AND NOT TARGET robot_interfaces::robot_interfaces)
  add_library(robot_interfaces::robot_interfaces INTERFACE IMPORTED)
  set_target_properties(robot_interfaces::robot_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${robot_interfaces_TARGETS}")
endif()
