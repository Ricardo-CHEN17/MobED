cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories - Using SYSTEM for Eigen to suppress its internal warnings
include_directories(
  include
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# Core Kinematics Library
add_library(mobed_kinematics SHARED
  src/kinematics/mobed_kinematics.cpp
)
target_include_directories(mobed_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mobed_kinematics SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# Controller Classes Library
add_library(mobed_controllers SHARED
  src/controllers/driving_controller.cpp
  src/controllers/balance_controller.cpp
)
target_include_directories(mobed_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mobed_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(mobed_controllers PUBLIC
  mobed_kinematics
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(mobed_kinematics mobed_controllers)
ament_export_dependencies(Eigen3)

ament_package()
