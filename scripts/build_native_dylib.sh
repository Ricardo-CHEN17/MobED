#!/bin/bash
set -e

# Target output directory
mkdir -p build_native

echo "==> 正在编译 MobED 原生 C++ 控制算法动态库 (libmobed_control.dylib)..."

clang++ -O3 -std=c++17 -shared -fPIC \
  -I mobed_ws/src/robot_control/include \
  -I /opt/homebrew/include/eigen3 \
  -I /opt/homebrew/opt/eigen/include/eigen3 \
  mobed_ws/src/robot_control/src/core/mobed_controller_core.cpp \
  mobed_ws/src/robot_control/src/kinematics/mobed_kinematics.cpp \
  mobed_ws/src/robot_control/src/controllers/driving_controller.cpp \
  mobed_ws/src/robot_control/src/controllers/balance_controller.cpp \
  mobed_ws/src/robot_control/src/dynamics/srbd_model.cpp \
  mobed_ws/src/robot_control/src/dynamics/grf_optimizer.cpp \
  mobed_ws/src/robot_control/src/estimation/state_estimator.cpp \
  mobed_ws/src/robot_control/src/estimation/terrain_estimator.cpp \
  mobed_ws/src/robot_control/src/estimation/contact_detector.cpp \
  mobed_ws/src/robot_control/src/estimation/climb_geometry_estimator.cpp \
  mobed_ws/src/robot_control/src/tasks/task_controller.cpp \
  mobed_ws/src/robot_control/src/tasks/trajectory_generator.cpp \
  -o build_native/libmobed_control.dylib

echo "==> 编译完成: build_native/libmobed_control.dylib"
ls -lh build_native/libmobed_control.dylib
