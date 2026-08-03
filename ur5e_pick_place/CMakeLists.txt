cmake_minimum_required(VERSION 3.8)
project(ur5e_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -Wall -Wextra is not decoration here. The failure.hpp switch is deliberately
  # exhaustive so that adding a Result variant without handling it produces a
  # warning rather than a silent fall-through — that only works if warnings are
  # actually enabled.
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(include)

# --- M1: joint-goal planning reliability ------------------------------------
add_executable(m1_joint_goal src/m1_joint_goal.cpp)
target_include_directories(m1_joint_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(m1_joint_goal
  rclcpp
  moveit_ros_planning_interface
  moveit_core
)

install(TARGETS
  m1_joint_goal
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
