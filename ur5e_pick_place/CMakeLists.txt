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
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
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

# --- M2: static scene TF + Cartesian approach --------------------------------
add_executable(static_scene_tf src/static_scene_tf.cpp)
ament_target_dependencies(static_scene_tf
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(m2_cartesian_approach src/m2_cartesian_approach.cpp)
target_include_directories(m2_cartesian_approach PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(m2_cartesian_approach
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# --- M3: pad-centre-corrected grasp + grasp-success verification ------------
add_executable(m3_grasp src/m3_grasp.cpp)
target_include_directories(m3_grasp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(m3_grasp
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  control_msgs
)

# --- diagnostic: CLI vs rclcpp_action::Client timing check, see
# docs/HANDOFF_M3.md "check the CLI vs node action-client timing
# difference" -- not part of the M-milestone chain, kept for reuse. ---
add_executable(probe_gripper_cmd src/probe_gripper_cmd.cpp)
ament_target_dependencies(probe_gripper_cmd
  rclcpp
  rclcpp_action
  control_msgs
)

install(TARGETS
  m1_joint_goal
  static_scene_tf
  m2_cartesian_approach
  m3_grasp
  probe_gripper_cmd
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
