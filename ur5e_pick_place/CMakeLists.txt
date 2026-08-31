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
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

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
# transport.cpp is a separate translation unit (lift/transport/place/release/
# retreat) rather than folded into m3_grasp.cpp -- see that file's own header
# for why: m3_grasp.cpp is verified and produced this project's first
# successful grasp, and is not the file to edit blind.
add_executable(m3_grasp src/m3_grasp.cpp src/transport.cpp)
target_include_directories(m3_grasp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(m3_grasp
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
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

# --- Milestone C: deterministic RGB-D tabletop-object detector --------------
add_executable(object_detector src/object_detector.cpp)
ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf2
  tf2_geometry_msgs
)
target_link_libraries(object_detector ${OpenCV_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(object_detector_d10_test test/test_object_detector_d10.cpp)
  target_include_directories(object_detector_d10_test PRIVATE include)
  ament_add_gtest(mask_orientation_test test/test_mask_orientation.cpp)
  target_include_directories(mask_orientation_test PRIVATE include)
  ament_add_gtest(edge_line_tls_test test/test_edge_line_tls.cpp)
  target_include_directories(edge_line_tls_test PRIVATE include)
  target_link_libraries(edge_line_tls_test ${OpenCV_LIBRARIES})
  ament_target_dependencies(edge_line_tls_test tf2)
  ament_add_gtest(object_position_world_test test/test_object_position_world.cpp)
  target_include_directories(object_position_world_test PRIVATE include)
  ament_target_dependencies(object_position_world_test
    rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
  ament_add_gtest(pixel_centre_shadow_test test/test_pixel_centre_shadow.cpp)
  target_include_directories(pixel_centre_shadow_test PRIVATE include)
  ament_target_dependencies(pixel_centre_shadow_test
    rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
  ament_add_gtest(m3_perceived_yaw_test test/test_m3_perceived_yaw.cpp)
  target_include_directories(m3_perceived_yaw_test PRIVATE include)
  ament_target_dependencies(m3_perceived_yaw_test
    rclcpp rclcpp_action moveit_ros_planning_interface moveit_core moveit_msgs
    tf2 tf2_ros tf2_geometry_msgs geometry_msgs control_msgs sensor_msgs)
endif()

# --- Milestone E: camera-frame -> world-frame transform ---------------------
# Deliberately a separate executable from object_detector: the Milestone C/D
# estimator stays byte-identical, and no TF listener is pulled into it.
add_executable(object_position_world src/object_position_world.cpp)
ament_target_dependencies(object_position_world
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  m1_joint_goal
  static_scene_tf
  m2_cartesian_approach
  m3_grasp
  probe_gripper_cmd
  object_detector
  object_position_world
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
