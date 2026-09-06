cmake_minimum_required(VERSION 3.8)
project(ur5e_robotiq_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(deterministic_motion_system SHARED
  src/deterministic_motion_system.cpp
)
target_link_libraries(deterministic_motion_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

install(
  TARGETS deterministic_motion_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY urdf config launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
