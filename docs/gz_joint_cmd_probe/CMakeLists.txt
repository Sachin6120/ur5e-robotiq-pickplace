cmake_minimum_required(VERSION 3.10)
project(gz_joint_cmd_probe)

# Harmonic ships gz-sim 8. Pinned rather than version-agnostic because the
# System API changed across major versions and a silent match against a
# different one would fail at load with an unhelpful message.
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(JointCmdProbe SHARED JointCmdProbe.cc)
set_property(TARGET JointCmdProbe PROPERTY CXX_STANDARD 17)

target_link_libraries(JointCmdProbe
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::register
)

# BUILD AND RUN
#
#   cd docs/gz_joint_cmd_probe && mkdir -p build && cd build
#   cmake .. && make
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD:$GZ_SIM_SYSTEM_PLUGIN_PATH
#
# Then add the <plugin> block to the model SDF (already wired into
# ur5e_robotiq.urdf.xacro, immediately after gz_ros2_control's own plugin
# block) and launch as normal. See the header comment in JointCmdProbe.cc --
# the ordering is the one assumption that would invert the conclusion.
