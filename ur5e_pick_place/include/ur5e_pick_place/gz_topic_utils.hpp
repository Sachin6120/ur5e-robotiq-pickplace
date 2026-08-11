// gz_topic_utils.hpp — small ground-truth helpers shared between m3_grasp.cpp
// and transport.cpp, factored out rather than duplicated. `run_command` and
// `parse_joint_position` used to live only in m3_grasp.cpp's own anonymous
// namespace; transport.cpp needs the same joint-position sampling (to detect
// a grasp lost mid-lift, see lift_transport_place's Stage 3 check) and
// growing a second copy is exactly the mistake transport.hpp's own header
// already argues against for the gripper-command helper.

#pragma once

#include <array>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>

namespace ur5e_pick_place
{

inline std::string run_command(const std::string & cmd)
{
  std::array<char, 4096> buf;
  std::string out;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    return out;
  }
  while (fgets(buf.data(), buf.size(), pipe.get()) != nullptr) {
    out += buf.data();
  }
  return out;
}

// Ground-truth JOINT position (not a link pose) for a named joint, from a
// `gz topic -e` text dump of a gz.msgs.Model joint_state topic.
inline std::optional<double> parse_joint_position(
  const std::string & dump, const std::string & joint_name)
{
  const std::string name_needle = "name: \"" + joint_name + "\"";
  auto name_pos = dump.find(name_needle);
  if (name_pos == std::string::npos) {
    return std::nullopt;
  }
  auto pos_key = dump.find("position:", name_pos);
  // Bound the search to this joint's own block: stop at the next "joint {"
  // or end of string, so a malformed/short dump can't accidentally read a
  // neighboring joint's position field.
  auto next_joint = dump.find("joint {", name_pos + name_needle.size());
  if (pos_key == std::string::npos || (next_joint != std::string::npos && pos_key > next_joint)) {
    return std::nullopt;
  }
  return std::strtod(dump.c_str() + pos_key + std::string("position:").size(), nullptr);
}

}  // namespace ur5e_pick_place
