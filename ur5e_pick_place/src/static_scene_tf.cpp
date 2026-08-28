// static_scene_tf.cpp — publishes the scene.yaml-derived static frames.
//
// FRAMES PUBLISHED
//   world -> object_frame   at object.pick_pose (translation + rpy)
//   object_frame -> grasp_frame
//       translation: zero (grasp point is the object's own origin for M2 —
//         no sub-object offset yet, that is a later-milestone refinement)
//       rotation: local Z axis aligned with grasp.approach_axis (expressed in
//         object_frame), then rotated about that same axis by
//         grasp.gripper_roll. approach_axis only constrains 2 of 3 rotational
//         DOF (the direction of Z); gripper_roll is exactly the remaining one.
//   tool0 -> grasp_tcp
//       translation: (0, 0, tcp_offset) along tool0's own local Z — this is
//         literally how tcp_offset was measured (§ M-1 report, gripper
//         geometry sweep), so the frame definition matches the measurement,
//         not a reinterpretation of it.
//       rotation: identity.
//
// WHY object_frame -> grasp_frame's ROTATION MATTERS AND ISN'T ARBITRARY
//   computeCartesianPath() moves the end effector to a target POSE, not just
//   a target point. If grasp_frame's orientation didn't have its Z axis
//   genuinely aligned with approach_axis, "back off along -approach by
//   standoff" (done in the planning node, in grasp_frame's own local Z)
//   would not actually back off along the geometric approach direction.
//
// DEGENERATE CASE, handled rather than assumed away
//   The construction below picks a world reference vector to build an
//   orthonormal basis with approach_axis. If approach_axis is nearly
//   parallel to that reference (this project's approach_axis is exactly
//   world -Z, so the natural reference of world +Z is exactly antiparallel —
//   precisely the degenerate case), cross(reference, z_axis) is near zero
//   and the basis is numerically garbage. Falls back to a different
//   reference vector in that case rather than silently producing a bad
//   frame.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

// Builds the rotation whose local Z axis is `approach_axis` (in the parent
// frame), with the remaining rotational freedom about that axis fixed by
// `roll`. See the file-level comment for the degenerate-case handling.
tf2::Quaternion orientation_from_approach_axis(
  const tf2::Vector3 & approach_axis_in, double roll)
{
  tf2::Vector3 z_axis = approach_axis_in;
  if (z_axis.length() < 1e-9) {
    throw std::runtime_error(
            "CONFIG_ERROR: grasp.approach_axis is the zero vector — cannot "
            "define an orientation from it");
  }
  z_axis.normalize();

  tf2::Vector3 reference(0, 0, 1);
  if (std::abs(z_axis.dot(reference)) > 0.99) {
    // approach_axis nearly parallel or antiparallel to world Z (true for
    // this project's straight-down approach) — cross product with Z would be
    // near-zero and numerically unstable. Use world X instead.
    reference = tf2::Vector3(1, 0, 0);
  }

  tf2::Vector3 x_axis = reference.cross(z_axis);
  x_axis.normalize();
  tf2::Vector3 y_axis = z_axis.cross(x_axis);
  y_axis.normalize();

  tf2::Matrix3x3 basis(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z());
  tf2::Quaternion q_base;
  basis.getRotation(q_base);

  tf2::Quaternion q_roll;
  q_roll.setRPY(0, 0, roll);

  // q_base * q_roll: apply roll about the LOCAL z axis established by
  // q_base (i.e. about approach_axis itself), then orient into the parent
  // frame via q_base. This is the composition order, not the reverse.
  tf2::Quaternion q = q_base * q_roll;
  q.normalize();
  return q;
}

geometry_msgs::msg::TransformStamped make_transform(
  const std::string & parent, const std::string & child,
  double x, double y, double z, const tf2::Quaternion & q,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  return t;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "static_scene_tf",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  // All of these originate in config/scene.yaml, read by the launch file.
  std::string world_frame = "world";
  std::string object_frame_name = "object_frame";
  std::string grasp_frame_name = "grasp_frame";
  std::string flange_frame = "tool0";
  std::string tcp_frame_name = "grasp_tcp";
  std::string place_frame_name = "place_frame";

  std::vector<double> pick_pose;        // x y z roll pitch yaw
  std::vector<double> approach_axis;    // x y z, in object_frame
  double gripper_roll = 0.0;
  double tcp_offset = 0.0;
  // place_pose is OPTIONAL, unlike pick_pose: static_scene_tf is shared by
  // m2_cartesian_approach.launch.py (no place leg, never sets this) and
  // m3_grasp.launch.py (does). Absent or malformed -> skip publishing
  // place_frame rather than CONFIG_ERROR, so M2 is untouched.
  std::vector<double> place_pose;       // x y z roll pitch yaw

  node->get_parameter_or("world_frame", world_frame, world_frame);
  node->get_parameter_or("object_frame_name", object_frame_name, object_frame_name);
  node->get_parameter_or("grasp_frame_name", grasp_frame_name, grasp_frame_name);
  node->get_parameter_or("flange_frame", flange_frame, flange_frame);
  node->get_parameter_or("tcp_frame_name", tcp_frame_name, tcp_frame_name);
  node->get_parameter_or("place_frame_name", place_frame_name, place_frame_name);
  node->get_parameter_or("pick_pose", pick_pose, pick_pose);
  node->get_parameter_or("approach_axis", approach_axis, approach_axis);
  node->get_parameter_or("gripper_roll", gripper_roll, gripper_roll);
  node->get_parameter_or("tcp_offset", tcp_offset, tcp_offset);
  node->get_parameter_or("place_pose", place_pose, place_pose);

  if (pick_pose.size() != 6) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: pick_pose must have 6 values (x y z roll pitch yaw), got %zu",
      pick_pose.size());
    rclcpp::shutdown();
    return 1;
  }
  if (approach_axis.size() != 3) {
    RCLCPP_ERROR(
      logger, "CONFIG_ERROR: approach_axis must have 3 values, got %zu",
      approach_axis.size());
    rclcpp::shutdown();
    return 1;
  }

  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto now = node->get_clock()->now();

  tf2::Quaternion q_object;
  q_object.setRPY(pick_pose[3], pick_pose[4], pick_pose[5]);
  auto t_world_object = make_transform(
    world_frame, object_frame_name, pick_pose[0], pick_pose[1], pick_pose[2],
    q_object, now);

  tf2::Quaternion q_grasp;
  try {
    q_grasp = orientation_from_approach_axis(
      tf2::Vector3(approach_axis[0], approach_axis[1], approach_axis[2]),
      gripper_roll);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  auto t_object_grasp = make_transform(
    object_frame_name, grasp_frame_name, 0.0, 0.0, 0.0, q_grasp, now);

  tf2::Quaternion q_identity;
  q_identity.setRPY(0, 0, 0);
  auto t_flange_tcp = make_transform(
    flange_frame, tcp_frame_name, 0.0, 0.0, tcp_offset, q_identity, now);

  std::vector<geometry_msgs::msg::TransformStamped> transforms{
    t_world_object, t_object_grasp, t_flange_tcp};

  // world -> place_frame: SAME composition as world -> grasp_frame (world ->
  // object_frame at pick_pose, THEN the approach_axis/gripper_roll rotation),
  // just re-anchored at place_pose instead of pick_pose. This is the one
  // piece flagged as the likeliest place to get wrong: a place target needs a
  // full gripper pose, orientation included, built the same way the grasp
  // pose is -- not a position-only pose that leaves the wrist orientation
  // arbitrary and drops the object sideways. Published directly world ->
  // place_frame (not object_frame -> place_frame with a second hop) because
  // place_pose, like pick_pose, is expressed in world, not relative to the
  // pick location.
  bool published_place_frame = false;
  if (!place_pose.empty()) {
    if (place_pose.size() != 6) {
      RCLCPP_ERROR(
        logger,
        "CONFIG_ERROR: place_pose must have 6 values (x y z roll pitch yaw) "
        "if provided at all, got %zu — not publishing %s",
        place_pose.size(), place_frame_name.c_str());
    } else {
      tf2::Quaternion q_place_object;
      q_place_object.setRPY(place_pose[3], place_pose[4], place_pose[5]);
      tf2::Quaternion q_place = q_place_object * q_grasp;
      auto t_world_place = make_transform(
        world_frame, place_frame_name, place_pose[0], place_pose[1], place_pose[2],
        q_place, now);
      transforms.push_back(t_world_place);
      published_place_frame = true;
    }
  }

  broadcaster->sendTransform(transforms);

  // Periodic re-publish, retained as a precaution against the documented
  // /tf_static late-joiner discovery race (see the measured startup-race
  // comment in m2_cartesian_approach.cpp, next to its own polling
  // canTransform() loop). Every consumer of these frames in this repository
  // -- m2_cartesian_approach.cpp, m3_grasp.cpp, milestone_f1_truth.py --
  // looks them up at tf2::TimePointZero / rclpy.time.Time(), so restamping
  // changes no lookup outcome; this does not fix or guard against staleness.
  // Stage-1 (G1-G5) and the 5/5 Scene-A repeatability campaign both ran with
  // this timer compiled in, but no evidence in either campaign shows it
  // changed an outcome -- the one recorded TF_LOOKUP_TIMEOUT predates this
  // timer, and no run since has approached the configured deadline. Retained
  // because Stage-1 validated the binary that contains it; a candidate for
  // removal only under a future requalification, not by inference now.
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [broadcaster, transforms, node]() {
      auto t = transforms;
      auto now = node->get_clock()->now();
      for (auto & tr : t) {
        tr.header.stamp = now;
      }
      broadcaster->sendTransform(t);
    });

  const std::string place_frame_log = published_place_frame
    ? (", " + world_frame + "->" + place_frame_name + " (place_pose)")
    : std::string();
  RCLCPP_INFO(
    logger,
    "published static frames: %s->%s (pick_pose), %s->%s (approach_axis=[%.3f %.3f %.3f] "
    "roll=%.3f), %s->%s (tcp_offset=%.4f)%s",
    world_frame.c_str(), object_frame_name.c_str(),
    object_frame_name.c_str(), grasp_frame_name.c_str(),
    approach_axis[0], approach_axis[1], approach_axis[2], gripper_roll,
    flange_frame.c_str(), tcp_frame_name.c_str(), tcp_offset,
    place_frame_log.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
