#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace ur5e_pick_place
{

enum class TargetPoseSource { PRODUCTION_PERCEPTION, SHADOW_ESTIMATOR, GAZEBO_GROUND_TRUTH };
enum class SceneTargetState { ABSENT, WORLD, ATTACHED };

class PlanningSceneManager
{
public:
  explicit PlanningSceneManager(const rclcpp::Node::SharedPtr & node, std::string world_frame = "world");

  static moveit_msgs::msg::CollisionObject makeTable(const std::string & world_frame);
  static moveit_msgs::msg::CollisionObject makeTarget(
    const std::string & world_frame, const geometry_msgs::msg::Pose & pose,
    TargetPoseSource source);
  static std::vector<std::string> padTouchLinks();
  static bool setPair(moveit_msgs::msg::AllowedCollisionMatrix & acm,
    const std::string & first, const std::string & second, bool allowed);
  static bool pairValue(const moveit_msgs::msg::AllowedCollisionMatrix & acm,
    const std::string & first, const std::string & second, bool & allowed);
  static void eraseEntry(moveit_msgs::msg::AllowedCollisionMatrix & acm, const std::string & name);

  static geometry_msgs::msg::Pose composePoses(
    const geometry_msgs::msg::Pose & parent, const geometry_msgs::msg::Pose & child);
  static geometry_msgs::msg::Pose effectivePrimitivePose(
    const moveit_msgs::msg::CollisionObject & object, std::size_t primitive_index = 0);
  static bool samePose(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);
  static bool currentAttachedTargetGlobalPose(
    const moveit::core::RobotState & state, const std::string & target_id,
    geometry_msgs::msg::Pose & global_pose, std::string & error);
  static bool copyRobotStateVariables(
    const moveit::core::RobotState & source,
    moveit::core::RobotState & target,
    std::string & error);

  bool initializeTable(std::string & error);
  bool addWorldTarget(const geometry_msgs::msg::Pose & perceived_pose, std::string & error);
  bool enableClosureContacts(std::string & error);
  bool attachTarget(std::string & error);
  bool removePickupSupportException(std::string & error);
  bool enablePlacementSupport(std::string & error);
  bool detachTargetToWorld(std::string & error);
  bool updateWorldTarget(const geometry_msgs::msg::Pose & perceived_pose, std::string & error);
  bool verifyExpectedScene(std::string & error);

  // Evaluates measured robot state on a local cloned planning scene with S removed
  bool checkPickupClearanceClone(
    const moveit::core::RobotState & current_state, double & separation_z, std::string & error);

  // Evaluates measured robot state at pre-contact waypoint with S absent
  bool checkPlacementPrecontact(
    const moveit::core::RobotState & current_state, double & separation_z, std::string & error);

  SceneTargetState targetState() const { return target_state_; }
  const std::string & fingerprint() const { return fingerprint_; }
  const geometry_msgs::msg::Pose & targetPose() const { return target_pose_; }

private:
  bool fetch(moveit_msgs::msg::PlanningScene & scene, std::string & error) const;
  bool applyAndVerify(moveit_msgs::msg::PlanningScene diff, std::string & error, bool verify = true);
  bool mutateAcm(const std::vector<std::pair<std::pair<std::string, std::string>, bool>> & changes,
    moveit_msgs::msg::PlanningScene & diff, std::string & error) const;
  void refreshFingerprint();

  rclcpp::Node::SharedPtr node_;
  std::string world_frame_;
  SceneTargetState target_state_{SceneTargetState::ABSENT};
  geometry_msgs::msg::Pose target_pose_{};
  std::string fingerprint_;
};

}  // namespace ur5e_pick_place
