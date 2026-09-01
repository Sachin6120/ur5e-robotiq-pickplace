#include "ur5e_pick_place/planning_scene_manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("planning_scene_manager_smoke");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  ur5e_pick_place::PlanningSceneManager manager(node);
  std::string error;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.45;
  pose.position.y = -0.15;
  pose.position.z = 0.7725;
  pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose updated_pose;
  updated_pose.position.x = 0.48;
  updated_pose.position.y = -0.12;
  updated_pose.position.z = 0.7725;
  updated_pose.orientation.w = 1.0;

  bool ok = manager.initializeTable(error);
  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at initializeTable: %s", error.c_str());
  }

  if (ok) {
    ok = manager.addWorldTarget(pose, error);
    if (!ok) RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at addWorldTarget: %s", error.c_str());
  }

  if (ok) {
    ok = manager.enableClosureContacts(error);
    if (!ok) RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at enableClosureContacts: %s", error.c_str());
  }

  if (ok) {
    ok = manager.attachTarget(error);
    if (!ok) RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at attachTarget: %s", error.c_str());
  }

  if (ok) {
    std::string fail_err;
    if (manager.addWorldTarget(pose, fail_err)) {
      RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED: addWorldTarget while attached was not rejected");
      ok = false;
    }
  }

  if (ok) {
    ok = manager.removePickupSupportException(error);
    if (!ok) RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at removePickupSupportException: %s", error.c_str());
  }

  if (ok) {
    ok = manager.enablePlacementSupport(error);
    if (!ok) RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at enablePlacementSupport: %s", error.c_str());
  }

  if (ok) {
    ok = manager.detachTargetToWorld(error);
    if (!ok) RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at detachTargetToWorld: %s", error.c_str());
  }

  if (ok) {
    std::string fail_err;
    if (manager.removePickupSupportException(fail_err)) {
      RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED: removePickupSupportException while detached was not rejected");
      ok = false;
    }
  }

  if (ok) {
    ok = manager.updateWorldTarget(updated_pose, error);
    if (!ok) RCLCPP_ERROR(node->get_logger(), "SCENE_SMOKE_FAILED at updateWorldTarget: %s", error.c_str());
  }

  if (ok) {
    RCLCPP_INFO(node->get_logger(), "SCENE_SMOKE_SUCCESS: all lifecycle stages and failure paths verified");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
