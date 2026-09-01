#include "ur5e_pick_place/planning_scene_manager.hpp"

#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace ur5e_pick_place
{
namespace
{
constexpr char kTable[] = "table";
constexpr char kTarget[] = "pick_target";
constexpr char kAttachLink[] = "gripper_base_link";
constexpr char kBase[] = "base_link_inertia";
constexpr char kFixedPad[] = "pad_fixed_link";
constexpr char kMovingPad[] = "pad_moving_link";
constexpr auto kTimeout = std::chrono::seconds(5);

bool same(double a, double b) { return std::abs(a - b) < 1e-9; }

bool sameOrientation(const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b)
{
  const double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  return std::abs(std::abs(dot) - 1.0) < 1e-9;
}

}  // namespace

bool PlanningSceneManager::samePose(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return same(a.position.x, b.position.x) &&
         same(a.position.y, b.position.y) &&
         same(a.position.z, b.position.z) &&
         sameOrientation(a.orientation, b.orientation);
}

geometry_msgs::msg::Pose PlanningSceneManager::composePoses(
  const geometry_msgs::msg::Pose & parent, const geometry_msgs::msg::Pose & child)
{
  auto normalizeQuat = [](const geometry_msgs::msg::Quaternion & q) -> geometry_msgs::msg::Quaternion {
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm_sq < 1e-12) {
      geometry_msgs::msg::Quaternion id;
      id.w = 1.0;
      return id;
    }
    const double inv_norm = 1.0 / std::sqrt(norm_sq);
    geometry_msgs::msg::Quaternion res;
    res.x = q.x * inv_norm;
    res.y = q.y * inv_norm;
    res.z = q.z * inv_norm;
    res.w = q.w * inv_norm;
    return res;
  };

  const auto q1 = normalizeQuat(parent.orientation);
  const auto q2 = normalizeQuat(child.orientation);

  const double px = child.position.x;
  const double py = child.position.y;
  const double pz = child.position.z;

  const double rx = q1.x, ry = q1.y, rz = q1.z, rw = q1.w;
  const double tx = 2.0 * (ry * pz - rz * py + rw * px);
  const double ty = 2.0 * (rz * px - rx * pz + rw * py);
  const double tz = 2.0 * (rx * py - ry * px + rw * pz);

  const double rot_px = px + (ry * tz - rz * ty);
  const double rot_py = py + (rz * tx - rx * tz);
  const double rot_pz = pz + (rx * ty - ry * tx);

  geometry_msgs::msg::Pose result;
  result.position.x = parent.position.x + rot_px;
  result.position.y = parent.position.y + rot_py;
  result.position.z = parent.position.z + rot_pz;

  result.orientation.w = rw * q2.w - rx * q2.x - ry * q2.y - rz * q2.z;
  result.orientation.x = rw * q2.x + rx * q2.w + ry * q2.z - rz * q2.y;
  result.orientation.y = rw * q2.y - rx * q2.z + ry * q2.w + rz * q2.x;
  result.orientation.z = rw * q2.z + rx * q2.y - ry * q2.x + rz * q2.w;

  result.orientation = normalizeQuat(result.orientation);
  return result;
}

geometry_msgs::msg::Pose PlanningSceneManager::effectivePrimitivePose(
  const moveit_msgs::msg::CollisionObject & object, std::size_t primitive_index)
{
  if (primitive_index >= object.primitive_poses.size()) {
    throw std::out_of_range("Primitive pose index out of range");
  }
  return composePoses(object.pose, object.primitive_poses[primitive_index]);
}

PlanningSceneManager::PlanningSceneManager(const rclcpp::Node::SharedPtr & node, std::string world_frame)
: node_(node), world_frame_(std::move(world_frame))
{
  refreshFingerprint();
}

moveit_msgs::msg::CollisionObject PlanningSceneManager::makeTable(const std::string & frame)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = kTable;
  object.header.frame_id = frame;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {1.20, 0.80, 0.75};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.55;
  pose.position.y = 0.0;
  pose.position.z = 0.375;
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);
  return object;
}

moveit_msgs::msg::CollisionObject PlanningSceneManager::makeTarget(
  const std::string & frame, const geometry_msgs::msg::Pose & pose, TargetPoseSource source)
{
  if (source != TargetPoseSource::PRODUCTION_PERCEPTION) {
    throw std::invalid_argument("PlanningScene target pose must come from production perception");
  }
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.w)) {
    throw std::invalid_argument("PlanningScene target pose is not finite");
  }
  moveit_msgs::msg::CollisionObject object;
  object.id = kTarget;
  object.header.frame_id = frame;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.030, 0.045, 0.045};
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  return object;
}

std::vector<std::string> PlanningSceneManager::padTouchLinks() { return {kFixedPad, kMovingPad}; }

bool PlanningSceneManager::setPair(moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & first, const std::string & second, bool allowed)
{
  auto add = [&acm](const std::string & name) {
      if (std::find(acm.entry_names.begin(), acm.entry_names.end(), name) != acm.entry_names.end()) return;
      const auto old = acm.entry_names.size();
      acm.entry_names.push_back(name);
      for (auto & value : acm.entry_values) value.enabled.resize(old + 1, false);
      moveit_msgs::msg::AllowedCollisionEntry value;
      value.enabled.resize(old + 1, false);
      acm.entry_values.push_back(value);
    };
  add(first); add(second);
  const auto i = static_cast<std::size_t>(std::distance(acm.entry_names.begin(),
    std::find(acm.entry_names.begin(), acm.entry_names.end(), first)));
  const auto j = static_cast<std::size_t>(std::distance(acm.entry_names.begin(),
    std::find(acm.entry_names.begin(), acm.entry_names.end(), second)));
  if (acm.entry_values.size() != acm.entry_names.size()) return false;
  for (auto & value : acm.entry_values) value.enabled.resize(acm.entry_names.size(), false);
  acm.entry_values[i].enabled[j] = allowed;
  acm.entry_values[j].enabled[i] = allowed;
  return true;
}

bool PlanningSceneManager::pairValue(const moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & first, const std::string & second, bool & allowed)
{
  const auto i = std::find(acm.entry_names.begin(), acm.entry_names.end(), first);
  const auto j = std::find(acm.entry_names.begin(), acm.entry_names.end(), second);
  if (i == acm.entry_names.end() || j == acm.entry_names.end()) return false;
  const auto ii = static_cast<std::size_t>(std::distance(acm.entry_names.begin(), i));
  const auto jj = static_cast<std::size_t>(std::distance(acm.entry_names.begin(), j));
  if (ii >= acm.entry_values.size() || jj >= acm.entry_values[ii].enabled.size()) return false;
  allowed = acm.entry_values[ii].enabled[jj];
  return true;
}

void PlanningSceneManager::eraseEntry(moveit_msgs::msg::AllowedCollisionMatrix & acm, const std::string & name)
{
  const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
  if (it == acm.entry_names.end()) return;
  const auto index = static_cast<std::size_t>(std::distance(acm.entry_names.begin(), it));
  acm.entry_names.erase(it);
  if (index < acm.entry_values.size()) acm.entry_values.erase(acm.entry_values.begin() + index);
  for (auto & value : acm.entry_values) {
    if (index < value.enabled.size()) value.enabled.erase(value.enabled.begin() + index);
  }
}

bool PlanningSceneManager::fetch(moveit_msgs::msg::PlanningScene & scene, std::string & error) const
{
  auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!client->wait_for_service(kTimeout)) { error = "GET_PLANNING_SCENE_TIMEOUT"; return false; }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
    moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING;
  auto future = client->async_send_request(request);
  if (future.wait_for(kTimeout) != std::future_status::ready) { error = "GET_PLANNING_SCENE_TIMEOUT"; return false; }
  scene = future.get()->scene;
  return true;
}

bool PlanningSceneManager::mutateAcm(
  const std::vector<std::pair<std::pair<std::string, std::string>, bool>> & changes,
  moveit_msgs::msg::PlanningScene & diff, std::string & error) const
{
  moveit_msgs::msg::PlanningScene current;
  if (!fetch(current, error)) return false;
  diff.allowed_collision_matrix = current.allowed_collision_matrix;
  diff.is_diff = true;
  for (const auto & change : changes) {
    if (!setPair(diff.allowed_collision_matrix, change.first.first, change.first.second, change.second)) {
      error = "ACM_MALFORMED"; return false;
    }
  }
  return true;
}

bool PlanningSceneManager::applyAndVerify(
  moveit_msgs::msg::PlanningScene diff, std::string & error, bool verify)
{
  auto client = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  if (!client->wait_for_service(kTimeout)) { error = "APPLY_PLANNING_SCENE_TIMEOUT"; return false; }
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = std::move(diff);
  auto future = client->async_send_request(request);
  if (future.wait_for(kTimeout) != std::future_status::ready || !future.get()->success) {
    error = "APPLY_PLANNING_SCENE_FAILED"; return false;
  }
  return !verify || verifyExpectedScene(error);
}

bool PlanningSceneManager::initializeTable(std::string & error)
{
  moveit_msgs::msg::PlanningScene diff;
  if (!mutateAcm({{{kTable, kBase}, true}}, diff, error)) return false;
  diff.world.collision_objects.push_back(makeTable(world_frame_));
  return applyAndVerify(std::move(diff), error);
}

bool PlanningSceneManager::addWorldTarget(const geometry_msgs::msg::Pose & pose, std::string & error)
{
  if (target_state_ == SceneTargetState::ATTACHED) { error = "ILLEGAL_TARGET_TRANSITION"; return false; }
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  try { diff.world.collision_objects.push_back(makeTarget(world_frame_, pose, TargetPoseSource::PRODUCTION_PERCEPTION)); }
  catch (const std::exception & e) { error = e.what(); return false; }
  target_pose_ = pose; target_state_ = SceneTargetState::WORLD; refreshFingerprint();
  return applyAndVerify(std::move(diff), error);
}

bool PlanningSceneManager::enableClosureContacts(std::string & error)
{
  if (target_state_ != SceneTargetState::WORLD) { error = "ILLEGAL_TARGET_TRANSITION"; return false; }
  moveit_msgs::msg::PlanningScene diff;
  if (!mutateAcm({{{kTarget, kFixedPad}, true}, {{kTarget, kMovingPad}, true}}, diff, error)) return false;
  return applyAndVerify(std::move(diff), error);
}

bool PlanningSceneManager::attachTarget(std::string & error)
{
  if (target_state_ != SceneTargetState::WORLD) { error = "ILLEGAL_TARGET_TRANSITION"; return false; }
  moveit_msgs::msg::PlanningScene diff;
  if (!mutateAcm({}, diff, error)) return false;
  eraseEntry(diff.allowed_collision_matrix, kTarget);  // remove C1/C2, then add only S
  if (!setPair(diff.allowed_collision_matrix, kTarget, kTable, true)) { error = "ACM_MALFORMED"; return false; }
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = kAttachLink;
  attached.touch_links = padTouchLinks();
  attached.object = makeTarget(world_frame_, target_pose_, TargetPoseSource::PRODUCTION_PERCEPTION);
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  diff.robot_state.is_diff = true;
  diff.robot_state.attached_collision_objects.push_back(attached);
  target_state_ = SceneTargetState::ATTACHED; refreshFingerprint();
  return applyAndVerify(std::move(diff), error);
}

bool PlanningSceneManager::removePickupSupportException(std::string & error)
{
  if (target_state_ != SceneTargetState::ATTACHED) { error = "ILLEGAL_TARGET_TRANSITION"; return false; }
  moveit_msgs::msg::PlanningScene diff;
  if (!mutateAcm({}, diff, error)) return false;
  eraseEntry(diff.allowed_collision_matrix, kTarget);
  return applyAndVerify(std::move(diff), error);
}

bool PlanningSceneManager::enablePlacementSupport(std::string & error)
{
  if (target_state_ != SceneTargetState::ATTACHED) { error = "ILLEGAL_TARGET_TRANSITION"; return false; }
  moveit_msgs::msg::PlanningScene diff;
  if (!mutateAcm({{{kTarget, kTable}, true}}, diff, error)) return false;
  return applyAndVerify(std::move(diff), error);
}

bool PlanningSceneManager::detachTargetToWorld(std::string & error)
{
  if (target_state_ != SceneTargetState::ATTACHED) { error = "ILLEGAL_TARGET_TRANSITION"; return false; }
  moveit_msgs::msg::PlanningScene diff;
  if (!mutateAcm({}, diff, error)) return false;
  eraseEntry(diff.allowed_collision_matrix, kTarget);
  moveit_msgs::msg::AttachedCollisionObject remove;
  remove.link_name = kAttachLink;
  remove.object.id = kTarget;
  remove.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  diff.robot_state.is_diff = true;
  diff.robot_state.attached_collision_objects.push_back(remove);
  target_state_ = SceneTargetState::WORLD;
  if (!applyAndVerify(std::move(diff), error, false)) return false;
  moveit_msgs::msg::PlanningScene readback;
  if (!fetch(readback, error)) return false;
  const auto world = std::find_if(readback.world.collision_objects.begin(),
    readback.world.collision_objects.end(), [](const auto & object) { return object.id == kTarget; });
  if (world == readback.world.collision_objects.end() || world->primitive_poses.size() != 1) {
    error = "SCENE_STALE_DETACH_POSE"; return false;
  }
  target_pose_ = effectivePrimitivePose(*world, 0);
  refreshFingerprint();
  return verifyExpectedScene(error);
}

bool PlanningSceneManager::updateWorldTarget(const geometry_msgs::msg::Pose & pose, std::string & error)
{
  if (target_state_ != SceneTargetState::WORLD) { error = "ILLEGAL_TARGET_TRANSITION"; return false; }
  return addWorldTarget(pose, error);
}

bool PlanningSceneManager::verifyExpectedScene(std::string & error)
{
  moveit_msgs::msg::PlanningScene scene;
  if (!fetch(scene, error)) return false;

  const auto table = std::find_if(scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
    [](const auto & object) { return object.id == kTable; });
  if (table == scene.world.collision_objects.end() ||
      table->header.frame_id != world_frame_ ||
      table->primitives.size() != 1 ||
      table->primitive_poses.size() != 1 ||
      table->primitives.front().type != shape_msgs::msg::SolidPrimitive::BOX ||
      table->primitives.front().dimensions.size() != 3 ||
      !same(table->primitives.front().dimensions[0], 1.20) ||
      !same(table->primitives.front().dimensions[1], 0.80) ||
      !same(table->primitives.front().dimensions[2], 0.75)) {
    error = "SCENE_STALE_TABLE"; return false;
  }
  const auto table_eff_pose = effectivePrimitivePose(*table, 0);
  geometry_msgs::msg::Pose expected_table_pose;
  expected_table_pose.position.x = 0.55;
  expected_table_pose.position.y = 0.0;
  expected_table_pose.position.z = 0.375;
  expected_table_pose.orientation.w = 1.0;
  if (!samePose(table_eff_pose, expected_table_pose)) {
    error = "SCENE_STALE_TABLE"; return false;
  }

  const auto world_target = std::find_if(scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
    [](const auto & object) { return object.id == kTarget; });
  const auto attached_target = std::find_if(scene.robot_state.attached_collision_objects.begin(), scene.robot_state.attached_collision_objects.end(),
    [](const auto & object) { return object.object.id == kTarget; });
  if ((target_state_ == SceneTargetState::WORLD) != (world_target != scene.world.collision_objects.end()) ||
      (target_state_ == SceneTargetState::ATTACHED) != (attached_target != scene.robot_state.attached_collision_objects.end())) {
    error = "SCENE_STALE_TARGET_STATE"; return false;
  }
  if (target_state_ == SceneTargetState::WORLD) {
    if (world_target->header.frame_id != world_frame_ ||
        world_target->primitives.size() != 1 ||
        world_target->primitive_poses.size() != 1 ||
        world_target->primitives.front().type != shape_msgs::msg::SolidPrimitive::BOX ||
        world_target->primitives.front().dimensions.size() != 3 ||
        !same(world_target->primitives.front().dimensions[0], 0.030) ||
        !same(world_target->primitives.front().dimensions[1], 0.045) ||
        !same(world_target->primitives.front().dimensions[2], 0.045)) {
      error = "SCENE_STALE_TARGET_STATE"; return false;
    }
    const auto target_eff_pose = effectivePrimitivePose(*world_target, 0);
    if (!samePose(target_eff_pose, target_pose_)) {
      error = "SCENE_STALE_TARGET_POSE"; return false;
    }
  }
  if (target_state_ == SceneTargetState::ATTACHED &&
      (attached_target->link_name != kAttachLink || attached_target->touch_links != padTouchLinks())) {
    error = "SCENE_STALE_ATTACHMENT"; return false;
  }
  bool p = false;
  if (!pairValue(scene.allowed_collision_matrix, kTable, kBase, p) || !p) { error = "SCENE_STALE_ACM_P"; return false; }
  for (const auto & padding : scene.link_padding) {
    if ((padding.link_name == kFixedPad || padding.link_name == kMovingPad) && !same(padding.padding, 0.0)) {
      error = "SCENE_PADDING_NOT_ZERO"; return false;
    }
  }
  for (const auto & scale : scene.link_scale) {
    if ((scale.link_name == kFixedPad || scale.link_name == kMovingPad) && !same(scale.scale, 1.0)) {
      error = "SCENE_SCALE_NOT_ONE"; return false;
    }
  }
  return true;
}

bool PlanningSceneManager::currentAttachedTargetGlobalPose(
  const moveit::core::RobotState & state, const std::string & target_id,
  geometry_msgs::msg::Pose & global_pose, std::string & error)
{
  const auto * attached_body = state.getAttachedBody(target_id);
  if (!attached_body) {
    error = "ATTACHED_BODY_ABSENT: target '" + target_id + "' is not attached to robot state";
    return false;
  }
  if (attached_body->getShapes().size() != 1) {
    error = "ATTACHED_BODY_INVALID_SHAPE_COUNT: expected 1 shape, got " +
      std::to_string(attached_body->getShapes().size());
    return false;
  }
  if (attached_body->getShapes().front()->type != shapes::BOX) {
    error = "ATTACHED_BODY_INVALID_SHAPE_TYPE: expected BOX primitive";
    return false;
  }
  const auto & transforms = attached_body->getGlobalCollisionBodyTransforms();
  if (transforms.empty()) {
    error = "ATTACHED_BODY_NO_TRANSFORMS: global collision body transforms are empty";
    return false;
  }
  global_pose = tf2::toMsg(transforms.front());
  return true;
}

bool PlanningSceneManager::copyRobotStateVariables(
  const moveit::core::RobotState & source,
  moveit::core::RobotState & target,
  std::string & error)
{
  const auto & source_names = source.getRobotModel()->getVariableNames();
  const auto & target_names = target.getRobotModel()->getVariableNames();

  if (source.getRobotModel()->getVariableCount() == target.getRobotModel()->getVariableCount() &&
      source_names == target_names)
  {
    target.setVariablePositions(source.getVariablePositions());
    target.update(true);
    return true;
  }

  for (const auto & name : target_names) {
    auto it = std::find(source_names.begin(), source_names.end(), name);
    if (it != source_names.end()) {
      target.setVariablePosition(name, source.getVariablePosition(name));
    } else {
      error = "INCOMPATIBLE_ROBOT_STATE_VARIABLE: missing variable '" + name + "' in source state";
      return false;
    }
  }
  target.update(true);
  return true;
}

bool PlanningSceneManager::checkPickupClearanceClone(
  const moveit::core::RobotState & current_state, double & separation_z, std::string & error)
{
  if (target_state_ != SceneTargetState::ATTACHED) {
    error = "ILLEGAL_TARGET_STATE_FOR_PICKUP_CLONE";
    return false;
  }

  // 1. Fetch live scene message
  moveit_msgs::msg::PlanningScene live_scene_msg;
  if (!fetch(live_scene_msg, error)) {
    return false;
  }

  // 2. Create local cloned planning scene and remove ONLY S (pick_target <-> table)
  moveit_msgs::msg::PlanningScene clone_scene_msg = live_scene_msg;
  clone_scene_msg.is_diff = false;
  eraseEntry(clone_scene_msg.allowed_collision_matrix, kTarget);

  auto clone_scene = std::make_shared<planning_scene::PlanningScene>(current_state.getRobotModel());
  if (!clone_scene->usePlanningSceneMsg(clone_scene_msg)) {
    error = "FAILED_TO_LOAD_CLONED_PLANNING_SCENE";
    return false;
  }

  // 3. Verify pick_target is attached before measured-state update
  if (!clone_scene->getCurrentState().hasAttachedBody(kTarget)) {
    error = "ATTACHED_BODY_PRECHECK_FAILED: target '" + std::string(kTarget) +
      "' not attached in cloned scene before state update";
    return false;
  }

  // 4. Update measured variable positions into clone state without replacing attached bodies
  if (!copyRobotStateVariables(current_state, clone_scene->getCurrentStateNonConst(), error)) {
    return false;
  }

  // 5. Verify pick_target is STILL attached after measured-state update
  if (!clone_scene->getCurrentState().hasAttachedBody(kTarget)) {
    error = "ATTACHED_BODY_POSTCHECK_FAILED: target '" + std::string(kTarget) +
      "' lost after measured variable update";
    return false;
  }

  // 6. Compute attached target's actual global collision-body pose using MoveIt AttachedBody representation
  geometry_msgs::msg::Pose target_global_pose;
  if (!currentAttachedTargetGlobalPose(clone_scene->getCurrentState(), kTarget, target_global_pose, error)) {
    return false;
  }

  // 7. Analytically verify target bottom > table top
  const double target_bottom_z = target_global_pose.position.z - 0.045 / 2.0;
  const double table_top_z = 0.750;
  separation_z = target_bottom_z - table_top_z;

  if (separation_z <= 0.0) {
    std::ostringstream ss;
    ss << "ANALYTICAL_CLEARANCE_VIOLATION: target bottom (" << target_bottom_z
       << " m) <= table top (" << table_top_z << " m), separation=" << separation_z << " m";
    error = ss.str();
    return false;
  }

  // 8. Run full collision check in the cloned scene
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 10;
  collision_detection::CollisionResult res;
  clone_scene->checkCollision(req, res, clone_scene->getCurrentState());

  if (res.collision) {
    std::ostringstream ss;
    ss << "CLONE_COLLISION_DETECTED: ";
    for (const auto & pair : res.contacts) {
      ss << pair.first.first << "<->" << pair.first.second << "; ";
    }
    error = ss.str();
    return false;
  }

  return true;
}

bool PlanningSceneManager::checkPlacementPrecontact(
  const moveit::core::RobotState & current_state, double & separation_z, std::string & error)
{
  if (target_state_ != SceneTargetState::ATTACHED) {
    error = "ILLEGAL_TARGET_STATE_FOR_PRECONTACT";
    return false;
  }

  // 1. Fetch live scene message (where S is still absent)
  moveit_msgs::msg::PlanningScene live_scene_msg;
  if (!fetch(live_scene_msg, error)) {
    return false;
  }

  // 2. Create local planning scene and load state
  auto precontact_scene = std::make_shared<planning_scene::PlanningScene>(current_state.getRobotModel());
  if (!precontact_scene->usePlanningSceneMsg(live_scene_msg)) {
    error = "FAILED_TO_LOAD_PRECONTACT_PLANNING_SCENE";
    return false;
  }

  // 3. Verify pick_target is attached before update
  if (!precontact_scene->getCurrentState().hasAttachedBody(kTarget)) {
    error = "ATTACHED_BODY_PRECHECK_FAILED: target '" + std::string(kTarget) +
      "' not attached in precontact scene before state update";
    return false;
  }

  // 4. Update measured variable positions into precontact state without replacing attached bodies
  if (!copyRobotStateVariables(current_state, precontact_scene->getCurrentStateNonConst(), error)) {
    return false;
  }

  // 5. Verify pick_target is STILL attached after update
  if (!precontact_scene->getCurrentState().hasAttachedBody(kTarget)) {
    error = "ATTACHED_BODY_POSTCHECK_FAILED: target '" + std::string(kTarget) +
      "' lost after measured variable update in precontact scene";
    return false;
  }

  // 6. Compute attached target's actual global collision-body pose using MoveIt AttachedBody representation
  geometry_msgs::msg::Pose target_global_pose;
  if (!currentAttachedTargetGlobalPose(precontact_scene->getCurrentState(), kTarget, target_global_pose, error)) {
    return false;
  }

  // 7. Analytically verify target bottom > table top
  const double target_bottom_z = target_global_pose.position.z - 0.045 / 2.0;
  const double table_top_z = 0.750;
  separation_z = target_bottom_z - table_top_z;

  if (separation_z <= 0.0) {
    std::ostringstream ss;
    ss << "ANALYTICAL_PRECONTACT_VIOLATION: target bottom (" << target_bottom_z
       << " m) <= table top (" << table_top_z << " m), separation=" << separation_z << " m";
    error = ss.str();
    return false;
  }

  // 8. Run collision check with S absent
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 10;
  collision_detection::CollisionResult res;
  precontact_scene->checkCollision(req, res, precontact_scene->getCurrentState());

  if (res.collision) {
    std::ostringstream ss;
    ss << "PRECONTACT_COLLISION_DETECTED: ";
    for (const auto & pair : res.contacts) {
      ss << pair.first.first << "<->" << pair.first.second << "; ";
    }
    error = ss.str();
    return false;
  }

  return true;
}

void PlanningSceneManager::refreshFingerprint()
{
  std::ostringstream stream;
  stream << static_cast<int>(target_state_) << ':' << target_pose_.position.x << ':' << target_pose_.position.y << ':' << target_pose_.position.z;
  fingerprint_ = stream.str();
}

}  // namespace ur5e_pick_place
