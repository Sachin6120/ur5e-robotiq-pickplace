#include <gtest/gtest.h>

#include "ur5e_pick_place/planning_scene_manager.hpp"

namespace
{
using ur5e_pick_place::PlanningSceneManager;

TEST(PlanningSceneManager, ConstructsExactTable)
{
  const auto table = PlanningSceneManager::makeTable("world");
  ASSERT_EQ(table.id, "table");
  ASSERT_EQ(table.primitives.size(), 1u);
  EXPECT_EQ(table.primitives[0].type, shape_msgs::msg::SolidPrimitive::BOX);
  ASSERT_EQ(table.primitives[0].dimensions.size(), 3u);
  EXPECT_DOUBLE_EQ(table.primitives[0].dimensions[0], 1.20);
  EXPECT_DOUBLE_EQ(table.primitives[0].dimensions[1], 0.80);
  EXPECT_DOUBLE_EQ(table.primitives[0].dimensions[2], 0.75);
  EXPECT_DOUBLE_EQ(table.primitive_poses[0].position.x, 0.55);
  EXPECT_DOUBLE_EQ(table.primitive_poses[0].position.z, 0.375);
}

TEST(PlanningSceneManager, TargetRejectsNonProductionSources)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  EXPECT_THROW(PlanningSceneManager::makeTarget("world", pose,
    ur5e_pick_place::TargetPoseSource::SHADOW_ESTIMATOR), std::invalid_argument);
  EXPECT_THROW(PlanningSceneManager::makeTarget("world", pose,
    ur5e_pick_place::TargetPoseSource::GAZEBO_GROUND_TRUTH), std::invalid_argument);
  const auto target = PlanningSceneManager::makeTarget("world", pose,
    ur5e_pick_place::TargetPoseSource::PRODUCTION_PERCEPTION);
  EXPECT_EQ(target.id, "pick_target");
  ASSERT_EQ(target.primitives[0].dimensions.size(), 3u);
  EXPECT_DOUBLE_EQ(target.primitives[0].dimensions[0], 0.030);
  EXPECT_DOUBLE_EQ(target.primitives[0].dimensions[1], 0.045);
  EXPECT_DOUBLE_EQ(target.primitives[0].dimensions[2], 0.045);
}

TEST(PlanningSceneManager, OnlyOwnPairsAreAddedAndUnrelatedEntriesSurvive)
{
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  ASSERT_TRUE(PlanningSceneManager::setPair(acm, "unrelated_a", "unrelated_b", true));
  ASSERT_TRUE(PlanningSceneManager::setPair(acm, "table", "base_link_inertia", true));
  bool value = false;
  ASSERT_TRUE(PlanningSceneManager::pairValue(acm, "unrelated_a", "unrelated_b", value));
  EXPECT_TRUE(value);
  ASSERT_TRUE(PlanningSceneManager::pairValue(acm, "table", "base_link_inertia", value));
  EXPECT_TRUE(value);
  EXPECT_FALSE(PlanningSceneManager::pairValue(acm, "pick_target", "table", value));
}

TEST(PlanningSceneManager, RemovingTargetRowLeavesWorldWorldPairAbsent)
{
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  ASSERT_TRUE(PlanningSceneManager::setPair(acm, "table", "base_link_inertia", true));
  ASSERT_TRUE(PlanningSceneManager::setPair(acm, "pick_target", "table", true));
  PlanningSceneManager::eraseEntry(acm, "pick_target");
  bool value = false;
  EXPECT_FALSE(PlanningSceneManager::pairValue(acm, "pick_target", "table", value));
  ASSERT_TRUE(PlanningSceneManager::pairValue(acm, "table", "base_link_inertia", value));
  EXPECT_TRUE(value);
}

TEST(PlanningSceneManager, TouchLinksAreExactlyTheTwoPads)
{
  EXPECT_EQ(PlanningSceneManager::padTouchLinks(),
    (std::vector<std::string>{"pad_fixed_link", "pad_moving_link"}));
}

TEST(PlanningSceneManager, PoseCompositionIdentityParentWorldChild)
{
  geometry_msgs::msg::Pose parent;
  parent.orientation.w = 1.0;
  geometry_msgs::msg::Pose child;
  child.position.x = 0.55;
  child.position.y = -0.15;
  child.position.z = 0.7725;
  child.orientation.w = 1.0;

  const auto eff = PlanningSceneManager::composePoses(parent, child);
  EXPECT_DOUBLE_EQ(eff.position.x, 0.55);
  EXPECT_DOUBLE_EQ(eff.position.y, -0.15);
  EXPECT_DOUBLE_EQ(eff.position.z, 0.7725);
  EXPECT_DOUBLE_EQ(eff.orientation.w, 1.0);
  EXPECT_TRUE(PlanningSceneManager::samePose(eff, child));
}

TEST(PlanningSceneManager, PoseCompositionWorldParentIdentityChild)
{
  geometry_msgs::msg::Pose parent;
  parent.position.x = 0.55;
  parent.position.y = -0.15;
  parent.position.z = 0.7725;
  parent.orientation.w = 1.0;
  geometry_msgs::msg::Pose child;
  child.orientation.w = 1.0;

  const auto eff = PlanningSceneManager::composePoses(parent, child);
  EXPECT_DOUBLE_EQ(eff.position.x, 0.55);
  EXPECT_DOUBLE_EQ(eff.position.y, -0.15);
  EXPECT_DOUBLE_EQ(eff.position.z, 0.7725);
  EXPECT_DOUBLE_EQ(eff.orientation.w, 1.0);
  EXPECT_TRUE(PlanningSceneManager::samePose(eff, parent));
}

TEST(PlanningSceneManager, PoseCompositionTranslationSplit)
{
  geometry_msgs::msg::Pose parent;
  parent.position.x = 0.30;
  parent.position.y = -0.05;
  parent.position.z = 0.50;
  parent.orientation.w = 1.0;

  geometry_msgs::msg::Pose child;
  child.position.x = 0.25;
  child.position.y = -0.10;
  child.position.z = 0.2725;
  child.orientation.w = 1.0;

  const auto eff = PlanningSceneManager::composePoses(parent, child);
  geometry_msgs::msg::Pose expected;
  expected.position.x = 0.55;
  expected.position.y = -0.15;
  expected.position.z = 0.7725;
  expected.orientation.w = 1.0;

  EXPECT_TRUE(PlanningSceneManager::samePose(eff, expected));
}

TEST(PlanningSceneManager, PoseCompositionRotatedParentWithChild)
{
  // Parent: pos (1.0, 2.0, 3.0), +90 deg around Z
  const double s45 = std::sin(M_PI / 4.0);
  const double c45 = std::cos(M_PI / 4.0);
  geometry_msgs::msg::Pose parent;
  parent.position.x = 1.0;
  parent.position.y = 2.0;
  parent.position.z = 3.0;
  parent.orientation.z = s45;
  parent.orientation.w = c45;

  // Child: pos (0.5, 0.1, 0.2), +45 deg around Z
  const double s22_5 = std::sin(M_PI / 8.0);
  const double c22_5 = std::cos(M_PI / 8.0);
  geometry_msgs::msg::Pose child;
  child.position.x = 0.5;
  child.position.y = 0.1;
  child.position.z = 0.2;
  child.orientation.z = s22_5;
  child.orientation.w = c22_5;

  const auto eff = PlanningSceneManager::composePoses(parent, child);

  // Rotating (0.5, 0.1, 0.2) by +90 deg around Z yields (-0.1, 0.5, 0.2)
  // Expected position: (1.0 - 0.1, 2.0 + 0.5, 3.0 + 0.2) = (0.9, 2.5, 3.2)
  // Expected orientation: +135 deg around Z
  const double s67_5 = std::sin(3.0 * M_PI / 8.0);
  const double c67_5 = std::cos(3.0 * M_PI / 8.0);
  geometry_msgs::msg::Pose expected;
  expected.position.x = 0.9;
  expected.position.y = 2.5;
  expected.position.z = 3.2;
  expected.orientation.z = s67_5;
  expected.orientation.w = c67_5;

  EXPECT_TRUE(PlanningSceneManager::samePose(eff, expected));
}

TEST(PlanningSceneManager, GenuinelyWrongEffectivePoseIsRejected)
{
  geometry_msgs::msg::Pose expected;
  expected.position.x = 0.55;
  expected.position.y = 0.0;
  expected.position.z = 0.375;
  expected.orientation.w = 1.0;

  geometry_msgs::msg::Pose wrong_x = expected;
  wrong_x.position.x += 0.01;
  EXPECT_FALSE(PlanningSceneManager::samePose(expected, wrong_x));

  geometry_msgs::msg::Pose wrong_z = expected;
  wrong_z.position.z -= 0.005;
  EXPECT_FALSE(PlanningSceneManager::samePose(expected, wrong_z));

  geometry_msgs::msg::Pose wrong_rot = expected;
  wrong_rot.orientation.z = std::sin(0.1);
  wrong_rot.orientation.w = std::cos(0.1);
  EXPECT_FALSE(PlanningSceneManager::samePose(expected, wrong_rot));
}

TEST(PlanningSceneManager, TableCanonicalizedReadbackPasses)
{
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "world";
  table.pose.position.x = 0.55;
  table.pose.position.y = 0.0;
  table.pose.position.z = 0.375;
  table.pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {1.20, 0.80, 0.75};
  table.primitives.push_back(box);

  geometry_msgs::msg::Pose identity_primitive_pose;
  identity_primitive_pose.orientation.w = 1.0;
  table.primitive_poses.push_back(identity_primitive_pose);

  const auto eff = PlanningSceneManager::effectivePrimitivePose(table, 0);

  geometry_msgs::msg::Pose expected;
  expected.position.x = 0.55;
  expected.position.y = 0.0;
  expected.position.z = 0.375;
  expected.orientation.w = 1.0;

  EXPECT_TRUE(PlanningSceneManager::samePose(eff, expected));
}

TEST(PlanningSceneManager, TargetCanonicalizedReadbackPasses)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.45;
  target_pose.position.y = -0.15;
  target_pose.position.z = 0.7725;
  target_pose.orientation.w = 1.0;

  moveit_msgs::msg::CollisionObject target;
  target.id = "pick_target";
  target.header.frame_id = "world";
  target.pose = target_pose;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.030, 0.045, 0.045};
  target.primitives.push_back(box);

  geometry_msgs::msg::Pose identity_primitive_pose;
  identity_primitive_pose.orientation.w = 1.0;
  target.primitive_poses.push_back(identity_primitive_pose);

  const auto eff = PlanningSceneManager::effectivePrimitivePose(target, 0);
  EXPECT_TRUE(PlanningSceneManager::samePose(eff, target_pose));
}

TEST(PlanningSceneManager, EffectivePrimitivePoseOutOfRangeThrows)
{
  moveit_msgs::msg::CollisionObject obj;
  EXPECT_THROW(PlanningSceneManager::effectivePrimitivePose(obj, 0), std::out_of_range);
}

TEST(PlanningSceneManager, TargetRejectsNonFinitePose)
{
  geometry_msgs::msg::Pose nan_pos;
  nan_pos.position.x = std::numeric_limits<double>::quiet_NaN();
  nan_pos.orientation.w = 1.0;
  EXPECT_THROW(PlanningSceneManager::makeTarget("world", nan_pos,
    ur5e_pick_place::TargetPoseSource::PRODUCTION_PERCEPTION), std::invalid_argument);

  geometry_msgs::msg::Pose inf_pos;
  inf_pos.position.y = std::numeric_limits<double>::infinity();
  inf_pos.orientation.w = 1.0;
  EXPECT_THROW(PlanningSceneManager::makeTarget("world", inf_pos,
    ur5e_pick_place::TargetPoseSource::PRODUCTION_PERCEPTION), std::invalid_argument);

  geometry_msgs::msg::Pose nan_rot;
  nan_rot.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PlanningSceneManager::makeTarget("world", nan_rot,
    ur5e_pick_place::TargetPoseSource::PRODUCTION_PERCEPTION), std::invalid_argument);
}

TEST(PlanningSceneManager, SamePoseRejectsNonFinite)
{
  geometry_msgs::msg::Pose valid_pose;
  valid_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose nan_pose = valid_pose;
  nan_pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PlanningSceneManager::samePose(valid_pose, nan_pose));
  EXPECT_FALSE(PlanningSceneManager::samePose(nan_pose, nan_pose));
}

TEST(PlanningSceneManager, EraseEntryCleansAllRowAndColumnEntries)
{
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  ASSERT_TRUE(PlanningSceneManager::setPair(acm, "link1", "link2", true));
  ASSERT_TRUE(PlanningSceneManager::setPair(acm, "link2", "link3", true));
  ASSERT_TRUE(PlanningSceneManager::setPair(acm, "link1", "link3", true));

  PlanningSceneManager::eraseEntry(acm, "link2");

  EXPECT_EQ(acm.entry_names.size(), 2u);
  EXPECT_EQ(acm.entry_values.size(), 2u);
  for (const auto & entry : acm.entry_values) {
    EXPECT_EQ(entry.enabled.size(), 2u);
  }

  bool val = false;
  EXPECT_FALSE(PlanningSceneManager::pairValue(acm, "link1", "link2", val));
  EXPECT_FALSE(PlanningSceneManager::pairValue(acm, "link2", "link3", val));
  ASSERT_TRUE(PlanningSceneManager::pairValue(acm, "link1", "link3", val));
  EXPECT_TRUE(val);
}

}  // namespace
