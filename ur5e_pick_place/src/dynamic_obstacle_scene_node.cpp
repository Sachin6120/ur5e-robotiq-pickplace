#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <string>
#include <chrono>
#include <memory>

namespace ur5e_pick_place
{

class DynamicObstacleSceneNode : public rclcpp::Node
{
public:
  explicit DynamicObstacleSceneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dynamic_obstacle_scene_node", options)
  {
    // Declare parameters with production defaults
    obstacle_id_ = this->declare_parameter<std::string>("obstacle_id", "dynamic_obstacle_0");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "world");
    box_size_x_ = this->declare_parameter<double>("box_size_x", 0.05);
    box_size_y_ = this->declare_parameter<double>("box_size_y", 0.05);
    box_size_z_ = this->declare_parameter<double>("box_size_z", 0.10);
    target_update_rate_hz_ = this->declare_parameter<double>("target_update_rate_hz", 10.0);
    stale_threshold_s_ = this->declare_parameter<double>("stale_threshold_s", 0.250);
    collision_object_topic_ = this->declare_parameter<std::string>("collision_object_topic", "/collision_object");
    input_pose_topic_ = this->declare_parameter<std::string>("input_pose_topic", "/model/dynamic_obstacle/pose");

    min_update_interval_s_ = (target_update_rate_hz_ > 0.0) ? (1.0 / target_update_rate_hz_ - 0.005) : 0.095;

    // Reliable publisher to /collision_object for MoveIt planning scene integration
    collision_object_pub_ = this->create_publisher<moveit_msgs::msg::CollisionObject>(
      collision_object_topic_, rclcpp::QoS(10).reliable());

    // Subscriber to bridged Gazebo obstacle pose stream
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_pose_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&DynamicObstacleSceneNode::onPoseReceived, this, std::placeholders::_1));

    // Periodic timer for staleness monitoring and telemetry reporting
    monitor_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DynamicObstacleSceneNode::onMonitorTick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "[Stage-3B] DynamicObstacleSceneNode initialized. ID='%s', Frame='%s', Size=[%.3f, %.3f, %.3f], TargetRate=%.1f Hz, Topic='%s'",
      obstacle_id_.c_str(), frame_id_.c_str(), box_size_x_, box_size_y_, box_size_z_,
      target_update_rate_hz_, collision_object_topic_.c_str());
  }

private:
  void onPoseReceived(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    received_count_++;
    rclcpp::Time current_stamp = msg->header.stamp;
    if (current_stamp.nanoseconds() == 0)
    {
      current_stamp = this->now();
    }
    last_received_stamp_ = current_stamp;
    has_received_any_pose_ = true;

    // First accepted message: perform ADD lifecycle operation with box primitive once subscribers exist
    if (add_count_ == 0)
    {
      if (collision_object_pub_->get_subscription_count() > 0)
      {
        publishCollisionObjectAdd(msg);
      }
      return;
    }

    // Subsequent messages: rate-limit to nominal 10 Hz and perform MOVE operation
    double elapsed_since_last_accepted = (current_stamp - last_accepted_stamp_).seconds();
    if (elapsed_since_last_accepted >= min_update_interval_s_)
    {
      publishCollisionObjectMove(msg);
    }
  }

  void publishCollisionObjectAdd(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    moveit_msgs::msg::CollisionObject co;
    rclcpp::Time stamp(msg->header.stamp);
    co.header.stamp = (stamp.nanoseconds() != 0) ? msg->header.stamp : builtin_interfaces::msg::Time(this->now());
    co.header.frame_id = frame_id_;
    co.id = obstacle_id_;
    co.pose = msg->pose;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = box_size_x_;
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = box_size_y_;
    box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = box_size_z_;

    geometry_msgs::msg::Pose identity_pose;
    identity_pose.orientation.w = 1.0;

    co.primitives.push_back(box);
    co.primitive_poses.push_back(identity_pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    collision_object_pub_->publish(co);

    add_count_++;
    accepted_count_++;
    if (first_accepted_stamp_.nanoseconds() == 0)
    {
      first_accepted_stamp_ = co.header.stamp;
    }
    last_accepted_stamp_ = co.header.stamp;

    RCLCPP_INFO(
      this->get_logger(),
      "[Stage-3B] CollisionObject ADD: id='%s', frame='%s', pos=[%.4f, %.4f, %.4f], dim=[%.3f, %.3f, %.3f]",
      co.id.c_str(), co.header.frame_id.c_str(),
      co.pose.position.x, co.pose.position.y, co.pose.position.z,
      box_size_x_, box_size_y_, box_size_z_);
  }

  void publishCollisionObjectMove(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    moveit_msgs::msg::CollisionObject co;
    rclcpp::Time stamp(msg->header.stamp);
    co.header.stamp = (stamp.nanoseconds() != 0) ? msg->header.stamp : builtin_interfaces::msg::Time(this->now());
    co.header.frame_id = frame_id_;
    co.id = obstacle_id_;
    co.pose = msg->pose;
    // MOVE semantics: geometry arrays are intentionally left empty; MoveIt updates object.pose directly
    co.primitives.clear();
    co.primitive_poses.clear();
    co.operation = moveit_msgs::msg::CollisionObject::MOVE;

    collision_object_pub_->publish(co);

    move_count_++;
    accepted_count_++;
    last_accepted_stamp_ = co.header.stamp;
  }

  void onMonitorTick()
  {
    if (!has_received_any_pose_)
    {
      return;
    }

    rclcpp::Time now_time = this->now();
    double staleness = (now_time - last_received_stamp_).seconds();

    if (staleness > stale_threshold_s_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[Stage-3B] Dynamic obstacle pose stream is STALE (gap: %.3fs > %.3fs threshold). "
        "Retaining obstacle in PlanningScene without extrapolation.",
        staleness, stale_threshold_s_);
    }

    // Telemetry logging every ~5.0 seconds
    static rclcpp::Time last_telemetry_time = now_time;
    if ((now_time - last_telemetry_time).seconds() >= 5.0)
    {
      double active_duration = (last_accepted_stamp_ - first_accepted_stamp_).seconds();
      double rate = (active_duration > 0.0) ? (static_cast<double>(accepted_count_) / active_duration) : 0.0;
      RCLCPP_INFO(
        this->get_logger(),
        "[Telemetry] Rx: %lu, Accepted: %lu (ADD: %lu, MOVE: %lu), Rate: %.2f Hz, Staleness: %.3fs",
        received_count_, accepted_count_, add_count_, move_count_, rate, staleness);
      last_telemetry_time = now_time;
    }
  }

  // Parameters
  std::string obstacle_id_;
  std::string frame_id_;
  double box_size_x_{0.05};
  double box_size_y_{0.05};
  double box_size_z_{0.10};
  double target_update_rate_hz_{10.0};
  double stale_threshold_s_{0.250};
  double min_update_interval_s_{0.095};
  std::string collision_object_topic_;
  std::string input_pose_topic_;

  // ROS infrastructure
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  // State & Telemetry
  uint64_t received_count_{0};
  uint64_t accepted_count_{0};
  uint64_t add_count_{0};
  uint64_t move_count_{0};
  bool has_received_any_pose_{false};
  rclcpp::Time last_received_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_accepted_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_accepted_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace ur5e_pick_place

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur5e_pick_place::DynamicObstacleSceneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
