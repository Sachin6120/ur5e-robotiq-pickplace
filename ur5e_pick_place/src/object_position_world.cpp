// Milestone E: camera-frame -> world-frame object position.
//
// WHY THIS IS A SEPARATE NODE, NOT AN EXTENSION OF object_detector.cpp
//   The Milestone C detector and the Milestone D camera-frame estimator are
//   validated and frozen; Milestone D's acceptance rested on a byte-level
//   proof that the detection functions were unchanged.  Keeping the world
//   transform out of that file leaves it with a zero-line diff, which is the
//   cheapest possible way to keep that proof valid.  The detector also has no
//   TF dependency at all today -- folding a Buffer/TransformListener (and its
//   listener thread) into the validated process would itself be a change to a
//   frozen component.  The coupling between the two is a single topic that
//   Milestone D already publishes, so nothing is lost by the split.
//
// WHAT THIS NODE IS NOT
//   It is not a perception algorithm.  It performs no detection, no
//   estimation, no filtering, no correction, and no calibration.  It applies
//   one TF2 transform to a point somebody else estimated.  In particular the
//   known sub-pixel Milestone C/D mask bias is deliberately NOT compensated
//   here: this milestone measures the transform, and an offset tuned against
//   ground truth would destroy exactly the property being measured.
//
// TF IS THE ONLY SOURCE OF EXTRINSICS
//   The camera mount constants (0.450, 0.025, 2.400 and the two fixed
//   rotations) appear NOWHERE in this file.  They reach the transform only
//   through the URDF -> robot_state_publisher -> /tf_static chain, which is
//   the thing Milestone E exists to validate.  Hardcoding them, or deriving
//   world coordinates from them directly, would make the check circular.
//   Gazebo ground truth is likewise absent: it lives only in the evaluation
//   harness under scripts/perception/, in a separate process.

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ObjectPositionWorld : public rclcpp::Node
{
public:
  ObjectPositionWorld()
  : Node("object_position_world"),
    target_frame_(declare_parameter("target_frame", std::string("world"))),
    buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_))
  {
    const std::string in_topic =
      declare_parameter("input_topic", std::string("object_detector/position_camera"));
    const std::string out_topic =
      declare_parameter("output_topic", std::string("object_detector/position_world"));

    publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(out_topic, 10);
    subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      in_topic, 10,
      std::bind(&ObjectPositionWorld::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "world transform: %s -> [TF2] -> %s (target_frame=%s)",
      in_topic.c_str(), out_topic.c_str(), target_frame_.c_str());
  }

private:
  void callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr & msg)
  {
    if (!source_frame_reported_) {
      source_frame_reported_ = true;
      RCLCPP_INFO(
        get_logger(), "first estimate received in frame '%s'", msg->header.frame_id.c_str());
    }

    geometry_msgs::msg::PointStamped out;
    const auto started = std::chrono::steady_clock::now();
    try {
      // Looks the transform up AT THE MESSAGE'S OWN STAMP -- not at "now".
      // A failure here is a real failure: no retry loop, no sleep, no cached
      // previous point, and no identity fallback.  Nothing is published.
      buffer_->transform(*msg, out, target_frame_);
    } catch (const tf2::TransformException & error) {
      ++failures_;
      RCLCPP_WARN(
        get_logger(),
        "TF_TRANSFORM_FAILED stamp=%.9f %s -> %s: %s -- nothing published (failures=%zu)",
        rclcpp::Time(msg->header.stamp).seconds(), msg->header.frame_id.c_str(),
        target_frame_.c_str(), error.what(), failures_);
      return;
    }
    const double transform_ms =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();

    // Belt and braces.  tf2's doTransform already carries the requested stamp
    // through, but the milestone requires the ORIGINAL observation stamp and
    // explicitly forbids substituting now(), so state it rather than rely on
    // a library detail.
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;
    publisher_->publish(out);

    RCLCPP_INFO(
      get_logger(),
      "POSITION_WORLD stamp=%.9f src_frame=%s dst_frame=%s "
      "camera=[%.9f,%.9f,%.9f] world=[%.9f,%.9f,%.9f] transform_ms=%.4f",
      rclcpp::Time(out.header.stamp).seconds(), msg->header.frame_id.c_str(),
      out.header.frame_id.c_str(), msg->point.x, msg->point.y, msg->point.z,
      out.point.x, out.point.y, out.point.z, transform_ms);
  }

  std::string target_frame_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  bool source_frame_reported_{false};
  std::size_t failures_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectPositionWorld>());
  rclcpp::shutdown();
  return 0;
}
