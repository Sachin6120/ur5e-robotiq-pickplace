// Deterministic RGB-D tabletop-object detector for perception Milestone C,
// extended in place with the Milestone D camera-frame position estimator.
//
// Detection uses sensor data only.  In particular, it contains no world pose,
// image location, bounding box, or Gazebo entity-state input.  The dominant
// planar depth is inferred independently in every synchronized observation.
//
// MILESTONE D (camera-frame 3D position) runs strictly AFTER Milestone C has
// finished.  It consumes the FINAL selected connected-component mask, the same
// synchronized depth image, and the same synchronized CameraInfo -- no second
// subscription, no second synchronizer, and no feedback of any kind into
// detection.  Every Milestone C threshold and the component-selection rule are
// frozen; nothing below reads or modifies them.
//
// The estimated quantity is the VISIBLE TOP-SURFACE position of the object in
// camera_optical_frame.  It is deliberately NOT the object's geometric centre:
// a single overhead depth view observes only the top face, so no half-height
// term is added anywhere in this file.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ur5e_pick_place/d10_trimmed_mean.hpp"
#include "ur5e_pick_place/edge_line_tls.hpp"
#include "ur5e_pick_place/pixel_centre_shadow.hpp"

namespace
{
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo>;

// Milestone D result.  Populated only from mask + depth + CameraInfo.
struct Position3D
{
  bool valid{false};
  std::size_t mask_pixels{0};
  std::size_t valid_points{0};
  std::size_t invalid_points{0};
  double d10_x{std::numeric_limits<double>::quiet_NaN()};
  double d10_y{std::numeric_limits<double>::quiet_NaN()};
  double d10_z{std::numeric_limits<double>::quiet_NaN()};
  double mean_x{std::numeric_limits<double>::quiet_NaN()};
  double mean_y{std::numeric_limits<double>::quiet_NaN()};
  double mean_z{std::numeric_limits<double>::quiet_NaN()};
  double std_x{std::numeric_limits<double>::quiet_NaN()};
  double std_y{std::numeric_limits<double>::quiet_NaN()};
  double std_z{std::numeric_limits<double>::quiet_NaN()};
};

struct Detection
{
  bool found{false};
  cv::Rect box{};
  cv::Point2d centroid{};
  int area{0};
  double table_depth{std::numeric_limits<double>::quiet_NaN()};
  double mean_depth{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector()
  : Node("object_detector"),
    rgb_sub_(this, declare_parameter("rgb_topic", "/overhead_camera/image")),
    depth_sub_(this, declare_parameter("depth_topic", "/overhead_camera/depth_image")),
    info_sub_(this, declare_parameter("camera_info_topic", "/overhead_camera/camera_info")),
    sync_(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_)
  {
    brightness_min_ = declare_parameter("brightness_min", 200);
    chroma_max_ = declare_parameter("chroma_max", 30);
    min_height_m_ = declare_parameter("min_height_m", 0.010);
    max_height_m_ = declare_parameter("max_height_m", 0.100);
    plane_min_depth_m_ = declare_parameter("plane_min_depth_m", 1.20);
    plane_max_depth_m_ = declare_parameter("plane_max_depth_m", 2.00);
    plane_bin_m_ = declare_parameter("plane_bin_m", 0.001);
    min_component_area_ = declare_parameter("min_component_area", 160);
    max_component_area_ = declare_parameter("max_component_area", 5000);
    min_component_width_ = declare_parameter("min_component_width", 16);
    min_component_height_ = declare_parameter("min_component_height", 10);
    pixel_centre_shadow_enabled_ = declare_parameter(
      "enable_pixel_centre_shadow", ur5e_pick_place::kPixelCentreShadowDefaultEnabled);

    detected_pub_ = create_publisher<std_msgs::msg::Bool>("object_detector/detected", 10);
    mask_pub_ = create_publisher<Image>("object_detector/mask", 10);
    debug_pub_ = create_publisher<Image>("object_detector/debug_image", 10);
    bbox_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>("object_detector/bounding_box", 10);
    centroid_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>("object_detector/centroid", 10);
    area_pub_ = create_publisher<std_msgs::msg::UInt32>("object_detector/component_area", 10);
    // Milestone D.  Published ONLY when a detection produced at least one
    // valid back-projected point; never a placeholder, never [0,0,0].
    position_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>("object_detector/position_camera", 10);
    if (pixel_centre_shadow_enabled_) {
      shadow_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "object_detector/position_camera_shadow", 10);
    }
    pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("object_detector/pose_camera", 10);

    sync_.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.20));
    sync_.registerCallback(
      std::bind(&ObjectDetector::callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(
      get_logger(),
      "Fixed detector: brightness>=%d chroma<=%d height=[%.3f,%.3f] m area=[%d,%d]",
      brightness_min_, chroma_max_, min_height_m_, max_height_m_, min_component_area_,
      max_component_area_);
    RCLCPP_INFO(
      get_logger(), "pixel-centre shadow estimator: %s (production output unchanged)",
      pixel_centre_shadow_enabled_ ? "ENABLED" : "disabled");
  }

private:
  double estimate_plane_depth(const cv::Mat & depth) const
  {
    const int bins = static_cast<int>(
      std::ceil((plane_max_depth_m_ - plane_min_depth_m_) / plane_bin_m_));
    std::vector<int> histogram(static_cast<std::size_t>(bins), 0);
    for (int row = 0; row < depth.rows; ++row) {
      const float * values = depth.ptr<float>(row);
      for (int col = 0; col < depth.cols; ++col) {
        const float value = values[col];
        if (!std::isfinite(value) || value < plane_min_depth_m_ || value >= plane_max_depth_m_) {
          continue;
        }
        const int bin = static_cast<int>((value - plane_min_depth_m_) / plane_bin_m_);
        ++histogram[static_cast<std::size_t>(bin)];
      }
    }
    const auto peak = std::max_element(histogram.begin(), histogram.end());
    if (peak == histogram.end() || *peak == 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto index = std::distance(histogram.begin(), peak);
    return plane_min_depth_m_ + (static_cast<double>(index) + 0.5) * plane_bin_m_;
  }

  Detection detect(const cv::Mat & rgb, const cv::Mat & depth, cv::Mat & selected_mask) const
  {
    Detection result;
    result.table_depth = estimate_plane_depth(depth);
    selected_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
    if (!std::isfinite(result.table_depth)) {
      return result;
    }

    cv::Mat candidate = cv::Mat::zeros(depth.size(), CV_8UC1);
    for (int row = 0; row < depth.rows; ++row) {
      const float * depths = depth.ptr<float>(row);
      const cv::Vec3b * colours = rgb.ptr<cv::Vec3b>(row);
      std::uint8_t * output = candidate.ptr<std::uint8_t>(row);
      for (int col = 0; col < depth.cols; ++col) {
        const float value = depths[col];
        if (!std::isfinite(value)) {
          continue;
        }
        const double height = result.table_depth - value;
        const cv::Vec3b colour = colours[col];  // RGB order: cv_bridge preserves rgb8.
        const int low = std::min({colour[0], colour[1], colour[2]});
        const int high = std::max({colour[0], colour[1], colour[2]});
        if (low >= brightness_min_ && high - low <= chroma_max_ &&
          height >= min_height_m_ && height <= max_height_m_)
        {
          output[col] = 255;
        }
      }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int count = cv::connectedComponentsWithStats(candidate, labels, stats, centroids, 8);
    int selected = -1;
    for (int label = 1; label < count; ++label) {
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      const int max_dim = std::max(width, height);
      const int min_dim = std::min(width, height);
      if (area >= min_component_area_ && area <= max_component_area_ &&
        max_dim >= min_component_width_ && min_dim >= min_component_height_ &&
        (selected < 0 || area > stats.at<int>(selected, cv::CC_STAT_AREA)))
      {
        selected = label;
      }
    }
    if (selected < 0) {
      for (int label = 1; label < count; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        RCLCPP_DEBUG(get_logger(), "NO_OBJECT component label=%d area=%d (gate=[%d,%d]) w=%d h=%d (gates=[%d,%d])",
          label, area, min_component_area_, max_component_area_, width, height, min_component_width_, min_component_height_);
      }
      return result;
    }

    cv::compare(labels, selected, selected_mask, cv::CMP_EQ);
    result.found = true;
    result.area = stats.at<int>(selected, cv::CC_STAT_AREA);
    result.box = cv::Rect(
      stats.at<int>(selected, cv::CC_STAT_LEFT), stats.at<int>(selected, cv::CC_STAT_TOP),
      stats.at<int>(selected, cv::CC_STAT_WIDTH), stats.at<int>(selected, cv::CC_STAT_HEIGHT));
    result.centroid = cv::Point2d(
      centroids.at<double>(selected, 0), centroids.at<double>(selected, 1));
    result.mean_depth = cv::mean(depth, selected_mask)[0];
    return result;
  }

  // ---------------------------------------------------------------------
  // MILESTONE D: pinhole back-projection of the final component mask.
  //
  // Intrinsics come from the runtime CameraInfo of THIS observation; nothing
  // is hardcoded.  Optical convention (REP-145), which the URDF pins via
  // gz_frame_id -> camera_optical_frame:
  //     +X = image right, +Y = image down, +Z = camera forward (into scene)
  // so for a pixel (u, v) with metric depth Z:
  //     Z_i = depth_i
  //     X_i = (u - cx) * Z_i / fx
  //     Y_i = (v - cy) * Z_i / fy
  //
  // Only genuinely invalid depths are rejected (non-finite or <= 0).  There is
  // deliberately no empirical XYZ gating and no ground-truth-derived offset:
  // any such filter would make the estimate a function of the answer.
  Position3D backproject(
    const cv::Mat & depth, const cv::Mat & mask, const CameraInfo & info,
    const cv::Point2d & centroid) const
  {
    Position3D out;
    const double fx = info.k[0];
    const double fy = info.k[4];
    const double cx = info.k[2];
    const double cy = info.k[5];
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
      RCLCPP_ERROR(get_logger(), "CameraInfo intrinsics are unusable; no position estimate");
      return out;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    for (int row = 0; row < depth.rows; ++row) {
      const float * depths = depth.ptr<float>(row);
      const std::uint8_t * selected = mask.ptr<std::uint8_t>(row);
      for (int col = 0; col < depth.cols; ++col) {
        if (selected[col] == 0) {
          continue;
        }
        ++out.mask_pixels;
        const double z = static_cast<double>(depths[col]);
        if (!std::isfinite(z) || z <= 0.0) {
          ++out.invalid_points;
          continue;
        }
        xs.push_back((static_cast<double>(col) - cx) * z / fx);
        ys.push_back((static_cast<double>(row) - cy) * z / fy);
        zs.push_back(z);
      }
    }

    out.valid_points = zs.size();
    if (out.valid_points == 0) {
      return out;
    }

    const auto moments = [](const std::vector<double> & v, double & mean, double & stddev) {
        double sum = 0.0;
        for (const double value : v) {
          sum += value;
        }
        mean = sum / static_cast<double>(v.size());
        double accum = 0.0;
        for (const double value : v) {
          const double d = value - mean;
          accum += d * d;
        }
        stddev = std::sqrt(accum / static_cast<double>(v.size()));
      };
    moments(xs, out.mean_x, out.std_x);
    moments(ys, out.mean_y, out.std_y);
    moments(zs, out.mean_z, out.std_z);

    // D10: symmetrically trim floor(10% * N) samples from each sorted depth
    // tail, then back-project the selected component's subpixel centroid.
    // The per-pixel XYZ moments above remain diagnostics only.
    const auto d10_z = ur5e_pick_place::d10_trimmed_mean(zs);
    if (!d10_z.has_value()) {
      return out;
    }
    out.d10_z = *d10_z;
    out.d10_x = (centroid.x - cx) * out.d10_z / fx;
    out.d10_y = (centroid.y - cy) * out.d10_z / fy;
    out.valid = true;
    return out;
  }

  void report_camera_info_once(const CameraInfo & info)
  {
    if (camera_info_reported_) {
      return;
    }
    camera_info_reported_ = true;
    const double sx = static_cast<double>(info.width) / 960.0;
    const double sy = static_cast<double>(info.height) / 720.0;
    const bool isotropic = (std::abs(sx - sy) < 1e-4);

    if (!isotropic) {
      RCLCPP_ERROR(
        get_logger(),
        "CAMERA_INFO non-isotropic scaling rejected: width=%u height=%u (sx=%.4f, sy=%.4f)",
        info.width, info.height, sx, sy);
      return;
    }

    if (std::abs(sx - 1.0) > 1e-4) {
      min_component_area_ = static_cast<int>(std::round(min_component_area_ * sx * sy));
      max_component_area_ = static_cast<int>(std::round(max_component_area_ * sx * sy));
      min_component_width_ = static_cast<int>(std::round(min_component_width_ * sx));
      min_component_height_ = static_cast<int>(std::round(min_component_height_ * sx));
    }

    RCLCPP_INFO(
      get_logger(),
      "CAMERA_INFO frame=%s width=%u height=%u scale=%.2fx (sx=%.4f, sy=%.4f) "
      "scaled_gates: area=[%d,%d] width_min=%d height_min=%d "
      "fx=%.9f fy=%.9f cx=%.9f cy=%.9f "
      "K=[%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f] "
      "optical_convention=+X_image_right,+Y_image_down,+Z_camera_forward",
      info.header.frame_id.c_str(), info.width, info.height, sx, sx, sy,
      min_component_area_, max_component_area_, min_component_width_, min_component_height_,
      info.k[0], info.k[4], info.k[2], info.k[5],
      info.k[0], info.k[1], info.k[2], info.k[3], info.k[4], info.k[5],
      info.k[6], info.k[7], info.k[8]);
  }

  void callback(
    const Image::ConstSharedPtr & rgb_msg, const Image::ConstSharedPtr & depth_msg,
    const CameraInfo::ConstSharedPtr & info_msg)
  {
    const auto started = std::chrono::steady_clock::now();
    if (rgb_msg->width != depth_msg->width || rgb_msg->height != depth_msg->height ||
      info_msg->width != rgb_msg->width || info_msg->height != rgb_msg->height)
    {
      RCLCPP_ERROR(get_logger(), "RGB, depth, and CameraInfo dimensions do not agree");
      return;
    }
    report_camera_info_once(*info_msg);

    cv::Mat rgb;
    cv::Mat depth;
    try {
      rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::RGB8)->image;
      depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", error.what());
      return;
    }

    // Stage A: Milestone C segmentation + component selection (frozen).
    cv::Mat mask;
    const auto seg_started = std::chrono::steady_clock::now();
    const Detection detection = detect(rgb, depth, mask);
    const auto seg_finished = std::chrono::steady_clock::now();

    // Stage B: Milestone D masked 3D reconstruction.  Runs only on a
    // successful detection, and only on the FINAL selected component mask.
    Position3D position;
    auto recon_started = seg_finished;
    auto recon_finished = seg_finished;
    if (detection.found) {
      recon_started = std::chrono::steady_clock::now();
      position = backproject(depth, mask, *info_msg, detection.centroid);
      recon_finished = std::chrono::steady_clock::now();
    }
    const double seg_ms =
      std::chrono::duration<double, std::milli>(seg_finished - seg_started).count();
    const double recon_ms =
      std::chrono::duration<double, std::milli>(recon_finished - recon_started).count();

    std_msgs::msg::Bool detected;
    detected.data = detection.found;
    detected_pub_->publish(detected);

    auto mask_msg = cv_bridge::CvImage(rgb_msg->header, "mono8", mask).toImageMsg();
    mask_pub_->publish(*mask_msg);

    cv::Mat debug;
    cv::cvtColor(rgb, debug, cv::COLOR_RGB2BGR);
    if (detection.found) {
      cv::rectangle(debug, detection.box, cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(
        debug, detection.centroid, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 11, 2);

      std_msgs::msg::Int32MultiArray bbox;
      bbox.data = {detection.box.x, detection.box.y, detection.box.width, detection.box.height};
      bbox_pub_->publish(bbox);
      geometry_msgs::msg::PointStamped centroid;
      centroid.header = rgb_msg->header;
      centroid.point.x = detection.centroid.x;
      centroid.point.y = detection.centroid.y;
      centroid.point.z = 0.0;
      centroid_pub_->publish(centroid);
      std_msgs::msg::UInt32 area;
      area.data = static_cast<std::uint32_t>(detection.area);
      area_pub_->publish(area);

      // Milestone D output.  Nothing is published when the observation
      // yielded no valid back-projected point, so a stale estimate can never
      // stand in for a fresh one.
      if (position.valid) {
        geometry_msgs::msg::PointStamped camera_position;
        camera_position.header.stamp = rgb_msg->header.stamp;
        camera_position.header.frame_id = info_msg->header.frame_id;
        camera_position.point.x = position.d10_x;
        camera_position.point.y = position.d10_y;
        camera_position.point.z = position.d10_z;
        position_pub_->publish(camera_position);

        // Default-off diagnostic only: use the unchanged D10 depth with the
        // same runtime intrinsics and a +0.5,+0.5 pixel centroid.  The shadow
        // point is never substituted for position_camera or pose_camera.
        if (pixel_centre_shadow_enabled_) {
          const ur5e_pick_place::PixelCentroid raw_centroid{
            detection.centroid.x, detection.centroid.y};
          const auto corrected_centroid = ur5e_pick_place::pixel_centre_corrected(raw_centroid);
          const ur5e_pick_place::PinholeIntrinsics intrinsics{
            info_msg->k[0], info_msg->k[4], info_msg->k[2], info_msg->k[5]};
          const auto corrected_camera = ur5e_pick_place::backproject_centroid(
            corrected_centroid, position.d10_z, intrinsics);

          geometry_msgs::msg::PointStamped shadow_camera_position;
          shadow_camera_position.header = camera_position.header;
          shadow_camera_position.point.x = corrected_camera.x;
          shadow_camera_position.point.y = corrected_camera.y;
          shadow_camera_position.point.z = corrected_camera.z;
          shadow_position_pub_->publish(shadow_camera_position);

          RCLCPP_INFO(
            get_logger(),
            "PIXEL_CENTRE_SHADOW stamp=%.9f width=%u height=%u "
            "K=[%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f] "
            "centroid_raw=[%.9f,%.9f] centroid_corrected=[%.9f,%.9f] d10_depth=%.9f "
            "production_camera=[%.9f,%.9f,%.9f] corrected_camera=[%.9f,%.9f,%.9f]",
            rclcpp::Time(camera_position.header.stamp).seconds(), info_msg->width, info_msg->height,
            info_msg->k[0], info_msg->k[1], info_msg->k[2], info_msg->k[3], info_msg->k[4],
            info_msg->k[5], info_msg->k[6], info_msg->k[7], info_msg->k[8],
            raw_centroid.u, raw_centroid.v, corrected_centroid.u, corrected_centroid.v,
            position.d10_z, camera_position.point.x, camera_position.point.y,
            camera_position.point.z, shadow_camera_position.point.x,
            shadow_camera_position.point.y, shadow_camera_position.point.z);
        }

        // Stage-2B: Edge-Line TLS orientation estimation and camera-frame pose publication
        const auto tls_result = ur5e_pick_place::estimate_edgelines_tls(mask);
        if (tls_result.valid && std::isfinite(tls_result.theta_image_rad)) {
          geometry_msgs::msg::PoseStamped camera_pose;
          camera_pose.header = camera_position.header;
          camera_pose.pose.position = camera_position.point;

          // CONTRACT: pose_camera.orientation is the orientation of the
          // perceived OBJECT frame expressed in camera_optical_frame -- never
          // a world-frame yaw smuggled into a camera-frame message.
          //
          //   R_opt_obj = Rz(theta_image - pi/2) * Rx(pi)
          //
          // The Rz term carries the estimated yaw with the long-axis ->
          // object-+X offset already removed; the Rx(pi) term is mandatory,
          // not cosmetic -- the object's +Z points world-UP while optical +Z
          // points DOWN into the scene, so without it the published frame is
          // improper for a downward camera. tf2's setRPY(r, p, y) builds
          // Rz(y)*Ry(p)*Rx(r), which is exactly this product.
          //
          // Composed with the URDF's R_world_opt this yields Rz(psi) in world
          // for every angle -- proven in test_edge_line_tls.cpp's
          // CameraFramePoseMapping suite, not assumed here.
          tf2::Quaternion q;
          q.setRPY(
            ur5e_pick_place::kMaskOrientationPi, 0.0,
            tls_result.theta_image_rad - ur5e_pick_place::kLongAxisToObjectXOffsetRad);
          camera_pose.pose.orientation = tf2::toMsg(q);
          pose_pub_->publish(camera_pose);

          const double object_yaw_deg =
            ur5e_pick_place::image_axial_to_object_yaw(tls_result.theta_image_rad) * 180.0 /
            ur5e_pick_place::kMaskOrientationPi;
          const double long_axis_world_deg =
            ur5e_pick_place::image_axial_to_world_yaw(tls_result.theta_image_rad) * 180.0 /
            ur5e_pick_place::kMaskOrientationPi;
          const double theta_img_deg = tls_result.theta_image_rad * 180.0 / ur5e_pick_place::kMaskOrientationPi;

          RCLCPP_INFO(
            get_logger(),
            "EDGE_LINE_TLS stamp=%.9f frame=%s valid=true theta_img=%.2f deg "
            "object_yaw_axial=%.2f deg long_axis_world_axial=%.2f deg "
            "eccentricity=%.4f boundary_pixels=%zu supports=[%zu,%zu,%zu,%zu] quality=%.1f",
            rclcpp::Time(rgb_msg->header.stamp).seconds(), info_msg->header.frame_id.c_str(),
            theta_img_deg, object_yaw_deg, long_axis_world_deg,
            tls_result.eccentricity, tls_result.boundary_pixel_count,
            tls_result.family_support_counts[0], tls_result.family_support_counts[1],
            tls_result.family_support_counts[2], tls_result.family_support_counts[3],
            tls_result.quality_score);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "EDGE_LINE_TLS stamp=%.9f frame=%s refused: valid=%s eccentricity=%.4f (yaw unobservable / mask degenerate); "
            "pose_camera omitted, position_camera published",
            rclcpp::Time(rgb_msg->header.stamp).seconds(), info_msg->header.frame_id.c_str(),
            tls_result.valid ? "true" : "false", tls_result.eccentricity);
        }
      }
    }
    auto debug_msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", debug).toImageMsg();
    debug_pub_->publish(*debug_msg);

    const double latency_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    if (detection.found) {
      RCLCPP_INFO(
        get_logger(),
        "DETECTED stamp=%.9f bbox=[%d,%d,%d,%d] centroid=[%.3f,%.3f] area=%d "
        "table_depth=%.6f mean_depth=%.6f latency_ms=%.3f",
        rclcpp::Time(rgb_msg->header.stamp).seconds(), detection.box.x, detection.box.y,
        detection.box.width, detection.box.height, detection.centroid.x, detection.centroid.y,
        detection.area, detection.table_depth, detection.mean_depth, latency_ms);
    } else {
      RCLCPP_INFO(
        get_logger(), "NO_OBJECT stamp=%.9f table_depth=%.6f latency_ms=%.3f",
        rclcpp::Time(rgb_msg->header.stamp).seconds(), detection.table_depth, latency_ms);
    }

    // Milestone D diagnostics.  latency_ms above spans the whole callback (its
    // Milestone C definition, unchanged) and therefore now also contains
    // recon_ms; seg_ms and recon_ms break that total down.
    if (detection.found && position.valid) {
      const double valid_pct = 100.0 * static_cast<double>(position.valid_points) /
        static_cast<double>(position.mask_pixels);
      RCLCPP_INFO(
        get_logger(),
        "POSITION3D stamp=%.9f frame=%s mask_px=%zu valid=%zu invalid=%zu valid_pct=%.4f "
        "d10=[%.9f,%.9f,%.9f] mean=[%.9f,%.9f,%.9f] std=[%.9f,%.9f,%.9f] "
        "seg_ms=%.3f recon_ms=%.3f total_ms=%.3f",
        rclcpp::Time(rgb_msg->header.stamp).seconds(), info_msg->header.frame_id.c_str(),
        position.mask_pixels, position.valid_points, position.invalid_points, valid_pct,
        position.d10_x, position.d10_y, position.d10_z,
        position.mean_x, position.mean_y, position.mean_z,
        position.std_x, position.std_y, position.std_z, seg_ms, recon_ms, latency_ms);
    } else if (detection.found) {
      RCLCPP_WARN(
        get_logger(),
        "POSITION3D_UNAVAILABLE stamp=%.9f mask_px=%zu valid=0 invalid=%zu "
        "seg_ms=%.3f recon_ms=%.3f total_ms=%.3f -- nothing published",
        rclcpp::Time(rgb_msg->header.stamp).seconds(), position.mask_pixels,
        position.invalid_points, seg_ms, recon_ms, latency_ms);
    }
  }

  int brightness_min_;
  int chroma_max_;
  double min_height_m_;
  double max_height_m_;
  double plane_min_depth_m_;
  double plane_max_depth_m_;
  double plane_bin_m_;
  int min_component_area_;
  int max_component_area_;
  int min_component_width_;
  int min_component_height_;

  message_filters::Subscriber<Image> rgb_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<CameraInfo> info_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr bbox_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr area_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr shadow_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  bool camera_info_reported_{false};
  bool pixel_centre_shadow_enabled_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetector>());
  rclcpp::shutdown();
  return 0;
}
