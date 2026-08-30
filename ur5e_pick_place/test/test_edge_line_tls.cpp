// test_edge_line_tls.cpp
//
// Focused unit tests and parity benchmark for ur5e_pick_place/edge_line_tls.hpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "ur5e_pick_place/edge_line_tls.hpp"

namespace
{

using ur5e_pick_place::axial_difference;
using ur5e_pick_place::canonicalize_axial_angle;
using ur5e_pick_place::estimate_edgelines_tls;
using ur5e_pick_place::image_axial_to_object_yaw;
using ur5e_pick_place::image_axial_to_world_yaw;
using ur5e_pick_place::kLongAxisToObjectXOffsetRad;
using ur5e_pick_place::kMaskOrientationPi;

double deg(double rad) { return rad * 180.0 / kMaskOrientationPi; }
double rad(double degrees) { return degrees * kMaskOrientationPi / 180.0; }

/**
 * @brief Test helper to rasterize a rectangle of half-dimensions (hx, hy)
 * rotated by theta_rad about subpixel center (cx, cy) into a CV_8UC1 binary mask.
 */
cv::Mat rasterize_rectangle_mask(
  double hx, double hy, double theta_rad, double cx, double cy, int grid_size = 130)
{
  cv::Mat mask = cv::Mat::zeros(grid_size, grid_size, CV_8UC1);
  const double c = std::cos(theta_rad);
  const double s = std::sin(theta_rad);
  for (int r = 0; r < grid_size; ++r) {
    const double py = r + 0.5;
    std::uint8_t * ptr = mask.ptr<std::uint8_t>(r);
    for (int col = 0; col < grid_size; ++col) {
      const double px = col + 0.5;
      const double dx = px - cx;
      const double dy = py - cy;
      const double lx = dx * c + dy * s;
      const double ly = -dx * s + dy * c;
      if (std::abs(lx) <= hx && std::abs(ly) <= hy) {
        ptr[col] = 255;
      }
    }
  }
  return mask;
}

// 3X Representative Footprint constants (30x45 mm object at f=2494.722 px, Z=1.605 m)
constexpr double k3X_Hx = 34.973;  // long half-span (69.95 px full width)
constexpr double k3X_Hy = 23.315;  // short half-span (46.63 px full height)
constexpr int k3X_Grid = 130;

// ===========================================================================
// SECTION A: FUNCTIONAL CORRECTNESS
// ===========================================================================

TEST(EdgeLineTLS, RepresentativeSweepAngles)
{
  const double cx = 65.37;
  const double cy = 65.61;
  const std::vector<double> test_angles_deg = {
    0.0, 5.0, -5.0, 15.0, -15.0, 30.0, -30.0, 40.0, -40.0, 42.0, -42.0,
    44.0, -44.0, 45.0, -45.0, 46.0, -46.0, 48.0, -48.0, 50.0, -50.0,
    60.0, -60.0, 85.0, -85.0
  };

  for (const double truth_deg : test_angles_deg) {
    const double theta_true = rad(truth_deg);
    cv::Mat mask = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, theta_true, cx, cy, k3X_Grid);
    const auto result = estimate_edgelines_tls(mask);
    ASSERT_TRUE(result.valid) << "Failed on truth_deg=" << truth_deg;
    const double err_deg = std::abs(deg(axial_difference(result.theta_image_rad, canonicalize_axial_angle(theta_true))));
    EXPECT_LT(err_deg, 0.50) << "truth_deg=" << truth_deg << " est=" << deg(result.theta_image_rad);
  }
}

TEST(EdgeLineTLS, ExplicitMod180Equivalence)
{
  const double cx = 65.25;
  const double cy = 65.75;
  cv::Mat mask1 = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, rad(85.0), cx, cy, k3X_Grid);
  cv::Mat mask2 = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, rad(-95.0), cx, cy, k3X_Grid);

  const auto res1 = estimate_edgelines_tls(mask1);
  const auto res2 = estimate_edgelines_tls(mask2);
  ASSERT_TRUE(res1.valid && res2.valid);
  const double diff = std::abs(deg(axial_difference(res1.theta_image_rad, res2.theta_image_rad)));
  EXPECT_LT(diff, 1e-4);
}

TEST(EdgeLineTLS, BranchBoundaryNear90Deg)
{
  const double cx = 65.10;
  const double cy = 65.90;
  for (const double truth_deg : {89.0, 89.9, -89.9, -89.0}) {
    cv::Mat mask = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, rad(truth_deg), cx, cy, k3X_Grid);
    const auto result = estimate_edgelines_tls(mask);
    ASSERT_TRUE(result.valid);
    const double err_deg = std::abs(deg(axial_difference(result.theta_image_rad, canonicalize_axial_angle(rad(truth_deg)))));
    EXPECT_LT(err_deg, 0.50) << "truth_deg=" << truth_deg;
  }
}

TEST(EdgeLineTLS, TranslationInvariance)
{
  for (const auto & offset : std::vector<std::pair<double, double>>{{50.0, 50.0}, {65.0, 65.0}, {80.0, 70.0}}) {
    cv::Mat mask = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, rad(25.0), offset.first, offset.second, k3X_Grid);
    const auto result = estimate_edgelines_tls(mask);
    ASSERT_TRUE(result.valid);
    const double err_deg = std::abs(deg(axial_difference(result.theta_image_rad, canonicalize_axial_angle(rad(25.0)))));
    EXPECT_LT(err_deg, 0.50);
  }
}

TEST(EdgeLineTLS, SubpixelPhases)
{
  const std::vector<std::pair<double, double>> phases = {
    {65.10, 65.90}, {65.25, 65.75}, {65.50, 65.50}, {65.75, 65.25}, {65.90, 65.10}
  };
  for (const auto & phase : phases) {
    cv::Mat mask = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, rad(42.0), phase.first, phase.second, k3X_Grid);
    const auto result = estimate_edgelines_tls(mask);
    ASSERT_TRUE(result.valid);
    const double err_deg = std::abs(deg(axial_difference(result.theta_image_rad, canonicalize_axial_angle(rad(42.0)))));
    EXPECT_LT(err_deg, 0.50);
  }
}

TEST(EdgeLineTLS, Determinism)
{
  cv::Mat mask = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, rad(33.0), 65.3, 65.7, k3X_Grid);
  const auto res1 = estimate_edgelines_tls(mask);
  const auto res2 = estimate_edgelines_tls(mask);
  ASSERT_TRUE(res1.valid && res2.valid);
  EXPECT_EQ(res1.theta_image_rad, res2.theta_image_rad);
  EXPECT_EQ(res1.boundary_pixel_count, res2.boundary_pixel_count);
  EXPECT_EQ(res1.quality_score, res2.quality_score);
}

TEST(EdgeLineTLS, RefusalEmptyOrWrongType)
{
  cv::Mat empty_mask;
  EXPECT_FALSE(estimate_edgelines_tls(empty_mask).valid);

  cv::Mat wrong_type(50, 50, CV_32FC1, cv::Scalar(1.0));
  EXPECT_FALSE(estimate_edgelines_tls(wrong_type).valid);

  cv::Mat zero_mask = cv::Mat::zeros(50, 50, CV_8UC1);
  EXPECT_FALSE(estimate_edgelines_tls(zero_mask).valid);
}

TEST(EdgeLineTLS, RefusalTinyOrDegenerateMask)
{
  cv::Mat tiny_mask = cv::Mat::zeros(50, 50, CV_8UC1);
  tiny_mask.at<std::uint8_t>(20, 20) = 255;
  tiny_mask.at<std::uint8_t>(20, 21) = 255;
  EXPECT_FALSE(estimate_edgelines_tls(tiny_mask).valid);
}

TEST(EdgeLineTLS, SquareNearIsotropicMaskRefusal)
{
  // Exact axis-aligned square: orientation unobservable -> invalid
  cv::Mat square_mask = rasterize_rectangle_mask(20.0, 20.0, rad(0.0), 65.0, 65.0, k3X_Grid);
  const auto res_square = estimate_edgelines_tls(square_mask);
  EXPECT_FALSE(res_square.valid);
  EXPECT_LT(res_square.eccentricity, 0.10);

  // Rotated exact square (15, 30, 45 deg): orientation unobservable -> invalid
  for (const double rot_deg : {15.0, 30.0, 45.0}) {
    cv::Mat rot_square = rasterize_rectangle_mask(20.0, 20.0, rad(rot_deg), 65.0, 65.0, k3X_Grid);
    const auto res_rot = estimate_edgelines_tls(rot_square);
    EXPECT_FALSE(res_rot.valid);
    EXPECT_LT(res_rot.eccentricity, 0.10);
  }

  // Near-square below threshold (e.g. 20 x 20.5 px, aspect ratio ~1.025) -> invalid
  cv::Mat near_square = rasterize_rectangle_mask(20.5 / 2.0, 20.0 / 2.0, rad(10.0), 65.0, 65.0, k3X_Grid);
  const auto res_near = estimate_edgelines_tls(near_square);
  EXPECT_FALSE(res_near.valid);
  EXPECT_LT(res_near.eccentricity, 0.10);

  // Target object (30x45 mm) half-spans 34.973 x 23.315 px (aspect ratio 1.5) -> eccentricity ~0.38
  cv::Mat target_mask = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, rad(0.0), 65.0, 65.0, k3X_Grid);
  const auto res_target = estimate_edgelines_tls(target_mask);
  EXPECT_TRUE(res_target.valid);
  EXPECT_GT(res_target.eccentricity, 0.35);
}

// ===========================================================================
// SECTION B: PARITY BENCHMARK WITH VALIDATED SCRATCH ESTIMATOR
// ===========================================================================

TEST(EdgeLineTLSParityBenchmark, Exhaustive0To179Deg7Phases)
{
  const std::vector<std::pair<double, double>> phase_offsets = {
    {-0.63, -0.39}, {-0.50, -0.50}, {-0.95, -0.05},
    {-0.10, -0.90}, {-0.75, -0.25}, { 0.00,  0.00}, {-0.87, -0.13}
  };
  const double center_base = k3X_Grid / 2.0;

  std::vector<double> all_errors;
  std::vector<double> band_40_50_errors;
  std::vector<double> band_130_140_errors;

  double worst_err_deg = 0.0;
  int worst_truth_deg = -1;
  std::pair<double, double> worst_phase = {0.0, 0.0};
  int branch_flip_failures = 0;

  for (const auto & p_off : phase_offsets) {
    const double cx = center_base + p_off.first;
    const double cy = center_base + p_off.second;

    for (int truth_deg = 0; truth_deg < 180; ++truth_deg) {
      const double theta_true = rad(static_cast<double>(truth_deg));
      cv::Mat mask = rasterize_rectangle_mask(k3X_Hx, k3X_Hy, theta_true, cx, cy, k3X_Grid);
      const auto result = estimate_edgelines_tls(mask);
      ASSERT_TRUE(result.valid) << "Refused valid 3X mask at truth_deg=" << truth_deg;

      const double truth_canon = canonicalize_axial_angle(theta_true);
      const double err_deg = std::abs(deg(axial_difference(result.theta_image_rad, truth_canon)));

      all_errors.push_back(err_deg);
      if (truth_deg >= 40 && truth_deg <= 50) {
        band_40_50_errors.push_back(err_deg);
      }
      if (truth_deg >= 130 && truth_deg <= 140) {
        band_130_140_errors.push_back(err_deg);
      }

      if (err_deg > worst_err_deg) {
        worst_err_deg = err_deg;
        worst_truth_deg = truth_deg;
        worst_phase = p_off;
      }

      if (err_deg > 15.0) {
        ++branch_flip_failures;
      }
    }
  }

  const double mean_err = std::accumulate(all_errors.begin(), all_errors.end(), 0.0) / all_errors.size();
  double sq_sum = 0.0;
  for (double e : all_errors) sq_sum += e * e;
  const double rms_err = std::sqrt(sq_sum / all_errors.size());

  const double max_40_50 = *std::max_element(band_40_50_errors.begin(), band_40_50_errors.end());
  const double max_130_140 = *std::max_element(band_130_140_errors.begin(), band_130_140_errors.end());

  std::cout << "\n========================================================" << std::endl;
  std::cout << "3X EXHAUSTIVE EDGE-LINE TLS PARITY BENCHMARK (N=" << all_errors.size() << ")" << std::endl;
  std::cout << "========================================================" << std::endl;
  std::cout << "  Max Axial Error   : " << worst_err_deg << " deg" << std::endl;
  std::cout << "  Mean Axial Error  : " << mean_err << " deg" << std::endl;
  std::cout << "  RMS Axial Error   : " << rms_err << " deg" << std::endl;
  std::cout << "  Max Error (40-50°): " << max_40_50 << " deg" << std::endl;
  std::cout << "  Max Error (130-140°): " << max_130_140 << " deg" << std::endl;
  std::cout << "  Worst Truth Angle : " << worst_truth_deg << " deg" << std::endl;
  std::cout << "  Worst Subpixel Phase: (" << worst_phase.first << ", " << worst_phase.second << ")" << std::endl;
  std::cout << "  Branch-Flip Failures: " << branch_flip_failures << std::endl;

  // Strict Parity Acceptance Criteria
  EXPECT_LT(worst_err_deg, 0.50);
  EXPECT_LT(mean_err, 0.10);
  EXPECT_EQ(branch_flip_failures, 0);
}

// ===========================================================================
// SECTION C: AXIAL FRAME MAPPING UNIT CHECK
// ===========================================================================

TEST(EdgeLineTLSFrameMapping, OpticalToWorldYawMapping)
{
  const std::vector<std::pair<double, double>> test_pairs_deg = {
    {15.0, 75.0},
    {-30.0, -60.0},
    {85.0, 5.0},
    {0.0, -90.0},
    {45.0, 45.0},
    {-45.0, -45.0}
  };

  for (const auto & pair : test_pairs_deg) {
    const double world_yaw_rad = rad(pair.first);
    const double expected_img_rad = canonicalize_axial_angle(rad(pair.second));

    // Optical frame projection: theta_image = canonicalize(pi/2 - yaw_world)
    const double derived_img_rad = canonicalize_axial_angle(kMaskOrientationPi / 2.0 - world_yaw_rad);
    const double err_img = std::abs(deg(axial_difference(derived_img_rad, expected_img_rad)));
    EXPECT_LT(err_img, 1e-5) << "world_yaw=" << pair.first;

    // Inverse mapping helper test: image_axial_to_world_yaw
    const double derived_world_rad = image_axial_to_world_yaw(derived_img_rad);
    const double err_world = std::abs(deg(axial_difference(derived_world_rad, canonicalize_axial_angle(world_yaw_rad))));
    EXPECT_LT(err_world, 1e-5) << "world_yaw=" << pair.first;
  }
}

// ===========================================================================
// SECTION D: CAMERA-FRAME POSE MAPPING
//
// Proves the full chain the detector relies on:
//
//   configured world yaw psi
//     -> physical long-axis world direction  (object-local +Y, since
//        scene.yaml object.size = [0.030, 0.045, 0.045])
//     -> projection through the URDF optical frame into theta_image
//     -> the REAL estimate_edgelines_tls() run on a rasterized mask
//     -> image_axial_to_object_yaw() back to psi
//     -> the EXACT quaternion object_detector.cpp publishes
//     -> composed with R_world_opt, must equal Rz(psi) in world.
//
// theta_image is NOT taken from the mapping formula here -- it is derived
// independently by explicit projection, so a sign error in the mapping
// cannot hide behind a matching sign error in the test setup.
// ===========================================================================

// Optical axes expressed in world, from ur5e_robotiq.urdf.xacro's
// camera_optical_joint. Columns are X_opt, Y_opt, Z_opt.
//   +X_opt = world -Y, +Y_opt = world -X, +Z_opt = world -Z
// Confirmed against a live D10 sample on 2026-08-31 (predicted optical XYZ
// (0.175, 0.000, 1.605) vs measured (0.174672, -0.000322, 1.605000)).
tf2::Matrix3x3 world_from_optical()
{
  return tf2::Matrix3x3(
    0.0, -1.0, 0.0,
    -1.0, 0.0, 0.0,
    0.0, 0.0, -1.0);
}

// Physical model, derived from first principles rather than from the mapping
// under test: where does the object's LONG axis land in the image?
double project_long_axis_to_image_angle(double psi_rad)
{
  // object-local +Y (the 45 mm side) in world at yaw psi
  const double wx = -std::sin(psi_rad);
  const double wy = std::cos(psi_rad);
  // world -> image:  u = -w_y (image right is world -Y), v = -w_x (image down is world -X)
  return canonicalize_axial_angle(std::atan2(-wx, -wy));
}

const std::vector<double> kPoseMappingYawsDeg = {
  0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 85.0};

TEST(CameraFramePoseMapping, ProjectionAndInverseAreConsistent)
{
  for (const double psi_deg : kPoseMappingYawsDeg) {
    const double psi = rad(psi_deg);
    const double theta_img = project_long_axis_to_image_angle(psi);

    // The independently-projected image angle must equal -psi (mod 180).
    EXPECT_LT(std::abs(deg(axial_difference(theta_img, canonicalize_axial_angle(-psi)))), 1e-9)
      << "psi=" << psi_deg;

    // And the production inverse must recover psi.
    EXPECT_LT(std::abs(deg(axial_difference(image_axial_to_object_yaw(theta_img), psi))), 1e-9)
      << "psi=" << psi_deg;

    // The long-axis helper must sit exactly one fixed offset away from it.
    EXPECT_LT(
      std::abs(deg(axial_difference(
          image_axial_to_world_yaw(theta_img) - kLongAxisToObjectXOffsetRad,
          image_axial_to_object_yaw(theta_img)))),
      1e-9) << "psi=" << psi_deg;
  }
}

TEST(CameraFramePoseMapping, EstimatorRecoversConfiguredYaw)
{
  const double cx = 65.37;
  const double cy = 65.61;
  for (const double psi_deg : kPoseMappingYawsDeg) {
    const double psi = rad(psi_deg);
    const double theta_img_truth = project_long_axis_to_image_angle(psi);

    // Rasterize the real 3X footprint with its LONG axis at theta_img_truth.
    const cv::Mat mask =
      rasterize_rectangle_mask(k3X_Hx, k3X_Hy, theta_img_truth, cx, cy, k3X_Grid);
    const auto result = estimate_edgelines_tls(mask);
    ASSERT_TRUE(result.valid) << "psi=" << psi_deg;

    // Estimator recovers the image angle within its qualified budget.
    EXPECT_LT(std::abs(deg(axial_difference(result.theta_image_rad, theta_img_truth))), 0.50)
      << "psi=" << psi_deg;

    // And the mapping turns that back into the configured yaw, mod 180.
    const double recovered = image_axial_to_object_yaw(result.theta_image_rad);
    EXPECT_LT(std::abs(deg(axial_difference(recovered, psi))), 0.50) << "psi=" << psi_deg;
  }
}

TEST(CameraFramePoseMapping, PublishedQuaternionComposesToWorldYaw)
{
  for (const double psi_deg : kPoseMappingYawsDeg) {
    const double psi = rad(psi_deg);
    const double theta_img = project_long_axis_to_image_angle(psi);

    // EXACTLY the construction in object_detector.cpp.
    tf2::Quaternion q;
    q.setRPY(kMaskOrientationPi, 0.0, theta_img - kLongAxisToObjectXOffsetRad);

    const tf2::Matrix3x3 R_opt_obj(q);
    const tf2::Matrix3x3 R_world_obj = world_from_optical() * R_opt_obj;

    // The composed world rotation must be a pure yaw about world +Z.
    EXPECT_NEAR(R_world_obj[2][2], 1.0, 1e-9) << "psi=" << psi_deg;
    EXPECT_NEAR(R_world_obj[0][2], 0.0, 1e-9) << "psi=" << psi_deg;
    EXPECT_NEAR(R_world_obj[1][2], 0.0, 1e-9) << "psi=" << psi_deg;
    EXPECT_NEAR(R_world_obj[2][0], 0.0, 1e-9) << "psi=" << psi_deg;
    EXPECT_NEAR(R_world_obj[2][1], 0.0, 1e-9) << "psi=" << psi_deg;

    // ... and that yaw must be psi, mod 180 (the object is 2-fold symmetric).
    const double recovered = canonicalize_axial_angle(
      std::atan2(R_world_obj[1][0], R_world_obj[0][0]));
    EXPECT_LT(std::abs(deg(axial_difference(recovered, psi))), 1e-9) << "psi=" << psi_deg;
  }
}

TEST(CameraFramePoseMapping, ZeroYawMatchesLiveObservation)
{
  // The 2026-08-31 controlled run spawned the object at configured yaw 0 and
  // the detector reported theta_img = 0.00 deg over 22/22 frames. Pin that.
  EXPECT_LT(std::abs(deg(project_long_axis_to_image_angle(0.0))), 1e-9);
  EXPECT_LT(std::abs(deg(image_axial_to_object_yaw(0.0))), 1e-9);

  // The historical -90 deg diagnostic was the LONG-AXIS world angle, not the
  // object yaw. Both readings are correct for what they each measure; only
  // the old label conflated them.
  EXPECT_LT(std::abs(deg(image_axial_to_world_yaw(0.0)) + 90.0), 1e-9);
}

TEST(CameraFramePoseMapping, AxialSymmetryIsRespected)
{
  // psi and psi+180 are the same physical pose for this object, so the whole
  // chain must be invariant under that shift.
  for (const double psi_deg : kPoseMappingYawsDeg) {
    const double a = project_long_axis_to_image_angle(rad(psi_deg));
    const double b = project_long_axis_to_image_angle(rad(psi_deg + 180.0));
    EXPECT_LT(std::abs(deg(axial_difference(a, b))), 1e-9) << "psi=" << psi_deg;
  }
}

}  // namespace
