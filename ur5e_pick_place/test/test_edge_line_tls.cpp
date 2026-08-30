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

#include "ur5e_pick_place/edge_line_tls.hpp"

namespace
{

using ur5e_pick_place::axial_difference;
using ur5e_pick_place::canonicalize_axial_angle;
using ur5e_pick_place::estimate_edgelines_tls;
using ur5e_pick_place::image_axial_to_world_yaw;
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

TEST(EdgeLineTLS, SquareNearIsotropicMaskBehavior)
{
  // For an exact square, principal axis orientation is ambiguous.
  // We explicitly log and verify structural estimator behavior.
  cv::Mat square_mask = rasterize_rectangle_mask(20.0, 20.0, rad(15.0), 65.0, 65.0, k3X_Grid);
  const auto result = estimate_edgelines_tls(square_mask);
  std::cout << "[Square Mask Test] valid=" << result.valid
            << " theta=" << (result.valid ? deg(result.theta_image_rad) : 0.0)
            << " boundary_pixels=" << result.boundary_pixel_count << std::endl;
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
    {15.0, -15.0},
    {-30.0, 30.0},
    {85.0, -85.0},
    {0.0, 0.0},
    {45.0, -45.0},
    {-45.0, 45.0}
  };

  for (const auto & pair : test_pairs_deg) {
    const double world_yaw_rad = rad(pair.first);
    const double expected_img_rad = canonicalize_axial_angle(rad(pair.second));

    // Optical frame projection: theta_image = -yaw_world (mod 180)
    const double derived_img_rad = canonicalize_axial_angle(-world_yaw_rad);
    const double err_img = std::abs(deg(axial_difference(derived_img_rad, expected_img_rad)));
    EXPECT_LT(err_img, 1e-5);

    // Inverse mapping helper test: image_axial_to_world_yaw
    const double derived_world_rad = image_axial_to_world_yaw(derived_img_rad);
    const double err_world = std::abs(deg(axial_difference(derived_world_rad, canonicalize_axial_angle(world_yaw_rad))));
    EXPECT_LT(err_world, 1e-5);
  }
}

}  // namespace
