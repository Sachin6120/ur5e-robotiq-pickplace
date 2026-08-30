// test_mask_orientation.cpp
//
// Two DELIBERATELY SEPARATE concerns, per the Stage-2B evaluation brief:
//
//   A. Functional correctness of mask_orientation.hpp's math (moments,
//      canonicalisation, axial difference, degeneracy rejection). These
//      tests use large/idealised synthetic shapes so quantization is
//      negligible -- they prove the ESTIMATOR is correct.
//
//   B. Empirical low-resolution accuracy at the actual pixel footprint this
//      project's overhead camera produces for the validated 30x45mm object
//      (~15.5 x 23.3 px -- derived below from the same camera/object
//      constants used elsewhere in this repo, not invented). This proves
//      (or disproves) whether a <0.5 degree perception criterion is
//      achievable at THIS resolution. It is possible for A to pass while B
//      fails; that is not a contradiction, and B's assertions are
//      deliberately loose sanity bounds, not the aspirational accuracy
//      target -- see the comment above MaskOrientationLowResSweep.
//
// No estimator parameter or test geometry here was tuned to force a
// particular sweep result.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "ur5e_pick_place/mask_orientation.hpp"

namespace
{

using ur5e_pick_place::axial_difference;
using ur5e_pick_place::canonicalize_axial_angle;
using ur5e_pick_place::estimate_mask_orientation;
using ur5e_pick_place::kMaskOrientationPi;
using Pixel = std::pair<double, double>;

double deg(double rad) {return rad * 180.0 / kMaskOrientationPi;}
double rad(double degrees) {return degrees * kMaskOrientationPi / 180.0;}

// Rasterizes an axis-aligned-then-rotated rectangle (half-widths hx, hy
// along its own LOCAL x/y before rotation) onto an integer pixel grid of
// size grid x grid, centred at sub-pixel position (cx, cy), rotated by
// theta_rad about that centre. Pixel (i, j)'s CENTRE (i+0.5, j+0.5) is
// tested for membership -- single-sample-per-pixel, no anti-aliasing, the
// same simple in/out rule the project's real detector effectively applies
// per-pixel. This is a TEST-ONLY helper; it is not shipped in
// mask_orientation.hpp.
std::vector<Pixel> rasterize_rectangle(
  double hx, double hy, double theta_rad, double cx, double cy, int grid)
{
  std::vector<Pixel> pixels;
  const double c = std::cos(theta_rad);
  const double s = std::sin(theta_rad);
  for (int j = 0; j < grid; ++j) {
    const double py = j + 0.5;
    for (int i = 0; i < grid; ++i) {
      const double px = i + 0.5;
      const double dx = px - cx;
      const double dy = py - cy;
      const double lx = dx * c + dy * s;
      const double ly = -dx * s + dy * c;
      if (std::abs(lx) <= hx && std::abs(ly) <= hy) {
        pixels.emplace_back(px, py);
      }
    }
  }
  return pixels;
}

// ===========================================================================
// A. FUNCTIONAL CORRECTNESS -- large/idealised shapes, negligible quantization
// ===========================================================================

TEST(MaskOrientationCorrectness, KnownSyntheticRectangleOrientations)
{
  // Large rectangle (400 px grid, half-widths 150x80 px) so per-pixel
  // quantization contributes well under 0.05 deg -- isolates the moment/
  // atan2 math from resolution effects, which are covered separately in
  // section B below.
  const double hx = 150.0;
  const double hy = 80.0;
  const int grid = 400;
  const double cx = 200.37;
  const double cy = 200.61;

  for (const double truth_deg : {0.0, 10.0, 22.5, 30.0, 60.0, 75.0, 89.0}) {
    const auto pixels = rasterize_rectangle(hx, hy, rad(truth_deg), cx, cy, grid);
    const auto result = estimate_mask_orientation(pixels);
    ASSERT_TRUE(result.valid) << "truth_deg=" << truth_deg;
    const double err_deg =
      std::abs(deg(axial_difference(result.theta_rad, canonicalize_axial_angle(rad(truth_deg)))));
    EXPECT_LT(err_deg, 0.05) << "truth_deg=" << truth_deg << " est_deg=" << deg(result.theta_rad);
  }
}

TEST(MaskOrientationCorrectness, Mod180IdentityOfCanonicalization)
{
  // canonicalize_axial_angle must be invariant under adding/subtracting
  // exactly pi -- this IS the mod-180 axial identity, tested directly on
  // the canonicalization function rather than through rasterization (which
  // would risk flaky pixel-boundary flips at exactly +/-pi).
  for (const double theta_deg : {0.0, 1.0, 44.9, 45.0, 45.1, 89.9, -89.9, 123.4, -170.0}) {
    const double base = canonicalize_axial_angle(rad(theta_deg));
    const double plus_pi = canonicalize_axial_angle(rad(theta_deg) + kMaskOrientationPi);
    const double minus_pi = canonicalize_axial_angle(rad(theta_deg) - kMaskOrientationPi);
    EXPECT_NEAR(base, plus_pi, 1e-12) << "theta_deg=" << theta_deg;
    EXPECT_NEAR(base, minus_pi, 1e-12) << "theta_deg=" << theta_deg;
  }
}

TEST(MaskOrientationCorrectness, Mod180IdentityViaRasterization)
{
  // A physically rotated-by-180-degrees rectangle is the identical shape;
  // the estimator run on both must agree to within quantization noise.
  const double hx = 150.0;
  const double hy = 80.0;
  const int grid = 400;
  const double cx = 200.13;
  const double cy = 200.87;

  const auto a = rasterize_rectangle(hx, hy, rad(37.0), cx, cy, grid);
  const auto b = rasterize_rectangle(hx, hy, rad(37.0 + 180.0), cx, cy, grid);
  const auto ra = estimate_mask_orientation(a);
  const auto rb = estimate_mask_orientation(b);
  ASSERT_TRUE(ra.valid);
  ASSERT_TRUE(rb.valid);
  EXPECT_LT(std::abs(deg(axial_difference(ra.theta_rad, rb.theta_rad))), 0.05);
}

TEST(MaskOrientationCorrectness, AxialDifferenceCrossesBranchBoundaryCorrectly)
{
  // +85 deg and -85 deg describe axes 10 degrees apart along the SHORT way
  // around the axial (mod-180) circle, not 170 degrees the long way.
  const double difference_deg = deg(axial_difference(rad(85.0), rad(-85.0)));
  EXPECT_NEAR(std::abs(difference_deg), 10.0, 1e-9);

  // Same check the other direction.
  const double difference_deg_rev = deg(axial_difference(rad(-85.0), rad(85.0)));
  EXPECT_NEAR(std::abs(difference_deg_rev), 10.0, 1e-9);

  // A generic pair straddling the +90/-90 wrap point.
  EXPECT_NEAR(std::abs(deg(axial_difference(rad(89.0), rad(-89.0)))), 2.0, 1e-9);
}

TEST(MaskOrientationCorrectness, NearPlusMinus45DegreesNoDiscontinuity)
{
  // mu20-mu02 crosses zero near +/-45 deg, the atan2 branch most sensitive
  // to noise (see the low-resolution sweep's worst-case region below). At
  // high resolution this must still be smooth and accurate -- no sign flip,
  // no large jump between adjacent angles.
  const double hx = 150.0;
  const double hy = 80.0;
  const int grid = 400;
  const double cx = 200.29;
  const double cy = 200.71;

  for (const double truth_deg : {43.0, 44.0, 45.0, 46.0, 47.0, -47.0, -46.0, -45.0, -44.0, -43.0}) {
    const auto pixels = rasterize_rectangle(hx, hy, rad(truth_deg), cx, cy, grid);
    const auto result = estimate_mask_orientation(pixels);
    ASSERT_TRUE(result.valid) << "truth_deg=" << truth_deg;
    const double err_deg =
      std::abs(deg(axial_difference(result.theta_rad, canonicalize_axial_angle(rad(truth_deg)))));
    EXPECT_LT(err_deg, 0.1) << "truth_deg=" << truth_deg;
  }
}

TEST(MaskOrientationCorrectness, RefusesAxisAlignedSquare)
{
  const auto pixels = rasterize_rectangle(60.0, 60.0, 0.0, 150.37, 150.61, 300);
  const auto result = estimate_mask_orientation(pixels);
  EXPECT_FALSE(result.valid);
  EXPECT_LT(result.eccentricity, ur5e_pick_place::kDefaultEccentricityMin);
}

TEST(MaskOrientationCorrectness, RefusesSquareAtAnyRotation)
{
  // A 4-fold-symmetric shape's second-moment tensor is isotropic at EVERY
  // rotation, not just axis-aligned -- confirm the gate holds at 45 deg too.
  const auto pixels = rasterize_rectangle(60.0, 60.0, rad(45.0), 150.13, 150.87, 300);
  const auto result = estimate_mask_orientation(pixels);
  EXPECT_FALSE(result.valid);
  EXPECT_LT(result.eccentricity, ur5e_pick_place::kDefaultEccentricityMin);
}

TEST(MaskOrientationCorrectness, AcceptsClearlyElongatedShape)
{
  // Sanity converse of the two refusal tests: the same call path accepts a
  // shape well above the eccentricity floor.
  const auto pixels = rasterize_rectangle(150.0, 30.0, rad(12.0), 200.5, 200.5, 400);
  const auto result = estimate_mask_orientation(pixels);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.eccentricity, ur5e_pick_place::kDefaultEccentricityMin);
}

TEST(MaskOrientationCorrectness, RefusesEmptyMask)
{
  const auto result = estimate_mask_orientation({});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.pixel_count, 0u);
}

TEST(MaskOrientationCorrectness, RefusesSinglePixel)
{
  const auto result = estimate_mask_orientation({{42.0, 17.0}});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.pixel_count, 1u);
}

TEST(MaskOrientationCorrectness, AcceptsLineLikeDegenerateCase)
{
  // A set of colinear points is the extreme opposite of a square: perfectly
  // anisotropic (eccentricity -> 1), with a well-defined axis along the
  // line. This must be ACCEPTED, not refused -- it is elongated, not
  // isotropic.
  const double truth_deg = 31.7;
  std::vector<Pixel> pixels;
  const double c = std::cos(rad(truth_deg));
  const double s = std::sin(rad(truth_deg));
  for (int k = -25; k <= 25; ++k) {
    const double t = static_cast<double>(k);
    pixels.emplace_back(100.0 + t * c, 100.0 + t * s);
  }
  const auto result = estimate_mask_orientation(pixels);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.eccentricity, 0.99);
  const double err_deg =
    std::abs(deg(axial_difference(result.theta_rad, canonicalize_axial_angle(rad(truth_deg)))));
  EXPECT_LT(err_deg, 1e-6);
}

TEST(MaskOrientationCorrectness, Deterministic)
{
  const auto pixels = rasterize_rectangle(150.0, 80.0, rad(17.3), 200.4, 200.6, 400);
  const auto first = estimate_mask_orientation(pixels);
  const auto second = estimate_mask_orientation(pixels);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_DOUBLE_EQ(first.theta_rad, second.theta_rad);
  EXPECT_DOUBLE_EQ(first.eccentricity, second.eccentricity);
}

TEST(MaskOrientationCorrectness, IndependentOfPixelScanOrder)
{
  const auto ordered = rasterize_rectangle(150.0, 80.0, rad(17.3), 200.4, 200.6, 400);
  std::vector<Pixel> reversed(ordered.rbegin(), ordered.rend());
  std::vector<Pixel> interleaved;
  interleaved.reserve(ordered.size());
  for (std::size_t i = 0; i < ordered.size(); i += 2) {interleaved.push_back(ordered[i]);}
  for (std::size_t i = 1; i < ordered.size(); i += 2) {interleaved.push_back(ordered[i]);}

  const auto result_ordered = estimate_mask_orientation(ordered);
  const auto result_reversed = estimate_mask_orientation(reversed);
  const auto result_interleaved = estimate_mask_orientation(interleaved);

  ASSERT_TRUE(result_ordered.valid);
  ASSERT_TRUE(result_reversed.valid);
  ASSERT_TRUE(result_interleaved.valid);
  EXPECT_DOUBLE_EQ(result_ordered.theta_rad, result_reversed.theta_rad);
  EXPECT_DOUBLE_EQ(result_ordered.theta_rad, result_interleaved.theta_rad);
  EXPECT_DOUBLE_EQ(result_ordered.eccentricity, result_reversed.eccentricity);
}

TEST(MaskOrientationCorrectness, TranslationInvariant)
{
  const auto base = rasterize_rectangle(150.0, 80.0, rad(17.3), 200.4, 200.6, 400);
  std::vector<Pixel> shifted;
  shifted.reserve(base.size());
  const double shift_x = 57.25;
  const double shift_y = -31.75;
  for (const auto & p : base) {
    shifted.emplace_back(p.first + shift_x, p.second + shift_y);
  }

  const auto result_base = estimate_mask_orientation(base);
  const auto result_shifted = estimate_mask_orientation(shifted);
  ASSERT_TRUE(result_base.valid);
  ASSERT_TRUE(result_shifted.valid);
  EXPECT_NEAR(result_base.theta_rad, result_shifted.theta_rad, 1e-9);
  EXPECT_NEAR(result_base.eccentricity, result_shifted.eccentricity, 1e-9);
}

TEST(MaskOrientationCorrectness, DegeneracyThresholdIsExplicitAndTunable)
{
  // The default floor (0.10) corresponds to an aspect-ratio boundary of
  // about 1.106:1 -- derived from eccentricity(r) = (r^2-1)/(r^2+1) for an
  // axis-aligned r:1 rectangle, solved for eccentricity == 0.10.
  const double r = std::sqrt(1.1 / 0.9);
  const double hx = 100.0 * r;
  const double hy = 100.0;
  const auto pixels = rasterize_rectangle(hx, hy, 0.0, 300.5, 300.5, 600);
  const auto result = estimate_mask_orientation(pixels);
  EXPECT_NEAR(result.eccentricity, 0.10, 0.01);

  // The threshold is a plain function parameter, not a hidden constant:
  // raising it must refuse a shape the default accepts.
  const auto lenient = estimate_mask_orientation(pixels, 0.05);
  const auto strict = estimate_mask_orientation(pixels, 0.50);
  EXPECT_TRUE(lenient.valid);
  EXPECT_FALSE(strict.valid);
}

// ===========================================================================
// B. EMPIRICAL LOW-RESOLUTION ACCURACY -- the actual project pixel footprint
//
// Derivation of the ~15.5 x 23.3 px footprint (matches HANDOFF.md's
// Stage-2B risk analysis, re-derived here from the same source constants,
// not copied as a bare number):
//   camera horizontal_fov = 1.047 rad, width = 960 px
//     (ur5e_robotiq.urdf.xacro's <sensor type="rgbd_camera">)
//   fx = (960/2) / tan(1.047/2) = 831.574 px
//   camera z = 2.400 m, object top z = pick_pose.z + size.z/2
//     = 0.7725 + 0.0225 = 0.795 m  (config/scene.yaml)
//   depth = 2.400 - 0.795 = 1.605 m
//   mm_per_px = depth / fx * 1000 = 1.9301 mm/px
//   object footprint (0.030 x 0.045 m) / mm_per_px = 15.543 x 23.315 px
//
// This section answers a DIFFERENT question than section A: not "is the
// math right" but "does this pixel count support the <0.5 deg perception
// criterion floated in HANDOFF.md's Stage-2B risk analysis". The estimator
// and rasterization here are UNCHANGED from section A -- nothing is loosened
// or tuned to help this test pass. Its assertions are therefore deliberately
// LOOSE sanity bounds (catch a gross regression/bug), not the 0.5 deg
// target -- the actual numeric verdict on that target is in this session's
// prose report, sourced from these tests' printed output, not encoded as a
// pass/fail assertion that would misrepresent a FAIL as a PASS.
// ===========================================================================

namespace
{
constexpr double kFootprintHalfWidthPx = 23.315 / 2.0;  // long axis, local x
constexpr double kFootprintHalfHeightPx = 15.543 / 2.0;  // short axis, local y
constexpr int kFootprintGrid = 40;

struct SweepStats
{
  int n_valid{0};
  int n_refused{0};
  double max_err_deg{0.0};
  double sum_err_deg{0.0};
  double sum_sq_err_deg{0.0};
  int worst_truth_deg{0};
  double min_eccentricity{1.0};
  double max_eccentricity{0.0};
};

SweepStats run_sweep(double cx, double cy)
{
  SweepStats stats;
  for (int truth_deg = 0; truth_deg < 180; ++truth_deg) {
    const double theta_true = rad(static_cast<double>(truth_deg));
    const auto pixels = rasterize_rectangle(
      kFootprintHalfWidthPx, kFootprintHalfHeightPx, theta_true, cx, cy, kFootprintGrid);
    const auto result = estimate_mask_orientation(pixels);
    if (!result.valid) {
      ++stats.n_refused;
      continue;
    }
    ++stats.n_valid;
    stats.min_eccentricity = std::min(stats.min_eccentricity, result.eccentricity);
    stats.max_eccentricity = std::max(stats.max_eccentricity, result.eccentricity);
    const double err_deg =
      std::abs(deg(axial_difference(result.theta_rad, canonicalize_axial_angle(theta_true))));
    stats.sum_err_deg += err_deg;
    stats.sum_sq_err_deg += err_deg * err_deg;
    if (err_deg > stats.max_err_deg) {
      stats.max_err_deg = err_deg;
      stats.worst_truth_deg = truth_deg;
    }
  }
  return stats;
}
}  // namespace

TEST(MaskOrientationLowResSweep, FullDegreeSweepAtActualProjectFootprint)
{
  // Several arbitrary, non-special sub-pixel phase offsets -- a real
  // object's centre lands at an arbitrary sub-pixel position relative to
  // the camera's pixel grid, so a single phase would not be representative.
  const std::vector<Pixel> phases = {
    {19.37, 19.61}, {19.50, 19.50}, {19.05, 19.95},
    {19.90, 19.10}, {19.25, 19.75}, {20.00, 20.00}, {19.13, 19.87},
  };

  double overall_max_err_deg = 0.0;
  int overall_worst_truth_deg = -1;
  double overall_worst_phase_x = 0.0;
  double overall_worst_phase_y = 0.0;

  for (const auto & phase : phases) {
    const SweepStats stats = run_sweep(phase.first, phase.second);
    // No angle should be spuriously refused: the object's true eccentricity
    // (~0.38, printed below) is well above the default 0.10 floor at every
    // orientation.
    EXPECT_EQ(stats.n_refused, 0)
      << "phase=(" << phase.first << "," << phase.second << ")";
    EXPECT_EQ(stats.n_valid, 180)
      << "phase=(" << phase.first << "," << phase.second << ")";

    const double mean_err_deg = stats.sum_err_deg / std::max(stats.n_valid, 1);
    const double rms_err_deg = std::sqrt(stats.sum_sq_err_deg / std::max(stats.n_valid, 1));

    std::cout << "[low-res sweep] phase=(" << phase.first << "," << phase.second << ") "
              << "max_err_deg=" << stats.max_err_deg
              << " mean_err_deg=" << mean_err_deg
              << " rms_err_deg=" << rms_err_deg
              << " worst_truth_deg=" << stats.worst_truth_deg
              << " eccentricity=[" << stats.min_eccentricity << ", " << stats.max_eccentricity
              << "]" << std::endl;

    // Loose sanity ceiling only -- catches an actual implementation bug
    // (e.g. a broken axis convention, which during development of this
    // estimator produced a constant ~90 deg offset and was caught exactly
    // this way). NOT an assertion that the 0.5 deg criterion is met.
    EXPECT_LT(stats.max_err_deg, 10.0)
      << "phase=(" << phase.first << "," << phase.second << ")";

    if (stats.max_err_deg > overall_max_err_deg) {
      overall_max_err_deg = stats.max_err_deg;
      overall_worst_truth_deg = stats.worst_truth_deg;
      overall_worst_phase_x = phase.first;
      overall_worst_phase_y = phase.second;
    }
  }

  std::cout << "[low-res sweep] WORST ACROSS ALL PHASES: max_err_deg=" << overall_max_err_deg
            << " at truth_deg=" << overall_worst_truth_deg
            << " phase=(" << overall_worst_phase_x << "," << overall_worst_phase_y << ")"
            << std::endl;
  std::cout << "[low-res sweep] 0.5 deg criterion: "
            << (overall_max_err_deg < 0.5 ? "SUPPORTED" : "NOT SUPPORTED")
            << " (worst-case max error " << overall_max_err_deg << " deg)" << std::endl;
}

TEST(MaskOrientationLowResSweep, ExplicitInspectionAngles)
{
  const double cx = 19.37;
  const double cy = 19.61;
  for (const int truth_deg : {0, 5, -5, 15, -15, 30, -30, 45, -45, 60, -60, 85, -85}) {
    const double theta_true = rad(static_cast<double>(truth_deg));
    const auto pixels = rasterize_rectangle(
      kFootprintHalfWidthPx, kFootprintHalfHeightPx, theta_true, cx, cy, kFootprintGrid);
    const auto result = estimate_mask_orientation(pixels);
    ASSERT_TRUE(result.valid) << "truth_deg=" << truth_deg;
    const double err_deg =
      std::abs(deg(axial_difference(result.theta_rad, canonicalize_axial_angle(theta_true))));
    std::cout << "[low-res sweep] truth_deg=" << truth_deg
              << " est_deg=" << deg(result.theta_rad)
              << " err_deg=" << err_deg
              << " eccentricity=" << result.eccentricity
              << " n_px=" << result.pixel_count << std::endl;
    // Loose sanity bound, same rationale as the full sweep above.
    EXPECT_LT(err_deg, 5.0) << "truth_deg=" << truth_deg;
  }
}

TEST(MaskOrientationLowResSweep, NoBranchOrSignFlipDiscontinuity)
{
  // Distinguishes an actual atan2-branch/sign-flip defect (the kind that,
  // during this estimator's own development, showed up as a constant ~90
  // deg offset from a mislabeled rectangle axis -- see the design-note
  // discussion in this session's report) from ordinary per-pixel
  // quantization roughness. FullDegreeSweepAtActualProjectFootprint above
  // already measures that roughness directly and found a worst-case single-
  // angle error near 3.9-4.3 deg (region: truth_deg ~= 134-136, where
  // mu20-mu02 crosses zero and atan2's sensitivity to per-pixel noise is
  // highest). A step-to-step jump on that same order is therefore EXPECTED
  // roughness, not a discontinuity bug, so the threshold here is set well
  // above it (15 deg -- under half the distance to an actual 90-degree
  // branch artifact) rather than at an arbitrary small value that would
  // just re-detect the already-characterized, already-reported noise.
  const double cx = 19.37;
  const double cy = 19.61;
  constexpr double kBranchFlipThresholdDeg = 15.0;
  double previous_theta = std::numeric_limits<double>::quiet_NaN();
  int discontinuities = 0;
  for (int truth_deg = 0; truth_deg < 180; ++truth_deg) {
    const auto pixels = rasterize_rectangle(
      kFootprintHalfWidthPx, kFootprintHalfHeightPx, rad(static_cast<double>(truth_deg)),
      cx, cy, kFootprintGrid);
    const auto result = estimate_mask_orientation(pixels);
    ASSERT_TRUE(result.valid) << "truth_deg=" << truth_deg;
    if (!std::isnan(previous_theta)) {
      const double step_deg = std::abs(deg(axial_difference(result.theta_rad, previous_theta)));
      if (step_deg > kBranchFlipThresholdDeg) {
        ++discontinuities;
        std::cout << "[low-res sweep] BRANCH-FLIP candidate at truth_deg=" << truth_deg
                  << " step_deg=" << step_deg << std::endl;
      }
    }
    previous_theta = result.theta_rad;
  }
  std::cout << "[low-res sweep] branch/sign-flip candidates (>"
            << kBranchFlipThresholdDeg << " deg step): " << discontinuities << std::endl;
  EXPECT_EQ(discontinuities, 0);
}

}  // namespace
