#ifndef UR5E_PICK_PLACE__MASK_ORIENTATION_HPP_
#define UR5E_PICK_PLACE__MASK_ORIENTATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace ur5e_pick_place
{

// Local, portable pi -- <cmath>'s M_PI is a POSIX extension, not guaranteed
// by the C++17 standard itself.
constexpr double kMaskOrientationPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// AXIAL ANGLE SEMANTICS
//
// A rectangle (or any 2-fold-symmetric shape) has no observable FRONT vs
// BACK along its principal axis: rotating it by 180 degrees produces an
// identical mask. Its orientation is therefore AXIAL, observable only mod
// 180 degrees, and the canonical representative used everywhere in this
// file is the half-open interval [-pi/2, +pi/2) radians.
//
// Ordinary angle wrapping (wrap to [-pi, +pi)) is WRONG for this quantity:
// two axial angles that are 180 degrees apart (e.g. +85 and -95) describe
// the SAME axis and must compare as a 0-degree difference, not a 180-degree
// one. Use axial_difference() below for any yaw comparison; never wrap_pi
// an axial angle.
// ---------------------------------------------------------------------------

// Canonicalises an axial angle (radians) into [-pi/2, +pi/2).
inline double canonicalize_axial_angle(double theta_rad)
{
  double wrapped = std::fmod(theta_rad, kMaskOrientationPi);
  if (wrapped < 0.0) {
    wrapped += kMaskOrientationPi;  // now in [0, pi)
  }
  if (wrapped >= kMaskOrientationPi / 2.0) {
    wrapped -= kMaskOrientationPi;  // now in [-pi/2, +pi/2)
  }
  return wrapped;
}

// Shortest AXIAL difference a-b (radians), canonicalised into [-pi/2, +pi/2).
// This is the only correct way to compare two axial angles -- see the
// AXIAL ANGLE SEMANTICS note above. Do not substitute ordinary angle
// wrapping here.
inline double axial_difference(double a_rad, double b_rad)
{
  return canonicalize_axial_angle(a_rad - b_rad);
}

// ---------------------------------------------------------------------------
// EXPERIMENTAL BASELINE: SECOND-MOMENT MASK ORIENTATION ESTIMATOR
//
// NOTE (Stage-2B): This second-order moment estimator was evaluated as an
// initial candidate and experimentally REJECTED for production yaw estimation
// (its rasterization error at camera resolution hits a 1.283 deg lower bound,
// failing the <0.5 deg yaw budget). Edge-Line TLS is the selected Stage-2B
// estimator algorithm. This moment-estimator is retained here solely as an
// explicitly documented experimental baseline and reference implementation.
// ---------------------------------------------------------------------------
//
// Deterministic second-order-moment principal-axis estimator over a set of
// foreground mask pixel coordinates (image convention: x = column, y =
// row; any consistent origin -- only relative pixel positions matter,
// since central moments are translation invariant by construction).
//
// theta = 0.5 * atan2(2*mu11, mu20 - mu02)
//
// mu20/mu02/mu11 are the pixel-count-normalised second central moments
// (variance_x, variance_y, covariance_xy). theta is the axis of the mask's
// principal (major) spread direction, canonicalised to [-pi/2, +pi/2).
//
// DEGENERACY: a shape with no anisotropy (a circle, or a shape with 3+ fold
// rotational symmetry, most notably a square) has an ISOTROPIC second-moment
// tensor -- mu20 == mu02 and mu11 == 0 for every rotation -- and therefore
// no observable orientation. The normalised eccentricity
//
//   eccentricity = sqrt((mu20-mu02)^2 + 4*mu11^2) / (mu20+mu02)
//
// is 0 for an isotropic shape and -> 1 for a highly elongated one. This
// estimator REFUSES (returns valid=false) whenever eccentricity falls below
// `eccentricity_min`, an explicit, caller-visible, testable parameter --
// not a hidden constant and not a ROS parameter (no runtime wiring exists
// yet; see HANDOFF.md's Stage-2B section).
//
// This function is pure and deterministic: no ROS, no OpenCV, no Gazebo/
// ground-truth dependency, no temporal filtering or history across calls,
// and no tuning against any manipulation outcome. The input is sorted
// internally before reduction, so the result does not depend on pixel scan
// order.
// ---------------------------------------------------------------------------
struct MaskOrientationResult
{
  bool valid{false};
  double theta_rad{std::numeric_limits<double>::quiet_NaN()};  // canonical [-pi/2, pi/2)
  double eccentricity{std::numeric_limits<double>::quiet_NaN()};
  double mu20{std::numeric_limits<double>::quiet_NaN()};
  double mu02{std::numeric_limits<double>::quiet_NaN()};
  double mu11{std::numeric_limits<double>::quiet_NaN()};
  std::size_t pixel_count{0};
};

// Default eccentricity floor: rejects any shape whose long/short second-
// -moment ratio corresponds to an aspect ratio below ~1.106 (see
// test_mask_orientation.cpp's DegeneracyThreshold test for the derivation).
// This project's validated object (30x45 mm, eccentricity ~0.38 at its top
// face -- see the low-resolution sweep test) sits comfortably above it.
constexpr double kDefaultEccentricityMin = 0.10;

inline MaskOrientationResult estimate_mask_orientation(
  std::vector<std::pair<double, double>> pixels_xy,
  double eccentricity_min = kDefaultEccentricityMin)
{
  MaskOrientationResult out;
  out.pixel_count = pixels_xy.size();
  if (pixels_xy.empty()) {
    return out;
  }

  // Sort first so the reduction below is independent of the order pixels
  // were discovered in (e.g. row-major mask scan vs. any other order).
  std::sort(
    pixels_xy.begin(), pixels_xy.end(),
    [](const std::pair<double, double> & a, const std::pair<double, double> & b) {
      return a.first != b.first ? a.first < b.first : a.second < b.second;
    });

  const double n = static_cast<double>(pixels_xy.size());
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & p : pixels_xy) {
    mean_x += p.first;
    mean_y += p.second;
  }
  mean_x /= n;
  mean_y /= n;

  double mu20 = 0.0;
  double mu02 = 0.0;
  double mu11 = 0.0;
  for (const auto & p : pixels_xy) {
    const double dx = p.first - mean_x;
    const double dy = p.second - mean_y;
    mu20 += dx * dx;
    mu02 += dy * dy;
    mu11 += dx * dy;
  }
  mu20 /= n;
  mu02 /= n;
  mu11 /= n;
  out.mu20 = mu20;
  out.mu02 = mu02;
  out.mu11 = mu11;

  const double denom = mu20 + mu02;
  if (!(denom > 0.0)) {
    // Every foreground pixel coincides (single distinct point): no
    // observable spread in any direction, hence no axis. Not a shape this
    // estimator can be asked about.
    out.eccentricity = 0.0;
    return out;
  }

  const double eccentricity =
    std::sqrt((mu20 - mu02) * (mu20 - mu02) + 4.0 * mu11 * mu11) / denom;
  out.eccentricity = eccentricity;
  if (!(eccentricity >= eccentricity_min)) {
    // NaN-safe by construction (comparison false for NaN): isotropic or
    // near-isotropic mask -- refuse rather than emit a noisy angle.
    return out;
  }

  const double theta = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
  out.theta_rad = canonicalize_axial_angle(theta);
  out.valid = true;
  return out;
}

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__MASK_ORIENTATION_HPP_
