#ifndef UR5E_PICK_PLACE__EDGE_LINE_TLS_HPP_
#define UR5E_PICK_PLACE__EDGE_LINE_TLS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "ur5e_pick_place/mask_orientation.hpp"

namespace ur5e_pick_place
{

struct EdgeLineTLSResult
{
  bool valid{false};
  double theta_image_rad{std::numeric_limits<double>::quiet_NaN()};  // canonical [-pi/2, +pi/2)
  double eccentricity{std::numeric_limits<double>::quiet_NaN()};     // normalized second-moment anisotropy
  std::size_t boundary_pixel_count{0};
  std::array<std::size_t, 4> family_support_counts{0, 0, 0, 0};     // right, left, top, bottom
  double quality_score{0.0};                                         // sum of support counts of valid fitted clusters
};

/**
 * @brief Converts canonical image axial angle (radians) to world-frame yaw (radians).
 * 
 * Optical Frame Convention (REP-145, camera mounted looking downward):
 *   +X_opt = -Y_world (image right)
 *   +Y_opt = -X_world (image down)
 *   +Z_opt = -Z_world (camera forward)
 * 
 * Major axis projection yields:
 *   yaw_world_axial = canonicalize_axial_angle(pi/2 - theta_image_rad)
 */
inline double image_axial_to_world_yaw(double theta_image_rad)
{
  return canonicalize_axial_angle(kMaskOrientationPi / 2.0 - theta_image_rad);
}

/**
 * @brief Helper for Total Least Squares direction fitting via SVD on centered 2D points.
 * Returns angle in radians or NaN if degenerate.
 */
inline double fit_tls_direction_svd(const std::vector<cv::Point2d> & points)
{
  if (points.size() < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & pt : points) {
    sum_x += pt.x;
    sum_y += pt.y;
  }
  const double n = static_cast<double>(points.size());
  const double mean_x = sum_x / n;
  const double mean_y = sum_y / n;

  cv::Mat A(static_cast<int>(points.size()), 2, CV_64F);
  for (std::size_t i = 0; i < points.size(); ++i) {
    A.at<double>(static_cast<int>(i), 0) = points[i].x - mean_x;
    A.at<double>(static_cast<int>(i), 1) = points[i].y - mean_y;
  }

  cv::Mat W, U, Vt;
  cv::SVD::compute(A, W, U, Vt);
  if (Vt.rows < 1 || Vt.cols < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double vx = Vt.at<double>(0, 0);
  const double vy = Vt.at<double>(0, 1);
  if (!std::isfinite(vx) || !std::isfinite(vy) || (vx == 0.0 && vy == 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(vy, vx);
}

/**
 * @brief Production Edge-Line Total Least Squares (TLS) mask orientation estimator.
 * 
 * Input: Single binary component mask (cv::Mat of type CV_8UC1, non-zero = foreground).
 * Pure C++, deterministic, no ROS / Gazebo / ground-truth dependencies.
 */
inline EdgeLineTLSResult estimate_edgelines_tls(const cv::Mat & mask_u8)
{
  EdgeLineTLSResult out;

  if (mask_u8.empty() || mask_u8.type() != CV_8UC1) {
    return out;
  }

  // 1. Find external contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask_u8, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (contours.empty()) {
    return out;
  }

  auto max_it = std::max_element(
    contours.begin(), contours.end(),
    [](const std::vector<cv::Point> & a, const std::vector<cv::Point> & b) {
      return cv::contourArea(a) < cv::contourArea(b);
    });

  const std::vector<cv::Point> & contour = *max_it;
  if (contour.size() < 8) {
    return out;
  }
  out.boundary_pixel_count = contour.size();

  // 2. Extract filled foreground pixel centers (x + 0.5, y + 0.5) for coarse moment seed
  std::vector<cv::Point2d> fill_pts;
  for (int r = 0; r < mask_u8.rows; ++r) {
    const std::uint8_t * ptr = mask_u8.ptr<std::uint8_t>(r);
    for (int c = 0; c < mask_u8.cols; ++c) {
      if (ptr[c] != 0) {
        fill_pts.emplace_back(static_cast<double>(c) + 0.5, static_cast<double>(r) + 0.5);
      }
    }
  }

  if (fill_pts.empty()) {
    return out;
  }

  const double n_fill = static_cast<double>(fill_pts.size());
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & p : fill_pts) {
    mean_x += p.x;
    mean_y += p.y;
  }
  mean_x /= n_fill;
  mean_y /= n_fill;

  double mu20 = 0.0;
  double mu02 = 0.0;
  double mu11 = 0.0;
  for (const auto & p : fill_pts) {
    const double dx = p.x - mean_x;
    const double dy = p.y - mean_y;
    mu20 += dx * dx;
    mu02 += dy * dy;
    mu11 += dx * dy;
  }
  mu20 /= n_fill;
  mu02 /= n_fill;
  mu11 /= n_fill;

  const double denom = mu20 + mu02;
  if (!(denom > 0.0)) {
    return out;
  }
  const double eccentricity =
    std::sqrt((mu20 - mu02) * (mu20 - mu02) + 4.0 * mu11 * mu11) / denom;
  out.eccentricity = eccentricity;

  // Structural anisotropy gate: refuse isotropic / near-isotropic shapes (eccentricity < 0.10)
  // where axial orientation is physically unobservable.
  constexpr double kMinEccentricity = 0.10;
  if (!(eccentricity >= kMinEccentricity)) {
    return out;
  }

  const double theta0 = canonicalize_axial_angle(0.5 * std::atan2(2.0 * mu11, mu20 - mu02));

  // 3. Transform boundary points (x + 0.5, y + 0.5) into seed-aligned frame
  std::vector<cv::Point2d> boundary_pts;
  boundary_pts.reserve(contour.size());
  for (const auto & pt : contour) {
    boundary_pts.emplace_back(static_cast<double>(pt.x) + 0.5, static_cast<double>(pt.y) + 0.5);
  }

  const double cos_rot = std::cos(-theta0);
  const double sin_rot = std::sin(-theta0);

  std::vector<double> lx(boundary_pts.size());
  std::vector<double> ly(boundary_pts.size());
  double min_lx = std::numeric_limits<double>::infinity();
  double max_lx = -std::numeric_limits<double>::infinity();
  double min_ly = std::numeric_limits<double>::infinity();
  double max_ly = -std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < boundary_pts.size(); ++i) {
    const double rx = boundary_pts[i].x - mean_x;
    const double ry = boundary_pts[i].y - mean_y;
    const double val_lx = rx * cos_rot - ry * sin_rot;
    const double val_ly = rx * sin_rot + ry * cos_rot;
    lx[i] = val_lx;
    ly[i] = val_ly;
    if (val_lx < min_lx) min_lx = val_lx;
    if (val_lx > max_lx) max_lx = val_lx;
    if (val_ly < min_ly) min_ly = val_ly;
    if (val_ly > max_ly) max_ly = val_ly;
  }

  const double hx_data = (max_lx - min_lx) / 2.0;
  const double hy_data = (max_ly - min_ly) / 2.0;

  // 4. Assign boundary points to 4 candidate edge families:
  // Cluster 0: right boundary (lx = +hx_data) -> short-side boundary
  // Cluster 1: left boundary (lx = -hx_data)  -> short-side boundary
  // Cluster 2: top boundary (ly = +hy_data)   -> long-side boundary
  // Cluster 3: bottom boundary (ly = -hy_data)-> long-side boundary
  std::array<std::vector<cv::Point2d>, 4> clusters;
  for (std::size_t i = 0; i < boundary_pts.size(); ++i) {
    const double d0 = std::abs(lx[i] - hx_data);
    const double d1 = std::abs(lx[i] + hx_data);
    const double d2 = std::abs(ly[i] - hy_data);
    const double d3 = std::abs(ly[i] + hy_data);

    int best_cid = 0;
    double min_d = d0;
    if (d1 < min_d) { min_d = d1; best_cid = 1; }
    if (d2 < min_d) { min_d = d2; best_cid = 2; }
    if (d3 < min_d) { min_d = d3; best_cid = 3; }

    clusters[static_cast<std::size_t>(best_cid)].push_back(boundary_pts[i]);
  }

  for (std::size_t c = 0; c < 4; ++c) {
    out.family_support_counts[c] = clusters[c].size();
  }

  // 5. Fit SVD TLS to each cluster with N >= 3
  std::vector<double> directions;
  std::vector<double> weights;

  // Family mapping:
  // Clusters 0 & 1 (left/right at lx = +/- hx) run parallel to short axis -> rotate by +90 deg
  // Clusters 2 & 3 (top/bottom at ly = +/- hy) run parallel to long axis  -> 0 deg shift
  const std::array<bool, 4> is_long = {false, false, true, true};

  for (std::size_t cid = 0; cid < 4; ++cid) {
    if (clusters[cid].size() < 3) {
      continue;
    }
    const double edge_dir = fit_tls_direction_svd(clusters[cid]);
    if (!std::isfinite(edge_dir)) {
      continue;
    }
    double edge_axial = canonicalize_axial_angle(edge_dir);
    if (!is_long[cid]) {
      edge_axial = canonicalize_axial_angle(edge_axial + kMaskOrientationPi / 2.0);
    }
    directions.push_back(edge_axial);
    const double w = static_cast<double>(clusters[cid].size());
    weights.push_back(w);
    out.quality_score += w;
  }

  if (directions.empty()) {
    return out;
  }

  // 6. Circular 2*theta axial weighted average
  double sx = 0.0;
  double sy = 0.0;
  for (std::size_t i = 0; i < directions.size(); ++i) {
    sx += weights[i] * std::cos(2.0 * directions[i]);
    sy += weights[i] * std::sin(2.0 * directions[i]);
  }

  if (!std::isfinite(sx) || !std::isfinite(sy) || (sx == 0.0 && sy == 0.0)) {
    return out;
  }

  const double mean_theta = 0.5 * std::atan2(sy, sx);
  out.theta_image_rad = canonicalize_axial_angle(mean_theta);
  out.valid = true;
  return out;
}

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__EDGE_LINE_TLS_HPP_
