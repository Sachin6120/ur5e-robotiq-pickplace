#ifndef UR5E_PICK_PLACE__PIXEL_CENTRE_SHADOW_HPP_
#define UR5E_PICK_PLACE__PIXEL_CENTRE_SHADOW_HPP_

// Pure, default-off shadow-estimator helpers.  These deliberately receive the
// already-computed D10 depth rather than inspecting a mask or depth samples:
// D10 and segmentation are production behaviour and must remain unchanged.

namespace ur5e_pick_place
{

constexpr double kPixelCentreCorrectionPx = 0.5;
constexpr bool kPixelCentreShadowDefaultEnabled = false;

struct PixelCentroid
{
  double u;
  double v;
};

struct PinholeIntrinsics
{
  double fx;
  double fy;
  double cx;
  double cy;
};

struct CameraPoint
{
  double x;
  double y;
  double z;
};

inline PixelCentroid pixel_centre_corrected(const PixelCentroid & raw)
{
  return {
    raw.u + kPixelCentreCorrectionPx,
    raw.v + kPixelCentreCorrectionPx,
  };
}

inline CameraPoint backproject_centroid(
  const PixelCentroid & centroid, const double depth_z,
  const PinholeIntrinsics & intrinsics)
{
  return {
    (centroid.u - intrinsics.cx) * depth_z / intrinsics.fx,
    (centroid.v - intrinsics.cy) * depth_z / intrinsics.fy,
    depth_z,
  };
}

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__PIXEL_CENTRE_SHADOW_HPP_
