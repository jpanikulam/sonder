#include "sensing/rendering.hh"

namespace sonder {
namespace sim {

std::pair<Eigen::Vector2f, bool> project_to_polar(const Eigen::Vector3f &point,
                                                  const float max_bearing,
                                                  const float max_elevation) {
  const float rcos_theta = point.head<2>().norm();
  const float elevation  = atan2(point.z(), rcos_theta);
  const float range      = point.norm();
  const float bearing    = atan2(point.y(), point.x());
  const bool  in_range   = (std::fabs(elevation) < max_elevation) && (std::fabs(bearing) < max_bearing);

  const Eigen::Vector2f result(range, bearing);
  return std::make_pair(result, in_range);
}

EigStdVector<Eigen::Vector2f> to_range_bearing(const EigStdVector<Eigen::Vector3f> &points,
                                               const se3 &                          pose,
                                               const float                          max_bearing,
                                               const float                          max_elevation) {
  EigStdVector<Eigen::Vector2f> range_bearings;
  range_bearings.reserve(points.size());

  const se3 pose_inv = pose.inverse();
  for (const auto point : points) {
    const Eigen::Vector3f new_point  = pose_inv * point;
    const auto            projection = project_to_polar(new_point, max_bearing, max_elevation);

    // If the projection is valid
    if (projection.second) {
      range_bearings.emplace_back(projection.first);
    }
  }

  return range_bearings;
}

EigStdVector<Eigen::Vector3f> make_plane_blanket() {
  EigStdVector<Eigen::Vector3f> points;

  for (float x = 1.0; x < 3.0; x += 1.0) {
    for (float y = -1.0; y <= 1.0; y += 1.0) {
      for (float z = 0.0; z <= 1.0; z += 1.0) {
        const Eigen::Vector3f pt = Eigen::Vector3f(x, y, z) + (Eigen::Vector3f::Random() * 0.001);
        points.emplace_back(pt);
      }
    }
  }
  // points.emplace_back(1.0, 0.0, 0.0);

  return points;
}
}
}