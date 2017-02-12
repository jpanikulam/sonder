#pragma once
#include "types.hh"

namespace sonder {
namespace sim {

// Project a 3D point into range, bearing coordinates
//
// The second element of the pair will be false if the abs(elevation) of `point` is greater than `max_elevation`
std::pair<Eigen::Vector2f, bool> project_to_polar(const Eigen::Vector3f &point,
                                                  const float max_bearing,
                                                  const float max_elevation);

// Project a list of points into 3D points
//
//
EigStdVector<Eigen::Vector2f> to_range_bearing(const EigStdVector<Eigen::Vector3f> &points,
                                               const se3 &                          pose,
                                               const float                          max_bearing,
                                               const float                          max_elevation);

EigStdVector<Eigen::Vector3f> make_plane_blanket();
}
}