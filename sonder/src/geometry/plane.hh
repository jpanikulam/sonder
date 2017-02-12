#pragma once

#include "geometry/geometry.hh"
#include "geometry/line.hh"

namespace sonder {

class plane {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  plane(const Eigen::Vector3f& _normal, const Eigen::Vector3f& _point) : normal(_normal.normalized()), point(_point) {
  }

  const line intersect(const plane& other) const {
    const Eigen::Vector3f direction = other.normal.cross(normal);
    Eigen::Matrix<float, 2, 3> m;
    m.row(0) = normal;
    m.row(1) = other.normal;
    const Eigen::Vector2f b(normal.dot(point), other.normal.dot(other.point));
    Eigen::Vector3f       line_point = m.colPivHouseholderQr().solve(b);

    return line(direction, line_point);
  }

  const Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  const Eigen::Vector3f point  = Eigen::Vector3f::Zero();
};
}