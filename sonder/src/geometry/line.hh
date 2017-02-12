#pragma once

#include "geometry/geometry.hh"

namespace sonder {

class line {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  line(const Eigen::Vector3f& _direction, const Eigen::Vector3f& _point)
      : direction(_direction.normalized()), point(_point) {
  }

  // Get an arbitrary line perpendicular to this one
  //
  // This is just a cheap trick, and provides no interesting guarantees
  //
  line any_perpendicular() const {
    return line(::sonder::any_perpendicular(direction), point);
  }

  const Eigen::Vector3f direction = Eigen::Vector3f::Zero();
  const Eigen::Vector3f point     = Eigen::Vector3f::Zero();
};
}