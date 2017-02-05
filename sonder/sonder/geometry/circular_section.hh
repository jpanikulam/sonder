#pragma once
#include <sonder/geometry/circle.hh>

namespace sonder {

class circular_section {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  circular_section(const Eigen::Vector3f &_direction,
                   const Eigen::Vector3f &_normal,
                   const Eigen::Vector3f &_center,
                   const float            _radius,
                   const float            _arc_rads)
      : direction(_direction), normal(_normal), center(_center), radius(_radius), arc_rads(_arc_rads) {
  }

  const Eigen::Vector3f direction = Eigen::Vector3f::Zero();
  const Eigen::Vector3f normal    = Eigen::Vector3f::Zero();
  const Eigen::Vector3f center    = Eigen::Vector3f::Zero();
  const float           radius    = 0.0f;
  const float           arc_rads  = 0.0f;
};
}