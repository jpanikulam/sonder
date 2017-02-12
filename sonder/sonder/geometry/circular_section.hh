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
      : direction(_direction), normal(_normal), center(_center), radius(_radius), arc_rads(_arc_rads),
        spanning_circle(_normal, _center, _radius) {
    _cos_arc_rads = std::cos(_arc_rads / 2.0);
  }

  // Compute the intersection point(s) of two circular arcs
  //
  // Do this by:
  // 1. Computing the intersection point(s) of two circles
  // 2. Determining whether those intersection point(s) lie on **BOTH** arcs
  // 3. Returning only the point(s) that do
  //
  EigStdVector<Eigen::Vector3f> intersect(const circular_section &other) const {
    EigStdVector<Eigen::Vector3f> intersections;

    const EigStdVector<Eigen::Vector3f> candidate_intersections = spanning_circle.intersect(other.spanning_circle);
    for (const auto &candidate : candidate_intersections) {
      const bool on_both_arcs = is_point_on_circle_on_arc(candidate) && other.is_point_on_circle_on_arc(candidate);

      if (on_both_arcs) {
        intersections.emplace_back(candidate);
      }
    }

    return intersections;
  }

  const Eigen::Vector3f direction = Eigen::Vector3f::Zero();
  const Eigen::Vector3f normal    = Eigen::Vector3f::Zero();
  const Eigen::Vector3f center    = Eigen::Vector3f::Zero();

  const float radius   = 0.0f;
  const float arc_rads = 0.0f;

  const circle spanning_circle = circle(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0.0);

  // Determine whether a point [which is already known to lie on the circle] lies on the arc
  //
  bool is_point_on_circle_on_arc(const Eigen::Vector3f &pt) const {
    const auto  difference = (pt - center).normalized();
    const float dot        = difference.dot(direction);
    return (dot >= _cos_arc_rads);
  }

private:
  float _cos_arc_rads;
};
}