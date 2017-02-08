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
      : direction(_direction),
        normal(_normal),
        center(_center),
        radius(_radius),
        arc_rads(_arc_rads),
        spanning_circle(_normal, _center, _radius) {
    _cos_arc_rads = std::cos(_arc_rads);
  }
  EigStdVector<Eigen::Vector3f> intersect(const circular_section &other) const {
    EigStdVector<Eigen::Vector3f>       intersections;
    const EigStdVector<Eigen::Vector3f> candidate_intersections = spanning_circle.intersect(other.spanning_circle);
    for (const auto &candidate : candidate_intersections) {
      const bool on_both_arcs = is_point_on_circle_on_arc(candidate) && other.is_point_on_circle_on_arc(candidate);

      if (on_both_arcs) {
        intersections.emplace_back(candidate);
      }

      // std::cout << dot << " : " << _cos_arc_rads << std::endl;
    }

    return intersections;
  }

  const Eigen::Vector3f direction = Eigen::Vector3f::Zero();
  const Eigen::Vector3f normal    = Eigen::Vector3f::Zero();
  const Eigen::Vector3f center    = Eigen::Vector3f::Zero();

  const float radius   = 0.0f;
  const float arc_rads = 0.0f;

  const circle spanning_circle = circle(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0.0);

  // Does a point that already lies on the circle lie on the arc?
  //
  //
  bool is_point_on_circle_on_arc(const Eigen::Vector3f &pt) const {
    const auto  difference = (pt - center).normalized();
    const float dot        = std::fabs(difference.dot(direction));
    return (dot >= _cos_arc_rads);
  }

 private:
  float _cos_arc_rads;
};
}