#pragma once

#include <sonder/geometry.hh>
#include <sonder/geometry/plane.hh>
#include <sonder/math.hh>

namespace sonder {

class circle {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  circle(const Eigen::Vector3f &_normal, const Eigen::Vector3f &_center, const float _radius)
      : normal(_normal), center(_center), radius(_radius) {
  }

  // TODO: Do something when parallel non-coplanar
  //  (The intersection line should be garbage in that case)
  //
  EigStdVector<Eigen::Vector3f> intersect(const circle &other) const {
    // A loose meausure of how precisely we expect circles to coincide
    constexpr float EPS = 1e-4f;

    const plane this_plane  = plane(normal, center);
    const plane other_plane = plane(other.normal, other.center);

    const line pl_line = this_plane.intersect(other_plane);

    const Eigen::Vector3f q = pl_line.point - center;

    const float a_dot_q      = pl_line.direction.dot(q);
    const float discriminant = int_pow(a_dot_q, 2u) - int_pow(q.norm(), 2u) + int_pow(radius, 2u);

    EigStdVector<Eigen::Vector3f> intersection_points;
    if (discriminant < 1e-12f) {
      // No intersection
    } else if (discriminant > 1e-1f) {
      // Two circle-plane intersections

      //
      // TODO: Factor these scopes into functions
      //

      // Compute the first (+) possibility
      {
        const float           t               = -a_dot_q + std::sqrt(discriminant);
        const Eigen::Vector3f candidate_point = pl_line.point + (t * pl_line.direction);

        // Only add the candidate point if it lies on the other circle
        if (std::fabs((candidate_point - other.center).norm() - other.radius) < EPS) {
          intersection_points.emplace_back(candidate_point);
        }
      }

      // Compute the second (-) possibility
      {
        const float           t               = -a_dot_q - std::sqrt(discriminant);
        const Eigen::Vector3f candidate_point = pl_line.point + (t * pl_line.direction);

        // Only add the candidate point if it lies on the other circle
        if (std::fabs((candidate_point - other.center).norm() - other.radius) < EPS) {
          intersection_points.emplace_back(candidate_point);
        }
      }

    } else {
      // One circle-plane intersection
      {
        const float           t               = -a_dot_q;
        const Eigen::Vector3f candidate_point = pl_line.point + (t * pl_line.direction);

        if (std::fabs((candidate_point - other.center).norm() - other.radius) < EPS) {
          intersection_points.emplace_back(candidate_point);
        }
      }
    }
    return intersection_points;
  }

  const Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  const Eigen::Vector3f center = Eigen::Vector3f::Zero();
  const float           radius = 0.0f;
};
}