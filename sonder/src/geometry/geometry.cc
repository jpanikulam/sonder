#include "geometry/geometry.hh"

namespace sonder {

Eigen::AngleAxisf vector3f_to_angleaxis(const Eigen::Vector3f &vec) {
  const float           norm      = vec.norm();
  const Eigen::Vector3f direction = vec / norm;

  if (norm > 1e-3) {
    return Eigen::AngleAxisf(norm, direction);
  } else {
    // Return an identity AA
    return Eigen::AngleAxisf(0.0, Eigen::Vector3f(0.0, 0.0, 1.0));
  }
}

Eigen::Matrix3f create_rotation_to(const Eigen::Vector3f &from, const Eigen::Vector3f &to) {
  // Enforce normalization when computing the axis

  const Eigen::Vector3f u_from = from.normalized();
  const Eigen::Vector3f u_to   = to.normalized();
  const Eigen::Vector3f u_axis = u_from.cross(u_to);
  const float           angle  = std::acos(u_from.dot(u_to));

  const Eigen::AngleAxisf angle_axis(angle, u_axis);
  return angle_axis.toRotationMatrix();
}

Eigen::Quaternionf rotation_from_xy(const Eigen::Vector3f &frame_x, const Eigen::Vector3f &frame_y) {
  const Eigen::Vector3f frame_z = frame_x.cross(frame_y);
  Eigen::Matrix3f       rotation;
  rotation << frame_x, frame_y, frame_z;
  const Eigen::Quaternionf q_rotation(rotation);
  return q_rotation;
}

Eigen::Vector3f any_perpendicular(const Eigen::Vector3f &direction) {
  return Eigen::Vector3f(direction.z(), direction.z(), -direction.x() - direction.x());
}
}