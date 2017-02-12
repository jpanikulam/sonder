#pragma once

#include "types.hh"

#include <Eigen/Dense>

namespace sonder {

Eigen::AngleAxisf vector3f_to_angleaxis(const Eigen::Vector3f &vec);

// Construct a rotation between two vectors (Non-unique)
//
//
Eigen::Matrix3f create_rotation_to(const Eigen::Vector3f &from, const Eigen::Vector3f &to);

// Construct a rotation into a frame defined by two vectors
//
//
Eigen::Quaternionf rotation_from_xy(const Eigen::Vector3f &frame_x, const Eigen::Vector3f &frame_y);

Eigen::Vector3f any_perpendicular(const Eigen::Vector3f &direction);
}