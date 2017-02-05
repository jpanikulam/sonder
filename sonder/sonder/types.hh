#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

// std::vector for our special eigen types
template <typename Derived>
using EigStdVector = std::vector<Derived, Eigen::aligned_allocator<Derived>>;

typedef float Scalar;
typedef Sophus::SO3<Scalar> so3;
typedef Sophus::SE3<Scalar> se3;