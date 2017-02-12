#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

// std::vector for our special eigen types
template <typename Derived>
using EigStdVector = std::vector<Derived, Eigen::aligned_allocator<Derived>>;

typedef float               Scalar;
typedef Sophus::SO3<Scalar> so3;
typedef Sophus::SE3<Scalar> se3;

typedef EigStdVector<Eigen::Vector3f> PointList;

//
// Retains no ownership over the object
//
template <class T>
struct Out {
  explicit Out(T &obj) : _obj(obj) {
  }
  T& operator*() {
    return _obj;
  }
  T* operator->() {
    return &_obj;
  }

 private:
  T& _obj;
};

template <class T>
Out<T> out(T& obj) {
  return Out<T>(obj);
}
