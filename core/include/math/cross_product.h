#pragma once

#include <Eigen/Core>

namespace math {

template <typename T>
Eigen::Matrix3<typename T::Scalar> GetSkewSymmetric(const Eigen::MatrixBase<T>& v) {
  return Eigen::Matrix3<typename T::Scalar>{{0, -v(2), v(1)}, {v(2), 0, -v(0)}, {-v(1), v(0), 0}};
}

template <typename T>
Eigen::Vector3<typename T::Scalar> GetVectorFromSkewSymmetric(const Eigen::MatrixBase<T>& m) {
  return {m(2, 1), m(0, 2), m(1, 0)};
}
}  // namespace math
