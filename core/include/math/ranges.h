#pragma once

#include <Eigen/Dense>
#include <vector>

namespace math {

template <typename T>
void ApplyDeadband(T cutoff, T& v) {
  if (v > -cutoff && v < cutoff) {
    v = 0;
  }
}

template <typename T>
int GetDeadbandSign(T v, T cutoff) {
  if (v >= cutoff) {
    return 1;
  }
  if (v <= -cutoff) {
    return -1;
  }
  return 0;
}

template <typename T>
void ApplyDeadband(typename T::Scalar cutoff, Eigen::MatrixBase<T>& v) {
  v.unaryExpr([cutoff](typename T::Scalar x) { return x > -cutoff && x < cutoff ? 0 : x; });
}

template <typename T>
auto Clamp(const Eigen::MatrixBase<T>& v, typename T::Scalar lo, typename T::Scalar hi) {
  return v.cwiseMin(hi).cwiseMax(lo);
}

template <typename T, int Rows>
Eigen::Matrix<T, Rows, 1> Clamp(const Eigen::Matrix<T, Rows, 1>& v, const Eigen::Matrix<T, Rows, 1>& v_min,
                                const Eigen::Matrix<T, Rows, 1>& v_max) {
  Eigen::Matrix<T, Rows, 1> result;
  for (int i = 0; i < v.size(); ++i) {
    result[i] = std::clamp(v[i], v_min[i], v_max[i]);
  }
  return result;
}

// Returns the sign of 'v'
// If returning zero is not needed, std::copysign(1.0, v) returns the sign of 'v' faster.
template <typename T>
int Sign(T v) {
  if (v < 0) {
    return -1;
  } else if (v > 0) {
    return 1;
  } else {
    return 0;
  }
}
}  // namespace math
