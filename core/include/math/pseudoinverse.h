#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>

#include "math/eigen_type_traits.h"

namespace math {

// Returns floating-point machine precision relative to `x`.
// Similar to eps(x) in MATLAB.
template <typename T>
T Epsilon(T x) {
  return std::nextafter(std::abs(x), std::numeric_limits<T>::infinity()) - std::abs(x);
}

// There are three main methods to find the pseudo-inverse:
// * Direct derivative: A = (Aᵀ A)⁻¹ Aᵀ
// * QR decomposition: A = Q R, A⁻¹ = R⁻¹ Qᵀ
// * SVD decomposition: A = U S Vᵀ, A⁻¹ = V S⁻¹ Uᵀ
// QR is the fastest but potentially numerically unstable.
// SVD is slower than QR and numerically stable.

// Returns the pseudoinverse of `A` constructed with the singular value decomposition.
// The returned value is an evaluated Eigen matrix of transposed dimensions instead of an expression.
// `tolerance`: Singular values smaller than `tolerance` are replaced by zeros.
//              If unset, uses the default value of max(size(A)) * eps(norm(A)), where norm(A) = max(Σ).
template <typename Derived>
auto PseudoinverseSvd(const Eigen::MatrixBase<Derived>& A, std::optional<double> tolerance = {})
    -> EigenConcreteType<decltype(A.transpose())> {
  if (A.size() == 0) return A.transpose();
  using Matrix = EigenConcreteType<Derived>;
  using MatrixTransposed = EigenConcreteType<decltype(A.transpose())>;
  Eigen::JacobiSVD<Matrix> A_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto S = A_svd.singularValues();
  MatrixTransposed S_inv = MatrixTransposed::Zero(A.cols(), A.rows());
  if (!tolerance.has_value()) {
    // `A` being non-empty here guarantees S[0] is accessible.
    tolerance = std::max(A.cols(), A.rows()) * Epsilon(S[0]);
  }
  // This counts on the fact that singularValues() are always sorted in decreasing order.
  int num_nonzeros = (S.array() >= *tolerance).count();
  S_inv.diagonal().segment(0, num_nonzeros) = S.segment(0, num_nonzeros).cwiseInverse();
  return A_svd.matrixV() * S_inv * A_svd.matrixU().transpose();
}

// Returns the pseudoinverse of `A` constructed with the QR decomposition.
// The returned value is an evaluated Eigen matrix of transposed dimensions instead of an expression.
template <typename Derived>
auto PseudoinverseQr(const Eigen::MatrixBase<Derived>& A) -> EigenConcreteType<decltype(A.transpose())> {
  return A.completeOrthogonalDecomposition().pseudoInverse();
}

// Damped least-squares method for Jacobian inverse
// Wampler C W. Manipulator inverse kinematic solutions based on vector formulations and damped least-squares
// methods[J]. IEEE Transactions on Systems, Man, and Cybernetics, 1986, 16(1): 93-101.
template <typename Derived>
auto DampedLeastSquaresInverse(const Eigen::MatrixBase<Derived>& A, double damping_factor)
    -> math::EigenConcreteType<decltype(A.transpose())> {
  using Matrix = math::EigenConcreteType<Derived>;
  using MatrixTransposed = math::EigenConcreteType<decltype(A.transpose())>;
  Eigen::JacobiSVD<Matrix> A_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto S = A_svd.singularValues();
  MatrixTransposed S_inv = MatrixTransposed::Zero(A.cols(), A.rows());
  for (int i = 0; i < S.size(); ++i) {
    S_inv(i, i) = S(i) / (S(i) * S(i) + damping_factor * damping_factor);
  }
  return A_svd.matrixV() * S_inv * A_svd.matrixU().transpose();
}

// Returns an evaluated matrix of transposed dimensions.
// Uses the damped least-squares method with a certain modification that keeps the inverse values smooth to an extent in
// highly dynamic scenarios.
template <typename Derived>
auto DampedLeastSquaresInverseModified(const Eigen::MatrixBase<Derived>& A, double threshold)
    -> math::EigenConcreteType<decltype(A.transpose())> {
  using Matrix = math::EigenConcreteType<Derived>;
  using MatrixTransposed = math::EigenConcreteType<decltype(A.transpose())>;
  Eigen::JacobiSVD<Matrix> A_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto S = A_svd.singularValues();
  MatrixTransposed S_inv = MatrixTransposed::Zero(A.cols(), A.rows());
  // TODO(therionchen) : Justify this operation with a citation
  S_inv.diagonal() = S.cwiseMax(threshold).cwiseInverse();
  return A_svd.matrixV() * S_inv * A_svd.matrixU().transpose();
}
}  // namespace math
