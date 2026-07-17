#pragma once

#include <Eigen/Core>

namespace math {

// Interpolates on a quintic spline given the following constraints:
// * The value, first derivative, second derivative of the start point.
// * The value, first derivative, second derivative of the end point.
//
// |T| can be floating types or fixed Eigen types.
//
// Spline are defined as:
//   P(t) = a₀(t-t₀)⁵ + a₁(t-t₀)⁴+ a₂(t-t₀)³ + a₃(t-t₀)² + a₄(t-t₀) + a₅
//
// a₀, a₁, a₂, a₃, a₄, a₅ are the six coefficients for the spline P(t).
// Derivatives for P₀:
//  Ṗ(t) = 5a₀(t-t₀)⁴ + 4a₁(t-t₀)³ + 3a₂(t-t₀)² + 2a₃(t-t₀) + a₄
//  P̈(t) = 20a₀(t-t₀)³ + 12a₁(t-t₀)² + 6a₂(t-t₀) + 2a₃
template <typename T>
class QuinticSplineInterpolation {
 public:
  template <typename Ts, typename Enable = void>
  struct ScalarType {
    using type = Ts;
  };

  template <typename Ts>
  struct ScalarType<Ts, std::void_t<typename Ts::Scalar>> {
    using type = typename Ts::Scalar;
  };
  using Scalar = typename ScalarType<T>::type;

  struct Knot {
    Scalar t;
    Scalar y;
    // |yd|: the first derivative dy/dt at |t|
    Scalar yd;
    // |ydd|: the second derivative d²y/dt² at |t|
    Scalar ydd;
  };

  struct KnotVector {
    Scalar t;
    T y;
    // |yd|: the first derivative dy/dt at |t|
    T yd;
    // |ydd|: the second derivative d²y/dt² at |t|
    T ydd;
  };

  // User must guarantee |start.t| < |end.t|.
  QuinticSplineInterpolation(const Knot& start, const Knot& end)
      : start_t_(start.t), end_t_(end.t), period_(end.t - start.t) {
    CalculateCoefficients(end.t - start.t);
    CalculateScalarParameters(start, end);
  }

  QuinticSplineInterpolation(const KnotVector& start, const KnotVector& end)
      : start_t_(start.t), end_t_(end.t), period_(end.t - start.t) {
    CalculateCoefficients(end.t - start.t);
    CalculateVectorParameters(start, end);
  }

  QuinticSplineInterpolation() = default;

  // Interpolates value of |y| at |t|.
  void Interpolate(Scalar t, Scalar& y) {
    CalculateValue(t);
    y = values_.row(0);
  }

  // Also returns the first derivative |yd| at |t|.
  void Interpolate(Scalar t, Scalar& y, Scalar& yd) {
    CalculateValue(t);
    y = values_.row(0);
    yd = values_.row(1);
  }

  // Also returns the second derivative |ydd| at |t|.
  void Interpolate(Scalar t, Scalar& y, Scalar& yd, Scalar& ydd) {
    CalculateValue(t);
    y = values_.row(0);
    yd = values_.row(1);
    ydd = values_.row(2);
  }

  // Interpolates value of |y| at |t|.
  void Interpolate(Scalar t, T& y) {
    CalculateValue(t);
    y = values_.row(0).transpose();
  }

  // Also returns the first derivative |yd| at |t|.
  void Interpolate(Scalar t, T& y, T& yd) {
    CalculateValue(t);
    y = values_.row(0).transpose();
    yd = values_.row(1).transpose();
  }

  // Also returns the second derivative |ydd| at |t|.
  void Interpolate(Scalar t, T& y, T& yd, T& ydd) {
    CalculateValue(t);
    y = values_.row(0).transpose();
    yd = values_.row(1).transpose();
    ydd = values_.row(2).transpose();
  }

  Scalar GetStartTime() { return start_t_; }
  Scalar GetEndTime() { return end_t_; }
  Scalar GetPeriod() { return period_; }

 private:
  void CalculateCoefficients(Scalar period) {
    Scalar divide_period = 1 / period;
    Scalar divide_period2 = divide_period * divide_period;
    Scalar divide_period3 = divide_period2 * divide_period;
    Scalar divide_period4 = divide_period3 * divide_period;
    Scalar divide_period5 = divide_period4 * divide_period;

    coeffs_.row(0) << -6 * divide_period5, -3 * divide_period4, -0.5 * divide_period3, 6 * divide_period5,
        -3 * divide_period4, 0.5 * divide_period3;
    coeffs_.row(1) << 15 * divide_period4, 8 * divide_period3, 1.5 * divide_period2, -15 * divide_period4,
        7 * divide_period3, -divide_period2;
    coeffs_.row(2) << -10 * divide_period3, -6 * divide_period2, -1.5 * divide_period, 10 * divide_period3,
        -4 * divide_period2, 0.5 * divide_period;
    coeffs_.row(3) << 0, 0, 0.5, 0, 0, 0;
    coeffs_.row(4) << 0, 1, 0, 0, 0, 0;
    coeffs_.row(5) << 1, 0, 0, 0, 0, 0;
  }

  void CalculateScalarParameters(const Knot& start, const Knot& end) {
    Eigen::Matrix<Scalar, 6, 1> bounds;
    bounds(0, 0) = start.y;
    bounds(1, 0) = start.yd;
    bounds(2, 0) = start.ydd;
    bounds(3, 0) = end.y;
    bounds(4, 0) = end.yd;
    bounds(5, 0) = end.ydd;

    params_ = coeffs_ * bounds;
  }

  void CalculateVectorParameters(const KnotVector& start, const KnotVector& end) {
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic> bounds(6, start.y.size());
    bounds.row(0) = start.y.transpose();
    bounds.row(1) = start.yd.transpose();
    bounds.row(2) = start.ydd.transpose();
    bounds.row(3) = end.y.transpose();
    bounds.row(4) = end.yd.transpose();
    bounds.row(5) = end.ydd.transpose();

    params_ = coeffs_ * bounds;
  }

  void CalculateValue(Scalar t) {
    Scalar phase = t - start_t_;
    Eigen::Matrix<Scalar, 3, 6> time;
    double phase2 = phase * phase;
    double phase3 = phase2 * phase;
    double phase4 = phase3 * phase;
    double phase5 = phase4 * phase;
    time.row(0) << phase5, phase4, phase3, phase2, phase, 1;
    time.row(1) << 5 * phase4, 4 * phase3, 3 * phase2, 2 * phase, 1, 0;
    time.row(2) << 20 * phase3, 12 * phase2, 6 * phase, 2, 0, 0;

    values_ = time * params_;
  }

  Scalar start_t_;
  Scalar end_t_;
  Scalar period_;
  Eigen::Matrix<Scalar, 6, 6> coeffs_;
  Eigen::Matrix<Scalar, 6, Eigen::Dynamic> params_;
  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> values_;
};
}  // namespace math
