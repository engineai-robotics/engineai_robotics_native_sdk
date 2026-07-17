#ifndef MATH_QUARTIC_SPLINE_INTERPOLATION_H_
#define MATH_QUARTIC_SPLINE_INTERPOLATION_H_

#include <Eigen/Geometry>
#include <type_traits>

namespace Eigen {
template <typename Type, int size>
using Vector = Matrix<Type, size, 1>;
}  // namespace Eigen

namespace math {

// Interpolates on a quartic spline of two pieces given the following constraints:
// * The value, first derivative, second derivative of the start point.
// * The value of the middle point.
// * The value, first derivative, second derivative of the end point.
//
// |T| can be floating types or fixed Eigen types.
//
// Spline piece 0 and 1 are defined as:
//   P₀(t) = a₀ + b₀(t-t₀) + c₀(t-t₀)² + d₀(t-t₀)³ + e₀(t-t₀)⁴,
//   P₁(t) = a₁ + b₁(t-t₁) + c₁(t-t₁)² + d₁(t-t₁)³ + e₁(t-t₁)⁴.
//
// a₀, b₀, c₀, d₀, e₀ are the five coefficients for the spline P₀, similarly for P₁.
// Derivatives for P₀:
//  Ṗ₀(t) = b₀ + 2c₀(t-t₀) + 3d₀(t-t₀)² + 4e₀(t-t₀)³
//  P̈₀(t) = 2c₀ + 6d₀(t-t₀) + 12e₀(t-t₀)²
template <typename T>
class QuarticSplineInterpolation {
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
    T y;
    // |yd|: the first derivative dy/dt at |t|
    T yd;
    // |ydd|: the second derivative d²y/dt² at |t|
    T ydd;
  };

  // The |middle| knot ignores the constraints on |yd| and |ydd|.
  // User must guarantee |start.t| < |middle.t| < |end.t|.
  QuarticSplineInterpolation(const Knot& start, const Knot& middle, const Knot& end)
      : start_t_(start.t), middle_t_(middle.t), end_t_(end.t), period_(end.t - start.t) {
    CalculateCoefficients(start, middle, end);
  }
  QuarticSplineInterpolation() = default;

  // Interpolates value of |y| at |t|.
  void Interpolate(Scalar t, T& y) {
    const T* k = (t < middle_t_ ? coeffs_[0].k : coeffs_[1].k);
    Scalar t_rel = (t < middle_t_ ? t - start_t_ : t - end_t_);

    y = k[0] + t_rel * (k[1] + t_rel * (k[2] + t_rel * (k[3] + t_rel * k[4])));
  }

  // Also returns the first derivative |yd| at |t|.
  void Interpolate(Scalar t, T& y, T& yd) {
    const T* k = (t < middle_t_ ? coeffs_[0].k : coeffs_[1].k);
    Scalar t_rel = (t < middle_t_ ? t - start_t_ : t - end_t_);

    y = k[0] + t_rel * (k[1] + t_rel * (k[2] + t_rel * (k[3] + t_rel * k[4])));
    yd = k[1] + t_rel * (2 * k[2] + t_rel * (3 * k[3] + t_rel * (4 * k[4])));
  }

  // Also returns the second derivative |ydd| at |t|.
  void Interpolate(Scalar t, T& y, T& yd, T& ydd) {
    const T* k = (t < middle_t_ ? coeffs_[0].k : coeffs_[1].k);
    Scalar t_rel = (t < middle_t_ ? t - start_t_ : t - end_t_);

    y = k[0] + t_rel * (k[1] + t_rel * (k[2] + t_rel * (k[3] + t_rel * k[4])));
    yd = k[1] + t_rel * (2 * k[2] + t_rel * (3 * k[3] + t_rel * (4 * k[4])));
    ydd = 2 * k[2] + t_rel * (6 * k[3] + t_rel * (12 * k[4]));
  }

  Scalar GetStartTime() { return start_t_; }
  Scalar GetEndTime() { return end_t_; }
  Scalar GetPeriod() { return period_; }

 private:
  static constexpr int kNumCoefficients = 5;
  static constexpr int kNumSplinePieces = 2;

  // Solves Ax = b where x is a column vector containing coefficients d₀, e₀, d₁, e₁.
  // (t₂-t₀)³d₀ + (t₂-t₀)⁴e₀ = y₂ - y₀ - ẏ₀(t₂-t₀) - 1/2ÿ₀(t₂-t₀)²
  // (t₂-t₁)³d₁ + (t₂-t₁)⁴e₁ = y₂ - y₁ - ẏ₁(t₂-t₁) - 1/2ÿ₁(t₂-t₁)²
  // 3(t₂-t₀)²d₀ + 4(t₂-t₀)³e₀ - 3(t₂-t₁)²d₁ - 4(t₂-t₁)³e₁ = ẏ₁ + ÿ₁(t₂-t₁) - ẏ₀ - ÿ₀(t₂-t₀)
  // 6(t₂-t₀)d₀ + 12(t₂-t₀)²e₀ - 6(t₂-t₁)d₁ - 12(t₂-t₁)²e₁ = ÿ₁ - ÿ₀
  void CalculateCoefficients(const Knot& start, const Knot& middle, const Knot& end) {
    coeffs_[0].k[0] = start.y;
    coeffs_[0].k[1] = start.yd;
    coeffs_[0].k[2] = start.ydd / 2;
    coeffs_[1].k[0] = end.y;
    coeffs_[1].k[1] = end.yd;
    coeffs_[1].k[2] = end.ydd / 2;

    // t20[i]: (t₂ - t₀)ⁱ, t20[0] is unused.
    Scalar t20[kNumCoefficients], t21[kNumCoefficients];
    t20[1] = middle.t - start.t;
    t21[1] = middle.t - end.t;
    for (int i = 2; i < kNumCoefficients; ++i) {
      t20[i] = t20[i - 1] * t20[1];
      t21[i] = t21[i - 1] * t21[1];
    }

    Eigen::Matrix<Scalar, 4, 4> A;
    A << t20[3], t20[4], 0, 0, 0, 0, t21[3], t21[4], 3 * t20[2], 4 * t20[3], -3 * t21[2], -4 * t21[3], 6 * t20[1],
        12 * t20[2], -6 * t21[1], -12 * t21[2];
    T b[4] = {
        middle.y - start.y - start.yd * t20[1] - start.ydd * t20[2] / 2,
        middle.y - end.y - end.yd * t21[1] - end.ydd * t21[2] / 2,
        end.yd + end.ydd * t21[1] - start.yd - start.ydd * t20[1],
        end.ydd - start.ydd,
    };
    auto A_lu_decomp = A.fullPivLu();

    if constexpr (std::is_floating_point_v<T>) {
      Eigen::Vector<Scalar, 4> bv(b);
      Eigen::Vector<Scalar, 4> x = A_lu_decomp.solve(bv);
      coeffs_[0].k[3] = x[0];
      coeffs_[0].k[4] = x[1];
      coeffs_[1].k[3] = x[2];
      coeffs_[1].k[4] = x[3];
    } else {
      for (int i = 0; i < T::RowsAtCompileTime; ++i) {
        for (int j = 0; j < T::ColsAtCompileTime; ++j) {
          Eigen::Vector<Scalar, 4> bv = {b[0](i, j), b[1](i, j), b[2](i, j), b[3](i, j)};
          Eigen::Vector<Scalar, 4> x = A_lu_decomp.solve(bv);
          coeffs_[0].k[3](i, j) = x[0];
          coeffs_[0].k[4](i, j) = x[1];
          coeffs_[1].k[3](i, j) = x[2];
          coeffs_[1].k[4](i, j) = x[3];
        }
      }
    }
  }

  struct Coefficients {
    T k[kNumCoefficients];
  };
  Scalar start_t_;
  Scalar middle_t_;
  Scalar end_t_;
  Scalar period_;
  Coefficients coeffs_[kNumSplinePieces];
};

}  // namespace math

#endif  // MATH_QUARTIC_SPLINE_INTERPOLATION_H_
