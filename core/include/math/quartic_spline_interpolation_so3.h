#ifndef MATH_QUARTIC_SPLINE_INTERPOLATION_SO3_H_
#define MATH_QUARTIC_SPLINE_INTERPOLATION_SO3_H_

#include "quartic_spline_interpolation.h"

namespace math {

// Interpolates the SO3 rotation by a quartic spline of two pieces.
template <typename T>
class QuarticSplineInterpolationSO3 {
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
    Eigen::Quaternion<T> y;
    Eigen::Vector3<T> yd;
    Eigen::Vector3<T> ydd;
  };

  QuarticSplineInterpolationSO3(const Knot& start, const Knot& middle, const Knot& end)
      : start_t_(start.t), end_t_(end.t), period_(end.t - start.t) {
    start_q_ = start.y;
    Eigen::AngleAxis<T> delta;
    delta = Eigen::AngleAxis<T>(middle.y * start.y.inverse());
    middle_delta_y_ = delta.axis() * delta.angle();

    delta = Eigen::AngleAxis<T>(end.y * start.y.inverse());
    end_delta_y_ = delta.axis() * delta.angle();

    spline_ = QuarticSplineInterpolation<Eigen::Vector3<T>>({start.t, zero, start.yd, start.ydd},
                                                            {middle.t, middle_delta_y_, middle.yd, middle.ydd},
                                                            {end.t, end_delta_y_, end.yd, end.ydd});
  }
  QuarticSplineInterpolationSO3() = default;

  void Interpolate(Scalar t, Eigen::Quaternion<T>& y) {
    spline_.Interpolate(t, delta_vec_);
    if (delta_vec_.norm() < kEpsilon) {
      delta_q_ = Eigen::Quaternion<T>::Identity();
    } else {
      delta_q_ = Eigen::AngleAxis<T>(delta_vec_.norm(), delta_vec_.normalized());
    }
    y = delta_q_ * start_q_;
  }

  void Interpolate(Scalar t, Eigen::Quaternion<T>& y, Eigen::Vector3<T>& yd) {
    spline_.Interpolate(t, delta_vec_, yd);
    if (delta_vec_.norm() < kEpsilon) {
      delta_q_ = Eigen::Quaternion<T>::Identity();
    } else {
      delta_q_ = Eigen::AngleAxis<T>(delta_vec_.norm(), delta_vec_.normalized());
    }
    y = delta_q_ * start_q_;
  }

  void Interpolate(Scalar t, Eigen::Quaternion<T>& y, Eigen::Vector3<T>& yd, Eigen::Vector3<T>& ydd) {
    spline_.Interpolate(t, delta_vec_, yd, ydd);
    if (delta_vec_.norm() < kEpsilon) {
      delta_q_ = Eigen::Quaternion<T>::Identity();
    } else {
      delta_q_ = Eigen::AngleAxis<T>(delta_vec_.norm(), delta_vec_.normalized());
    }
    y = delta_q_ * start_q_;
  }

  Scalar GetStartTime() { return start_t_; }
  Scalar GetEndTime() { return end_t_; }
  Scalar GetPeriod() { return period_; }

 private:
  static constexpr double kEpsilon = 1e-6;

  Scalar start_t_;
  Scalar end_t_;
  Scalar period_;

  Eigen::Vector3<T> zero = Eigen::Vector3<T>::Zero();
  Eigen::Vector3<T> middle_delta_y_;
  Eigen::Vector3<T> end_delta_y_;
  Eigen::Vector3<T> delta_vec_;
  Eigen::Quaternion<T> delta_q_;
  Eigen::Quaternion<T> start_q_;

  QuarticSplineInterpolation<Eigen::Vector3<T>> spline_;
};
}  // namespace math

#endif  // MATH_QUARTIC_SPLINE_INTERPOLATION_SO3_H_
