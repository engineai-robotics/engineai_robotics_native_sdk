#ifndef CORE_MATH_INCLUDE_MATH_INTERPOLATION_H_
#define CORE_MATH_INCLUDE_MATH_INTERPOLATION_H_

#include <type_traits>

namespace Eigen {
template <typename T>
class MatrixBase;
}

namespace math {

// Linearly interpolates from |start| and |end|. t ∈ [0, 1].
// |T| can be Eigen types.
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
T LinearInterpolate(T start, T end, double t) {
  return start + (end - start) * t;
}

template <typename T1, typename T2>
auto LinearInterpolate(const Eigen::MatrixBase<T1>& start, const Eigen::MatrixBase<T2>& end, double t) {
  return start + (end - start) * t;
}

// Interpolates on the line from position `p_start` to `p_end` with start and end velocities `v_start` and `v_end`.
// Returns the position at time `phase` in `period` during the movement.
// `period` must be > 0. `phase` must be in [0, period].
//
// Formulated as:
//   P(t) = a t³ + b t² + c t + d, t ∈ [0, T]
//   where t is `phase`, T is `period`.
// Constraints:
//   P(0) = p_start,
//   P(T) = p_end,
//   P'(0) = v_start,
//   P'(T) = v_end.
// Solved as:
//   a = (2 * P(0) - 2 * P(T) + T * P'(0) + T * P'(T)) / T³,
//   b = (-3 * P(0) + 3 * P(T) -2 * T * P'(0) - T * P'(T)) / T²,
//   c = P'(0),
//   d = P(0).
template <typename T>
void CubicSplineInterpolate(const T& p_start, const T& p_end, const T& v_start, const T& v_end, double period,
                            double phase, T& p) {
  auto a_period3 = 2 * (p_start - p_end) + period * (v_start + v_end);
  auto b_period2 = 3 * (p_end - p_start) - period * (2 * v_start + v_end);
  const T& c = v_start;
  const T& d = p_start;
  double t = phase / period;
  p = (a_period3 * t + b_period2) * (t * t) + c * phase + d;
}

// Interpolates on the line from |start| to |end| with zero velocity at endpoints.
// Returns the position at time |phase| in |period| during the movement.
// |period| must be > 0. |phase| must be in [0, |period|].
//
// Formulated as:
//   P(t) = P_start + (P_end - P_start) * r(t), t ∈ [0, 1],
//   r(t) = a t³ + b t² + c t + d.
// Constraints: r(0) = 0, r(1) = 1, r'(0) = 0, r'(1) = 0.
// Solved as a = -2, b = 3, c = 0, d = 0.
template <typename T>
void CubicInterpolate(const T& start, const T& end, double period, double phase, T& pos) {
  double t = phase / period;
  double r = (-2 * t + 3) * t * t;
  pos = start + (end - start) * r;
}

// Also returns the velocity at time |phase| in |period|.
template <typename T>
void CubicInterpolate(const T& start, const T& end, double period, double phase, T& pos, T& vel) {
  double t = phase / period;
  double r = (-2 * t + 3) * t * t;
  double rd = (-6 * t + 6) * t;
  pos = start + (end - start) * r;
  vel = (end - start) * (rd / period);
}

// Also returns the acceleration.
template <typename T>
void CubicInterpolate(const T& start, const T& end, double period, double phase, T& pos, T& vel, T& acc) {
  double t = phase / period;
  double r = (-2 * t + 3) * t * t;
  double rd = (-6 * t + 6) * t;
  double rdd = -12 * t + 6;
  pos = start + (end - start) * r;
  vel = (end - start) * (rd / period);
  acc = (end - start) * (rdd / period / period);
}

// Interpolates on the line from |start| to |end| with zero velocity and acceleration at endpoints.
// Returns the position at time |phase| in |period| during the movement.
// |period| must be > 0. |phase| must be in [0, |period|].
//
// Formulated as:
//   P(t) = P_start + (P_end - P_start) * r(t), |t| ∈ [0, 1],
//   r(t) = a t⁵ + b t⁴ + c t³ + d t² + e t + f.
// Constraints: r(0) = 0, r(1) = 1, r'(0) = 0, r'(1) = 0, r''(0) = 0, r''(1) = 0.
// Solved as: a = 6, b = -15, c = 10, d = e = f = 0.
template <typename T>
void QuinticInterpolate(const T& start, const T& end, double period, double phase, T& pos) {
  double t = phase / period;
  double r = ((6 * t - 15) * t + 10) * t * t * t;
  pos = start + (end - start) * r;
}

// Also returns the velocity at time |phase| in |period|.
template <typename T>
void QuinticInterpolate(const T& start, const T& end, double period, double phase, T& pos, T& vel) {
  double t = phase / period;
  double r = ((6 * t - 15) * t + 10) * t * t * t;
  double rd = ((30 * t - 60) * t + 30) * t * t;
  pos = start + (end - start) * r;
  vel = (end - start) * (rd / period);
}

// Also returns the acceleration.
template <typename T>
void QuinticInterpolate(const T& start, const T& end, double period, double phase, T& pos, T& vel, T& acc) {
  double t = phase / period;
  double r = ((6 * t - 15) * t + 10) * t * t * t;
  double rd = ((30 * t - 60) * t + 30) * t * t;
  double rdd = ((120 * t - 180) * t + 60) * t;
  pos = start + (end - start) * r;
  vel = (end - start) * (rd / period);
  acc = (end - start) * (rdd / period / period);
}

}  // namespace math
#endif  // CORE_MATH_INCLUDE_MATH_INTERPOLATION_H_
