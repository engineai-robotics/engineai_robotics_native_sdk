#pragma once

#include <Eigen/Core>
#include <string>
#include <type_traits>

#include "math/rotation_matrix.h"

namespace math {

// `T`: can be floating point types, or Eigen::VectorXd.
template <typename T>
class PdController {
 public:
  PdController(const T& kp, const T& kd, const T& u_min, const T& u_max);

  void set_x_reference(const T& value) { x_reference_ = value; }
  void set_x_actual(const T& value) { x_actual_ = value; }
  void set_xd_reference(const T& value) { xd_reference_ = value; }
  void set_xd_actual(const T& value) { xd_actual_ = value; }
  void set_u_reference(const T& value) { u_reference_ = value; }
  void set_kp(const T& value) { kp_ = value; }
  void set_kd(const T& value) { kd_ = value; }
  void set_u_min(const T& value) { u_min_ = value; }
  void set_u_max(const T& value) { u_max_ = value; }

  void Update();

  constexpr int dimensions() const {
    if constexpr (std::is_base_of_v<Eigen::DenseBase<T>, T>) {
      return x_reference_.size();
    } else if constexpr (std::is_floating_point_v<T>) {
      return 1;
    } else {
      static_assert(dependent_false_v<T>, "Unsupported type");
      return 0;
    }
  }

  const T& u_desired() const { return u_desired_; }

 private:
  // For static_assert(false) in an else clause of constexpr if statement.
  template <typename>
  static inline constexpr bool dependent_false_v = false;

  T x_reference_;
  T x_actual_;

  T xd_reference_;
  T xd_actual_;

  T u_reference_;
  T u_desired_;

  T kp_;
  T kd_;

  T u_min_;
  T u_max_;
};

// `T`: can be floating point types.
template <typename T>
class PdControllerRotation {
 public:
  PdControllerRotation(const Eigen::Vector3<T>& kp, const Eigen::Vector3<T>& kd, const Eigen::Vector3<T>& u_min,
                       const Eigen::Vector3<T>& u_max);

  void set_x_reference(const math::RotationMatrix<T>& value) { x_reference_ = value; }
  void set_x_actual(const math::RotationMatrix<T>& value) { x_actual_ = value; }
  void set_xd_reference(const Eigen::Vector3<T>& value) { xd_reference_ = value; }
  void set_xd_actual(const Eigen::Vector3<T>& value) { xd_actual_ = value; }
  void set_u_reference(const Eigen::Vector3<T>& value) { u_reference_ = value; }
  void set_kp(const Eigen::Vector3<T>& value) { kp_ = value; }
  void set_kd(const Eigen::Vector3<T>& value) { kd_ = value; }
  void set_u_min(const Eigen::Vector3<T>& value) { u_min_ = value; }
  void set_u_max(const Eigen::Vector3<T>& value) { u_max_ = value; }

  void Update();

  constexpr int dimensions() const { return 3; }

  Eigen::Vector3<T> u_desired() const { return u_desired_; }

 private:
  math::RotationMatrix<T> x_reference_;
  math::RotationMatrix<T> x_actual_;

  Eigen::Vector3<T> xd_reference_ = Eigen::Vector3<T>::Zero();
  Eigen::Vector3<T> xd_actual_ = Eigen::Vector3<T>::Zero();

  Eigen::Vector3<T> u_reference_ = Eigen::Vector3<T>::Zero();
  Eigen::Vector3<T> u_desired_ = Eigen::Vector3<T>::Zero();

  Eigen::Vector3<T> kp_;
  Eigen::Vector3<T> kd_;

  Eigen::Vector3<T> u_min_;
  Eigen::Vector3<T> u_max_;
};
}  // namespace math
