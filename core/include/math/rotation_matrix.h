
// Modified from the Drake framework.
// Copyright 2012-2022 Robot Locomotion Group @ CSAIL. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef MATH_ROTATION_MATRIX_H_
#define MATH_ROTATION_MATRIX_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "math/fast_pose_composition_functions.h"
#include "math/roll_pitch_yaw.h"

namespace math {

namespace internal {
// This is used to select a non-initializing constructor for use by
// RigidTransform.
struct DoNotInitializeMemberFields {};
}  // namespace internal

// This class represents a 3x3 rotation matrix between two arbitrary frames
// A and B and helps ensure users create valid rotation matrices.  This class
// relates right-handed orthogonal unit vectors Ax, Ay, Az fixed in frame A
// to right-handed orthogonal unit vectors Bx, By, Bz fixed in frame B.
// The monogram notation for the rotation matrix relating A to B is `R_AB`.
// An example that gives context to this rotation matrix is `v_A = R_AB * v_B`,
// where `v_B` denotes an arbitrary vector v expressed in terms of Bx, By, Bz
// and `v_A` denotes vector v expressed in terms of Ax, Ay, Az.
//
// Note: This class does not store the frames associated with a rotation matrix
// nor does it enforce strict proper usage of this class with vectors.
//
// Note: When assertions are enabled, several methods in this class perform a
// validity check and throw std::exception if the rotation matrix is invalid.
// When assertions are disabled, many of these validity checks are skipped
// (which helps improve speed). These validity tests are only performed for
// scalar types for which scalar_predicate<T>::is_bool is `true`. For
// instance, validity checks are not performed when T is symbolic::Expression.
//
// Author: Paul Mitiguy (2018) Original author.
// Author: Drake team (see https://drake.mit.edu/credits).
template <typename T>
class RotationMatrix {
 public:
  // Constructs a 3x3 identity RotationMatrix -- which corresponds to
  // aligning two frames (so that unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() : R_AB_(Eigen::Matrix3<T>::Identity()) {}

  // Constructs a RotationMatrix from a Matrix3.
  // `R`: an allegedly valid rotation matrix.
  // Throws: std::exception in debug builds if R fails IsValid(R).
  explicit RotationMatrix(const Eigen::Matrix3<T>& R) { set(R); }

  // Constructs a RotationMatrix from an Eigen::Quaternion.
  // `quaternion`: a non-zero, finite quaternion which may or may not have unit
  //   length [i.e., `quaternion.norm()` does not have to be 1].
  // Throws: std::exception in debug builds if the rotation matrix
  //   R that is built from `quaternion` fails IsValid(R).  For example, an
  //   exception is thrown if `quaternion` is zero or contains a NaN or infinity.
  // Note: This method has the effect of normalizing its `quaternion` argument,
  // without the inefficiency of the square-root associated with normalization.
  explicit RotationMatrix(const Eigen::Quaternion<T>& quaternion) {
    // TODO: Although this method is fairly efficient, consider adding
    // an optional second argument if `quaternion` is known to be normalized
    // apriori or for some reason the calling site does not want `quaternion`
    // normalized.
    // Cost for various way to create a rotation matrix from a quaternion.
    // Eigen quaternion.toRotationMatrix() = 12 multiplies, 12 adds.
    // Drake  QuaternionToRotationMatrix() = 12 multiplies, 12 adds.
    // Extra cost for two_over_norm_squared =  4 multiplies,  3 adds, 1 divide.
    // Extra cost if normalized = 4 multiplies, 3 adds, 1 sqrt, 1 divide.
    const T two_over_norm_squared = 2.0 / quaternion.squaredNorm();
    set(QuaternionToRotationMatrix(quaternion, two_over_norm_squared));
  }

  // Constructs a RotationMatrix from an Eigen::AngleAxis.
  // `theta_lambda`: an Eigen::AngleAxis whose associated axis (vector direction
  //   herein called `lambda`) is non-zero and finite, but which may or may not
  //   have unit length [i.e., `lambda.norm()` does not have to be 1].
  // Throws: std::exception in debug builds if the rotation matrix
  //   R that is built from `theta_lambda` fails IsValid(R).  For example, an
  //   exception is thrown if `lambda` is zero or contains a NaN or infinity.
  // (Internal) In general, the RotationMatrix constructed by passing a non-unit
  // `lambda` to this method is different than the RotationMatrix produced by
  // converting `lambda` to an un-normalized quaternion and calling the
  // RotationMatrix constructor (above) with that un-normalized quaternion.
  explicit RotationMatrix(const Eigen::AngleAxis<T>& theta_lambda) {
    // TODO: Consider adding an optional second argument if `lambda` is
    // known to be normalized apriori or calling site does not want
    // normalization.
    const Eigen::Vector3<T>& lambda = theta_lambda.axis();
    const T norm = lambda.norm();
    T theta = theta_lambda.angle();
    set(Eigen::AngleAxis<T>(theta, lambda / norm).toRotationMatrix());
  }

  // Constructs a RotationMatrix from an RollPitchYaw.  In other words,
  // makes the RotationMatrix for a Space-fixed (extrinsic) X-Y-Z rotation by
  // "roll-pitch-yaw" angles `[r, p, y]`, which is equivalent to a Body-fixed
  // (intrinsic) Z-Y-X rotation by "yaw-pitch-roll" angles `[y, p, r]`.
  // `rpy`: radian measures of three angles [roll, pitch, yaw].
  // `rpy`: a RollPitchYaw which is a Space-fixed (extrinsic) X-Y-Z
  //   rotation with "roll-pitch-yaw" angles `[r, p, y]` or equivalently a Body-
  //   fixed (intrinsic) Z-Y-X rotation with "yaw-pitch-roll" angles `[y, p, r]`.
  // Note: Denoting roll `r`, pitch `p`, yaw `y`, this method returns a
  // rotation matrix `R_AD` equal to the matrix multiplication shown below.
  //
  //        ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
  // R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
  //        ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
  //      =       R_AB          *        R_BC          *        R_CD
  //
  // Note: In this discussion, A is the Space frame and D is the Body frame.
  // One way to visualize this rotation sequence is by introducing intermediate
  // frames B and C (useful constructs to understand this rotation sequence).
  // Initially, the frames are aligned so `Di = Ci = Bi = Ai (i = x, y, z)`.
  // Then D is subjected to successive right-handed rotations relative to A.
  // * 1st rotation R_CD: Frame D rotates relative to frames C, B, A by a
  // roll angle `r` about `Dx = Cx`.  Note: D and C are no longer aligned.
  // * 2nd rotation R_BC: Frames D, C (collectively -- as if welded together)
  // rotate relative to frame B, A by a pitch angle `p` about `Cy = By`.
  // Note: C and B are no longer aligned.
  // * 3rd rotation R_AB: Frames D, C, B (collectively -- as if welded)
  // rotate relative to frame A by a roll angle `y` about `Bz = Az`.
  // Note: B and A are no longer aligned.
  // Note: This method constructs a RotationMatrix from a RollPitchYaw.
  // Vice-versa, there are high-accuracy RollPitchYaw constructor/methods that
  // form a RollPitchYaw from a rotation matrix.
  explicit RotationMatrix(const RollPitchYaw<T>& rpy) {
    // TODO: Add publicly viewable documentation on how Sherm and
    // Goldstein like to visualize/conceptualize rotation sequences.
    T r = rpy.roll();
    T p = rpy.pitch();
    T y = rpy.yaw();
    using std::cos;
    using std::sin;
    const T c0 = cos(r), c1 = cos(p), c2 = cos(y);
    const T s0 = sin(r), s1 = sin(p), s2 = sin(y);
    const T c2_s1 = c2 * s1, s2_s1 = s2 * s1;
    const T Rxx = c2 * c1;
    const T Rxy = c2_s1 * s0 - s2 * c0;
    const T Rxz = c2_s1 * c0 + s2 * s0;
    const T Ryx = s2 * c1;
    const T Ryy = s2_s1 * s0 + c2 * c0;
    const T Ryz = s2_s1 * c0 - c2 * s0;
    const T Rzx = -s1;
    const T Rzy = c1 * s0;
    const T Rzz = c1 * c0;
    SetFromOrthonormalRows(Eigen::Vector3<T>(Rxx, Rxy, Rxz), Eigen::Vector3<T>(Ryx, Ryy, Ryz),
                           Eigen::Vector3<T>(Rzx, Rzy, Rzz));
  }

  // (Advanced) Makes the RotationMatrix `R_AB` from right-handed orthogonal
  // unit vectors `Bx`, `By`, `Bz` so the columns of `R_AB` are `[Bx, By, Bz]`.
  // `Bx`: first unit vector in right-handed orthogonal set.
  // `By`: second unit vector in right-handed orthogonal set.
  // `Bz`: third unit vector in right-handed orthogonal set.
  // Throws: std::exception in debug builds if `R_AB` fails IsValid(R_AB).
  // Note: In release builds, the caller can subsequently test if `R_AB` is,
  // in fact, a valid RotationMatrix by calling `R_AB.IsValid()`.
  // Note: The rotation matrix `R_AB` relates two sets of right-handed
  // orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz.
  // The rows of `R_AB` are Ax, Ay, Az expressed in frame B (i.e.,`Ax_B`,
  // `Ay_B`, `Az_B`).  The columns of `R_AB` are Bx, By, Bz expressed in
  // frame A (i.e., `Bx_A`, `By_A`, `Bz_A`).
  static RotationMatrix<T> MakeFromOrthonormalColumns(const Eigen::Vector3<T>& Bx, const Eigen::Vector3<T>& By,
                                                      const Eigen::Vector3<T>& Bz) {
    RotationMatrix<T> R(internal::DoNotInitializeMemberFields{});
    R.SetFromOrthonormalColumns(Bx, By, Bz);
    return R;
  }

  // (Advanced) Makes the RotationMatrix `R_AB` from right-handed orthogonal
  // unit vectors `Ax`, `Ay`, `Az` so the rows of `R_AB` are `[Ax, Ay, Az]`.
  // `Ax`: first unit vector in right-handed orthogonal set.
  // `Ay`: second unit vector in right-handed orthogonal set.
  // `Az`: third unit vector in right-handed orthogonal set.
  // Throws: std::exception in debug builds if `R_AB` fails IsValid(R_AB).
  // Note: In release builds, the caller can subsequently test if `R_AB` is,
  // in fact, a valid RotationMatrix by calling `R_AB.IsValid()`.
  // Note: The rotation matrix `R_AB` relates two sets of right-handed
  // orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz.
  // The rows of `R_AB` are Ax, Ay, Az expressed in frame B (i.e.,`Ax_B`,
  // `Ay_B`, `Az_B`).  The columns of `R_AB` are Bx, By, Bz expressed in
  // frame A (i.e., `Bx_A`, `By_A`, `Bz_A`).
  static RotationMatrix<T> MakeFromOrthonormalRows(const Eigen::Vector3<T>& Ax, const Eigen::Vector3<T>& Ay,
                                                   const Eigen::Vector3<T>& Az) {
    RotationMatrix<T> R(internal::DoNotInitializeMemberFields{});
    R.SetFromOrthonormalRows(Ax, Ay, Az);
    return R;
  }

  // Makes the RotationMatrix `R_AB` associated with rotating a frame B
  // relative to a frame A by an angle `theta` about unit vector `Ax = Bx`.
  // `theta`: radian measure of rotation angle about Ax.
  // Note: Orientation is same as Eigen::AngleAxis<T>(theta, Eigen::Vector3d::UnitX().
  // Note: `R_AB` relates two frames A and B having unit vectors Ax, Ay, Az and
  // Bx, By, Bz.  Initially, `Bx = Ax`, `By = Ay`, `Bz = Az`, then B undergoes
  // a right-handed rotation relative to A by an angle `theta` about `Ax = Bx`.
  //
  //        ⎡ 1       0                 0  ⎤
  // R_AB = ⎢ 0   cos(theta)   -sin(theta) ⎥
  //        ⎣ 0   sin(theta)    cos(theta) ⎦
  //
  static RotationMatrix<T> MakeXRotation(T theta) {
    using std::cos;
    using std::sin;
    const T c = cos(theta), s = sin(theta);
    Eigen::Matrix3<T> R = Eigen::Matrix3<T>{
        {1, 0, 0},
        {0, c, -s},
        {0, s, c},
    };
    return RotationMatrix(R);
  }

  // Makes the RotationMatrix `R_AB` associated with rotating a frame B
  // relative to a frame A by an angle `theta` about unit vector `Ay = By`.
  // `theta`: radian measure of rotation angle about Ay.
  // Note: Orientation is same as Eigen::AngleAxis<T>(theta, Eigen::Vector3d::UnitY().
  // Note: `R_AB` relates two frames A and B having unit vectors Ax, Ay, Az and
  // Bx, By, Bz.  Initially, `Bx = Ax`, `By = Ay`, `Bz = Az`, then B undergoes
  // a right-handed rotation relative to A by an angle `theta` about `Ay = By`.
  //
  //        ⎡  cos(theta)   0   sin(theta) ⎤
  // R_AB = ⎢          0    1           0  ⎥
  //        ⎣ -sin(theta)   0   cos(theta) ⎦
  //
  static RotationMatrix<T> MakeYRotation(T theta) {
    using std::cos;
    using std::sin;
    const T c = cos(theta), s = sin(theta);
    Eigen::Matrix3<T> R = Eigen::Matrix3<T>{
        {c, 0, s},
        {0, 1, 0},
        {-s, 0, c},
    };
    return RotationMatrix(R);
  }

  // Makes the RotationMatrix `R_AB` associated with rotating a frame B
  // relative to a frame A by an angle `theta` about unit vector `Az = Bz`.
  // `theta`: radian measure of rotation angle about Az.
  // Note: Orientation is same as Eigen::AngleAxis<T>(theta, Eigen::Vector3d::UnitZ().
  // Note: `R_AB` relates two frames A and B having unit vectors Ax, Ay, Az and
  // Bx, By, Bz.  Initially, `Bx = Ax`, `By = Ay`, `Bz = Az`, then B undergoes
  // a right-handed rotation relative to A by an angle `theta` about `Az = Bz`.
  //
  //        ⎡ cos(theta)  -sin(theta)   0 ⎤
  // R_AB = ⎢ sin(theta)   cos(theta)   0 ⎥
  //        ⎣         0            0    1 ⎦
  //
  static RotationMatrix<T> MakeZRotation(T theta) {
    using std::cos;
    using std::sin;
    const T c = cos(theta), s = sin(theta);
    Eigen::Matrix3<T> R = Eigen::Matrix3<T>{
        {c, -s, 0},
        {s, c, 0},
        {0, 0, 1},
    };
    return RotationMatrix(R);
  }

  // Creates a 3D right-handed orthonormal basis B from a given vector b_A,
  // returned as a rotation matrix R_AB. It consists of orthogonal unit vectors
  // u_A, v_A, w_A where u_A is the normalized b_A in the axis_index column of
  // R_AB and v_A has one element which is zero.  If an element of b_A is zero,
  // then one element of w_A is 1 and the other two elements are 0.
  // `b_A`: vector expressed in frame A that when normalized as
  //   u_A = b_A.normalized() represents Bx, By, or Bz (depending on axis_index).
  // `axis_index`: The index ∈ {0, 1, 2} of the unit vector associated
  //   with u_A, 0 means u_A is Bx, 1 means u_A is By, and 2 means u_A is Bz.
  // Throws: std::exception if b_A cannot be made into a unit vector because
  //   b_A contains a NaN or infinity or |b_A| < 1.0e-10.
  static RotationMatrix<T> MakeFromOneVector(const Eigen::Vector3<T>& b_A, int axis_index) {
    const Eigen::Vector3<T> u_A = NormalizeOrThrow(b_A, __func__);
    return MakeFromOneUnitVector(u_A, axis_index);
  }

  // Creates a 3D right-handed orthonormal basis B from a given vector b_A,
  // returned as a rotation matrix R_AB. It consists of orthogonal unit vectors
  // u_A, v_A, w_A where u_A is the normalized b_A as z axis in the end column of R_AB.
  static RotationMatrix<T> MakeFromOneVectorAsZAxis(const Eigen::Vector3<T>& b_A) {
    const Eigen::Vector3<T> u_A = NormalizeOrThrow(b_A, __func__);
    return MakeFromOneUnitVectorAsZAxis(u_A);
  }

  // (Advanced) Creates a right-handed orthonormal basis B from a given
  // unit vector u_A, returned as a rotation matrix R_AB.
  // `u_A`: unit vector which is expressed in frame A and is either
  //   Bx or By or Bz (depending on the value of axis_index).
  // `axis_index`: The index ∈ {0, 1, 2} of the unit vector associated
  //   with u_A, 0 means u_A is Bx, 1 means u_A is By, and 2 means u_A is Bz.
  // Throws: std::exception in Debug builds if u_A is not a unit vector, i.e.,
  //   |u_A| is not within a tolerance of 4ε ≈ 8.88e-16 to 1.0.
  // Note: This method is designed for maximum performance and does not verify
  // u_A as a unit vector in Release builds.  Consider MakeFromOneVector().
  static RotationMatrix<T> MakeFromOneUnitVector(const Eigen::Vector3<T>& u_A, int axis_index);

  // Creates a right-handed orthonormal basis B from a given
  // unit vector u_A as z axis, returned as a rotation matrix R_AB.
  // `u_A`: unit vector of Bz which is expressed in frame A.
  // Throws: std::exception in Debug builds if u_A is not a unit vector, i.e.,
  //   |u_A| is not within a tolerance of 4ε ≈ 8.88e-16 to 1.0.
  static RotationMatrix<T> MakeFromOneUnitVectorAsZAxis(const Eigen::Vector3<T>& u_A);

  // Creates a RotationMatrix templatized on a scalar type U from a
  // RotationMatrix templatized on scalar type T.  For example,
  //
  //   RotationMatrix<double> source = RotationMatrix<double>::Identity();
  //
  // `U`: Scalar type on which the returned RotationMatrix is templated.
  // Note: `RotationMatrix<From>::cast<To>()` creates a new
  // `RotationMatrix<To>` from a `RotationMatrix<From>` but only if
  // type `To` is constructible from type `From`.
  // This cast method works in accordance with Eigen's cast method for Eigen's
  // Matrix3 that underlies this RotationMatrix.  For example, Eigen
  // currently allows cast from type double to AutoDiffXd, but not vice-versa.
  template <typename U>
  RotationMatrix<U> cast() const {
    // TODO: Make the RotationMatrix::cast() method more robust.  It is
    // currently limited by Eigen's cast() for the matrix underlying this class.
    // Consider the following logic to improve casts (and address issue #11785).
    // 1. If relevant, use Eigen's underlying cast method.
    // 2. Strip derivative data when casting from `<AutoDiffXd>` to `<double>`.
    // 3. Call ExtractDoubleOrThrow() when casting from `<symbolic::Expression>`
    //    to `<double>`.
    // 4. The current RotationMatrix::cast() method incurs overhead due to its
    //    underlying call to a RotationMatrix constructor. Perhaps create
    //    specialized code to return a reference if casting to the same type,
    //    e.g., casting from `<double>` to `<double>' should be inexpensive.
    const Eigen::Matrix3<U> m = R_AB_.template cast<U>();
    return RotationMatrix<U>(m, true);
  }

  // Sets `this` RotationMatrix from a Matrix3.
  // `R`: an allegedly valid rotation matrix.
  // Throws: std::exception in debug builds if R fails IsValid(R).
  void set(const Eigen::Matrix3<T>& R) {
    // if constexpr (DCHECK_IS_ON()) {
    //   ThrowIfNotValid(R);
    // }
    SetUnchecked(R);
  }

  // Returns the 3x3 identity RotationMatrix.
  // (Internal) This method's name was chosen to mimic Eigen's Identity().
  static const RotationMatrix<T>& Identity() {
    // TODO: Use placement new to avoid memory leak checking.
    static const RotationMatrix<T>* kIdentity = new RotationMatrix<T>();
    return *kIdentity;
  }

  // Returns `R_BA = R_AB⁻¹`, the inverse (transpose) of this RotationMatrix.
  // Note: For a valid rotation matrix `R_BA = R_AB⁻¹ = R_ABᵀ`.
  // (Internal) This method's name was chosen to mimic Eigen's inverse().
  RotationMatrix<T> inverse() const { return RotationMatrix<T>(R_AB_.transpose()); }

  // Returns `R_BA = R_AB⁻¹`, the transpose of this RotationMatrix.
  // Note: For a valid rotation matrix `R_BA = R_AB⁻¹ = R_ABᵀ`.
  // (Internal) This method's name was chosen to mimic Eigen's transpose().
  RotationMatrix<T> transpose() const { return RotationMatrix<T>(R_AB_.transpose()); }

  // Returns the Matrix3 underlying a RotationMatrix.
  const Eigen::Matrix3<T>& matrix() const { return R_AB_; }

  // Returns `this` rotation matrix's iᵗʰ row (i = 0, 1, 2).
  // For `this` rotation matrix R_AB (which relates right-handed
  // sets of orthogonal unit vectors Ax, Ay, Az to Bx, By, Bz),
  // - row(0) returns Ax_B (Ax expressed in terms of Bx, By, Bz).
  // - row(1) returns Ay_B (Ay expressed in terms of Bx, By, Bz).
  // - row(2) returns Az_B (Az expressed in terms of Bx, By, Bz).
  // `index`: requested row index (0 <= index <= 2).
  // Throws: In Debug builds, asserts (0 <= index <= 2).
  // Note: For efficiency and consistency with Eigen, this method returns
  // the same quantity returned by Eigen's row() operator.
  // The returned quantity can be assigned in various ways, e.g., as
  // `const auto& Az_B = row(2);` or `RowEigen::Vector3<T> Az_B = row(2);`
  const Eigen::Block<const Eigen::Matrix3<T>, 1, 3, false> row(int index) const {
    // The returned value from this method mimics Eigen's row() method which was
    // found in  Eigen/src/plugins/BlockMethods.h.  The Eigen Matrix3 R_AB_ that
    // underlies this class is a column major matrix.  To return a row,
    // InnerPanel = false is passed as the last template parameter above.
    // DCHECK(0 <= index && index <= 2) << index;
    return R_AB_.row(index);
  }

  // Returns `this` rotation matrix's iᵗʰ column (i = 0, 1, 2).
  // For `this` rotation matrix R_AB (which relates right-handed
  // sets of orthogonal unit vectors Ax, Ay, Az to Bx, By, Bz),
  // - col(0) returns Bx_A (Bx expressed in terms of Ax, Ay, Az).
  // - col(1) returns By_A (By expressed in terms of Ax, Ay, Az).
  // - col(2) returns Bz_A (Bz expressed in terms of Ax, Ay, Az).
  // `index`: requested column index (0 <= index <= 2).
  // Throws: In Debug builds, asserts (0 <= index <= 2).
  // Note: For efficiency and consistency with Eigen, this method returns
  // the same quantity returned by Eigen's col() operator.
  // The returned quantity can be assigned in various ways, e.g., as
  // `const auto& Bz_A = col(2);` or `Eigen::Vector3<T> Bz_A = col(2);`
  const Eigen::Block<const Eigen::Matrix3<T>, 3, 1, true> col(int index) const {
    // The returned value from this method mimics Eigen's col() method which was
    // found in  Eigen/src/plugins/BlockMethods.h.  The Eigen Matrix3 R_AB_ that
    // underlies this class is a column major matrix.  To return a column,
    // InnerPanel = true is passed as the last template parameter above.
    // DCHECK(0 <= index && index <= 2) << index;
    return R_AB_.col(index);
  }

  // In-place multiply of `this` rotation matrix `R_AB` by `other` rotation
  // matrix `R_BC`.  On return, `this` is set to equal `R_AB * R_BC`.
  // Returns: `this` rotation matrix which has been multiplied by `other`.
  // Note: It is possible (albeit improbable) to create an invalid rotation
  // matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T>& operator*=(const RotationMatrix<T>& other) {
    if constexpr (std::is_same_v<T, double>) {
      internal::ComposeRR(*this, other, this);
    } else {
      SetUnchecked(matrix() * other.matrix());
    }
    return *this;
  }

  // Calculates `this` rotation matrix `R_AB` multiplied by `other` rotation
  // matrix `R_BC`, returning the composition `R_AB * R_BC`.
  // Returns: rotation matrix that results from `this` multiplied by `other`.
  // Note: It is possible (albeit improbable) to create an invalid rotation
  // matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T> operator*(const RotationMatrix<T>& other) const {
    RotationMatrix<T> R_AC(internal::DoNotInitializeMemberFields{});
    if constexpr (std::is_same_v<T, double>) {
      internal::ComposeRR(*this, other, &R_AC);
    } else {
      R_AC.R_AB_ = matrix() * other.matrix();
    }
    return R_AC;
  }

  // Calculates the product of `this` inverted and another RotationMatrix.
  // If you consider `this` to be the rotation matrix R_AB, and `other` to be
  // R_AC, then this method returns R_BC = R_AB⁻¹ * R_AC. For T==double, this
  // method can be _much_ faster than inverting first and then performing the
  // composition because it can take advantage of the orthogonality of
  // rotation matrices. On some platforms it can use SIMD instructions for
  // further speedups.
  // Returns: R_BC where R_BC = this⁻¹ * other.
  // Note: It is possible (albeit improbable) to create an invalid rotation
  // matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T> InvertAndCompose(const RotationMatrix<T>& other) const {
    const RotationMatrix<T>& R_AC = other;  // Nicer name.
    RotationMatrix<T> R_BC(internal::DoNotInitializeMemberFields{});
    if constexpr (std::is_same_v<T, double>) {
      internal::ComposeRinvR(*this, R_AC, &R_BC);
    } else {
      const RotationMatrix<T> R_BA = inverse();
      R_BC = R_BA * R_AC;
    }
    return R_BC;
  }

  // Calculates `this` rotation matrix `R_AB` multiplied by an arbitrary
  // Eigen::Vector3 expressed in the B frame.
  // `v_B`: 3x1 vector that post-multiplies `this`.
  // Returns: 3x1 vector `v_A = R_AB * v_B`.
  Eigen::Vector3<T> operator*(const Eigen::Vector3<T>& v_B) const { return Eigen::Vector3<T>(matrix() * v_B); }

  // Multiplies `this` RotationMatrix `R_AB` by the n vectors `v1`, ... `vn`,
  // where each vector has 3 elements and is expressed in frame B.
  // `v_B`: `3 x n` matrix whose n columns are regarded as arbitrary
  //   vectors `v1`, ... `vn` expressed in frame B.
  // Returns: v_A `3 x n` matrix whose n columns are vectors `v1`, ... `vn`
  //   expressed in frame A.
  //
  //   const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  //   const RotationMatrix<double> R_AB(rpy);
  //   Eigen::Matrix<double, 3, 2> v_B;
  //   v_B.col(0) = Eigen::Vector3d(4, 5, 6);
  //   v_B.col(1) = Eigen::Vector3d(9, 8, 7);
  //   const Eigen::Matrix<double, 3, 2> v_A = R_AB * v_B;
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime> operator*(
      const Eigen::MatrixBase<Derived>& v_B) const {
    if (v_B.rows() != 3) {
      throw std::logic_error("Error: Inner dimension for matrix multiplication is not 3.");
    }
    // Express vectors in terms of frame A as v_A = R_AB * v_B.
    return matrix() * v_B;
  }

  // Returns how close the matrix R is to being a 3x3 orthonormal matrix by
  // computing `‖R ⋅ Rᵀ - I‖∞` (i.e., the maximum absolute value of the
  // difference between the elements of R ⋅ Rᵀ and the 3x3 identity matrix).
  // `R`: matrix being checked for orthonormality.
  // Returns: `‖R ⋅ Rᵀ - I‖∞`
  static T GetMeasureOfOrthonormality(const Eigen::Matrix3<T>& R) {
    const Eigen::Matrix3<T> m = R * R.transpose();
    return GetMaximumAbsoluteDifference(m, Eigen::Matrix3<T>::Identity());
  }

  // Tests if a generic Matrix3 has orthonormal vectors to within the threshold
  // specified by `tolerance`.
  // `R`: an allegedly orthonormal rotation matrix.
  // `tolerance`: maximum allowable absolute difference between R * Rᵀ
  // and the identity matrix I, i.e., checks if `‖R ⋅ Rᵀ - I‖∞ <= tolerance`.
  // Returns: `true` if R is an orthonormal matrix.
  static bool IsOrthonormal(const Eigen::Matrix3<T>& R, double tolerance) {
    return GetMeasureOfOrthonormality(R) <= tolerance;
  }

  // Tests if a generic Matrix3 seems to be a proper orthonormal rotation
  // matrix to within the threshold specified by `tolerance`.
  // `R`: an allegedly valid rotation matrix.
  // `tolerance`: maximum allowable absolute difference of `R * Rᵀ`
  // and the identity matrix I (i.e., checks if `‖R ⋅ Rᵀ - I‖∞ <= tolerance`).
  // Returns: `true` if R is a valid rotation matrix.
  static bool IsValid(const Eigen::Matrix3<T>& R, double tolerance) {
    return IsOrthonormal(R, tolerance) && R.determinant() > 0;
  }

  // Tests if a generic Matrix3 is a proper orthonormal rotation matrix to
  // within the threshold of get_internal_tolerance_for_orthonormality().
  // `R`: an allegedly valid rotation matrix.
  // Returns: `true` if R is a valid rotation matrix.
  static bool IsValid(const Eigen::Matrix3<T>& R) { return IsValid(R, get_internal_tolerance_for_orthonormality()); }

  // Tests if `this` rotation matrix R is a proper orthonormal rotation matrix
  // to within the threshold of get_internal_tolerance_for_orthonormality().
  // Returns: `true` if `this` is a valid rotation matrix.
  bool IsValid() const { return IsValid(matrix()); }

  // Returns `true` if `this` is exactly equal to the identity matrix.
  bool IsExactlyIdentity() const { return matrix() == Eigen::Matrix3<T>::Identity(); }

  // Returns true if `this` is within tolerance of the identity RigidTransform.
  // `tolerance`: non-negative number that is generally the default
  //   value, namely RotationMatrix::get_internal_tolerance_for_orthonormality().
  bool IsNearlyIdentity(double tolerance = get_internal_tolerance_for_orthonormality()) const {
    return IsNearlyEqualTo(matrix(), Eigen::Matrix3<T>::Identity(), tolerance);
  }

  // Compares each element of `this` to the corresponding element of `other`
  // to check if they are the same to within a specified `tolerance`.
  // `tolerance`: maximum allowable absolute difference between the
  //   matrix elements in `this` and `other`.
  // Returns: `true` if `‖this - other‖∞ <= tolerance`.
  bool IsNearlyEqualTo(const RotationMatrix<T>& other, double tolerance) const {
    return IsNearlyEqualTo(matrix(), other.matrix(), tolerance);
  }

  // Compares each element of `this` to the corresponding element of `other`
  // to check if they are exactly the same.
  // Returns: true if each element of `this` is exactly equal to the
  //   corresponding element in `other`.
  bool IsExactlyEqualTo(const RotationMatrix<T>& other) const { return matrix() == other.matrix(); }

  // Computes the infinity norm of `this` - `other` (i.e., the maximum absolute
  // value of the difference between the elements of `this` and `other`).
  // Returns: `‖this - other‖∞`
  T GetMaximumAbsoluteDifference(const RotationMatrix<T>& other) const {
    return GetMaximumAbsoluteDifference(matrix(), other.matrix());
  }

  // Given an approximate rotation matrix M, finds the RotationMatrix R
  // closest to M.  Closeness is measured with a matrix-2 norm (or equivalently
  // with a Frobenius norm).  Hence, this method creates a RotationMatrix R
  // from a 3x3 matrix M by minimizing `‖R - M‖₂` (the matrix-2 norm of (R-M))
  // subject to `R * Rᵀ = I`, where I is the 3x3 identity matrix.  For this
  // problem, closeness can also be measured by forming the orthonormal matrix
  // R whose elements minimize the double-summation `∑ᵢ ∑ⱼ (R(i,j) - M(i,j))²`
  // where `i = 1:3, j = 1:3`, subject to `R * Rᵀ = I`.  The square-root of
  // this double-summation is called the Frobenius norm.
  // `quality_factor`: The quality of M as a rotation matrix.
  //   `quality_factor` = 1 is perfect (M = R). `quality_factor` = 1.25 means
  //   that when M multiplies a unit vector (magnitude 1), a vector of magnitude
  //   as large as 1.25 may result.  `quality_factor` = 0.8 means that when M
  //   multiplies a unit vector, a vector of magnitude as small as 0.8 may
  //   result.  `quality_factor` = 0 means M is singular, so at least one of the
  //   bases related by matrix M does not span 3D space (when M multiples a unit
  //   vector, a vector of magnitude as small as 0 may result).
  // Returns: proper orthonormal matrix R that is closest to M.
  // Throws: std::exception if R fails IsValid(R).
  // Note: William Kahan (UC Berkeley) and Hongkai Dai (Toyota Research
  // Institute) proved that for this problem, the same R that minimizes the
  // Frobenius norm also minimizes the matrix-2 norm (a.k.a an induced-2 norm),
  // which is defined [Dahleh, Section 4.2] as the column matrix u which
  // maximizes `‖(R - M) u‖ / ‖u‖`, where `u ≠ 0`.  Since the matrix-2 norm of
  // any matrix A is equal to the maximum singular value of A, minimizing the
  // matrix-2 norm of (R - M) is equivalent to minimizing the maximum singular
  // value of (R - M).
  //
  // - [Dahleh] "Lectures on Dynamic Systems and Controls: Electrical
  // Engineering and Computer Science, Massachusetts Institute of Technology"
  // https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-241j-dynamic-systems-and-control-spring-2011/readings/MIT6_241JS11_chap04.pdf
  static RotationMatrix<T> ProjectToRotationMatrix(const Eigen::Matrix3<T>& M, T* quality_factor = nullptr) {
    const Eigen::Matrix3<T> M_orthonormalized = ProjectMatrix3ToOrthonormalMatrix3(M, quality_factor);
    ThrowIfNotValid(M_orthonormalized);
    return RotationMatrix<T>(M_orthonormalized, true);
  }

  // Returns an internal tolerance that checks rotation matrix orthonormality.
  // Returns: internal tolerance (small multiplier of double-precision epsilon)
  //   used to check whether or not a rotation matrix is orthonormal.
  // Note: The tolerance is chosen by developers to ensure a reasonably
  // valid (orthonormal) rotation matrix.
  // Note: To orthonormalize a 3x3 matrix, use ProjectToRotationMatrix().
  static constexpr double get_internal_tolerance_for_orthonormality() { return kInternalToleranceForOrthonormality; }

  // Returns a quaternion q that represents `this` RotationMatrix.  Since the
  // quaternion `q` and `-q` represent the same RotationMatrix, this method
  // chooses to return a canonical quaternion, i.e., with q(0) >= 0.
  // Note: There is a constructor in the RollPitchYaw class that converts
  // a rotation matrix to roll-pitch-yaw angles.
  Eigen::Quaternion<T> ToQuaternion() const { return ToQuaternion(R_AB_); }

  // Returns a unit quaternion q associated with the 3x3 matrix M.  Since the
  // quaternion `q` and `-q` represent the same RotationMatrix, this method
  // chooses to return a canonical quaternion, i.e., with q(0) >= 0.
  // Returns: a unit quaternion q in canonical form, i.e., with q(0) >= 0.
  // Throws: std::exception in debug builds if the quaternion `q`
  // returned by this method cannot construct a valid RotationMatrix.
  // For example, if `M` contains NaNs, `q` will not be a valid quaternion.
  static Eigen::Quaternion<T> ToQuaternion(const Eigen::Ref<const Eigen::Matrix3<T>>& M) {
    Eigen::Quaternion<T> q = RotationMatrixToUnnormalizedQuaternion(M);

    // Since the quaternions q and -q correspond to the same rotation matrix,
    // choose to return a canonical quaternion, i.e., with q(0) >= 0.
    const T canonical_factor = (q.w() < 0 ? T(-1) : T(1));

    // The quantity q calculated thus far in this algorithm is not a quaternion
    // with magnitude 1.  It differs from a quaternion in that all elements of
    // q are scaled by the same factor. To return a valid quaternion, q must be
    // normalized so q(0)^2 + q(1)^2 + q(2)^2 + q(3)^2 = 1.
    const T scale = canonical_factor / q.norm();
    q.coeffs() *= scale;

    // if constexpr (DCHECK_IS_ON()) {
    //   ThrowIfNotValid(QuaternionToRotationMatrix(q, T(2)));
    // }
    return q;
  }

  // Utility method to return the Vector4 associated with ToQuaternion().
  Eigen::Vector4<T> ToQuaternionAsVector4() const { return ToQuaternionAsVector4(R_AB_); }

  // Utility method to return the Vector4 associated with ToQuaternion(M).
  static Eigen::Vector4<T> ToQuaternionAsVector4(const Eigen::Matrix3<T>& M) {
    const Eigen::Quaternion<T> q = ToQuaternion(M);
    return Eigen::Vector4<T>(q.w(), q.x(), q.y(), q.z());
  }

  // Returns an AngleAxis `theta_lambda` containing an angle `theta` and unit
  // vector (axis direction) `lambda` that represents `this` RotationMatrix.
  // Note: The orientation and RotationMatrix associated with `theta * lambda`
  // is identical to that of `(-theta) * (-lambda)`.  The AngleAxis returned by
  // this method chooses to have `0 <= theta <= pi`.
  // Returns: an AngleAxis with `0 <= theta <= pi` and a unit vector `lambda`.
  Eigen::AngleAxis<T> ToAngleAxis() const {
    const Eigen::AngleAxis<T> theta_lambda(this->matrix());
    return theta_lambda;
  }

  Eigen::Vector3<T> ToAngleAxisAsVector3() const {
    const Eigen::AngleAxis<T> theta_lambda(this->matrix());
    return theta_lambda.angle() * theta_lambda.axis();
  }

  // (Internal use only) Constructs a RotationMatrix without initializing the
  // underlying 3x3 matrix. Here for use by RigidTransform but no one else.
  explicit RotationMatrix(internal::DoNotInitializeMemberFields) {}

 private:
  // Makes RotationMatrix<U> templatized on any typename U be a friend of a
  // RotationMatrix templatized on any other typename T.
  // This is needed for the method RotationMatrix<T>::cast<U>() to be able to
  // use the necessary private constructor.
  template <typename U>
  friend class RotationMatrix;

  // Declares the allowable tolerance (small multiplier of double-precision
  // epsilon) used to check whether or not a rotation matrix is orthonormal.
  static constexpr double kInternalToleranceForOrthonormality{128 * std::numeric_limits<double>::epsilon()};

  // Constructs a RotationMatrix from a Matrix3.  No check is performed to test
  // whether or not the parameter R is a valid rotation matrix.
  // `R`: an allegedly valid rotation matrix.
  // Note: The second parameter is just a dummy to distinguish this constructor
  // from one of the public constructors.
  RotationMatrix(const Eigen::Matrix3<T>& R, bool) : R_AB_(R) {}

  // Sets `this` RotationMatrix from a Matrix3.  No check is performed to
  // test whether or not the parameter R is a valid rotation matrix.
  // `R`: an allegedly valid rotation matrix.
  void SetUnchecked(const Eigen::Matrix3<T>& R) { R_AB_ = R; }

  // Sets `this` RotationMatrix `R_AB` from right-handed orthogonal unit
  // vectors `Bx`, `By`, `Bz` so that the columns of `this` are `[Bx, By, Bz]`.
  // `Bx`: first unit vector in right-handed orthogonal basis.
  // `By`: second unit vector in right-handed orthogonal basis.
  // `Bz`: third unit vector in right-handed orthogonal basis.
  // Throws: std::exception in debug builds if `R_AB` fails IsValid(R_AB).
  // Note: The rotation matrix `R_AB` relates two sets of right-handed
  // orthogonal unit vectors, namely `Ax`, `Ay`, `Az` and `Bx`, `By`, `Bz`.
  // The rows of `R_AB` are `Ax`, `Ay`, `Az` whereas the
  // columns of `R_AB` are `Bx`, `By`, `Bz`.
  void SetFromOrthonormalColumns(const Eigen::Vector3<T>& Bx, const Eigen::Vector3<T>& By,
                                 const Eigen::Vector3<T>& Bz) {
    R_AB_.col(0) = Bx;
    R_AB_.col(1) = By;
    R_AB_.col(2) = Bz;
    // if constexpr (DCHECK_IS_ON()) {
    //   ThrowIfNotValid(R_AB_);
    // }
  }

  // Sets `this` RotationMatrix `R_AB` from right-handed orthogonal unit
  // vectors `Ax`, `Ay`, `Az` so that the rows of `this` are `[Ax, Ay, Az]`.
  // `Ax`: first unit vector in right-handed orthogonal basis.
  // `Ay`: second unit vector in right-handed orthogonal basis.
  // `Az`: third unit vector in right-handed orthogonal basis.
  // Throws: std::exception in debug builds if `R_AB` fails R_AB.IsValid().
  void SetFromOrthonormalRows(const Eigen::Vector3<T>& Ax, const Eigen::Vector3<T>& Ay, const Eigen::Vector3<T>& Az) {
    R_AB_.row(0) = Ax;
    R_AB_.row(1) = Ay;
    R_AB_.row(2) = Az;
    // if constexpr (DCHECK_IS_ON()) {
    //   ThrowIfNotValid(R_AB_);
    // }
  }

  // Computes the infinity norm of R - `other` (i.e., the maximum absolute
  // value of the difference between the elements of R and `other`).
  // Returns: `‖R - other‖∞`
  static T GetMaximumAbsoluteDifference(const Eigen::Matrix3<T>& R, const Eigen::Matrix3<T>& other) {
    const Eigen::Matrix3<T> R_difference = R - other;
    return R_difference.template lpNorm<Eigen::Infinity>();
  }

  // Compares corresponding elements in two matrices to check if they are the
  // same to within a specified `tolerance`.
  // matrix elements in R and `other`.
  // Returns: `true` if `‖R - `other`‖∞ <= tolerance`.
  static bool IsNearlyEqualTo(const Eigen::Matrix3<T>& R, const Eigen::Matrix3<T>& other, double tolerance) {
    const T R_max_difference = GetMaximumAbsoluteDifference(R, other);
    return R_max_difference <= tolerance;
  }

  // Throws an exception if R is not a valid RotationMatrix.
  // `R`: an allegedly valid rotation matrix.
  // Note: If the underlying scalar type T is non-numeric (symbolic), no
  // validity check is made and no exception is thrown.
  static void ThrowIfNotValid(const Eigen::Matrix3<T>& R);

  // Given an approximate rotation matrix M, finds the orthonormal matrix R
  // closest to M.  Closeness is measured with a matrix-2 norm (or equivalently
  // with a Frobenius norm).  Hence, this method creates an orthonormal matrix R
  // from a 3x3 matrix M by minimizing `‖R - M‖₂` (the matrix-2 norm of (R-M))
  // subject to `R * Rᵀ = I`, where I is the 3x3 identity matrix.  For this
  // problem, closeness can also be measured by forming the orthonormal matrix R
  // whose elements minimize the double-summation `∑ᵢ ∑ⱼ (R(i,j) - M(i,j))²`
  // where `i = 1:3, j = 1:3`, subject to `R * Rᵀ = I`.  The square-root of
  // this double-summation is called the Frobenius norm.
  // `quality_factor`: The quality of M as a rotation matrix.
  //   `quality_factor` = 1 is perfect (M = R). `quality_factor` = 1.25 means
  //   that when M multiplies a unit vector (magnitude 1), a vector of magnitude
  //   as large as 1.25 may result.  `quality_factor` = -1 means M relates two
  //   perfectly orthonormal bases, but one is right-handed whereas the other is
  //   left-handed (M is a "reflection").  `quality_factor` = -0.8 means M
  //   relates a right-handed basis to a left-handed basis and when M multiplies
  //   a unit vector, a vector of magnitude as small as 0.8 may result.
  //   `quality_factor` = 0 means M is singular, so at least one of the bases
  //   related by matrix M does not span 3D space (when M multiples a unit vector,
  //   a vector of magnitude as small as 0 may result).
  // Returns: orthonormal matrix R that is closest to M (note det(R) may be -1).
  // Note: The SVD part of this algorithm can be derived as a modification of
  // section 3.2 in http://haralick.org/conferences/pose_estimation.pdf.
  // Note: William Kahan (UC Berkeley) and Hongkai Dai (Toyota Research
  // Institute) proved that for this problem, the same R that minimizes the
  // Frobenius norm also minimizes the matrix-2 norm (a.k.a an induced-2 norm),
  // which is defined [Dahleh, Section 4.2] as the column matrix u which
  // maximizes `‖(R - M) u‖ / ‖u‖`, where `u ≠ 0`.  Since the matrix-2 norm of
  // any matrix A is equal to the maximum singular value of A, minimizing the
  // matrix-2 norm of (R - M) is equivalent to minimizing the maximum singular
  // value of (R - M).
  //
  // - [Dahleh] "Lectures on Dynamic Systems and Controls: Electrical
  // Engineering and Computer Science, Massachusetts Institute of Technology"
  // https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-241j-dynamic-systems-and-control-spring-2011/readings/MIT6_241JS11_chap04.pdf
  static Eigen::Matrix3<T> ProjectMatrix3ToOrthonormalMatrix3(const Eigen::Matrix3<T>& M, T* quality_factor);

  // This is a helper method for RotationMatrix::ToQuaternion that returns a
  // Quaternion that is neither sign-canonicalized nor magnitude-normalized.
  static Eigen::Quaternion<T> RotationMatrixToUnnormalizedQuaternion(const Eigen::Ref<const Eigen::Matrix3<T>>& M) {
    // This implementation is adapted from simbody at
    // https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Rotation.cpp
    T w, x, y, z;  // Elements of the quaternion, w relates to cos(theta/2).
    const T trace = M.trace();
    if (trace >= M(0, 0) && trace >= M(1, 1) && trace >= M(2, 2)) {
      // This branch occurs if the trace is larger than any diagonal element.
      w = T(1) + trace;
      x = M(2, 1) - M(1, 2);
      y = M(0, 2) - M(2, 0);
      z = M(1, 0) - M(0, 1);
    } else if (M(0, 0) >= M(1, 1) && M(0, 0) >= M(2, 2)) {
      // This branch occurs if M(0,0) is largest among the diagonal elements.
      w = M(2, 1) - M(1, 2);
      x = T(1) - (trace - 2 * M(0, 0));
      y = M(0, 1) + M(1, 0);
      z = M(0, 2) + M(2, 0);
    } else if (M(1, 1) >= M(2, 2)) {
      // This branch occurs if M(1,1) is largest among the diagonal elements.
      w = M(0, 2) - M(2, 0);
      x = M(0, 1) + M(1, 0);
      y = T(1) - (trace - 2 * M(1, 1));
      z = M(1, 2) + M(2, 1);
    } else {
      // This branch occurs if M(2,2) is largest among the diagonal elements.
      w = M(1, 0) - M(0, 1);
      x = M(0, 2) + M(2, 0);
      y = M(1, 2) + M(2, 1);
      z = T(1) - (trace - 2 * M(2, 2));
    }
    // Create a quantity q (which is not yet a unit quaternion).
    // Note: Eigen's Quaternion constructor does not normalize.
    return Eigen::Quaternion<T>(w, x, y, z);
  }

  // Constructs a 3x3 rotation matrix from a Quaternion.
  // `quaternion`: a quaternion which may or may not have unit length.
  // `two_over_norm_squared`: is supplied by the calling method and is
  //   usually pre-computed as `2 / quaternion.squaredNorm()`.  If `quaternion`
  //   has already been normalized [`quaternion.norm() = 1`] or there is a reason
  //   (unlikely) that the calling method determines that normalization is
  //   unwanted, the calling method should just past `two_over_norm_squared = 2`.
  //   (Internal) The cost of Eigen's quaternion.toRotationMatrix() is 12 adds and
  //   12 multiplies.  This method also costs 12 adds and 12 multiplies, but
  //   has a provision for an efficient algorithm for always calculating an
  // orthogonal rotation matrix (whereas Eigen's algorithm does not).
  // Throws: std::exception if all the elements of quaternion are zero.
  // Throws std::exception in debug builds if any of the elements in quaternion
  // are infinity or NaN.
  static Eigen::Matrix3<T> QuaternionToRotationMatrix(const Eigen::Quaternion<T>& quaternion, T two_over_norm_squared);

  // Throws an exception if the vector v does not have a measurable magnitude
  // within 4ε of 1 (where machine epsilon ε ≈ 2.22e-16).
  // `v`: The vector to test.
  // `function_name`: The name of the calling function; included in the
  //   exception message.
  // Throws: std::exception if |v| cannot be verified to be within 4ε of 1.
  //   An exception is thrown if v contains nonfinite numbers (NaN or infinity).
  // Note: no exception is thrown if v is a symbolic type.
  static void ThrowIfNotUnitLength(const Eigen::Vector3<T>& v, const char* function_name);

  // Returns the unit vector in the direction of v or throws an exception if v
  // cannot be "safely" normalized.
  // `v`: The vector to normalize.
  // `function_name`: The name of the calling function; included in the
  //   exception message.
  // Throws: std::exception if v contains nonfinite numbers (NaN or infinity)
  //   or |v| < 1.0e-10.
  // Note: no exception is thrown if v is a symbolic type.
  static Eigen::Vector3<T> NormalizeOrThrow(const Eigen::Vector3<T>& v, const char* function_name);

  // Stores the underlying rotation matrix relating two frames (e.g. A and B).
  // For speed, `R_AB_` is uninitialized (public constructors set its value).
  // The elements are stored in column-major order, per Eigen's default,
  // see https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html.
  Eigen::Matrix3<T> R_AB_;
};

// To enable low-level optimizations we insist that RotationMatrix<double> is
// packed into 9 consecutive doubles, with no extra alignment padding.
static_assert(sizeof(RotationMatrix<double>) == 9 * sizeof(double),
              "Low-level optimizations depend on RotationMatrix<double> being stored as "
              "9 sequential doubles in memory, with no extra memory alignment padding.");

// Abbreviation (alias/typedef) for a RotationMatrix double scalar type.
using RotationMatrixd = RotationMatrix<double>;
using RotationMatrixf = RotationMatrix<float>;

// Projects an approximate 3 x 3 rotation matrix M onto an orthonormal matrix R
// so that R is a rotation matrix associated with a angle-axis rotation by an
// angle θ about a vector direction `axis`, with `angle_lb <= θ <= angle_ub`.
// `M`: the matrix to be projected.
// `axis`: vector direction associated with angle-axis rotation for R.
//   axis can be a non-unit vector, but cannot be the zero vector.
// `angle_lb`: the lower bound of the rotation angle θ.
// `angle_ub`: the upper bound of the rotation angle θ.
// Returns: Rotation angle θ of the projected matrix, angle_lb <= θ <= angle_ub
// Throws: std::exception if axis is the zero vector or if angle_lb > angle_ub.
// Note: This method is useful for reconstructing a rotation matrix for a
// revolute joint with joint limits.
// Note: This can be formulated as an optimization problem
//
//   min_θ trace((R - M)ᵀ*(R - M))
//   subject to R = I + sinθ * A + (1 - cosθ) * A²   (1)
//              angle_lb <= θ <= angle_ub
//
// where A is the cross product matrix of a = axis / axis.norm() = [a₁, a₂, a₃]
//
// A = [ 0  -a₃  a₂]
//     [ a₃  0  -a₁]
//     [-a₂  a₁  0 ]
//
// Equation (1) is the Rodriguez Formula that computes the rotation matrix R
// from the angle-axis rotation with angle θ and vector direction `axis`.
// For details, see http://mathworld.wolfram.com/RodriguesRotationFormula.html
// The objective function can be simplified as
//
//    max_θ trace(Rᵀ * M + Mᵀ * R)
//
// By substituting the matrix `R` with the angle-axis representation, the
// optimization problem is formulated as
//
//    max_θ sinθ * trace(Aᵀ*M) - cosθ * trace(Mᵀ * A²)
//    subject to angle_lb <= θ <= angle_ub
//
// By introducing α = atan2(-trace(Mᵀ * A²), trace(Aᵀ*M)), we can compute the
// optimal θ as
//
//    θ = π/2 + 2kπ - α, if angle_lb <= π/2 + 2kπ - α <= angle_ub, k ∈ integers
// else
//    θ = angle_lb, if sin(angle_lb + α) >= sin(angle_ub + α)
//    θ = angle_ub, if sin(angle_lb + α) <  sin(angle_ub + α)
//
double ProjectMatToRotMatWithAxis(const Eigen::Matrix3d& M, const Eigen::Vector3d& axis, double angle_lb,
                                  double angle_ub);

}  // namespace math

extern template class math::RotationMatrix<double>;
extern template class math::RotationMatrix<float>;

#endif  // MATH_ROTATION_MATRIX_H_
