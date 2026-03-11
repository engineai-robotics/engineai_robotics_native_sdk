
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
#ifndef MATH_ROLL_PITCH_YAW_H_
#define MATH_ROLL_PITCH_YAW_H_

#include <cmath>
#include <iosfwd>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace math {

template <typename T>
class RotationMatrix;

// This class represents the orientation between two arbitrary frames A and D
// associated with a Space-fixed (extrinsic) X-Y-Z rotation by "roll-pitch-yaw"
// angles `[r, p, y]`, which is equivalent to a Body-fixed (intrinsic) Z-Y-X
// rotation by "yaw-pitch-roll" angles `[y, p, r]`.  The rotation matrix `R_AD`
// associated with this roll-pitch-yaw `[r, p, y]` rotation sequence is equal
// to the matrix multiplication shown below.
//
//        ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
// R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
//        ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
//      =       R_AB          *        R_BC          *        R_CD
//
// Note: In this discussion, A is the Space frame and D is the Body frame.
// One way to visualize this rotation sequence is by introducing intermediate
// frames B and C (useful constructs to understand this rotation sequence).
// Initially, the frames are aligned so `Di = Ci = Bi = Ai (i = x, y, z)`
// where Dx, Dy, Dz and Ax, Ay, Az are orthogonal unit vectors fixed in frames
// D and A respectively. Similarly for Bx, By, Bz and Cx, Cy, Cz in frame B, C.
// Then D is subjected to successive right-handed rotations relative to A.
// * 1st rotation R_CD: Frame D rotates relative to frames C, B, A by a
// roll angle `r` about `Dx = Cx`.  Note: D and C are no longer aligned.
// * 2nd rotation R_BC: Frames D, C (collectively -- as if welded together)
// rotate relative to frame B, A by a pitch angle `p` about `Cy = By`.
// Note: C and B are no longer aligned.
// * 3rd rotation R_AB: Frames D, C, B (collectively -- as if welded)
// rotate relative to frame A by a roll angle `y` about `Bz = Az`.
// Note: B and A are no longer aligned.
// The monogram notation for the rotation matrix relating A to D is `R_AD`.
//
// Note: This class does not store the frames associated with this rotation
// sequence.
template <typename T>
class RollPitchYaw {
 public:
  RollPitchYaw() = default;

  // Constructs a RollPitchYaw from a 3x1 array of angles (units of radians).
  explicit RollPitchYaw(const Eigen::Vector3<T>& rpy) { set(rpy); }

  // Constructs a RollPitchYaw from roll, pitch, yaw angles (radian units).
  RollPitchYaw(T roll, T pitch, T yaw) { set(roll, pitch, yaw); }

  // Uses a RotationMatrix to construct a RollPitchYaw with roll-pitch-yaw angles `[r, p, y]` in the range
  // `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  // Note: This new high-accuracy algorithm avoids numerical round-off issues
  // encountered by some algorithms when pitch is within 1.0e-6 of π/2 or -π/2.
  explicit RollPitchYaw(const RotationMatrix<T>& R) { SetFromRotationMatrix(R); }

  // Uses a Quaternion to construct a RollPitchYaw with roll-pitch-yaw angles `[r, p, y]` in the range
  // `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  // `quaternion`: unit Quaternion.
  // Note: This new high-accuracy algorithm avoids numerical round-off issues
  // encountered by some algorithms when pitch is within 1.0e-6 of π/2 or -π/2.
  // Throws: std::exception in debug builds if !IsValid(rpy).
  explicit RollPitchYaw(const Eigen::Quaternion<T>& quaternion) { SetFromQuaternion(quaternion); }

  // Sets `this` RollPitchYaw from a 3x1 array of angles (units of radians).
  // Throws: std::exception in debug builds if !IsValid(rpy).
  RollPitchYaw<T>& set(const Eigen::Vector3<T>& rpy) { return SetOrThrowIfNotValidInDebugBuild(rpy); }

  // Sets `this` RollPitchYaw from roll, pitch, yaw angles (units of radians).
  // Throws: std::exception in debug builds if !IsValid(Eigen::Vector3<T>(roll, pitch, yaw)).
  RollPitchYaw<T>& set(T roll, T pitch, T yaw) { return set(Eigen::Vector3<T>(roll, pitch, yaw)); }

  // Uses a Quaternion to set `this` RollPitchYaw with roll-pitch-yaw angles `[r, p, y]` in the range
  // `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  // `quaternion`: unit Quaternion.
  // Note: This new high-accuracy algorithm avoids numerical round-off issues
  // encountered by some algorithms when pitch is within 1.0e-6 of π/2 or -π/2.
  // Throws: std::exception in debug builds if !IsValid(rpy).
  void SetFromQuaternion(const Eigen::Quaternion<T>& quaternion);

  // Uses a high-accuracy efficient algorithm to set the roll-pitch-yaw
  // angles `[r, p, y]` that underlie `this` RollPitchYaw, even when the pitch
  // angle p is very near a singularity (when p is within 1.0e-6 of π/2 or -π/2).
  // After calling this method, the underlying roll-pitch-yaw `[r, p, y]` has
  // range `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  // Note: This high-accuracy algorithm was invented at TRI in October 2016 and
  // avoids numerical round-off issues encountered by some algorithms when
  // pitch is within 1.0e-6 of π/2 or -π/2.
  void SetFromRotationMatrix(const RotationMatrix<T>& R);

  // Returns the Eigen::Vector3 underlying `this` RollPitchYaw.
  const Eigen::Vector3<T>& vector() const { return roll_pitch_yaw_; }

  // Returns the roll-angle underlying `this` RollPitchYaw.
  T roll() const { return roll_pitch_yaw_(0); }

  // Returns the pitch-angle underlying `this` RollPitchYaw.
  T pitch() const { return roll_pitch_yaw_(1); }

  // Returns the yaw-angle underlying `this` RollPitchYaw.
  T yaw() const { return roll_pitch_yaw_(2); }

  // Sets the roll-angle underlying `this` RollPitchYaw (in units of radians).
  void set_roll(T r) { roll_pitch_yaw_(0) = r; }

  // Sets the pitch-angle underlying `this` RollPitchYaw (in units of radians).
  void set_pitch(T p) { roll_pitch_yaw_(1) = p; }

  // Sets the yaw-angle underlying `this` RollPitchYaw (in units of radians).
  void set_yaw(T y) { roll_pitch_yaw_(2) = y; }

  // Returns a quaternion representation of `this` RollPitchYaw.
  Eigen::Quaternion<T> ToQuaternion() const {
    using std::cos;
    using std::sin;
    const T q0Half = roll_pitch_yaw_(0) / 2;
    const T q1Half = roll_pitch_yaw_(1) / 2;
    const T q2Half = roll_pitch_yaw_(2) / 2;
    const T c0 = cos(q0Half), s0 = sin(q0Half);
    const T c1 = cos(q1Half), s1 = sin(q1Half);
    const T c2 = cos(q2Half), s2 = sin(q2Half);
    const T c1_c2 = c1 * c2, s1_c2 = s1 * c2;
    const T s1_s2 = s1 * s2, c1_s2 = c1 * s2;
    const T w = c0 * c1_c2 + s0 * s1_s2;
    const T x = s0 * c1_c2 - c0 * s1_s2;
    const T y = c0 * s1_c2 + s0 * c1_s2;
    const T z = c0 * c1_s2 - s0 * s1_c2;
    return Eigen::Quaternion<T>(w, x, y, z);
  }

  // Returns the RotationMatrix representation of `this` RollPitchYaw.
  RotationMatrix<T> ToRotationMatrix() const;

  // Returns the 3x3 matrix representation of the RotationMatrix that
  // corresponds to `this` RollPitchYaw.  This is a convenient "sugar" method
  // that is exactly equivalent to RotationMatrix(rpy).ToMatrix3().
  Eigen::Matrix3<T> ToMatrix3ViaRotationMatrix() const { return ToRotationMatrix().matrix(); }

  // Compares each element of `this` to the corresponding element of `other`
  // to check if they are the same to within a specified `tolerance`.
  // Returns: true if `‖this - other‖∞ <= tolerance`.
  bool IsNearlyEqualTo(const RollPitchYaw<T>& other, double tolerance) const {
    const Eigen::Vector3<T> difference = vector() - other.vector();
    return difference.template lpNorm<Eigen::Infinity>() <= tolerance;
  }

  // Compares each element of the RotationMatrix R1 produced by `this` to the
  // corresponding element of the RotationMatrix R2 produced by `other` to
  // check if they are the same to within a specified `tolerance`.
  // Returns: true if `‖R1 - R2‖∞ <= tolerance`.
  bool IsNearlySameOrientation(const RollPitchYaw<T>& other, double tolerance) const;

  // Returns true if roll-pitch-yaw angles `[r, p, y]` are in the range
  // `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  bool IsRollPitchYawInCanonicalRange() const {
    T r = roll();
    T p = pitch();
    T y = yaw();
    return (-M_PI <= r && r <= M_PI) && (-M_PI / 2 <= p && p <= M_PI / 2) && (-M_PI <= y && y <= M_PI);
  }

  // Returns true if the pitch-angle `p` is close to gimbal-lock, which means
  // `cos(p) ≈ 0` or `p ≈ (n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  // More specifically, returns true if `abs(cos_pitch)` is less than an
  // internally-defined tolerance of gimbal-lock.
  // `cos_pitch`: cosine of the pitch angle, i.e., `cos(p)`.
  // Note: Pitch-angles close to gimbal-lock can cause problems with numerical
  // precision and numerical integration.
  static bool DoesCosPitchAngleViolateGimbalLockTolerance(T cos_pitch) {
    using std::abs;
    return abs(cos_pitch) < kGimbalLockToleranceCosPitchAngle;
  }

  // Returns true if the pitch-angle `p` is within an internally-defined
  // tolerance of gimbal-lock.  In other words, this method returns true if
  // `p ≈ (n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  // Note: To improve efficiency when cos(pitch()) is already calculated,
  // instead use the function DoesCosPitchAngleViolateGimbalLockTolerance().
  bool DoesPitchAngleViolateGimbalLockTolerance() const {
    using std::cos;
    return DoesCosPitchAngleViolateGimbalLockTolerance(cos(pitch()));
  }

  // Returns the internally-defined allowable closeness (in radians) of the
  // pitch angle `p` to gimbal-lock, i.e., the allowable proximity of `p` to
  // `(n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  static double GimbalLockPitchAngleTolerance() { return M_PI_2 - std::acos(kGimbalLockToleranceCosPitchAngle); }

  // Returns true if `rpy` contains valid roll, pitch, yaw angles.
  // Note: an angle is invalid if it is NaN or infinite.
  static bool IsValid(const Eigen::Vector3<T>& rpy) {
    using std::isfinite;
    return isfinite(rpy[0]) && isfinite(rpy[1]) && isfinite(rpy[2]);
  }

  // Forms Ṙ, the ordinary derivative of the RotationMatrix `R` with respect
  // to an independent variable `t` (`t` usually denotes time) and `R` is the
  // RotationMatrix formed by `this` RollPitchYaw.  The roll-pitch-yaw angles
  // r, p, y are regarded as functions of `t` [i.e., r(t), p(t), y(t)].
  // `rpyDt`: Ordinary derivative of rpy with respect to an independent
  //   variable `t` (`t` usually denotes time, but not necessarily).
  // Returns: Ṙ, the ordinary derivative of `R` with respect to `t`, calculated
  //   as Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ.  In other words, the returned
  //   (i, j) element is ∂Rij/∂r * ṙ + ∂Rij/∂p * ṗ + ∂Rij/∂y * ẏ.
  Eigen::Matrix3<T> CalcRotationMatrixDt(const Eigen::Vector3<T>& rpyDt) const {
    // For the rotation matrix R generated by `this` RollPitchYaw, calculate the
    // partial derivatives of R with respect to roll `r`, pitch `p` and yaw `y`.
    Eigen::Matrix3<T> R_r, R_p, R_y;  // ∂R/∂r, ∂R/∂p, ∂R/∂y
    CalcRotationMatrixDrDpDy(&R_r, &R_p, &R_y);

    // When rotation matrix R is regarded as an implicit function of t as
    // R(r(t), p(t), y(t)), the ordinary derivative of R with respect to t is
    // Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ
    const T rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
    return R_r * rDt + R_p * pDt + R_y * yDt;
  }

  // Calculates angular velocity from `this` RollPitchYaw whose roll-pitch-yaw
  // angles `[r; p; y]` relate the orientation of two generic frames A and D.
  // `rpyDt`: Time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  // Returns: w_AD_A, frame D's angular velocity in frame A, expressed in A.
  Eigen::Vector3<T> CalcAngularVelocityInParentFromRpyDt(const Eigen::Vector3<T>& rpyDt) const {
    // Get the 3x3 coefficient matrix M that contains the partial derivatives of
    // w_AD_A with respect to ṙ, ṗ, ẏ.  In other words, `w_AD_A = M * rpyDt`.
    // TODO: Improve speed -- last column of M is (0, 0, 1).
    const Eigen::Matrix3<T> M = CalcMatrixRelatingAngularVelocityInParentToRpyDt();
    return M * rpyDt;
  }

  // Calculates angular velocity from `this` RollPitchYaw whose roll-pitch-yaw
  // angles `[r; p; y]` relate the orientation of two generic frames A and D.
  // `rpyDt`: Time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  // Returns: w_AD_D, frame D's angular velocity in frame A, expressed in D.
  Eigen::Vector3<T> CalcAngularVelocityInChildFromRpyDt(const Eigen::Vector3<T>& rpyDt) const {
    // Get the 3x3 coefficient matrix M that contains the partial derivatives of
    // w_AD_D with respect to ṙ, ṗ, ẏ.  In other words, `w_AD_D = M * rpyDt`.
    // TODO: Improve speed -- first column of M is (1, 0, 0).
    const Eigen::Matrix3<T> M = CalcMatrixRelatingAngularVelocityInChildToRpyDt();
    return M * rpyDt;
  }

  // Uses angular velocity to compute the 1ˢᵗ time-derivative of `this`
  // RollPitchYaw whose angles `[r; p; y]` orient two generic frames A and D.
  // `w_AD_A`: frame D's angular velocity in frame A, expressed in A.
  // Returns: `[ṙ; ṗ; ẏ]`, the 1ˢᵗ time-derivative of `this` RollPitchYaw.
  // Throws: std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  // Note: This method has a divide-by-zero error (singularity) when the cosine
  // of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  // "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  // There are associated precision problems (inaccuracies) in the neighborhood
  // of these pitch angles, i.e., when `cos(p) ≈ 0`.
  Eigen::Vector3<T> CalcRpyDtFromAngularVelocityInParent(const Eigen::Vector3<T>& w_AD_A) const {
    // Get the 3x3 M matrix that contains the partial derivatives of `[ṙ, ṗ, ẏ]`
    // with respect to `[wx; wy; wz]ₐ` (which is w_AD_A expressed in A).
    // In other words, `rpyDt = M * w_AD_A`.
    // TODO: Improve speed -- last column of M is (0, 0, 1).
    // TODO: Improve accuracy when `cos(p) ≈ 0`.
    const Eigen::Matrix3<T> M = CalcMatrixRelatingRpyDtToAngularVelocityInParent(__func__, __FILE__, __LINE__);
    return M * w_AD_A;
  }

  // For `this` RollPitchYaw with roll-pitch-yaw angles `[r; p; y]` which
  // relate the orientation of two generic frames A and D, returns the 3x3
  // matrix M that contains the partial derivatives of [ṙ, ṗ, ẏ] with respect
  // to `[wx; wy; wz]ₐ` (which is w_AD_A expressed in A).
  // In other words, `rpyDt = M * w_AD_A`.
  // `function_name`: name of the calling function/method.
  // Throws: std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  // Note: This method has a divide-by-zero error (singularity) when the cosine
  // of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  // "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  // There are associated precision problems (inaccuracies) in the neighborhood
  // of these pitch angles, i.e., when `cos(p) ≈ 0`.
  const Eigen::Matrix3<T> CalcMatrixRelatingRpyDtToAngularVelocityInParent() const {
    const Eigen::Matrix3<T> M = CalcMatrixRelatingRpyDtToAngularVelocityInParent(__func__, __FILE__, __LINE__);
    return M;
  }

  // Uses angular acceleration to compute the 2ⁿᵈ time-derivative of `this`
  // RollPitchYaw whose angles `[r; p; y]` orient two generic frames A and D.
  // `rpyDt`: time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  // `alpha_AD_A`: frame D's angular acceleration in frame A, expressed in frame A.
  // Returns: `[r̈, p̈, ÿ]`, the 2ⁿᵈ time-derivative of `this` RollPitchYaw.
  // Throws: std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  // Note: This method has a divide-by-zero error (singularity) when the cosine
  // of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  // "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  // There are associated precision problems (inaccuracies) in the neighborhood
  // of these pitch angles, i.e., when `cos(p) ≈ 0`.
  Eigen::Vector3<T> CalcRpyDDtFromRpyDtAndAngularAccelInParent(const Eigen::Vector3<T>& rpyDt,
                                                               const Eigen::Vector3<T>& alpha_AD_A) const {
    // TODO: Improve accuracy when `cos(p) ≈ 0`.
    // TODO: Improve speed: The last column of M is (0, 0, 1), the last
    // column of MDt is (0, 0, 1) and there are repeated sin/cos calculations.
    const Eigen::Matrix3<T> Minv = CalcMatrixRelatingRpyDtToAngularVelocityInParent(__func__, __FILE__, __LINE__);
    const Eigen::Matrix3<T> MDt = CalcDtMatrixRelatingAngularVelocityInParentToRpyDt(rpyDt);
    return Minv * (alpha_AD_A - MDt * rpyDt);
  }

  // Uses angular acceleration to compute the 2ⁿᵈ time-derivative of `this`
  // RollPitchYaw whose angles `[r; p; y]` orient two generic frames A and D.
  // `rpyDt`: time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  // `alpha_AD_D`: frame D's angular acceleration in frame A, expressed in frame D.
  // Returns: `[r̈, p̈, ÿ]`, the 2ⁿᵈ time-derivative of `this` RollPitchYaw.
  // Throws: std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  // Note: This method has a divide-by-zero error (singularity) when the cosine
  // of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  // "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  // There are associated precision problems (inaccuracies) in the neighborhood
  // of these pitch angles, i.e., when `cos(p) ≈ 0`.
  Eigen::Vector3<T> CalcRpyDDtFromAngularAccelInChild(const Eigen::Vector3<T>& rpyDt,
                                                      const Eigen::Vector3<T>& alpha_AD_D) const {
    T r = roll();
    T p = pitch();
    using std::cos;
    using std::sin;
    const T sr = sin(r), cr = cos(r);
    const T sp = sin(p), cp = cos(p);
    if (DoesCosPitchAngleViolateGimbalLockTolerance(cp)) {
      ThrowPitchAngleViolatesGimbalLockTolerance(__func__, __FILE__, __LINE__, p);
    }
    const T one_over_cp = T(1) / cp;
    const T cr_over_cp = cr * one_over_cp;
    const T sr_over_cp = sr * one_over_cp;
    Eigen::Matrix3<T> M = Eigen::Matrix3<T>{
        {T{1}, sr_over_cp * sp, cr_over_cp * sp},
        {T{0}, cr, -sr},
        {T{0}, sr_over_cp, cr_over_cp},
    };

    // Remainder terms (terms not multiplying α).
    const T tanp = sp * one_over_cp;
    const T rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
    const T pDt_yDt = pDt * yDt;
    const T rDt_pDt = rDt * pDt;
    const Eigen::Vector3<T> remainder(tanp * rDt_pDt + one_over_cp * pDt_yDt, -cp * rDt * yDt,
                                      tanp * pDt_yDt + one_over_cp * rDt_pDt);

    // Combine terms that contains alpha with remainder terms.
    // TODO: M * alpha_AD_D can be calculated faster since the first
    // column of M is (1, 0, 0).
    return M * alpha_AD_D + remainder;
  }

 private:
  // Throws an exception if rpy is not a valid RollPitchYaw.
  // `rpy`: an allegedly valid rotation matrix.
  // Note: If the underlying scalar type T is non-numeric (symbolic), no
  // validity check is made and no assertion is thrown.
  static void ThrowIfNotValid(const Eigen::Vector3<T>& rpy) {
    if (!RollPitchYaw<T>::IsValid(rpy)) {
      throw std::logic_error("Error: One (or more) of the roll-pitch-yaw angles is infinity or NaN.");
    }
  }

  // Throws an exception with a message that the pitch-angle `p` violates the
  // internally-defined gimbal-lock tolerance, which occurs when `cos(p) ≈ 0`,
  // which means `p ≈ (n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  // `function_name`: name of the calling function/method.
  // `file_name`: name of the file with the calling function/method.
  // `line_number`: the line number in file_name that made the call.
  // `pitch`: pitch angle `p` (in radians).
  // Throws: std::exception with a message that `p` is too near gimbal-lock.
  static void ThrowPitchAngleViolatesGimbalLockTolerance(const char* function_name, const char* file_name,
                                                         int line_number, T pitch);

  // Uses a quaternion and its associated rotation matrix `R` to accurately
  // and efficiently set the roll-pitch-yaw angles (SpaceXYZ Euler angles)
  // that underlie `this` RollPitchYaw, even when the pitch angle p is very
  // near a singularity (e.g., when p is within 1.0e-6 of π/2 or -π/2).
  // After calling this method, the underlying roll-pitch-yaw `[r, p, y]` has
  // range `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  // `quaternion`: unit quaternion with elements `[e0, e1, e2, e3]`.
  // `R`: The RotationMatrix corresponding to `quaternion`.
  // Throws: std::exception in debug builds if rpy fails IsValid(rpy).
  // Note: Aborts in debug builds if `quaternion` does not correspond to `R`.
  void SetFromQuaternionAndRotationMatrix(const Eigen::Quaternion<T>& quaternion, const RotationMatrix<T>& R);

  // For the RotationMatrix `R` generated by `this` RollPitchYaw, this method
  // calculates the partial derivatives of `R` with respect to roll, pitch, yaw.
  // `R_r`: ∂R/∂r Partial derivative of `R` with respect to roll `r`.
  // `R_p`: ∂R/∂p Partial derivative of `R` with respect to pitch `p`.
  // `R_y`: ∂R/∂y Partial derivative of `R` with respect to yaw `y`.
  void CalcRotationMatrixDrDpDy(Eigen::Matrix3<T>* R_r, Eigen::Matrix3<T>* R_p, Eigen::Matrix3<T>* R_y) const {
    // DCHECK(R_r != nullptr && R_p != nullptr && R_y != nullptr);
    if (R_r == nullptr || R_p == nullptr || R_y == nullptr) {
      throw std::logic_error("Error: R_r, R_p, and R_y cannot be nullptr.");
    }
    T r = roll();
    T p = pitch();
    T y = yaw();
    using std::cos;
    using std::sin;
    const T c0 = cos(r), c1 = cos(p), c2 = cos(y);
    const T s0 = sin(r), s1 = sin(p), s2 = sin(y);
    const T c2_s1 = c2 * s1, s2_s1 = s2 * s1, s2_s0 = s2 * s0, s2_c0 = s2 * c0;
    const T c2_c1 = c2 * c1, s2_c1 = s2 * c1, c2_s0 = c2 * s0, c2_c0 = c2 * c0;
    *R_r = Eigen::Matrix3<T>{
        {0, c2_s1 * c0 + s2_s0, -c2_s1 * s0 + s2_c0},
        {0, s2_s1 * c0 - c2_s0, -s2_s1 * s0 - c2_c0},
        {0, c1 * c0, -c1 * s0},
    };

    *R_p = Eigen::Matrix3<T>{
        {-c2_s1, c2_c1 * s0, c2_c1 * c0},
        {-s2_s1, s2_c1 * s0, s2_c1 * c0},
        {-c1, -s1 * s0, -s1 * c0},
    };

    *R_y = Eigen::Matrix3<T>{
        {-s2_c1, -s2_s1 * s0 - c2_c0, -s2_s1 * c0 + c2_s0},
        {c2_c1, c2_s1 * s0 - s2_c0, c2_s1 * c0 + s2_s0},
        {0, 0, 0},
    };
  }

  // For `this` RollPitchYaw with roll-pitch-yaw angles `[r; p; y]` which
  // relate the orientation of two generic frames A and D, returns the 3x3
  // coefficient matrix M that contains the partial derivatives of `w_AD_A`
  // (D's angular velocity in A, expressed in A) with respect to ṙ, ṗ, ẏ.
  // In other words, `w_AD_A = M * rpyDt` where `rpyDt` is `[ṙ; ṗ; ẏ]`.
  const Eigen::Matrix3<T> CalcMatrixRelatingAngularVelocityInParentToRpyDt() const {
    using std::cos;
    using std::sin;
    T p = pitch();
    T y = yaw();
    const T sp = sin(p), cp = cos(p);
    const T sy = sin(y), cy = cos(y);
    Eigen::Matrix3<T> M = Eigen::Matrix3<T>{
        {cp * cy, -sy, T{0}},
        {cp * sy, cy, T{0}},
        {-sp, T{0}, T{1}},
    };
    return M;
  }

  // Returns the time-derivative of the 3x3 matrix returned by the `this`
  // RollPitchYaw method MatrixRelatingAngularVelocityAToRpyDt().
  // `rpyDt`: Time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  const Eigen::Matrix3<T> CalcDtMatrixRelatingAngularVelocityInParentToRpyDt(const Eigen::Vector3<T>& rpyDt) const {
    using std::cos;
    using std::sin;
    T p = pitch();
    T y = yaw();
    const T sp = sin(p), cp = cos(p);
    const T sy = sin(y), cy = cos(y);
    const T pDt = rpyDt(1);
    const T yDt = rpyDt(2);
    const T sp_pDt = sp * pDt;
    const T cp_yDt = cp * yDt;
    Eigen::Matrix3<T> M = Eigen::Matrix3<T>{
        {-cy * sp_pDt - sy * cp_yDt, -cy * yDt, T{0}},
        {-sy * sp_pDt + cy * cp_yDt, -sy * yDt, T{0}},
        {-cp * pDt, T{0}, T{0}},
    };
    return M;
  }

  // For `this` RollPitchYaw with roll-pitch-yaw angles `[r; p; y]` which
  // relate the orientation of two generic frames A and D, returns the 3x3
  // coefficient matrix M that contains the partial derivatives of `w_AD_D`
  // (D's angular velocity in A, expressed in D) with respect to ṙ, ṗ, ẏ.
  // In other words, `w_AD_D = M * rpyDt` where `rpyDt` is `[ṙ; ṗ; ẏ]`.
  const Eigen::Matrix3<T> CalcMatrixRelatingAngularVelocityInChildToRpyDt() const {
    using std::cos;
    using std::sin;
    T r = roll();
    T p = pitch();
    const T sr = sin(r), cr = cos(r);
    const T sp = sin(p), cp = cos(p);
    Eigen::Matrix3<T> M = Eigen::Matrix3<T>{
        {T(1), T(0), -sp},
        {T(0), cr, sr * cp},
        {T(0), -sr, cr * cp},
    };
    return M;
  }

  // For `this` RollPitchYaw with roll-pitch-yaw angles `[r; p; y]` which
  // relate the orientation of two generic frames A and D, returns the 3x3
  // matrix M that contains the partial derivatives of [ṙ, ṗ, ẏ] with respect
  // to `[wx; wy; wz]ₐ` (which is w_AD_A expressed in A).
  // In other words, `rpyDt = M * w_AD_A`.
  // `function_name`: name of the calling function/method.
  // `file_name`: name of the file with the calling function/method.
  // `line_number`: the line number in file_name that made the call.
  // Throws: std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  // Note: This method has a divide-by-zero error (singularity) when the cosine
  // of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  // "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  // There are associated precision problems (inaccuracies) in the neighborhood
  // of these pitch angles, i.e., when `cos(p) ≈ 0`.
  // Note: This utility method typically gets called from a user-relevant API
  // so it provides the ability to detect gimbal-lock and throws an error
  // message that includes information from the calling function (rather than
  // less useful information from within this method itself).
  const Eigen::Matrix3<T> CalcMatrixRelatingRpyDtToAngularVelocityInParent(const char* function_name,
                                                                           const char* file_name,
                                                                           int line_number) const {
    using std::cos;
    using std::sin;
    T p = pitch();
    T y = yaw();
    const T sp = sin(p), cp = cos(p);
    // TODO: Improve accuracy when `cos(p) ≈ 0`.
    if (DoesCosPitchAngleViolateGimbalLockTolerance(cp)) {
      ThrowPitchAngleViolatesGimbalLockTolerance(function_name, file_name, line_number, p);
    }
    const T one_over_cp = T(1) / cp;
    const T sy = sin(y), cy = cos(y);
    const T cy_over_cp = cy * one_over_cp;
    const T sy_over_cp = sy * one_over_cp;
    Eigen::Matrix3<T> M = Eigen::Matrix3<T>{
        {cy_over_cp, sy_over_cp, T{0}},
        {-sy, cy, T{0}},
        {cy_over_cp * sp, sy_over_cp * sp, T{1}},
    };
    return M;
  }

  // Sets `this` RollPitchYaw from a Eigen::Vector3.
  // `rpy`: allegedly valid roll-pitch-yaw angles.
  // Throws: std::exception in debug builds if rpy fails IsValid(rpy).
  RollPitchYaw<T>& SetOrThrowIfNotValidInDebugBuild(const Eigen::Vector3<T>& rpy) {
    // if constexpr (DCHECK_IS_ON()) {
    //   ThrowIfNotValid(rpy);
    // }
    roll_pitch_yaw_ = rpy;
    return *this;
  }

  // Stores the underlying roll-pitch-yaw angles.
  // There is no default initialization needed.
  Eigen::Vector3<T> roll_pitch_yaw_;

  // Internally-defined value for the allowable proximity of the cosine of the
  // pitch-angle `p` to gimbal-lock [the proximity of `cos(p)` to 0].
  // Note: For small values (<= 0.1), this value approximates the allowable
  // proximity of the pitch-angle (in radians) to gimbal-lock.  Example: A value
  // of 0.01 corresponds to `p` within ≈ 0.01 radians (≈ 0.57°) of gimbal-lock,
  // i.e., `p` is within 0.01 radians of `(n*π + π/2)`, `n = 0, ±1, ±2, ...`.
  // A value 0.008 corresponds to `p` ≈ 0.008 radians (≈ 0.46°) of gimbal-lock.
  // Note: The conversion from angular velocity to rpyDt (the time-derivative of
  // RollPitchYaw) has a calculation that divides by `cos(p)` (the cosine of
  // the pitch angle).  This results in values of rpyDt that scale with angular
  // velocity multiplied by `1/cos(p)`, which can cause problems with numerical
  // precision and numerical integration.
  // Note: There is a numerical test (in roll_pitch_yaw_test.cc) that converts
  // an angular velocity w(1, 1, 1) to rpyDt (time-derivative of roll-pitch-yaw)
  // and then back to angular velocity.  This test revealed an imprecision in
  // the back-and-forth calculation near gimbal-lock by calculating max_error
  // (how much the angular velocity changed in the back-and-forth calculation).
  // The table below shows various values of kGimbalLockToleranceCosPitchAngle,
  // it associated proximity to gimbal-lock (in degrees) and the associated
  // max_error (in terms of machine kEpsilon = 1/2^52 ≈ 2.22e-16).
  // ---------------------------------------------------------------------------
  //  kGimbalLockToleranceCosPitchAngle  |  max_error
  // ---------------------------------------------------------------------------
  //  0.001 ≈ 0.06°                      | (2^10 = 1024) * kEpsilon ≈ 2.274e-13
  //  0.002 ≈ 0.11°                      |  (2^9 = 512)  * kEpsilon ≈ 1.137e-13
  //  0.004 ≈ 0.23°                      |  (2^8 = 256)  * kEpsilon ≈ 5.684e-14
  //  0.008 ≈ 0.46°                      |  (2^7 = 128)  * kEpsilon ≈ 2.842e-14
  //  0.016 ≈ 0.92°                      |  (2^6 =  64)  * kEpsilon ≈ 1.421e-14
  //  0.032 ≈ 1.83°                      |  (2^5 =  32)  * kEpsilon ≈ 7.105e-15
  // ---------------------------------------------------------------------------
  // Hence if kGimbalLockToleranceCosPitchAngle = 0.008 and the pitch angle is
  // in the proximity of ≈ 0.46° of gimbal lock, there may be inaccuracies in
  // 7 of the 52 bits in max_error's mantissa, which we deem acceptable.
  static constexpr double kGimbalLockToleranceCosPitchAngle = 0.008;
};

// Stream insertion operator to write an instance of RollPitchYaw into a
// `std::ostream`. Especially useful for debugging.
template <typename T>
std::ostream& operator<<(std::ostream& out, const RollPitchYaw<T>& rpy);

// Abbreviation (alias/typedef) for a RollPitchYaw double scalar type.
using RollPitchYawd = RollPitchYaw<double>;
using RollPitchYawf = RollPitchYaw<float>;

}  // namespace math

extern template class math::RollPitchYaw<double>;
extern template class math::RollPitchYaw<float>;

#endif  // MATH_ROLL_PITCH_YAW_H_
