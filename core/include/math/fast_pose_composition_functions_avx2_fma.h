
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
#ifndef MATH_FAST_POSE_COMPOSITION_FUNCTIONS_AVX2_FMA_H_
#define MATH_FAST_POSE_COMPOSITION_FUNCTIONS_AVX2_FMA_H_

// Declarations for fast, low-level functions for handling objects stored in small
// matrices with known memory layouts, implemented using platform-specific SIMD
// instructions for speed.
//
// N.B. Do not include any other drake headers here because this file will be
// included by a compilation unit that may have a different opinion about whether
// SIMD instructions are enabled than Eigen does in the rest of Drake.

namespace math {

// We do not have access to the declarations for RotationMatrix and
// RigidTransform here. Instead, the functions below depend on knowledge of the
// internal storage format of these classes, which is guaranteed and enforced by
// the class declarations:
//  - Drake RotationMatrix objects are stored as 3x3 column-ordered matrices in
//    nine consecutive doubles.
//  - Drake RigidTransform objects are stored as 3x4 column-ordered matrices in
//    twelve consecutive doubles. The first nine elements comprise a 3x3
//    RotationMatrix and the last three are the translation vector.

template <typename>
class RotationMatrix;
template <typename>
class RigidTransform;

namespace internal {

// Detects if the AVX2 implementations below are supported. Supported means that
// both (1) AVX2 was enabled at built time, and (2) the processor executing this
// code supports AVX2 instructions.
bool AvxSupported();

// Composes two math::RotationMatrix<double> objects as quickly as
// possible, resulting in a new RotationMatrix.
//
// Here we calculate `R_AC = R_AB * R_BC`. It is OK for R_AC to overlap
// with one or both inputs.
//
// Note: if AVX2 is not supported, calling this function will crash the program. */
void ComposeRRAvx(const RotationMatrix<double>& R_AB, const RotationMatrix<double>& R_BC, RotationMatrix<double>* R_AC);

// Composes the inverse of a math::RotationMatrix<double> object with
// another (non-inverted) math::RotationMatrix<double> as quickly as
// possible, resulting in a new RotationMatrix.
//
// Note: A valid RotationMatrix is orthonormal, and the inverse of an orthonormal
// matrix is just its transpose. This function assumes orthonormality and hence
// simply multiplies the transpose of its first argument by the second.
//
// Here we calculate `R_AC = R_BA⁻¹ * R_BC`. It is OK for R_AC to overlap
// with one or both inputs.
//
// Note: if AVX2 is not supported, calling this function will crash the program.
void ComposeRinvRAvx(const RotationMatrix<double>& R_BA, const RotationMatrix<double>& R_BC,
                     RotationMatrix<double>* R_AC);

// Composes two math::RigidTransform<double> objects as quickly as
// possible, resulting in a new RigidTransform.
//
// Note: This function is specialized for RigidTransforms and is not just a
// matrix multiply.
//
// Here we calculate `X_AC = X_AB * X_BC`. It is OK for X_AC to overlap
// with one or both inputs.
//
// Note: if AVX2 is not supported, calling this function will crash the program.
void ComposeXXAvx(const RigidTransform<double>& X_AB, const RigidTransform<double>& X_BC, RigidTransform<double>* X_AC);

// Composes the inverse of a math::RigidTransform<double> object with
// another (non-inverted) math::RigidTransform<double> as quickly as
// possible, resulting in a new RigidTransform.
//
// Note: This function is specialized for RigidTransforms and is not just a
// matrix multiply.
//
// Here we calculate `X_AC = X_BA⁻¹ * X_BC`. It is OK for X_AC to overlap
// with one or both inputs.
//
// Note: if AVX2 is not supported, calling this function will crash the program.
void ComposeXinvXAvx(const RigidTransform<double>& X_BA, const RigidTransform<double>& X_BC,
                     RigidTransform<double>* X_AC);

}  // namespace internal
}  // namespace math

#endif  // MATH_FAST_POSE_COMPOSITION_FUNCTIONS_AVX2_FMA_H_
