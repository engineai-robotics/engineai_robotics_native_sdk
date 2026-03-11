#pragma once

#include <type_traits>

namespace math {

// Substitutes std::remove_cvref in C++20 missing from C++17.
template <typename T>
using RemoveCvref = std::remove_cv_t<std::remove_reference_t<T>>;

// Returns the evaluated, concrete type of an Eigen expression.
// ::EvalReturnType is from Eigen::DenseBase.
template <typename EigenExpression>
using EigenConcreteType = RemoveCvref<typename RemoveCvref<EigenExpression>::EvalReturnType>;

}  // namespace math
