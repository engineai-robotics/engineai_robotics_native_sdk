#pragma once

#include <memory>
#include <optional>

#include <Eigen/Core>
#include <qpOASES/SQProblem.hpp>
#include <qpOASES/Types.hpp>

namespace math {

enum class QpHessianType {
  kDefault,
  kZero,
  kIdentity,
};

struct QpDataOptions {
  QpHessianType hessian_type = QpHessianType::kDefault;
  bool has_var_lb = true;
  bool has_var_ub = true;
  bool has_cst_lb = true;
  bool has_cst_ub = true;

  void SetNoVariableBounds() {
    has_var_lb = false;
    has_var_ub = false;
  }

  void SetNoConstraintBounds() {
    has_cst_lb = false;
    has_cst_ub = false;
  }

  void SetNoBounds() {
    SetNoVariableBounds();
    SetNoConstraintBounds();
  }
};

template <typename T>
struct QpData {
  QpData(int num_variables, int num_constraints, QpDataOptions options);

  // Sets all problem inputs to zero.
  // Removes the solution and places the QP problem in uninitialized state.
  // If the solver uses sequential strategies, this causes the solver to
  // reinitialize the solution sequence.
  // Calling this is unnecessary if the variables and constraints are separately assigned.
  void Clear();

  bool IsSolved() const;

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Hess;
  Eigen::VectorX<T> grad;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cst;

  Eigen::VectorX<T> var_lb;
  Eigen::VectorX<T> var_ub;
  Eigen::VectorX<T> cst_lb;
  Eigen::VectorX<T> cst_ub;

  Eigen::VectorX<T> solution;
};

// A wrapper of qpOASES to make it easier to handle memory allocations and floating-point type conversions.
// `T` can be float or double.
template <typename T>
class QpOasesSolver {
 public:
  // Initializes problem input variables to zero and the `solution` to NaN.
  QpOasesSolver(int num_variables, int num_constraints, QpDataOptions options = {});

  enum class PrintLevel {
    // Full tabular debugging output
    kDebugIter = -2,
    // Normal tabular output
    kTabular,
    // No output
    kNone,
    // Error messages only
    kLow,
    // Error and warning messages as well as concise info messages
    kMedium,
    // All messages with full details
    kHigh,
  };

  enum class SolverProfile {
    kDefault,
    kReliable,
    kFast,
  };

  enum class SequentialStrategy {
    // Initializes the QP solution every time with no prior information.
    kNonSequential,

    // Initializes the QP solution every time but uses the last solution
    // as the initial guess.
    kNonSequentialWithLastSolution,

    // Uses the online active set strategy when calling Solve() again
    // on an already solved QP problem with constant Hessian matrix
    // and/or constraint matrix.
    // WARNING: Will crash in qpOASES if this is used while the Hessian
    // and/or constraint matrix are varied.
    kSequentialWithConstantHessianConstraints,

    // Uses the online active set strategy when calling Solve() again
    // on an already solved QP problem with varied Hessian matrix
    // and/or constraint matrix.
    kSequentialWithVaryingHessianConstraints,
  };

  struct SolverOptions {
    int max_iterations = 100;

    // Solves QP with automatically sparsified Hessian and constraint matrices.
    bool sparsify = false;

    SolverProfile solver_profile = SolverProfile::kDefault;

    // The following options override details in `solver_profile` if set.
    // See qpOASES' Options.hpp for the original documentation.
    PrintLevel print_level = PrintLevel::kNone;
    std::optional<qpOASES::real_t> termination_tolerance;
    std::optional<bool> enable_regularisation;

    SequentialStrategy sequential_strategy = SequentialStrategy::kNonSequential;
  };

  SolverOptions& solver_options() { return solver_options_; }

  // The user should prepare the problem input via this accessor.
  QpData<T>& data() { return user_; }

  // Solves with prepared `data()` as input. Returns true iff succeeded.
  // The primal solution is saved in `data().solution`.
  bool Solve();

  // Solves with a subset of the prepared `data()` as input. Returns true iff succeeded.
  // `variable_subset` and `constraint_subset` are the indices of subset elements in the original problem.
  // Example: if the original problem has 3 variables x0, x1, x2, pass {1, 2} to solve the subset x1, x2.
  // The primal solution is saved in the subset part of `data().solution`.
  // Non-subset part of `data().solution` is set to `nonsubset_value` if provided, otherwise remains unchanged.
  bool SolveSubset(const Eigen::ArrayXi& variable_subset, const Eigen::ArrayXi& constraint_subset,
                   std::optional<T> nonsubset_value = {});

  // Returns the objective value of the last solution.
  qpOASES::real_t GetLastObjectiveValue() const;

 private:
  void CheckDimensions();
  void SetProblemOptions(qpOASES::SQProblem& problem);

  bool DoSolve(qpOASES::SQProblem& problem, QpData<qpOASES::real_t>& qp);

  qpOASES::SQProblem problem_;
  std::unique_ptr<qpOASES::SQProblem> subproblem_;

  QpData<T> user_;
  QpData<qpOASES::real_t> real_;

  SolverOptions solver_options_;
  bool qp_solved_ = false;
};
}  // namespace math
