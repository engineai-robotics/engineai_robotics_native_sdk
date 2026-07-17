/**
 * @file passive_runner.cc
 * @brief Implementation of the Passive Runner — a safe "power-off" mode that
 *        disables all active joint control.
 *
 * This Runner puts the robot into a passive (compliant) state where no active
 * position or torque control is applied. All joints are free to move under
 * external forces. An optional per-joint damping can be configured to provide
 * viscous resistance, preventing joints from swinging freely.
 *
 * Typical use cases:
 *   - **Emergency stop**: Immediately disable all motor control
 *   - **Manual manipulation**: Allow a human operator to physically move the
 *     robot's limbs (e.g., for calibration or pose adjustment)
 *   - **Safe initial state**: The default state before any active Runner takes over
 *
 * Motor command in passive mode:
 *   tau = kp*(0-q) + kd*(0-qd) + 0
 *   Since kp=0 and q_des=0, qd_des=0, tau_ff=0, this simplifies to:
 *   tau = -kd * qd    (pure velocity damping if kd > 0, or zero torque if kd = 0)
 *
 * Notes for secondary developers:
 *   - To adjust damping behavior, modify the `damping` parameter in the YAML config.
 *   - If damping is not specified in config, all joints will have zero damping (fully passive).
 *   - This Runner enables parallel classic parser mode.
 */

#include "passive/passive_runner.h"

#include <glog/logging.h>
#include "tool/concatenate_vector.h"

namespace runner {

// ============================================================================
// Runner Lifecycle Methods
// ============================================================================

/**
 * @brief Sets up the runtime context. Enables parallel classic parser control.
 */
void PassiveRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(true); }

/**
 * @brief Tears down the runtime context. Currently no cleanup needed.
 */
void PassiveRunner::TeardownContext() {}

/**
 * @brief Initializes the passive mode.
 *
 * Reloads parameters if param_tag_ has been updated, pre-allocates a zero vector
 * for joint commands, and resets all motor outputs to zero.
 *
 * @return Always returns true (passive mode has no failure conditions).
 */
bool PassiveRunner::Enter() {
  // Reload parameters if a tag has been set
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::PassiveParam>(param_tag_);
  }

  // Pre-allocate a zero vector used for position, velocity, stiffness, and feedforward
  zeros_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);

  // Reset all motor outputs to zero (no active control)
  GetMutableOutput().Reset();
  return true;
}

// ============================================================================
// Main Control Loop
// ============================================================================

/**
 * @brief Executes one cycle of passive mode.
 *
 * Each cycle:
 *   1. Sets all contact signals to non-contact (robot is not actively standing)
 *   2. Reads the optional damping coefficients from config
 *   3. Sends motor commands with all zeros except for the Kd (damping) term
 *
 * The motor command is:
 *   - q_des = 0, qd_des = 0, kp = 0, tau_ff = 0
 *   - kd = configured damping (or 0 if not specified)
 *
 * This means the only active torque is velocity damping: tau = -kd * qd_real,
 * which provides viscous resistance proportional to joint velocity.
 */
void PassiveRunner::Run() {
  // Mark all limbs as non-contact (robot is not actively balancing)
  data_store_->reference_contact_signal_info->SetAllNonContactSignal();

  // Load damping coefficients from config (or use zeros if not configured)
  if (param_->damping) {
    damping_ = common::ConcatenateVectors(param_->damping.value());
  } else {
    damping_ = zeros_;
  }

  // Send motor command: all zeros except kd (damping), producing pure velocity damping
  GetMutableOutput().SetCommand(/*q_des=*/zeros_, /*qd_des=*/zeros_, /*kp=*/zeros_, damping_, /*tau_ff=*/zeros_);
}

// ============================================================================
// Runner Exit Logic
// ============================================================================

/**
 * @brief Immediately allows exit (passive mode can be left at any time).
 */
TransitionState PassiveRunner::TryExit() { return TransitionState::kCompleted; }

/**
 * @brief Post-exit cleanup. Currently no additional actions needed.
 */
bool PassiveRunner::Exit() { return true; }

/**
 * @brief Runner termination. Currently no additional actions needed.
 */
void PassiveRunner::End() {}
}  // namespace runner
