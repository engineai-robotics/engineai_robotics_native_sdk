/**
 * @file pd_stand_runner.cc
 * @brief Implementation of the PD Stand Runner — smoothly interpolates the robot
 *        from its current pose to a target standing pose using PD position control.
 *
 * This Runner is typically the first Runner executed after the robot powers on.
 * It reads the current joint positions, then uses quintic polynomial interpolation
 * to smoothly move all joints to a predefined standing configuration over a
 * specified duration.
 *
 * Key features:
 *   - **Quintic interpolation**: Provides smooth, jerk-limited joint trajectories
 *     with zero velocity and acceleration at start and end points
 *   - **Safety check**: Validates that initial joint positions are within a
 *     configurable threshold of the target pose before executing (prevents
 *     large unexpected motions from unsafe starting poses)
 *   - **Auto-transition**: Optionally transitions to the next Runner in the
 *     sequence once the standing motion is complete
 *   - **Force-start override**: When strict motion check is disabled and the
 *     gamepad LT trigger is held, bypasses the safety check (useful for recovery)
 *
 * Notes for secondary developers:
 *   - The target standing pose, PD gains (Kp/Kd), and duration are all configured
 *     via `PdStandParam` in the YAML config file.
 *   - This Runner enables `parallel_by_classic_parser`, meaning the classic motion
 *     parser can run in parallel (e.g., for upper-body gestures during standup).
 */

#include "pd_stand/pd_stand_runner.h"

#include <iostream>
#include <string>
#include "math/interpolation.h"
#include "tool/concatenate_vector.h"

namespace runner {

// ============================================================================
// Runner Lifecycle Methods
// ============================================================================

/**
 * @brief Sets up the runtime context. Enables parallel classic parser control,
 *        allowing other motion modules to run alongside this Runner.
 */
void PdStandRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(true); }

/**
 * @brief Tears down the runtime context. Currently no cleanup needed.
 */
void PdStandRunner::TeardownContext() {}

/**
 * @brief Initializes the standing motion.
 *
 * Performs the following:
 *   1. Reload parameters if param_tag_ has been updated
 *   2. Record the current joint positions as the interpolation start point
 *   3. Load the target standing pose, PD gains, and motion duration from config
 *   4. Validate that the initial joint positions are within the safety threshold
 *      of the target pose (unless overridden by gamepad LT + disabled strict check)
 *   5. Set the initial motor command output to hold the current position
 *
 * @return true if initialization succeeds and the standup motion can begin,
 *         false if the joint position safety check fails.
 */
bool PdStandRunner::Enter() {
  // Reload parameters if a tag has been set
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::PdStandParam>(param_tag_);
  }
  std::cout << "Current tag: " << param_->GetTag() << std::endl;

  // Read configuration: auto-transition flag, current joint positions, target pose, PD gains
  auto_transition_ = param_->auto_transition.value_or(false);
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_init_);
  q_des_ = common::ConcatenateVectors(param_->desired_joint_position);  // Target standing pose [rad]
  kp_ = common::ConcatenateVectors(param_->stiffness);                  // PD proportional gain
  kd_ = common::ConcatenateVectors(param_->damping);                    // PD derivative gain
  duration_ = param_->duration;                                         // Interpolation duration [s]

  iter_ = 0;
  tau_ff_cmd_ = Eigen::VectorXd::Zero(q_init_.size());

  // --- Safety check: verify initial joint positions ---
  // Skip the check in MuJoCo simulation (always safe in sim)
  if (!common::IsInMujoco() && !CheckJointPositionBias()) {
    // Safety check failed — joints are too far from the target pose.
    // Allow a force-start override if strict_motion_check is disabled AND
    // the gamepad LT trigger is held past the threshold.
    auto gamepad_info = data_store_->gamepad_info.Get();
    constexpr double kGamepadThreshold = 0.8;
    if (global_options_param_->strict_motion_check == false && gamepad_info->LT > kGamepadThreshold) {
      LOG(INFO) << "Strict motion check is disabled, force stand";
      // Set initial output to hold current position (avoids a sudden jump)
      q_cmd_ = q_init_;
      qd_cmd_ = Eigen::VectorXd::Zero(q_init_.size());
      GetMutableOutput().SetCommand(q_cmd_, qd_cmd_, kp_, kd_, tau_ff_cmd_);
      return true;
    }
    return false;
  }

  // Set initial motor output to hold the current position
  // (the interpolation will gradually move from q_init_ to q_des_ in Run())
  q_cmd_ = q_init_;
  qd_cmd_ = Eigen::VectorXd::Zero(q_init_.size());
  GetMutableOutput().SetCommand(q_cmd_, qd_cmd_, kp_, kd_, tau_ff_cmd_);

  return true;
}

/**
 * @brief Validates that all joints are within an acceptable range of the target pose.
 *
 * Compares the current joint positions (q_init_) against the target standing
 * pose (q_des_). If any joint exceeds the configurable threshold (default: 1.0 rad),
 * logs a warning with per-joint diagnostic details and returns false.
 *
 * This prevents the robot from attempting a large, potentially dangerous motion
 * if it starts in an unexpected configuration (e.g., fallen over, manually moved).
 *
 * @return true if all joint biases are within the threshold, false otherwise.
 */
bool PdStandRunner::CheckJointPositionBias() {
  static constexpr double kJointPositionBiasThreshold = 1.0;
  double threshold = param_->initial_joint_position_bias_threshold.value_or(kJointPositionBiasThreshold);

  Eigen::VectorXd q_init_bias = q_init_ - q_des_;
  auto large_bias = (q_init_bias.array().abs() > threshold).matrix();
  if (large_bias.any()) {
    LOG(WARNING) << "Joint position bias exceeds threshold (" << threshold << ") at joints: ";
    for (int i = 0; i < model_param_->num_total_joints; ++i) {
      if (large_bias(i)) {
        LOG(WARNING) << "Joint [" << model_param_->GetJointNameByJointId(i) << "]: bias = " << q_init_bias[i];
      }
    }
    return false;
  }

  return true;
}

// ============================================================================
// Main Control Loop
// ============================================================================

/**
 * @brief Executes one step of the standup interpolation.
 *
 * Each cycle:
 *   1. Sets contact signals (all legs in contact, all arms non-contact)
 *      for the balance controller or other modules that depend on contact state
 *   2. Computes the current interpolation phase based on elapsed iterations
 *   3. Uses quintic (5th-order) polynomial interpolation to compute the
 *      instantaneous target position and velocity for all joints
 *   4. Sends the interpolated command to the motors
 *   5. If auto_transition is enabled and the motion is complete, triggers
 *      a transition to the next Runner in the sequence
 *
 * Quintic interpolation ensures: zero velocity and acceleration at t=0 and t=T,
 * producing smooth, jerk-limited motion suitable for humanoid standup.
 */
void PdStandRunner::Run() {
  // Set reference contact signals for downstream modules
  // (legs should be in contact during standing, arms should not)
  data_store_->reference_contact_signal_info->SetAllLegContactSignal();
  data_store_->reference_contact_signal_info->SetAllArmNonContactSignal();

  // Compute interpolation phase: clamped to [0, duration_]
  double phase = std::min(static_cast<double>(iter_) * runner_period_, duration_);

  // Quintic polynomial interpolation: q_init_ → q_des_ over duration_
  // Outputs: q_cmd_ (position), qd_cmd_ (velocity) at the current phase
  math::QuinticInterpolate(q_init_, q_des_, duration_, phase, q_cmd_, qd_cmd_);

  // Write the interpolated command to the output buffer
  GetMutableOutput().SetCommand(q_cmd_, qd_cmd_, kp_, kd_, tau_ff_cmd_);

  ++iter_;

  // Auto-transition: once the interpolation duration has elapsed, signal exit
  // so the system can transition to the next Runner (e.g., RL walking)
  if (static_cast<double>(iter_) * runner_period_ > duration_ && auto_transition_) {
    LOG(INFO) << "PdStandRunner: Transitioning to next runner after " << duration_ << " seconds.";
    SetRunnerState(RunnerState::kTryExit);
  }
}

// ============================================================================
// Runner Exit Logic
// ============================================================================

/**
 * @brief Immediately allows exit (standing interpolation has a definite end point,
 *        so no gradual exit transition is needed).
 */
TransitionState PdStandRunner::TryExit() {
  return TransitionState::kCompleted;
  // if (static_cast<double>(iter_) * runner_period_ > duration_) {
  //   return TransitionState::kCompleted;
  // } else {
  //   Run();
  //   return TransitionState::kTrying;
  // }
}

/**
 * @brief Post-exit cleanup. Currently no additional actions needed.
 */
bool PdStandRunner::Exit() { return true; }

/**
 * @brief Runner termination. Currently no additional actions needed.
 */
void PdStandRunner::End() {}
}  // namespace runner
