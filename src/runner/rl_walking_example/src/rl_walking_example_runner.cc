/**
 * @file rl_walking_example_runner.cc
 * @brief Implementation of the RL (Reinforcement Learning) walking policy Runner.
 *
 * This file implements the deployment of an RL-trained walking policy on a real
 * humanoid robot. The core control loop follows a standard perception-decision-action
 * pipeline each cycle:
 *   1. Read sensor data (IMU, joint encoders) to capture the robot's current state
 *   2. Assemble the state into an observation vector matching the RL training environment
 *   3. Feed the observation into an MLP policy network for forward inference to get an action
 *   4. Convert the action into target joint positions and send them to the motors via PD control
 *
 * Notes for secondary developers:
 *   - If you need to modify the observation composition (e.g., adding new sensor inputs),
 *     you MUST update CalculateObservation() AND the training-side observation definition
 *     simultaneously, otherwise an input dimension mismatch will occur.
 *   - The policy model file (.mnn format) path is configured via the `policy_file` parameter
 *     and must match the model exported from the training side.
 *   - Joint mapping is determined by the `active_joint_names` parameter, which defines
 *     which subset of joints the RL policy controls and their ordering. This MUST be
 *     identical to the training-side configuration.
 */

#include "rl_walking_example/rl_walking_example_runner.h"

#include <glog/logging.h>

#include "math/rotation_matrix.h"
#include "tool/concatenate_vector.h"

namespace runner {

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Constructor. Initializes the parameter pointer.
 * @param name       Runner identifier, passed in by the runner registration macro
 *                   (e.g., "rl_walking_example_runner").
 * @param data_store Shared data bus for exchanging sensor data, control commands,
 *                   and other information between this Runner and other system modules.
 *
 * @note `param_` is created via ParamManager and automatically loads parameters
 *       from the corresponding YAML configuration file. To use different parameter
 *       sets for different robot models or scenarios, switch via param_tag_ (see Enter()).
 */
RlWalkingExampleRunner::RlWalkingExampleRunner(std::string_view name,
                                               const std::shared_ptr<data::DataStore>& data_store)
    : MotionRunner(name, data_store) {
  // Create the RL walking parameter instance using the default tag (loaded from config)
  param_ = data::ParamManager::create<data::RlWalkingExampleParam>();
}

// ============================================================================
// Runner Lifecycle Methods
// ============================================================================

/**
 * @brief Sets up the runtime context, called before the Runner is scheduled to run.
 *
 * Sets `parallel_by_classic_parser` to false, indicating that this Runner does NOT
 * use the classic parser's parallel motion control mode — the RL policy has exclusive
 * control over all active joints.
 */
void RlWalkingExampleRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(false); }

/**
 * @brief Tears down the runtime context, called after the Runner is unscheduled.
 *        Currently no additional cleanup is required.
 */
void RlWalkingExampleRunner::TeardownContext() {}

/**
 * @brief Initialization logic executed upon entering this Runner. Allocates resources
 *        and resets all internal state.
 *
 * This method performs the following steps:
 *   1. Hot-switch the parameter set if param_tag_ has changed (supports runtime config switching)
 *   2. Load the MLP policy network model (.mnn file)
 *   3. Initialize the observation buffer and action buffer
 *   4. Initialize the low-pass filter for remote control commands (smooths gamepad input)
 *   5. Build the "policy joint index → full-body joint index" mapping
 *   6. Load default joint positions, PD gains (Kp/Kd), and action scale coefficients
 *   7. Record the current joint positions as the starting point for smooth interpolation
 *
 * @return true if initialization succeeds and the Runner is ready to run.
 */
bool RlWalkingExampleRunner::Enter() {
  // --- Step 1: Hot-switch parameters ---
  // If an external module has set a new param_tag_ (e.g., via the task system), reload parameters
  if (!param_tag_.empty() && param_tag_ != last_param_tag_) {
    param_ = data::ParamManager::create<data::RlWalkingExampleParam>(param_tag_);
    last_param_tag_ = param_tag_;
  }

  // --- Step 2: Load the MLP policy network ---
  // policy_file is the model file path relative to the config directory (.mnn format).
  // MNNModel wraps the MNN inference framework for forward inference.
  mlp_net_ = std::make_unique<math::MNNModel>(
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->policy_file));

  // --- Step 3: Initialize observation and action buffers ---
  // Observation matrix dimensions: [num_observations x num_include_obs_steps]
  // Columns represent time steps, supporting multi-step historical observation stacking
  // (used for temporal features in the policy network).
  mlp_net_observation_.setZero(param_->num_observations, param_->num_include_obs_steps);
  // Action vector dimension: [num_actions], i.e., the number of joints controlled by the policy
  mlp_net_action_.setZero(param_->num_actions);

  // --- Step 4: Initialize remote command low-pass filter ---
  // Applies low-pass filtering to gamepad input, removing high-frequency jitter
  // for smoother robot locomotion
  lpf_command_ = std::make_unique<math::FirstOrderLowPassFilter<Eigen::Vector3d>>(
      param_->remote_command_sampling_frequency, param_->remote_command_cut_off_frequency);

  // --- Step 5: Build joint index mapping ---
  // active_joint_idx_ maps policy action indices to actual indices in the full-body joint array.
  // For example, the policy may only control 12 joints out of 20+ total joints; this mapping
  // tells us which full-body joint corresponds to action[i].
  active_joint_idx_.setZero(param_->num_actions);
  int i = 0;
  for (const std::string& name : param_->active_joint_names) {
    active_joint_idx_(i++) = model_param_->joint_id_in_total_limb.at(name);
  }

  // --- Step 6: Load joint control parameters ---
  // Parameters are organized as vector<vector> in the config (grouped by limb);
  // ConcatenateVectors flattens them into a single 1D vector.
  default_joint_q_ = common::ConcatenateVectors(param_->default_joint_q);  // Default standing pose [rad]
  joint_kp_ = common::ConcatenateVectors(param_->joint_kp);                // PD controller proportional gain
  joint_kd_ = common::ConcatenateVectors(param_->joint_kd);                // PD controller derivative gain
  action_scale_ = common::ConcatenateVectors(param_->action_scale);        // Action scaling coefficients

  // --- Step 7: Reset runtime state ---
  time_ = 0.0;
  is_first_time_ = true;
  command_.setZero();  // Remote command (vx, vy, yaw_rate) reset to zero
  imu_install_bias_ = param_->imu_install_bias.value_or(Eigen::Vector3d::Zero());  // IMU mounting bias compensation

  lpf_command_->Reset();

  // Zero out all motor commands (safety measure to avoid inheriting residual commands)
  GetMutableOutput().Reset();

  // Record the actual joint positions at entry, can be used for smooth startup interpolation
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  initial_joint_q_ = q_real_;

  return true;
}

// ============================================================================
// Main Control Loop
// ============================================================================

/**
 * @brief Main loop called once per control cycle, executing the full
 *        perception → decision → action pipeline.
 *
 * The call frequency is determined by the system scheduler and typically corresponds
 * to param_->control_dt (e.g., 50Hz or 100Hz).
 *
 * Execution order is strictly fixed and must not be reordered:
 *   1. Read joint sensor data (position, velocity)
 *   2. Update remote commands (gamepad input)
 *   3. Assemble the RL observation vector
 *   4. Run policy network inference and compute target joint positions
 *   5. Send motor commands
 */
void RlWalkingExampleRunner::Run() {
  // Read current joint positions and velocities
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);

  UpdateRemoteCommand();    // Step 2: Update velocity commands from gamepad
  CalculateObservation();   // Step 3: Assemble the observation vector
  CalculateMotorCommand();  // Step 4: Run policy inference + compute target joint positions
  SendMotorCommand();       // Step 5: Send motor commands to actuators

  time_ += param_->control_dt;  // Accumulate elapsed runtime
}

// ============================================================================
// Runner Exit Logic
// ============================================================================

/**
 * @brief Transition logic during Runner exit attempts.
 *
 * When the system requests this Runner to exit, this method is called repeatedly:
 *   - If the joint override command is still enabled, continue executing Run()
 *     to maintain control and return kTrying (exit not yet complete).
 *   - Once the override command is disabled, return kCompleted to allow exit.
 *
 * This design ensures the robot does not abruptly lose control during the exit process.
 */
TransitionState RlWalkingExampleRunner::TryExit() {
  auto joint_override_command = data_store_->joint_override_command.Get();
  if (joint_override_command->IsEnable() == true) {
    Run();
    return TransitionState::kTrying;
  } else {
    return TransitionState::kCompleted;
  }
}

/**
 * @brief Cleanup logic after exiting the Runner. Currently no additional actions needed.
 */
bool RlWalkingExampleRunner::Exit() { return true; }

/**
 * @brief Called when the Runner terminates. Currently no additional actions needed.
 */
void RlWalkingExampleRunner::End() {}

// ============================================================================
// Remote Command Processing
// ============================================================================

/**
 * @brief Reads gamepad input and converts it into velocity commands.
 *
 * Gamepad stick mapping:
 *   - Left stick X axis  → command_.x(): Forward/backward linear velocity (m/s)
 *   - Left stick Y axis  → command_.y(): Left/right lateral velocity (m/s)
 *   - Right stick Y axis → command_.z(): Yaw rotational velocity (rad/s)
 *
 * Positive and negative directions can have different scaling factors
 * (command_scale_pos / command_scale_neg), allowing asymmetric speed ranges
 * (common requirement since forward walking is typically more stable than backward).
 *
 * An optional low-pass filter can be enabled to smooth sudden input changes from
 * the gamepad, preventing velocity command spikes that could destabilize the robot.
 */
void RlWalkingExampleRunner::UpdateRemoteCommand() {
  const auto& gamepad = data_store_->gamepad_info.Get();

  // Select positive/negative scaling factor based on stick direction
  command_.x() = (gamepad->LeftStick_X >= 0) ? gamepad->LeftStick_X * param_->command_scale_pos.x()
                                             : gamepad->LeftStick_X * param_->command_scale_neg.x();
  command_.y() = (gamepad->LeftStick_Y >= 0) ? gamepad->LeftStick_Y * param_->command_scale_pos.y()
                                             : gamepad->LeftStick_Y * param_->command_scale_neg.y();
  command_.z() = (gamepad->RightStick_Y >= 0) ? gamepad->RightStick_Y * param_->command_scale_pos.z()
                                              : gamepad->RightStick_Y * param_->command_scale_neg.z();

  // Optional: apply low-pass filtering to smooth the input
  if (param_->enable_remote_command_lpf) {
    command_ = lpf_command_->Update(command_);
  }
}

// ============================================================================
// Observation Assembly
// ============================================================================

/**
 * @brief Assembles the observation vector required by the RL policy network.
 *
 * This function performs three tasks:
 *   1. Compute the robot body orientation from IMU data (with mounting bias compensation)
 *   2. Concatenate sensor data into a single-step observation vector
 *   3. Maintain a sliding window buffer of multi-step historical observations
 *
 * ## Single-Step Observation Vector Composition
 * The ordering of components MUST match the training environment definition exactly:
 *
 *   | Component                     | Dimension        | Description                               |
 *   |-------------------------------|------------------|-------------------------------------------|
 *   | q_real - default_joint_q      | num_actions      | Joint position offset from default [rad]  |
 *   | qd_real                       | num_actions      | Joint angular velocity [rad/s]            |
 *   | mlp_net_action (previous)     | num_actions      | Previous step's policy action output      |
 *   | angular_velocity (body frame) | 3                | IMU angular velocity in body frame [rad/s]|
 *   | projected_gravity             | 3                | Gravity projection in body frame          |
 *
 * @warning Modifying the observation composition or ordering requires corresponding
 *          changes on the training side and model retraining.
 */
void RlWalkingExampleRunner::CalculateObservation() {
  // --- 1. IMU orientation computation (with mounting bias compensation) ---
  // R_install: Rotation matrix for IMU mounting bias (compensates for the physical
  //            mounting angle offset between the IMU and its theoretical position)
  Eigen::Matrix3d R_install = math::RotationMatrixd(math::RollPitchYawd(imu_install_bias_)).matrix();
  // R_local: Raw orientation reported by the IMU (world frame → IMU frame)
  Eigen::Matrix3d R_local = math::RotationMatrixd(data_store_->imu_info.Get()->quaternion).matrix();
  // R_real: Actual robot body orientation after bias compensation
  // Formula: R_real = R_local * R_install^T
  Eigen::Matrix3d R_real = R_local * R_install.transpose();

  // Transform IMU-measured angular velocity from IMU frame to robot body frame
  Eigen::Vector3d w_real = R_real.transpose() * R_local * data_store_->imu_info.Get()->angular_velocity;
  Eigen::Vector3d euler_xyz = math::RollPitchYawd(math::RotationMatrixd(R_real)).vector();

  // Compute the gravity vector projected into the body frame.
  // When the robot is upright, this vector is (0, 0, -1); when tilted,
  // the components change, allowing the policy to infer body orientation.
  Eigen::Vector3d projected_gravity_real = -R_real.transpose() * Eigen::Vector3d::UnitZ();

  // --- 2. Concatenate the single-step observation vector ---
  // Note: The velocity command (command_) is NOT included here; it is concatenated
  // separately in CalculateMotorCommand() as the policy network's command input.
  Eigen::VectorXd mlp_net_observation_single = Eigen::VectorXd::Zero(param_->num_observations);
  mlp_net_observation_single <<                         //
      (q_real_ - default_joint_q_)(active_joint_idx_),  // Joint position offset [num_actions]
      qd_real_(active_joint_idx_),                      // Joint angular velocity [num_actions]
      mlp_net_action_,                                  // Previous action [num_actions]
      w_real,                                           // Body-frame angular velocity [3]
      projected_gravity_real;                           // Body-frame gravity projection [3]

  // --- Observation scaling and clipping ---
  // Scale by configured factors (matches the normalization used during training)
  mlp_net_observation_single.array() *= param_->observation_scale.array();
  // Clip observations to prevent abnormal sensor values from destabilizing the policy
  mlp_net_observation_single =
      mlp_net_observation_single.cwiseMax(-param_->observation_clip).cwiseMin(param_->observation_clip);

  // --- 3. Update the multi-step historical observation sliding window ---
  if (is_first_time_) {
    // First run: fill all history steps with the current observation
    // (avoids feeding all-zero history data to the policy network)
    is_first_time_ = false;
    mlp_net_observation_.setZero(param_->num_observations, param_->num_include_obs_steps);
    mlp_net_action_.setZero(param_->num_actions);
    mlp_net_observation_.colwise() = mlp_net_observation_single;
  } else {
    // Subsequent runs: shift the sliding window left by one step,
    // placing the latest observation in the rightmost column
    mlp_net_observation_.leftCols(param_->num_include_obs_steps - 1) =
        mlp_net_observation_.rightCols(param_->num_include_obs_steps - 1);
    mlp_net_observation_.rightCols(1) = mlp_net_observation_single;
  }
}

// ============================================================================
// Policy Inference and Motor Command Computation
// ============================================================================

/**
 * @brief Runs the MLP policy network inference, converting observations into
 *        target joint positions.
 *
 * Inference pipeline:
 *   1. Scale the remote velocity command by configured coefficients
 *   2. Concatenate multi-step historical observations (column-flattened) and the
 *      velocity command into the final input vector
 *   3. Run MNN forward inference to obtain the action vector
 *   4. Clip the action (prevents unsafe outputs)
 *   5. Map action to target joint positions: q_des = default_q + action * action_scale
 *
 * @note Policy network input vector layout:
 *       [obs_step_0, obs_step_1, ..., obs_step_N, command(3)]
 *       where each obs_step has dimension num_observations, giving a total input
 *       dimension of: num_observations * num_include_obs_steps + 3.
 */
void RlWalkingExampleRunner::CalculateMotorCommand() {
  // Scale the velocity command by configured coefficients
  // (matches the command normalization used during training)
  Eigen::Vector3d command_obs = Eigen::Vector3d::Zero();
  command_obs = command_.cwiseProduct(param_->command_obs_scale);

  // Assemble the final policy network input vector
  Eigen::VectorXd obs;
  obs = Eigen::VectorXd::Zero(param_->num_observations * param_->num_include_obs_steps + 3);
  // Flatten the observation matrix into a 1D vector (transpose first because Eigen
  // uses column-major storage, but the network expects data ordered by time step)
  obs.head(param_->num_observations * param_->num_include_obs_steps) =
      Eigen::Map<Eigen::VectorXd>(mlp_net_observation_.transpose().data(), mlp_net_observation_.size());
  // Append the velocity command at the tail of the observation vector
  obs.tail(3) = command_obs;

  // Run MLP forward inference (float precision for inference, cast back to double)
  mlp_net_action_ = (mlp_net_->Inference(obs.cast<float>())).cast<double>();

  // Clip the action to prevent policy outputs from exceeding safe ranges
  mlp_net_action_ = mlp_net_action_.cwiseMax(-param_->action_clip).cwiseMin(param_->action_clip);

  // Map action to target joint positions:
  //   q_des[active_joints] = default_q[active_joints] + action * action_scale
  // Non-active joints retain their default positions (not controlled by the policy)
  q_des_ = default_joint_q_;
  q_des_(active_joint_idx_) += mlp_net_action_.cwiseProduct(action_scale_);
}

// ============================================================================
// Motor Command Dispatch
// ============================================================================

/**
 * @brief Sends the computed target joint positions to the motor controllers.
 *
 * Sets PD control parameters via the ControlOutput interface:
 *   - q_des_:      Target joint positions [rad]
 *   - qd_des_:     Target joint velocities [rad/s] (set to 0, pure position control)
 *   - joint_kp_:   PD controller proportional gain
 *   - joint_kd_:   PD controller derivative gain
 *   - tau_ff_des_: Feedforward torque [Nm] (set to 0)
 *
 * The actual motor torque is computed by the low-level driver using:
 *   tau = kp * (q_des - q_real) + kd * (qd_des - qd_real) + tau_ff
 *
 * @note To add feedforward torque compensation (e.g., gravity compensation),
 *       secondary developers can modify tau_ff_des_ before calling SetCommand().
 */
void RlWalkingExampleRunner::SendMotorCommand() {
  // Target velocity and feedforward torque are both zero (pure PD position control mode)
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);

  // Write control commands to the output buffer for dispatch to motor drivers
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);
}

}  // namespace runner
