#include "rl_lab/rl_lab_runner.h"

#include <glog/logging.h>
#include <iostream>

#include "math/fast_pose_composition_functions.h"
#include "math/rotation_matrix.h"
#include "tool/concatenate_vector.h"

namespace runner {

RlLabRunner::RlLabRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
    : MotionRunner(name, data_store) {
  // Creates param ptr
  param_ = data::ParamManager::create<data::RlLabParam>();
}

void RlLabRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(false); }

void RlLabRunner::TeardownContext() { data_store_->parallel_by_classic_parser.store(true); }

bool RlLabRunner::Enter() {
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::RlLabParam>(param_tag_);
  }

  // Creates mlp net model
  mlp_net_ = std::make_unique<math::MNNModel>(
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->policy_file));
  non_command_observation_dim_ = param_->num_observations - kCommandObservationDim;
  if (non_command_observation_dim_ <= 0) {
    LOG(ERROR) << "Invalid num_observations: " << param_->num_observations
               << ", must be larger than command dim " << kCommandObservationDim;
    non_command_observation_dim_ = 0;
  }
  const int expected_non_command_dim = param_->num_actions * 3 + 6;
  if (non_command_observation_dim_ != expected_non_command_dim) {
    LOG(WARNING) << "RlLabRunner observation dim mismatch, num_observations="
                 << param_->num_observations << ", expected "
                 << (expected_non_command_dim + kCommandObservationDim);
  }
  const int history_steps = param_->num_include_obs_steps;
  joint_pos_history_.setZero(history_steps, param_->num_actions);
  joint_vel_history_.setZero(history_steps, param_->num_actions);
  action_history_.setZero(history_steps, param_->num_actions);
  base_ang_vel_history_.setZero(history_steps, kCommandObservationDim);
  projected_gravity_history_.setZero(history_steps, kCommandObservationDim);
  mlp_net_action_.setZero(param_->num_actions);

  // Initializes some low pass filters
  lpf_command_ = std::make_unique<math::FirstOrderLowPassFilter<Eigen::Vector3d>>(
      param_->remote_command_sampling_frequency, param_->remote_command_cut_off_frequency);

  // Initializes active joint num
  active_joint_idx_.setZero(param_->num_actions);
  int i = 0;
  for (const std::string& name : param_->active_joint_names) {
    active_joint_idx_(i++) = model_param_->joint_id_in_total_limb.at(name);
  }

  default_joint_q_ = common::ConcatenateVectors(param_->default_joint_q);
  joint_kp_ = common::ConcatenateVectors(param_->joint_kp);
  joint_kd_ = common::ConcatenateVectors(param_->joint_kd);
  action_scale_ = common::ConcatenateVectors(param_->action_scale);

  time_ = 0.0;
  is_first_time_ = true;
  last_remote_command_ = *data_store_->gamepad_info.Get();
  imu_install_bias_ = param_->imu_install_bias.value_or(Eigen::Vector3d::Zero());

  // Initializes some low pass filters
  lpf_command_->Reset();

  // Sets all motor command to zero
  GetMutableOutput().Reset();

  data_store_->parallel_by_classic_parser.store(false);
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  initial_joint_q_ = q_real_;
  return true;
}

void RlLabRunner::Run() {
  // Gets current joint pos and vel
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_real_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_real_);
  UpdateIMUInstalBias();
  UpdateRobotBaseOrientation();
  UpdateRemoteCommandBias();
  UpdateRemoteCommand();
  last_remote_command_ = *data_store_->gamepad_info.Get();
  CalculateObservation();
  CalculateMotorCommand();
  SendMotorCommand();
  time_ += param_->control_dt;
}

TransitionState RlLabRunner::TryExit() { return TransitionState::kCompleted; }

bool RlLabRunner::Exit() {
  data_store_->parallel_by_classic_parser.store(true);
  return true;
}

void RlLabRunner::End() {}

void RlLabRunner::UpdateIMUInstalBias() {
  bool imu_flag = false;

  if (data_store_->gamepad_info.Get()->START && data_store_->gamepad_info.Get()->CROSS_Y == -1 &&
      data_store_->gamepad_info.Get()->CROSS_Y < last_remote_command_.CROSS_Y) {
    imu_install_bias_.x() += param_->imu_install_delta_bias;
    imu_flag = true;
  }
  if (data_store_->gamepad_info.Get()->START && data_store_->gamepad_info.Get()->CROSS_Y == 1 &&
      data_store_->gamepad_info.Get()->CROSS_Y > last_remote_command_.CROSS_Y) {
    imu_install_bias_.x() -= param_->imu_install_delta_bias;
    imu_flag = true;
  }
  if (data_store_->gamepad_info.Get()->START && data_store_->gamepad_info.Get()->CROSS_X == 1 &&
      data_store_->gamepad_info.Get()->CROSS_X > last_remote_command_.CROSS_X) {
    imu_install_bias_.y() += param_->imu_install_delta_bias;
    imu_flag = true;
  }
  if (data_store_->gamepad_info.Get()->START && data_store_->gamepad_info.Get()->CROSS_X == -1 &&
      data_store_->gamepad_info.Get()->CROSS_X < last_remote_command_.CROSS_X) {
    imu_install_bias_.y() -= param_->imu_install_delta_bias;
    imu_flag = true;
  }
  if (imu_flag) {
    LOG(INFO) << "imu_install_bias: " << imu_install_bias_;
  }
}

void RlLabRunner::UpdateRobotBaseOrientation() {
  Eigen::Matrix3d R_install = math::RollPitchYawd(imu_install_bias_).ToRotationMatrix().matrix();
  Eigen::Matrix3d R_local = math::RotationMatrixd(data_store_->imu_info.Get()->quaternion).matrix();
  rotation_install_ = R_install;
  rotation_measured_ = R_local;
  rotation_real_ = R_local * R_install.transpose();
  angular_vel_real_ = rotation_real_.transpose() * R_local * data_store_->imu_info.Get()->angular_velocity;
  euler_xyz_ = math::RollPitchYawd(math::RotationMatrixd(rotation_real_)).vector();
  projected_gravity_real_ = -rotation_real_.transpose() * Eigen::Vector3d::UnitZ();
  // LOG(INFO) << "euler: " << euler_xyz_.transpose();
}

void RlLabRunner::UpdateRemoteCommandBias() {
  bool command_flag = false;
  if (data_store_->gamepad_info.Get()->BACK && data_store_->gamepad_info.Get()->CROSS_Y == -1 &&
      data_store_->gamepad_info.Get()->CROSS_Y < last_remote_command_.CROSS_Y) {
    command_bias_.y() -= param_->linear_vel_delta_bias;
    command_flag = true;
  }
  if (data_store_->gamepad_info.Get()->BACK && data_store_->gamepad_info.Get()->CROSS_Y == 1 &&
      data_store_->gamepad_info.Get()->CROSS_Y > last_remote_command_.CROSS_Y) {
    command_bias_.y() += param_->linear_vel_delta_bias;
    command_flag = true;
  }
  if (data_store_->gamepad_info.Get()->BACK && data_store_->gamepad_info.Get()->CROSS_X == 1 &&
      data_store_->gamepad_info.Get()->CROSS_X > last_remote_command_.CROSS_X) {
    command_bias_.x() += param_->linear_vel_delta_bias;
    command_flag = true;
  }
  if (data_store_->gamepad_info.Get()->BACK && data_store_->gamepad_info.Get()->CROSS_X == -1 &&
      data_store_->gamepad_info.Get()->CROSS_X < last_remote_command_.CROSS_X) {
    command_bias_.x() -= param_->linear_vel_delta_bias;
    command_flag = true;
  }
  if (command_flag) {
    LOG(INFO) << "command_bias: " << command_bias_;
  }
}

void RlLabRunner::UpdateRemoteCommand() {
  if (data_store_->gamepad_info.Get()->A == 1 && data_store_->gamepad_info.Get()->A > last_remote_command_.A) {
    still_pressed_ = true;
  }

  command_.x() = data_store_->gamepad_info.Get()->LeftStick_X * param_->command_scale.x();
  command_.y() = data_store_->gamepad_info.Get()->LeftStick_Y * param_->command_scale.y();
  command_.z() = data_store_->gamepad_info.Get()->RightStick_Y * param_->command_scale.z();
  if (param_->enable_remote_command_lpf) {
    command_ = lpf_command_->Update(command_);
  }
  command_ += command_bias_ + param_->command_bias;
}

void RlLabRunner::CalculateObservation() {
  if (non_command_observation_dim_ <= 0) {
    return;
  }

  const Eigen::VectorXd joint_pos = (q_real_ - default_joint_q_)(active_joint_idx_);
  const Eigen::VectorXd joint_vel = qd_real_(active_joint_idx_);
  const Eigen::RowVectorXd joint_pos_row = joint_pos.transpose();
  const Eigen::RowVectorXd joint_vel_row = joint_vel.transpose();
  const Eigen::RowVectorXd action_row = mlp_net_action_.transpose();
  const Eigen::RowVector3d base_ang_vel_row = angular_vel_real_.transpose();
  const Eigen::RowVector3d projected_gravity_row = projected_gravity_real_.transpose();

  const int history_steps = joint_pos_history_.rows();
  if (is_first_time_) {
    is_first_time_ = false;
    joint_pos_history_.rowwise() = joint_pos_row;
    joint_vel_history_.rowwise() = joint_vel_row;
    action_history_.rowwise() = action_row;
    base_ang_vel_history_.rowwise() = base_ang_vel_row;
    projected_gravity_history_.rowwise() = projected_gravity_row;
  } else if (history_steps > 1) {
    joint_pos_history_.topRows(history_steps - 1) = joint_pos_history_.bottomRows(history_steps - 1);
    joint_vel_history_.topRows(history_steps - 1) = joint_vel_history_.bottomRows(history_steps - 1);
    action_history_.topRows(history_steps - 1) = action_history_.bottomRows(history_steps - 1);
    base_ang_vel_history_.topRows(history_steps - 1) = base_ang_vel_history_.bottomRows(history_steps - 1);
    projected_gravity_history_.topRows(history_steps - 1) = projected_gravity_history_.bottomRows(history_steps - 1);

    joint_pos_history_.bottomRows(1) = joint_pos_row;
    joint_vel_history_.bottomRows(1) = joint_vel_row;
    action_history_.bottomRows(1) = action_row;
    base_ang_vel_history_.bottomRows(1) = base_ang_vel_row;
    projected_gravity_history_.bottomRows(1) = projected_gravity_row;
  } else {
    joint_pos_history_.bottomRows(1) = joint_pos_row;
    joint_vel_history_.bottomRows(1) = joint_vel_row;
    action_history_.bottomRows(1) = action_row;
    base_ang_vel_history_.bottomRows(1) = base_ang_vel_row;
    projected_gravity_history_.bottomRows(1) = projected_gravity_row;
  }
}

void RlLabRunner::CalculateMotorCommand() {
  const int history_steps = param_->num_include_obs_steps;
  const int non_command_size = non_command_observation_dim_ * history_steps;
  Eigen::VectorXd obs = Eigen::VectorXd::Zero(non_command_size + kCommandObservationDim);
  int offset = 0;
  const auto append_history = [&](const auto& history) {
    Eigen::Map<const Eigen::VectorXd> flat(history.data(), history.size());
    obs.segment(offset, flat.size()) = flat;
    offset += flat.size();
  };
  append_history(joint_pos_history_);
  append_history(joint_vel_history_);
  append_history(action_history_);
  append_history(base_ang_vel_history_);
  append_history(projected_gravity_history_);
  obs.tail(kCommandObservationDim) = command_;

  mlp_net_action_ = (mlp_net_->Inference(obs.cast<float>())).cast<double>();
  mlp_net_action_ = mlp_net_action_.cwiseMax(-param_->action_clip).cwiseMin(param_->action_clip);
  q_des_ = default_joint_q_;
  q_des_(active_joint_idx_) += mlp_net_action_.cwiseProduct(action_scale_);
  if (param_->transition_time.has_value() && param_->transition_time.value() > 0 &&
      time_ < param_->transition_time.value()) {
    float ratio = time_ / param_->transition_time.value();
    q_des_ = ratio * q_des_ + (1.0f - ratio) * initial_joint_q_;
  }
}

void RlLabRunner::SendMotorCommand() {
  qd_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);

  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);
}
}  // namespace runner
