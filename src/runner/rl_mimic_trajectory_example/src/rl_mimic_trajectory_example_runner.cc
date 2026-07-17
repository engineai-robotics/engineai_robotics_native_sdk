#include "rl_mimic_trajectory_example/rl_mimic_trajectory_example_runner.h"

#include <glog/logging.h>
#include <iostream>
#include <limits>

#include "math/interpolation.h"
#include "math/roll_pitch_yaw.h"
#include "math/rotation_matrix.h"
#include "tool/concatenate_vector.h"

namespace runner {

void RlMimicTrajectoryRunner::SetupContext() { data_store_->parallel_by_classic_parser.store(false); }

void RlMimicTrajectoryRunner::TeardownContext() {}

/**
 * @brief 进入runner状态，初始化参数和状态。
 * @return true 初始化成功
 */
bool RlMimicTrajectoryRunner::Enter() {
  // If param_tag changed, reload configuration and reset all dependent resources
  if (param_tag_ != last_param_tag_) {
    LOG(INFO) << "Reloading config for param_tag: " << param_tag_;
    policy_net_ = nullptr;
    current_traj_ = nullptr;
    policy_model_.reset();
    observation_manager_.reset();
    param_ = data::ParamManager::create<data::RlMinicTrajectoryParam>(param_tag_);
    last_param_tag_ = param_tag_;
  }

  Init();

  run_reference_trajectory_ = true;
  transition_iter_ = 0;
  is_first_time_ = true;
  initial_quat_set_ = false;

  GetMutableOutput().Reset();
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, initial_joint_q_);

  return true;
}

/**
 * @brief 主循环，每周期调用，依次更新状态、观测、动作等。
 *
 * 执行顺序：
 * 1. UpdateState()          - 更新关节状态和速度
 * 2. CalculateObservation() - 计算神经网络观测数据
 * 3. CalculateMotorCommand() - 神经网络推理和电机命令计算
 * 4. SendMotorCommand()     - 发送电机命令到硬件
 */
void RlMimicTrajectoryRunner::Run() {
  UpdateState();
  CalculateObservation();
  CalculateMotorCommand();
  SendMotorCommand();
}

/**
 * @brief 判断是否可以退出runner状态。
 * @return TransitionState::kCompleted 表示可退出
 */
TransitionState RlMimicTrajectoryRunner::TryExit() { return TransitionState::kCompleted; }

/**
 * @brief 退出runner状态，清理相关状态和变量。
 * @return true 退出成功
 */
bool RlMimicTrajectoryRunner::Exit() {
  data_store_->parallel_by_classic_parser.store(true);

  if (observation_manager_) {
    observation_manager_->Reset();
  }

  // 清空裸指针，保留 policy_model_ 和 interpolated_traj_ 以便 re-enter 时复用
  policy_net_ = nullptr;
  current_traj_ = nullptr;

  return true;
}

/**
 * @brief 初始化runner的所有核心资源：策略模型、轨迹、控制参数。
 */
void RlMimicTrajectoryRunner::Init() {
  const auto& profile = param_->motion_state;

  // 1. 初始化 ObservationManager
  if (!observation_manager_) {
    observation_manager_ = std::make_unique<rl_mimic_trajectory::ObservationManager>(*param_);
  }
  observation_manager_->SetProfile("default", profile);
  observation_manager_->SetRunnerPeriod(runner_period_);

  // 2. 加载策略模型（param_tag 切换后 policy_model_ 已被 reset，否则复用）
  if (!policy_model_) {
    std::string policy_path =
        common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), profile.policy_path);
    policy_model_ = std::make_shared<math::MNNModel>(policy_path);
  }
  policy_net_ = policy_model_.get();
  if (!policy_net_) {
    throw std::runtime_error("Policy model is null after loading: " + profile.policy_path);
  }

  // 3. 加载并插值轨迹（mimic / control_mimic 类型）
  if (profile.observation_type == "mimic" || profile.observation_type == "control_mimic") {
    if (interpolated_traj_.rows() == 0) {
      interpolated_traj_ = rl_mimic_trajectory::CsvLoader::LoadProfileTrajectory(profile, *param_, runner_period_);
    }
    current_traj_ = &interpolated_traj_;
  } else {
    current_traj_ = nullptr;
  }
  observation_manager_->SetCurrentTrajectory(current_traj_);
  policy_action_.setZero(param_->num_actions);

  // 4. 加载控制参数
  auto load_group_concat = [&](const std::optional<std::string>& group_name,
                               const std::vector<Eigen::VectorXd>& fallback) -> Eigen::VectorXd {
    if (group_name && !group_name->empty()) {
      try {
        auto group =
            common::ScopedParameterGetter<std::vector<Eigen::VectorXd>>::Get(param_->GetScope(), *group_name);
        return common::ConcatenateVectors(group);
      } catch (...) {
        return common::ConcatenateVectors(fallback);
      }
    }
    return common::ConcatenateVectors(fallback);
  };

  joint_kp_     = load_group_concat(profile.kp_group,           param_->joint_kp);
  joint_kd_     = load_group_concat(profile.kd_group,           param_->joint_kd);
  action_scale_ = load_group_concat(profile.action_scale_group, param_->action_scale);
  qd_mask_      = common::ConcatenateVectors(
      common::ScopedParameterGetter<std::vector<Eigen::VectorXd>>::Get(param_->GetScope(),
                                                                        profile.qd_mask_group.value()));

  if (profile.default_joint_q_group && !profile.default_joint_q_group->empty()) {
    try {
      auto group = common::ScopedParameterGetter<std::vector<Eigen::VectorXd>>::Get(
          param_->GetScope(), *profile.default_joint_q_group);
      default_joint_q_ = common::ConcatenateVectors(group);
    } catch (...) {
      default_joint_q_ = common::ConcatenateVectors(param_->default_joint_q);
    }
  } else {
    default_joint_q_ = common::ConcatenateVectors(param_->default_joint_q);
  }
  q_des_ = default_joint_q_;

  // 5. 预计算扭矩上限
  PrecomputeTauMax();

  // 6. 初始化关节索引
  active_joint_idx_.setZero(param_->num_actions);
  int i = 0;
  for (const std::string& name : param_->active_joint_names) {
    if (model_param_->joint_id_in_total_limb.find(name) == model_param_->joint_id_in_total_limb.end()) {
      throw std::runtime_error("Joint name not found in model: " + name);
    }
    active_joint_idx_(i++) = model_param_->joint_id_in_total_limb.at(name);
  }

  // 7. 调整向量尺寸
  q_actual_.resize(default_joint_q_.size());
  qd_actual_.resize(default_joint_q_.size());
  qd_actual_mask_.resize(default_joint_q_.size());
  qd_des_.resize(model_param_->num_total_joints);
  tau_ff_des_.resize(model_param_->num_total_joints);
}

double RlMimicTrajectoryRunner::ComputeTransitionRatio() const {
  const auto& profile = param_->motion_state;
  double transition_time_s = param_->transition_time;
  if (profile.transition_time.has_value()) {
    transition_time_s = profile.transition_time.value();
  }
  return transition_time_s > 0.0
             ? std::min(1.0, static_cast<double>(transition_iter_) * runner_period_ / transition_time_s)
             : 1.0;
}

void RlMimicTrajectoryRunner::PrecomputeTauMax() {
  const int n = static_cast<int>(model_param_->num_total_joints);
  tau_max_ = Eigen::VectorXd::Zero(n);
  int idx = 0;
  for (const auto& group : param_->max_torque_joint) {
    const int gsz = static_cast<int>(group.size());
    if (gsz > n - idx) {
      throw std::runtime_error("max_torque_joint total size exceeds DOF count");
    }
    tau_max_.segment(idx, gsz) = group;
    idx += gsz;
  }
  if (idx != n) {
    throw std::runtime_error("max_torque_joint total size does not match DOF count");
  }
  for (int i = 0; i < n; ++i) {
    if (tau_max_(i) <= 0.0) tau_max_(i) = std::numeric_limits<double>::infinity();
  }
}

/**
 * @brief 更新当前关节状态和速度，应用 qd_mask。
 */
void RlMimicTrajectoryRunner::UpdateState() {
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_actual_);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, qd_actual_);
  qd_actual_mask_ = qd_actual_.cwiseProduct(qd_mask_);
}

void RlMimicTrajectoryRunner::CalculateObservation() {
  if (!observation_manager_) {
    LOG(ERROR) << "observation_manager_ is null!";
    return;
  }

  Eigen::Matrix3d R_install = math::RollPitchYawd(imu_install_bias_).ToRotationMatrix().matrix();
  Eigen::Matrix3d R_local   = math::RotationMatrixd(data_store_->imu_info.Get()->quaternion).matrix();
  Eigen::Matrix3d R_real    = R_local * R_install.transpose();
  Eigen::Vector3d w_real    = R_real.transpose() * R_local * data_store_->imu_info.Get()->angular_velocity;
  Eigen::Vector3d projected_gravity_real = -R_real.transpose() * Eigen::Vector3d::UnitZ();

  std::optional<Eigen::Vector2d> yaw_forward_vec = CalculateBaseYawForwardVector();

  observation_manager_->UpdateObservation(q_actual_, qd_actual_mask_, policy_action_, w_real,
                                          projected_gravity_real, command_, default_joint_q_,
                                          active_joint_idx_, yaw_forward_vec);
}

void RlMimicTrajectoryRunner::CalculateMotorCommand() {
  if (!observation_manager_) {
    LOG(ERROR) << "observation_manager_ is null!";
    return;
  }
  if (!policy_net_) {
    LOG(ERROR) << "policy_net_ is null!";
    return;
  }

  // 推理
  Eigen::MatrixXd obs_matrix = observation_manager_->GetCurrentObservation();
  Eigen::VectorXd obs = Eigen::Map<const Eigen::VectorXd>(obs_matrix.data(), obs_matrix.size());
  policy_action_ = policy_net_->Inference(obs.cast<float>()).cast<double>();
  policy_action_ = policy_action_.cwiseMax(-param_->action_clip).cwiseMin(param_->action_clip);

  const auto& profile = param_->motion_state;
  observation_manager_->StepTrajectory(profile.replay);

  // 计算期望关节角度
  q_des_ = default_joint_q_;
  if (profile.residual_control) {
    q_des_(active_joint_idx_) =
        observation_manager_->GetCurrentReference() + policy_action_.cwiseProduct(action_scale_);
  } else {
    q_des_(active_joint_idx_) = default_joint_q_(active_joint_idx_) + policy_action_.cwiseProduct(action_scale_);
  }

  // 过渡插值
  double ratio = ComputeTransitionRatio();
  if (ratio < 1.0) {
    q_des_ = math::LinearInterpolate(initial_joint_q_, q_des_, ratio);
    ++transition_iter_;
  }

  // 轨迹播放完毕则退出 runner
  if (!profile.replay && IsTrajectoryFinished()) {
    LOG(INFO) << "Trajectory finished, requesting exit.";
    SetRunnerState(RunnerState::kTryExit);
  }
}

void RlMimicTrajectoryRunner::SendMotorCommand() {
  qd_des_    = Eigen::VectorXd::Zero(model_param_->num_total_joints);
  tau_ff_des_ = Eigen::VectorXd::Zero(model_param_->num_total_joints);

  if (param_->torque_limit) {
    ApplyTorqueLimits(q_des_, q_actual_, qd_actual_, joint_kp_, joint_kd_);
  }

  for (int i = 0; i < model_param_->num_total_joints; ++i) {
    if (q_des_(i) > 6.5) {
      std::cout << "Warning: Joint " << i << " desired position exceeds limit: " << q_des_(i) << std::endl;
      q_des_(i) = 6.5;
    }
  }
  GetMutableOutput().SetCommand(q_des_, qd_des_, joint_kp_, joint_kd_, tau_ff_des_);
}

void RlMimicTrajectoryRunner::ApplyTorqueLimits(Eigen::VectorXd& q_des, const Eigen::VectorXd& q_actual,
                                                const Eigen::VectorXd& qd_actual, const Eigen::VectorXd& joint_kp,
                                                const Eigen::VectorXd& joint_kd) {
  const int n = q_des.size();

  Eigen::VectorXd tau_des(n);
  {
    Eigen::ArrayXd Kp = joint_kp.array();
    Eigen::ArrayXd Kd = joint_kd.array();
    tau_des = (Kp * (q_des - q_actual).array() - Kd * qd_actual.array()).matrix();
  }

  Eigen::VectorXd tau = tau_des;
  for (int i = 0; i < n; ++i) {
    const double m = tau_max_(i);
    if (std::isfinite(m)) {
      if (tau(i) > m) tau(i) = m;
      if (tau(i) < -m) tau(i) = -m;
    }
  }

  const int lower_body_end_index = param_->lower_body_joint_count;
  if (n >= lower_body_end_index && param_->max_lower_body_torque > 0.0) {
    const double sum_abs_lower = tau.segment(0, lower_body_end_index).cwiseAbs().sum();
    const double limit = param_->max_lower_body_torque;
    if (sum_abs_lower > limit) {
      tau.segment(0, lower_body_end_index) *= (limit / sum_abs_lower);
    }
  }

  const double eps = 1e-6;
  for (int i = 0; i < n; ++i) {
    const double kp = joint_kp(i);
    const double kd = joint_kd(i);
    if (std::abs(kp) > eps && std::isfinite(kp)) {
      q_des(i) = q_actual(i) + (tau(i) + kd * qd_actual(i)) / kp;
    }
  }
}

bool RlMimicTrajectoryRunner::IsTrajectoryFinished() const {
  const auto& profile = param_->motion_state;
  if (profile.observation_type == "mimic" || profile.observation_type == "control_mimic") {
    return current_traj_ && observation_manager_->GetCurrentTrajectoryIndex() >= current_traj_->rows() - 1;
  } else if (profile.observation_type == "mimic_tj") {
    if (profile.step_phase == 0.0 || profile.phase_range.size() < 2) {
      LOG(ERROR) << "step_phase or phase_range not configured for mimic_tj";
      return false;
    }
    double current_step = static_cast<double>(observation_manager_->GetMimicTjPhaseCounter());
    double phase_start  = profile.phase_range[0];
    double phase_end    = profile.phase_range[1];
    double current_phase =
        phase_start + (phase_end - phase_start) *
                          (profile.step_phase * (2 * 3.14159) / (phase_end - phase_start) * current_step);
    return current_phase >= phase_end;
  }
  return current_traj_ && observation_manager_->GetCurrentTrajectoryIndex() >= current_traj_->rows() - 1;
}

Eigen::Vector2d RlMimicTrajectoryRunner::CalculateBaseYawForwardVector() {
  Eigen::Quaterniond current_quat = data_store_->imu_info.Get()->quaternion;
  Eigen::Vector4d current_base_quat(current_quat.w(), current_quat.x(), current_quat.y(), current_quat.z());

  if (!initial_quat_set_) {
    initial_base_quat_ = current_base_quat;
    initial_quat_set_ = true;
  }

  Eigen::Quaterniond init_quat(initial_base_quat_[0], initial_base_quat_[1], initial_base_quat_[2],
                               initial_base_quat_[3]);
  Eigen::Quaterniond q_rel = init_quat.conjugate() * current_quat;

  double w = q_rel.w(), x = q_rel.x(), y = q_rel.y(), z = q_rel.z();
  double yaw_rad = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  return {std::cos(yaw_rad), std::sin(yaw_rad)};
}

}  // namespace runner
