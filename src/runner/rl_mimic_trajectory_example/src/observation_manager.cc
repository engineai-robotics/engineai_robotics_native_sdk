#include "rl_mimic_trajectory_example/observation_manager.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include "math/constants.h"
#include "math/interpolation.h"
#include "math/rotation_matrix.h"
#include "tool/concatenate_vector.h"

namespace rl_mimic_trajectory {

ObservationManager::ObservationManager(const data::RlMinicTrajectoryParam& param) : param_(param) {
  // 默认不初始化，需SetProfile后Init
}

void ObservationManager::SetCurrentTrajectory(Eigen::MatrixXd* traj) { current_traj_ = traj; }

void ObservationManager::SetProfile(const std::string& profile_key, const data::MotionStateProfile& profile) {
  profile_key_ = profile_key;
  current_profile_ = &profile;
  observation_type_ = profile.observation_type;
  // 使用统一的observation配置
  current_obs_cfg_ = &param_.observations;
  // 使用统一的observation scale向量
  current_observation_scale_ = param_.observation_scale_vec;

  // Initialize buffers: 1 proprioceptive buffer + 3 goal buffers
  // Don't reset obs_buffer_ here to preserve history
  // obs_buffer_ = Eigen::MatrixXd::Zero(1, 1);

  // Goal buffers: obtain configuration for each goal buffer from config
  // Don't clear goal_buffers_ blindly to preserve history
  goal_time_steps_.clear();
  proprioceptive_time_steps_.clear();

  for (const auto& [goal_type, goal_cfg] : param_.observations.observation_type) {
    // Each goal uses its own history steps and proprioceptive time steps
    int goal_steps = goal_cfg.goal_history_steps;

    // Initialize or resize buffer only if needed
    bool need_init = true;
    if (goal_buffers_.count(goal_type)) {
      if (goal_buffers_[goal_type].rows() == goal_cfg.goal_length && goal_buffers_[goal_type].cols() == goal_steps) {
        need_init = false;
      }
    }

    if (need_init) {
      goal_buffers_[goal_type] = Eigen::MatrixXd::Zero(goal_cfg.goal_length, goal_steps);
    }

    goal_time_steps_[goal_type] = goal_steps;
    proprioceptive_time_steps_[goal_type] = goal_cfg.num_include_obs_steps;
  }
  // obs_buffer_initialized_ = false;  // Removed to preserve history

  // 设置当前goal buffer，检查是否存在对应的配置
  auto goal_it = goal_buffers_.find(observation_type_);
  if (goal_it != goal_buffers_.end()) {
    current_goal_buffer_ = &goal_it->second;
  } else {
    throw std::runtime_error("No goal buffer configuration found for observation_type: " + observation_type_);
  }
  // Reset indices only when profile actually switches
  if (last_profile_key_ != profile_key) {
    // 检查 observation_type 是否改变
    bool observation_type_changed = (last_observation_type_ != observation_type_);

    trajectory_index_ = 0;
    mimic_tj_phase_counter_ = 0;  // Reset mimic_tj phase counter
    // 重置初始yaw forward vector，在policy切换时重置
    initial_yaw_forward_.setZero();
    last_profile_key_ = profile_key;
    last_observation_type_ = observation_type_;

    // 关键逻辑：根据 observation_type 是否改变决定是否填充 goal buffer
    if (observation_type_changed) {
      // observation_type 改变时（如 mimic → rl_saw_locomotion）：
      // 需要填充整个 goal buffer，因为：
      // 1. 这是完全不同的 policy，action_history 无意义
      // 2. 新的 goal buffer 可能包含陈旧数据
      // 3. goal 的维度和含义完全不同
      fill_goal_buffer_on_next_update_ = true;
    } else {
      // 同一 observation_type 内切换（如 kneeling → keep_kneeling，都是 mimic）：
      // 不填充 goal buffer，让它自然更新，保持 goal_history 和 action_history 配套
      fill_goal_buffer_on_next_update_ = false;
    }
  }
}

void ObservationManager::UpdateObservation(const Eigen::VectorXd& q_actual, const Eigen::VectorXd& qd_actual_mask,
                                           const Eigen::VectorXd& mlp_net_action, const Eigen::Vector3d& w_real,
                                           const Eigen::Vector3d& projected_gravity_real,
                                           const Eigen::Vector3d& command, const Eigen::VectorXd& default_joint_q,
                                           const Eigen::VectorXi& active_joint_idx,
                                           const std::optional<Eigen::Vector2d>& yaw_forward_vec, bool update_history) {
  if (!current_obs_cfg_ || !current_goal_buffer_ || !current_profile_) {
    LOG(ERROR) << "Invalid state: current_obs_cfg_=" << (current_obs_cfg_ ? "OK" : "NULL")
               << ", current_goal_buffer_=" << (current_goal_buffer_ ? "OK" : "NULL")
               << ", current_profile_=" << (current_profile_ ? "OK" : "NULL");
    return;
  }

  // 1. Update proprioceptive buffer (using current observation_type's time steps)
  int q_size = q_actual(active_joint_idx).size();
  int qd_size = qd_actual_mask(active_joint_idx).size();
  int action_size = mlp_net_action.size();

  // Safely get time steps for current observation_type
  auto time_steps_it = proprioceptive_time_steps_.find(observation_type_);
  if (time_steps_it == proprioceptive_time_steps_.end()) {
    LOG(ERROR) << "No proprioceptive time steps found for observation_type: " << observation_type_;
    return;
  }
  int time_steps = time_steps_it->second;

  // First-time initialization or dimension change detection
  bool needs_reinit = !obs_buffer_initialized_;

  // Check if we need to reinitialize due to dimension changes
  if (obs_buffer_initialized_) {
    const int w_dim = w_real.size();
    const int gravity_dim = projected_gravity_real.size();
    const int yaw_dim = 2;

    // Calculate expected total dimension (always include yaw for consistency)
    int expected_total_dim = (q_size + qd_size + action_size + w_dim + gravity_dim + yaw_dim) * time_steps;

    if (obs_buffer_.rows() != expected_total_dim) {
      needs_reinit = true;
      VLOG(1) << "Dimension mismatch detected, reinitializing obs_buffer: current=" << obs_buffer_.rows()
              << ", expected=" << expected_total_dim;
    }
  }

  if (needs_reinit) {
    // Initialize all history buffers including yaw forward vector (always reserve space)
    history_buffers_ = {{&q_history_, q_size}, {&qd_history_, qd_size}, {&action_history_, action_size},
                        {&w_history_, 3},      {&gravity_history_, 3},  {&yaw_forward_history_, 2}};

    int total_dim = 0;
    for (auto& [buffer, dim] : history_buffers_) {
      *buffer = Eigen::MatrixXd::Zero(dim, time_steps);
      total_dim += dim * time_steps;
    }
    obs_buffer_ = Eigen::MatrixXd::Zero(total_dim, 1);
    obs_buffer_initialized_ = true;

    VLOG(1) << "Initialized observation buffer for " << observation_type_ << ": time_steps=" << time_steps
            << ", total_dim=" << total_dim << ", q_size=" << q_size << ", qd_size=" << qd_size
            << ", action_size=" << action_size;

    // Debug info for yaw vector inclusion
    if (current_profile_ && current_profile_->proprioception_obs_with_yaw) {
      VLOG(1) << "Yaw forward vector included in proprioception observation";
    }
  }

  // Only update history if requested
  if (update_history) {
    // Update history buffers (new data on the right) - avoid temporary vector creation
    const Eigen::VectorXd q_diff = q_actual(active_joint_idx) - default_joint_q(active_joint_idx);
    const Eigen::VectorXd qd_actual_mask_active = qd_actual_mask(active_joint_idx);

    const Eigen::VectorXd w_real_vec = w_real;
    const Eigen::VectorXd projected_gravity_vec = projected_gravity_real;

    // 处理yaw forward vector：总是计算，但根据配置决定是否使用
    Eigen::Vector2d yaw_forward_data;

    if (yaw_forward_vec.has_value()) {
      yaw_forward_data = yaw_forward_vec.value();
      // 如果是第一次接收到yaw数据，保存为初始值
      if (initial_yaw_forward_.isZero()) {
        initial_yaw_forward_ = yaw_forward_data;
      }
    } else {
      // 如果没有提供yaw数据，使用初始值
      yaw_forward_data = initial_yaw_forward_;
    }

    // 始终包含所有数据（保持结构一致）
    // 将 yaw_forward_data 转换为 VectorXd 类型以匹配数组类型
    Eigen::VectorXd yaw_forward_vector = yaw_forward_data;
    const std::array<const Eigen::VectorXd*, 6> new_data = {&q_diff,     &qd_actual_mask_active, &mlp_net_action,
                                                            &w_real_vec, &projected_gravity_vec, &yaw_forward_vector};

    for (size_t i = 0; i < history_buffers_.size(); ++i) {
      auto& buffer = *history_buffers_[i].first;
      if (time_steps > 1) {
        buffer.leftCols(time_steps - 1) = buffer.rightCols(time_steps - 1);
      }
      buffer.rightCols(1) = *new_data[i];
    }

    // Concatenate all history data to obs_buffer_
    int idx = 0;
    for (const auto& [buffer, dim] : history_buffers_) {
      for (int t = 0; t < time_steps; ++t) {
        obs_buffer_.block(idx, 0, dim, 1) = buffer->col(t);
        idx += dim;
      }
    }
  }

  // 2. Update goal buffer
  Eigen::VectorXd goal_obs = CreateGoalObservation(q_actual, command, default_joint_q, active_joint_idx);

  // Validate goal observation length against config; auto-correct by truncate/pad
  auto goal_cfg_it = param_.observations.observation_type.find(observation_type_);
  if (goal_cfg_it == param_.observations.observation_type.end()) {
    LOG(ERROR) << "No goal cfg found for observation_type: " << observation_type_;
    return;
  }
  int expected_goal_len = goal_cfg_it->second.goal_length;
  if (goal_obs.size() != expected_goal_len) {
    if (goal_obs.size() > expected_goal_len) {
      goal_obs = goal_obs.head(expected_goal_len);
    } else {
      Eigen::VectorXd fixed = Eigen::VectorXd::Zero(expected_goal_len);
      fixed.head(goal_obs.size()) = goal_obs;
      goal_obs = fixed;
    }
  }

  // Update goal buffer (using its own time step)
  auto goal_steps_it = goal_time_steps_.find(observation_type_);
  if (goal_steps_it == goal_time_steps_.end()) {
    LOG(ERROR) << "No goal time steps found for observation_type: " << observation_type_;
    return;
  }
  int goal_steps = goal_steps_it->second;

  // 根据标志位决定是否填充整个 goal buffer
  if (fill_goal_buffer_on_next_update_) {
    // observation_type 改变时，用当前帧填充整个 goal buffer
    for (int i = 0; i < goal_steps; ++i) {
      current_goal_buffer_->col(i) = goal_obs;
    }
    fill_goal_buffer_on_next_update_ = false;
  } else {
    // 正常更新：左移历史帧，最新帧放在最右边
    if (goal_steps > 1) {
      current_goal_buffer_->leftCols(goal_steps - 1) = current_goal_buffer_->rightCols(goal_steps - 1);
    }
    current_goal_buffer_->col(goal_steps - 1) = goal_obs;
  }
}

void ObservationManager::StepTrajectory(bool replay) {
  // For mimic_tj, always increment phase counter
  if (observation_type_ == "mimic_tj") {
    ++mimic_tj_phase_counter_;
    return;
  }

  // For other observation types, use trajectory-based stepping
  if (!current_traj_) {
    return;
  }
  size_t end = current_traj_->rows() > 0 ? current_traj_->rows() - 1 : 0;
  if (trajectory_index_ < end) {
    ++trajectory_index_;
  } else {
    if (replay) {
      trajectory_index_ = 0;
    } else {
      trajectory_index_ = end;
    }
  }
}

Eigen::MatrixXd ObservationManager::GetCurrentObservation() const {
  if (!current_goal_buffer_) {
    LOG(ERROR) << "Current goal buffer is null";
    return Eigen::MatrixXd::Zero(1, 1);
  }

  const int obs_rows = obs_buffer_.rows();
  const int goal_rows = current_goal_buffer_->rows();

  // 安全获取goal_cols
  auto goal_cols_it = goal_time_steps_.find(observation_type_);
  if (goal_cols_it == goal_time_steps_.end()) {
    LOG(ERROR) << "No goal time steps found for observation_type: " << observation_type_;
    return Eigen::MatrixXd::Zero(1, 1);
  }
  const int goal_cols = goal_cols_it->second;

  // Calculate total dimension: proprioceptive + expanded goal data
  // Check if yaw should be included in final observation
  bool include_yaw = current_profile_ && current_profile_->proprioception_obs_with_yaw;

  int actual_proprio_rows = obs_rows;
  if (!include_yaw) {
    // Exclude yaw vector from final observation (but keep in buffer)
    auto time_steps_it = proprioceptive_time_steps_.find(observation_type_);
    if (time_steps_it != proprioceptive_time_steps_.end()) {
      int current_time_steps = time_steps_it->second;
      int yaw_size = 2 * current_time_steps;
      actual_proprio_rows = obs_rows - yaw_size;
    }
  }

  const int total_rows = actual_proprio_rows + goal_rows * goal_cols;

  // Create single-column observation vector (simplified to 1D output)
  Eigen::VectorXd observation = Eigen::VectorXd::Zero(total_rows);

  // Fill proprioceptive data - exclude yaw if not needed
  observation.head(actual_proprio_rows) = obs_buffer_.col(0).head(actual_proprio_rows);

  // Fill expanded goal data
  int idx = actual_proprio_rows;
  for (int t = 0; t < goal_cols; ++t) {
    observation.segment(idx, goal_rows) = current_goal_buffer_->col(t);
    idx += goal_rows;
  }

  // Apply scale and clip
  // Apply proprioceptive scaling
  auto time_steps_it = proprioceptive_time_steps_.find(observation_type_);
  if (time_steps_it != proprioceptive_time_steps_.end()) {
    int current_time_steps = time_steps_it->second;
    int proprio_dim = obs_rows;
    Eigen::VectorXd current_scale = BuildProprioceptiveScale(proprio_dim, current_time_steps);

    // Apply scaling to proprioceptive data (excluding yaw if not needed)
    // Use only the first actual_proprio_rows elements of the scale (yaw scales at the end)
    observation.head(actual_proprio_rows).array() *= current_scale.head(actual_proprio_rows).array();
  } else {
    LOG(WARNING) << "No time steps found for observation_type: " << observation_type_;
  }

  // Apply goal part scaling
  ApplyGoalScaling(observation, actual_proprio_rows, goal_rows, goal_cols);

  // Apply clip
  observation = observation.cwiseMax(-param_.observation_clip).cwiseMin(param_.observation_clip);

  // Return single-column matrix to maintain interface consistency
  return observation.reshaped(total_rows, 1);
}

void ObservationManager::Reset() {
  obs_buffer_.setZero();
  // Reset all history buffers
  if (obs_buffer_initialized_) {
    for (auto& [buffer, _] : history_buffers_) {
      buffer->setZero();
    }
  }
  // Reset goal buffers
  for (auto& [_, buffer] : goal_buffers_) {
    buffer.setZero();
  }
  trajectory_index_ = 0;
  mimic_tj_phase_counter_ = 0;      // Reset mimic_tj phase counter
  initial_yaw_forward_.setZero();   // Reset initial yaw forward vector
  obs_buffer_initialized_ = false;  // Force reinitialization on next UpdateObservation
  last_observation_type_.clear();   // Reset observation type tracking
  fill_goal_buffer_on_next_update_ = false;
}

size_t ObservationManager::GetCurrentTrajectoryIndex() const { return trajectory_index_; }

void ObservationManager::SetCurrentTrajectoryIndex(size_t index) { trajectory_index_ = index; }

size_t ObservationManager::GetMimicTjPhaseCounter() const { return mimic_tj_phase_counter_; }

// Get current reference joints (trajectory frame for mimic/control_mimic, otherwise default_joint_q)
// Always returns active joints only to maintain consistent dimensions
Eigen::VectorXd ObservationManager::GetCurrentReference() const {
  Eigen::VectorXd all_joints;

  if ((observation_type_ == "mimic" || observation_type_ == "control_mimic") && current_profile_ &&
      !current_profile_->data_path.empty() && current_traj_) {
    if (trajectory_index_ < current_traj_->rows()) {
      // Trajectory already contains only active joints (loaded from CSV with active_joint_names)
      return current_traj_->row(trajectory_index_);
    } else {
      // Trajectory already contains only active joints (loaded from CSV with active_joint_names)
      return current_traj_->row(current_traj_->rows() - 1);
    }
  } else {
    // For mimic_tj and other types, return default_joint_q
    // default_joint_q contains all joints (29), need to extract only active joints (25)
    all_joints = common::ConcatenateVectors(param_.default_joint_q);

    // Extract active joints from all joints
    // Structure of default_joint_q after concatenation:
    // [0-5]:   left leg (6 joints - all active)
    // [6-11]:  right leg (6 joints - all active)
    // [12]:    torso (1 joint - active)
    // [13-19]: left arm (7 joints - first 5 active, last 2 wrist inactive)
    // [20-26]: right arm (7 joints - first 5 active, last 2 wrist inactive)
    // [27-28]: head (2 joints - all active)
    Eigen::VectorXd active_joints(25);

    // Left leg (6) + Right leg (6) + Torso (1) = 13 joints, all active
    active_joints.head(13) = all_joints.head(13);

    // Left arm: take first 5 of 7 (skip J18_WRIST_PITCH_L and J19_WRIST_ROLL_L)
    active_joints.segment(13, 5) = all_joints.segment(13, 5);

    // Right arm: take first 5 of 7 (skip J25_WRIST_PITCH_R and J26_WRIST_ROLL_R)
    active_joints.segment(18, 5) = all_joints.segment(20, 5);

    // Head: 2 joints at the end
    active_joints.segment(23, 2) = all_joints.segment(27, 2);

    return active_joints;
  }
}

Eigen::VectorXd ObservationManager::CreateGoalObservation(const Eigen::VectorXd& q_actual,
                                                          const Eigen::Vector3d& command,
                                                          const Eigen::VectorXd& default_joint_q,
                                                          const Eigen::VectorXi& active_joint_idx) {
  if (observation_type_ == "mimic") {
    // GetCurrentReference() now always returns active joints only
    return q_actual(active_joint_idx) - GetCurrentReference();
  } else if (observation_type_ == "control_mimic") {
    // control_mimic: mimic goal plus appended 3D velocity command
    // GetCurrentReference() now always returns active joints only
    Eigen::VectorXd diff = q_actual(active_joint_idx) - GetCurrentReference();

    Eigen::VectorXd goal_obs(diff.size() + 3);
    goal_obs.head(diff.size()) = diff;
    goal_obs.tail(3) = command;
    return goal_obs;
  } else if (observation_type_ == "rl_saw_locomotion") {
    return command;
  } else if (observation_type_ == "mimic_tj") {
    // mimic_tj uses step_phase to calculate sin and cos
    if (!current_profile_) {
      LOG(ERROR) << "Current profile is null for mimic_tj";
      return Eigen::VectorXd::Zero(2);
    }

    // Check if step_phase is configured
    if (current_profile_->step_phase == 0.0) {
      LOG(ERROR) << "step_phase is not configured for mimic_tj observation_type in profile: " << profile_key_;
      throw std::runtime_error("step_phase is required for mimic_tj observation_type but not configured");
    }

    // Check if phase_range is configured
    if (current_profile_->phase_range.empty() || current_profile_->phase_range.size() < 2) {
      LOG(ERROR) << "phase_range is not configured for mimic_tj observation_type in profile: " << profile_key_;
      throw std::runtime_error("phase_range is required for mimic_tj observation_type but not configured");
    }

    // Get configuration parameters
    double step_phase = current_profile_->step_phase;
    double phase_start = current_profile_->phase_range[0];
    double phase_end = current_profile_->phase_range[1];

    // Calculate current step (using mimic_tj_phase_counter_ as step)
    double step = static_cast<double>(mimic_tj_phase_counter_);

    // Calculate normalized phase within the specified range
    // phase = phase_start + (phase_end - phase_start) * (step_phase * step)
    double phase =
        phase_start + (phase_end - phase_start) * (step_phase * (2 * 3.14159) / (phase_end - phase_start) * step);

    // For sin/cos calculation, we can use the phase directly without fmod
    // since sin and cos are periodic functions that work with any phase value

    const int mimic_tj_dim = 2;  // sin/cos dimension for mimic_tj
    Eigen::VectorXd goal_obs(mimic_tj_dim);
    goal_obs(0) = std::sin(phase);
    goal_obs(1) = std::cos(phase);

    return goal_obs;
  } else {
    throw std::runtime_error("Unknown observation_type: " + observation_type_);
  }
}

Eigen::VectorXd ObservationManager::BuildProprioceptiveScale(int proprio_dim, int time_steps) const {
  Eigen::VectorXd current_scale = Eigen::VectorXd::Zero(proprio_dim);
  int idx = 0;

  // Follow observation concatenation order: buffer first, then time
  // Always include yaw forward vector in scales (will be excluded from final observation if not needed)
  const std::vector<double> scales = {
      param_.observations.observation_scale.observation_scale_dof_pos_ref_diff,  // q_diff (position difference)
      param_.observations.observation_scale.observation_scale_dof_vel,           // qd (velocity)
      1.0,                                                                       // action
      param_.observations.observation_scale.observation_scale_angular_vel,       // w (angular velocity)
      param_.observations.observation_scale.observation_scale_quat,              // gravity
      param_.observations.observation_scale.observation_scale_yaw_forward        // yaw forward vector
  };

  // Dimensions correspond to: q_diff, qd, action, angular_vel, gravity, yaw_forward
  int joint_dim = static_cast<int>(param_.active_joint_names.size());

  const std::vector<int> dims = {
      joint_dim,  // q_diff dimension
      joint_dim,  // qd dimension
      joint_dim,  // action dimension
      3,          // angular velocity dimension
      3,          // gravity dimension
      2           // yaw forward vector dimension
  };

  for (size_t buffer_idx = 0; buffer_idx < scales.size(); ++buffer_idx) {
    for (int t = 0; t < time_steps; ++t) {
      int segment_size = dims[buffer_idx];
      // Add bounds checking
      if (idx + segment_size > current_scale.size()) {
        LOG(ERROR) << "Index out of bounds in BuildProprioceptiveScale: idx=" << idx
                   << ", segment_size=" << segment_size << ", current_scale.size()=" << current_scale.size();
        break;
      }
      current_scale.segment(idx, segment_size) = Eigen::VectorXd::Constant(segment_size, scales[buffer_idx]);
      idx += segment_size;
    }
  }

  return current_scale;
}

void ObservationManager::ApplyGoalScaling(Eigen::VectorXd& observation, int obs_rows, int goal_rows,
                                          int goal_cols) const {
  auto goal_cfg_it = param_.observations.observation_type.find(observation_type_);
  if (goal_cfg_it != param_.observations.observation_type.end()) {
    double goal_scale = goal_cfg_it->second.goal_scale;
    // Apply scaling to goal part: each time step goal uses same scale
    for (int t = 0; t < goal_cols; ++t) {
      int start_idx = obs_rows + t * goal_rows;
      observation.segment(start_idx, goal_rows) *= goal_scale;
    }
  }
}

}  // namespace rl_mimic_trajectory