#pragma once
#include <Eigen/Dense>
#include <map>
#include <optional>
#include <string>
#include "rl_mimic_trajectory_example_param/rl_mimic_trajectory_example_param.h"

namespace rl_mimic_trajectory {

class ObservationManager {
 public:
  ObservationManager(const data::RlMinicTrajectoryParam& param);
  void SetProfile(const std::string& profile_key, const data::MotionStateProfile& profile);
  void UpdateObservation(const Eigen::VectorXd& q_actual, const Eigen::VectorXd& qd_actual_mask,
                         const Eigen::VectorXd& mlp_net_action, const Eigen::Vector3d& w_real,
                         const Eigen::Vector3d& projected_gravity_real, const Eigen::Vector3d& command,
                         const Eigen::VectorXd& default_joint_q, const Eigen::VectorXi& active_joint_idx,
                         const std::optional<Eigen::Vector2d>& yaw_forward_vec = std::nullopt,
                         bool update_history = true);
  void StepTrajectory(bool replay);
  Eigen::MatrixXd GetCurrentObservation() const;
  void Reset();
  size_t GetCurrentTrajectoryIndex() const;
  void SetCurrentTrajectoryIndex(size_t index);
  // 新增：获取mimic_tj的相位计数器
  size_t GetMimicTjPhaseCounter() const;
  // 新增：获取当前参考关节（mimic/residual_control时为当前轨迹帧，否则为default_joint_q_）
  Eigen::VectorXd GetCurrentReference() const;
  // 当前profile的轨迹指针（由外部管理，不负责释放）
  Eigen::MatrixXd* current_traj_ = nullptr;

 public:
  // 设置当前轨迹指针
  void SetCurrentTrajectory(Eigen::MatrixXd* traj);
  // 设置runner周期时间
  void SetRunnerPeriod(double period) { runner_period_ = period; }

 private:
  // Create goal observation
  Eigen::VectorXd CreateGoalObservation(const Eigen::VectorXd& q_actual, const Eigen::Vector3d& command,
                                        const Eigen::VectorXd& default_joint_q,
                                        const Eigen::VectorXi& active_joint_idx);
  // Helper functions for GetCurrentObservation
  Eigen::VectorXd BuildProprioceptiveScale(int proprio_dim, int time_steps) const;
  void ApplyGoalScaling(Eigen::VectorXd& observation, int obs_rows, int goal_rows, int goal_cols) const;

  const data::RlMinicTrajectoryParam& param_;
  std::string profile_key_;
  std::string observation_type_;
  const data::MotionStateProfile* current_profile_ = nullptr;
  const data::ObservationConfig* current_obs_cfg_ = nullptr;

  // 新的buffer结构：1个本体感知buffer + 3个goal buffer
  Eigen::MatrixXd obs_buffer_;  // 本体感知buffer（拼接后的最终数据）

  // 本体感知各项的历史buffer
  Eigen::MatrixXd q_history_;            // 关节角度历史
  Eigen::MatrixXd qd_history_;           // 关节速度历史
  Eigen::MatrixXd action_history_;       // 动作历史
  Eigen::MatrixXd w_history_;            // 角速度历史
  Eigen::MatrixXd gravity_history_;      // 重力向量历史
  Eigen::MatrixXd yaw_forward_history_;  // yaw forward vector历史

  // 历史buffer管理
  std::vector<std::pair<Eigen::MatrixXd*, int>> history_buffers_;  // {buffer指针, 维度}

  std::map<std::string, Eigen::MatrixXd> goal_buffers_;   // 目标buffer（mimic/rl_saw_locomotion）
  std::map<std::string, int> goal_time_steps_;            // 各goal buffer的time step设置
  std::map<std::string, int> proprioceptive_time_steps_;  // 各observation_type的本体感知time step设置
  Eigen::MatrixXd* current_goal_buffer_ = nullptr;        // 当前使用的goal buffer

  // 本体感知配置（现在每个observation_type独立）
  bool obs_buffer_initialized_ = false;  // 本体感知buffer初始化标记

  // Cached policy joint indices (computed once)
  std::vector<int> policy_joint_indices_;

  size_t trajectory_index_ = 0;
  size_t mimic_tj_phase_counter_ = 0;  // mimic_tj专用的步数计数器
  Eigen::VectorXd current_observation_scale_;

  std::string last_profile_key_;                  // 用于检测profile切换
  std::string last_observation_type_;             // 用于检测observation_type切换
  double runner_period_ = 0.02;                   // runner周期时间，默认值
  Eigen::Vector2d initial_yaw_forward_;           // 初始yaw forward vector，在policy切换时重置
  bool fill_goal_buffer_on_next_update_ = false;  // observation_type切换后需要填充goal buffer
};

}  // namespace rl_mimic_trajectory