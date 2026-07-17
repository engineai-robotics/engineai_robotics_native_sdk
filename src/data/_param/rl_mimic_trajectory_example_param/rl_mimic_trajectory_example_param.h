#pragma once

#include <glog/logging.h>
#include <Eigen/Dense>
#include <iostream>
#include <unordered_map>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"

namespace data {

class ObservationScaleConfig {
 public:
  double observation_scale_linear_vel = 1.0;
  double observation_scale_angular_vel = 1.0;
  double observation_scale_dof_pos = 1.0;
  double observation_scale_dof_vel = 1.0;
  double observation_scale_quat = 1.0;
  double observation_scale_dof_pos_ref_diff = 1.0;
  double observation_scale_yaw_forward = 1.0;
};

class ObservationGoalTypeConfig {
 public:
  int goal_length = 0;
  int goal_history_steps = 1;     // goal的历史步数
  int num_include_obs_steps = 5;  // 本体感知的历史步数
  double goal_scale = 1.0;
};

class ObservationConfig {
 public:
  int num_single_observations = 78;                                   // 本体感知维度
  std::map<std::string, ObservationGoalTypeConfig> observation_type;  // 各goal类型配置
  ObservationScaleConfig observation_scale;                           // 统一scale配置
};

class MotionStateProfile {
 public:
  MotionStateProfile() { Reset(); }
  ~MotionStateProfile() = default;

  void Reset() {
    observation_type.clear();
    data_path.clear();
    csv_dt = 1.0 / 30.0;  // 重置为默认值
    policy_path.clear();
    traj_frame.clear();
    replay = false;
    residual_control = false;
    cycle_time.clear();
    stride_max = 0.6;
    yaw_per_cycle_max = 0.35;
    k_angle = 0.6;
    proprioception_obs_with_yaw = false;
    transition_time.reset();
  }

  std::string observation_type;                 // "mimic" or "rl"
  std::string data_path;                        // CSV trajectory path
  double csv_dt = 1.0 / 30.0;                   // CSV data sampling time (seconds)
  std::string policy_path;                      // Policy model path
  std::vector<int> traj_frame;                  // Trajectory frame indices
  bool replay = false;                          // Whether to loop the trajectory
  bool residual_control = false;                // Whether to use residual control
  std::vector<double> cycle_time;               // Motion cycle time range [min, max]
  double stride_max = 0.6;                      // Maximum stride length constraint
  double yaw_per_cycle_max = 0.35;              // Maximum yaw angle per cycle constraint
  double k_angle = 0.6;                         // Angle coefficient for intensity calculation
  double step_phase = 0.0;                      // Step phase for mimic_tj observation type
  std::vector<double> phase_range;              // Phase range for mimic_tj observation type [start, end]
  bool proprioception_obs_with_yaw = false;     // 是否在本体感知观测中包含yaw forward vector
  // 每个motion state独立的过渡时间（可选）；若未设置则回退到全局transition_time
  std::optional<double> transition_time;
  // 每个motion_state可选的控制参数组名（对应yaml中的组名，如 joint_kp_0 等）
  std::optional<std::string> kp_group;
  std::optional<std::string> kd_group;
  std::optional<std::string> action_scale_group;
  std::optional<std::string> qd_mask_group;
  std::optional<std::string> default_joint_q_group;  // 可选：选择 default_joint_q_0/1/... 组
};

class RlMinicTrajectoryParam : public BasicParam {
 public:
  RlMinicTrajectoryParam(std::string_view tag = "rl_supine_to_stance");

  DEFINE_PARAM_SCOPE(scope_);

  // Single motion state profile
  MotionStateProfile motion_state;
  // Observation configuration (single config for all)
  ObservationConfig observations;
  // 统一的observation scale向量
  Eigen::VectorXd observation_scale_vec;

  // Parameters
  int num_actions;
  float observation_clip = 100.0f;
  std::vector<std::string> active_joint_names;
  std::vector<Eigen::VectorXd> default_joint_q;
  // 多组 default_joint_q 配置 (default_joint_q_0, default_joint_q_1, ...)
  std::map<std::string, std::vector<Eigen::VectorXd>> default_joint_q_groups;
  std::vector<Eigen::VectorXd> joint_kp;
  std::vector<Eigen::VectorXd> joint_kd;
  std::vector<Eigen::VectorXd> action_scale;
  std::optional<std::vector<Eigen::VectorXd>> qd_mask;
  float imu_install_delta_bias;
  float linear_vel_delta_bias;
  float action_clip;
  float transition_time;

  // Torque limiting parameters
  bool torque_limit = false;
  std::vector<Eigen::VectorXd> max_torque_joint;
  // New: group torque budget for lower body (sum of |torque| over lower-body joints)
  double max_lower_body_torque = 500.0;
  // Number of lower body joints (legs) for torque limiting
  int lower_body_joint_count = 12;

  // Gait phase adjustment parameters
  double unstable_cycle_time_scale = 0.5;

  // Default dynamic cycle time when configuration is invalid
  double default_dynamic_cycle_time = 0.8;
};

}  // namespace data

namespace YAML {
template <>
struct convert<data::ObservationScaleConfig> {
  static bool decode(const Node& node, data::ObservationScaleConfig& param) {
    if (node["observation_scale_linear_vel"])
      param.observation_scale_linear_vel = node["observation_scale_linear_vel"].as<double>();
    if (node["observation_scale_angular_vel"])
      param.observation_scale_angular_vel = node["observation_scale_angular_vel"].as<double>();
    if (node["observation_scale_dof_pos"])
      param.observation_scale_dof_pos = node["observation_scale_dof_pos"].as<double>();
    if (node["observation_scale_dof_vel"])
      param.observation_scale_dof_vel = node["observation_scale_dof_vel"].as<double>();
    if (node["observation_scale_quat"]) param.observation_scale_quat = node["observation_scale_quat"].as<double>();
    if (node["observation_scale_dof_pos_ref_diff"])
      param.observation_scale_dof_pos_ref_diff = node["observation_scale_dof_pos_ref_diff"].as<double>();
    return true;
  }
};

template <>
struct convert<data::ObservationGoalTypeConfig> {
  static bool decode(const Node& node, data::ObservationGoalTypeConfig& param) {
    if (node["goal_length"]) param.goal_length = node["goal_length"].as<int>();
    if (node["goal_history_steps"]) param.goal_history_steps = node["goal_history_steps"].as<int>();
    if (node["num_include_obs_steps"]) param.num_include_obs_steps = node["num_include_obs_steps"].as<int>();
    if (node["goal_scale"]) param.goal_scale = node["goal_scale"].as<double>();
    return true;
  }
};

template <>
struct convert<data::ObservationConfig> {
  static bool decode(const Node& node, data::ObservationConfig& param) {
    if (node["num_single_observations"]) param.num_single_observations = node["num_single_observations"].as<int>();
    if (node["observation_type"])
      param.observation_type = node["observation_type"].as<std::map<std::string, data::ObservationGoalTypeConfig>>();
    if (node["observation_scale"])
      param.observation_scale = node["observation_scale"].as<data::ObservationScaleConfig>();
    return true;
  }
};

template <>
struct convert<data::MotionStateProfile> {
  static bool decode(const Node& node, data::MotionStateProfile& param) {
    if (node["observation_type"]) param.observation_type = node["observation_type"].as<std::string>();
    if (node["data_path"]) param.data_path = node["data_path"].as<std::string>();
    if (node["csv_dt"]) param.csv_dt = node["csv_dt"].as<double>();
    if (node["policy_path"]) param.policy_path = node["policy_path"].as<std::string>();
    if (node["traj_frame"]) param.traj_frame = node["traj_frame"].as<std::vector<int>>();
    if (node["replay"]) param.replay = node["replay"].as<bool>();
    if (node["residual_control"]) param.residual_control = node["residual_control"].as<bool>();
    if (node["cycle_time"]) param.cycle_time = node["cycle_time"].as<std::vector<double>>();
    if (node["stride_max"]) param.stride_max = node["stride_max"].as<double>();
    if (node["yaw_per_cycle_max"]) param.yaw_per_cycle_max = node["yaw_per_cycle_max"].as<double>();
    if (node["k_angle"]) param.k_angle = node["k_angle"].as<double>();
    if (node["step_phase"]) param.step_phase = node["step_phase"].as<double>();
    if (node["phase_range"]) param.phase_range = node["phase_range"].as<std::vector<double>>();
    if (node["transition_time"]) param.transition_time = node["transition_time"].as<double>();
    if (node["kp_group"]) param.kp_group = node["kp_group"].as<std::string>();
    if (node["kd_group"]) param.kd_group = node["kd_group"].as<std::string>();
    if (node["action_scale_group"]) param.action_scale_group = node["action_scale_group"].as<std::string>();
    if (node["qd_mask_group"]) param.qd_mask_group = node["qd_mask_group"].as<std::string>();
    if (node["default_joint_q_group"]) param.default_joint_q_group = node["default_joint_q_group"].as<std::string>();
    return true;
  }
};
}  // namespace YAML