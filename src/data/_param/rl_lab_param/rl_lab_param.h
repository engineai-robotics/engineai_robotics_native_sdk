#pragma once

#include <iostream>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"
namespace data {

class RlLabParam : public BasicParam {
 public:
  RlLabParam(std::string_view tag = "rl_lab") : BasicParam(tag) {
    num_actions = active_joint_names.size();
  };

  DEFINE_PARAM_SCOPE(scope_);

  // Sets mlp net parameters
  std::string LOAD_PARAM(policy_file);
  int LOAD_PARAM(num_observations);
  std::vector<std::string> LOAD_PARAM(active_joint_names);
  int num_actions;
  int LOAD_PARAM(num_include_obs_steps);

  // Sets joint control parameters
  float LOAD_PARAM(action_clip);
  std::vector<Eigen::VectorXd> LOAD_PARAM(default_joint_q);
  std::vector<Eigen::VectorXd> LOAD_PARAM(joint_kp);
  std::vector<Eigen::VectorXd> LOAD_PARAM(joint_kd);
  std::vector<Eigen::VectorXd> LOAD_PARAM(action_scale);
  float LOAD_PARAM(control_dt);

  // Sets sim to real fine tune parameters
  bool LOAD_PARAM(enable_remote_command_lpf);
  float LOAD_PARAM(remote_command_sampling_frequency);
  float LOAD_PARAM(remote_command_cut_off_frequency);
  Eigen::Vector3d LOAD_PARAM(command_scale);
  Eigen::Vector3d LOAD_PARAM(command_bias);
  float LOAD_PARAM(linear_vel_delta_bias);
  float LOAD_PARAM(angular_vel_delta_bias);
  float LOAD_PARAM(imu_install_delta_bias);
  std::optional<Eigen::Vector3d> LOAD_PARAM(imu_install_bias);
  std::optional<float> LOAD_PARAM(transition_time);

  // Sets replaced limbs parameters
  std::optional<std::vector<std::string>> LOAD_PARAM(replaced_limbs);
  std::optional<bool> LOAD_PARAM(replaced_gamepad);

  void Update() {
    LOAD_PARAM(default_joint_q);
    LOAD_PARAM(joint_kp);
    LOAD_PARAM(joint_kd);
    LOAD_PARAM(enable_remote_command_lpf);
    LOAD_PARAM(remote_command_cut_off_frequency);
    LOAD_PARAM(command_scale);
    LOAD_PARAM(command_bias);
    LOAD_PARAM(replaced_limbs);
    LOAD_PARAM(replaced_gamepad);
  }
};
}  // namespace data
