#pragma once

#include <iostream>

#include "basic_param/basic_param.h"
#include "parameter/parameter_loader.h"
namespace data {

class RlWalkingExampleParam : public BasicParam {
 public:
  RlWalkingExampleParam(std::string_view tag = "rl_walking_example") : BasicParam(tag) {
    num_actions = active_joint_names.size();
    observation_scale = Eigen::VectorXd::Zero(num_observations);
    observation_scale << Eigen::VectorXd::Constant(
        num_actions,
        observation_scale_dof_pos),  // joint position - joint default position
        Eigen::VectorXd::Constant(num_actions,
                                  observation_scale_dof_vel),      // joint velocity
        Eigen::VectorXd::Ones(num_actions),                        // last joint action
        Eigen::Vector3d::Constant(observation_scale_angular_vel),  // base angular velocity
        Eigen::Vector3d::Constant(observation_scale_quat);         // base euler angle xyz
    command_obs_scale = Eigen::VectorXd::Zero(3);
    command_obs_scale << Eigen::Vector2d::Constant(observation_scale_linear_vel), observation_scale_angular_vel;
  };

  DEFINE_PARAM_SCOPE(scope_);

  // Sets mlp net parameters
  std::string LOAD_PARAM(policy_file);
  int LOAD_PARAM(num_observations);
  std::vector<std::string> LOAD_PARAM(active_joint_names);
  int num_actions;
  int LOAD_PARAM(num_include_obs_steps);

  // Sets observation parameters
  float LOAD_PARAM(observation_scale_linear_vel);
  float LOAD_PARAM(observation_scale_angular_vel);
  float LOAD_PARAM(observation_scale_dof_pos);
  float LOAD_PARAM(observation_scale_dof_vel);
  float LOAD_PARAM(observation_scale_quat);
  float LOAD_PARAM(observation_clip);
  Eigen::VectorXd observation_scale;
  Eigen::VectorXd command_obs_scale;

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
  Eigen::Vector3d LOAD_PARAM(command_scale_pos);
  Eigen::Vector3d LOAD_PARAM(command_scale_neg);
  std::optional<Eigen::Vector3d> LOAD_PARAM(imu_install_bias);

  void Update() {
    LOAD_PARAM(default_joint_q);
    LOAD_PARAM(joint_kp);
    LOAD_PARAM(joint_kd);
    LOAD_PARAM(enable_remote_command_lpf);
    LOAD_PARAM(remote_command_cut_off_frequency);
    LOAD_PARAM(command_scale_pos);
    LOAD_PARAM(command_scale_neg);
  }
};
}  // namespace data
