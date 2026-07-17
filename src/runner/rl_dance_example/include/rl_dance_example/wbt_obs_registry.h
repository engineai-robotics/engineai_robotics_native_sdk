#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "data_store/data_store.h"

/// WBT observation registry: dimensions and getters are registered in wbt_obs_registry.cc.
namespace wbt_obs {

/// Data source for observation computation, populated by WbtRunner.
/// - Constant parts are filled once in Enter() (data_store, ref_*_all, num_actions, pointers, etc.)
/// - Only policy_step is updated each frame; ref_init_yaw_rot / body_init_yaw_rot on the first frame.
struct ObsContext {
  std::shared_ptr<data::DataStore> data_store = nullptr;

  std::shared_ptr<const Eigen::MatrixXd> ref_joint_pos_all = nullptr;
  std::shared_ptr<const Eigen::MatrixXd> ref_joint_vel_all = nullptr;
  std::shared_ptr<const Eigen::MatrixXd> ref_body_quat_w_all = nullptr;
  int policy_step = 0;

  int num_actions = 0;
  std::shared_ptr<Eigen::VectorXd> default_joint_q = nullptr;
  std::shared_ptr<Eigen::VectorXi> policy2deploy_joint_idx = nullptr;
  std::shared_ptr<Eigen::VectorXd> actions = nullptr;

  Eigen::Matrix3d ref_init_yaw_rot = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d body_init_yaw_rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d imu_install_bias = Eigen::Vector3d::Zero();

  std::shared_ptr<const std::vector<int>> joint_vel_mask_indices = nullptr;

  std::shared_ptr<const Eigen::VectorXd> soft_joint_pos_limit = nullptr;

  int future_cmd_step = 0;
};

/// Single-step observation dimension (without history). Some observations depend on num_actions.
int GetObservationDim(const std::string& name, int num_actions);

/// Compute and return a single-step observation vector by name using the registered getter.
Eigen::VectorXd GetObservation(const std::string& name, const ObsContext& ctx);

}  // namespace wbt_obs
