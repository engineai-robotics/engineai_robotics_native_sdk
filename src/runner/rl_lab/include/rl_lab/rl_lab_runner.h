#pragma once

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "math/first_order_low_pass_filter.h"
#include "math/mnn_model.h"
#include "parameter/global_config_initializer.h"
#include "rl_lab_param/rl_lab_param.h"
#include "tool/string_join.h"

namespace runner {

class RlLabRunner : public MotionRunner {
 public:
  RlLabRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store);
  ~RlLabRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

  void Log();

 private:
  void UpdateIMUInstalBias();
  void UpdateRobotBaseOrientation();
  void UpdateRemoteCommandBias();
  void UpdateRemoteCommand();
  void CalculateObservation();
  void CalculateMotorCommand();
  void SendMotorCommand();

  std::shared_ptr<data::RlLabParam> param_;
  float time_ = 0.0;
  float global_phase_ = 0.0;
  bool is_first_time_ = true;
  bool still_flag_ = false;
  bool still_pressed_ = false;

  std::unique_ptr<math::MNNModel> mlp_net_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> joint_pos_history_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> joint_vel_history_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> action_history_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> base_ang_vel_history_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> projected_gravity_history_;
  Eigen::VectorXd mlp_net_action_;
  int non_command_observation_dim_ = 0;
  static constexpr int kCommandObservationDim = 3;

  Eigen::VectorXd q_real_;
  Eigen::VectorXd qd_real_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd qd_des_;
  Eigen::VectorXd tau_ff_des_;
  Eigen::VectorXi active_joint_idx_;
  Eigen::VectorXd initial_joint_q_;

  Eigen::VectorXd default_joint_q_;
  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;
  Eigen::VectorXd action_scale_;

  Eigen::Matrix3d rotation_install_;
  Eigen::Matrix3d rotation_measured_;
  Eigen::Matrix3d rotation_real_;
  Eigen::Vector3d angular_vel_real_;
  Eigen::Vector3d euler_xyz_;
  Eigen::Vector3d projected_gravity_real_;
  Eigen::Matrix3d heading_rotation_initial_;

  Eigen::Vector3d imu_install_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d command_bias_ = Eigen::Vector3d::Zero();
  data::GamepadInfo last_remote_command_;
  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
  std::unique_ptr<math::FirstOrderLowPassFilter<Eigen::Vector3d>> lpf_command_;
};
}  // namespace runner

REGISTER_RUNNER(RlLabRunner, "rl_lab_runner", kMotion)
