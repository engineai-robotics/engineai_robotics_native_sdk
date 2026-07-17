#pragma once

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "math/mnn_model.h"
#include "parameter/global_config_initializer.h"
#include "rl_mimic_trajectory_example/csv_loader.h"
#include "rl_mimic_trajectory_example/observation_manager.h"
#include "rl_mimic_trajectory_example_param/rl_mimic_trajectory_example_param.h"

namespace runner {

class RlMimicTrajectoryRunner : public MotionRunner {
 public:
  RlMimicTrajectoryRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : MotionRunner(name, data_store) {
    param_ = data::ParamManager::create<data::RlMinicTrajectoryParam>();
  }
  ~RlMimicTrajectoryRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void SetupContext() override;
  void TeardownContext() override;

 private:
  void Init();
  void UpdateState();
  void CalculateObservation();
  void CalculateMotorCommand();
  void SendMotorCommand();

  // Helper methods
  void ApplyTorqueLimits(Eigen::VectorXd& q_des, const Eigen::VectorXd& q_actual, const Eigen::VectorXd& qd_actual,
                         const Eigen::VectorXd& joint_kp, const Eigen::VectorXd& joint_kd);
  bool IsTrajectoryFinished() const;
  double ComputeTransitionRatio() const;
  void PrecomputeTauMax();

  Eigen::Vector2d CalculateBaseYawForwardVector();

  std::shared_ptr<data::RlMinicTrajectoryParam> param_;
  std::string last_param_tag_ = "";
  bool is_first_time_ = true;
  bool run_reference_trajectory_ = false;
  int transition_iter_ = 0;
  size_t reference_trajectory_iter_ = 0;

  std::shared_ptr<math::MNNModel> policy_model_;
  math::MNNModel* policy_net_ = nullptr;  // 只做指针，不负责所有权
  Eigen::VectorXd policy_action_;

  Eigen::VectorXd q_actual_;
  Eigen::VectorXd qd_actual_;
  Eigen::VectorXd qd_actual_mask_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd qd_des_;
  Eigen::VectorXd tau_ff_des_;
  Eigen::VectorXd initial_joint_q_;
  Eigen::VectorXd default_joint_q_;
  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;
  Eigen::VectorXd action_scale_;
  Eigen::VectorXd qd_mask_;
  Eigen::VectorXi active_joint_idx_;

  Eigen::Vector3d imu_install_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();

  std::unique_ptr<rl_mimic_trajectory::ObservationManager> observation_manager_;

  // 插值后的关节轨迹 [N, num_actions]
  Eigen::MatrixXd interpolated_traj_;
  Eigen::MatrixXd* current_traj_ = nullptr;

  // 预计算的扭矩上限向量
  Eigen::VectorXd tau_max_;

  // Yaw forward vector calculation state
  bool initial_quat_set_ = false;
  Eigen::Vector4d initial_base_quat_ = Eigen::Vector4d::Zero();
};
}  // namespace runner

REGISTER_RUNNER(RlMimicTrajectoryRunner, "rl_mimic_trajectory_runner", kMotion)
