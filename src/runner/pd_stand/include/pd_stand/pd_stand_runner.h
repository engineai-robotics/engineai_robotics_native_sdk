#pragma once

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "global_options_param/global_options_param.h"
#include "pd_stand_param/pd_stand_param.h"
namespace runner {

class PdStandRunner : public MotionRunner {
 public:
  PdStandRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : MotionRunner(name, data_store) {
    param_ = data::ParamManager::create<data::PdStandParam>();
    global_options_param_ = data::ParamManager::create<data::GlobalOptionsParam>();
  }
  ~PdStandRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

  void Log();

 private:
  bool CheckJointPositionBias();
  std::shared_ptr<data::PdStandParam> param_;
  std::shared_ptr<data::GlobalOptionsParam> global_options_param_;
  Eigen::VectorXd q_init_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd q_cmd_;
  Eigen::VectorXd qd_cmd_;
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd tau_ff_cmd_;

  int iter_ = 0;
  int num_duration_iterations_ = 0;
  double duration_ = 0.0;
  bool auto_transition_ = false;
};
}  // namespace runner

REGISTER_RUNNER(PdStandRunner, "pd_stand_runner", kMotion)
