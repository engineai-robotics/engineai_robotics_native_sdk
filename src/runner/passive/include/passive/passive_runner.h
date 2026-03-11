#pragma once

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "passive_param/passive_param.h"
namespace runner {

class PassiveRunner : public MotionRunner {
 public:
  PassiveRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : MotionRunner(name, data_store) {
    param_ = data::ParamManager::create<data::PassiveParam>();
  }
  ~PassiveRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

  void Log();

 private:
  std::shared_ptr<data::PassiveParam> param_;
  Eigen::VectorXd damping_;
  Eigen::VectorXd zeros_;
};
}  // namespace runner

REGISTER_RUNNER(PassiveRunner, "passive_runner", kMotion)
