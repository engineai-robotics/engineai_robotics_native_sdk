#pragma once

#include "basic/motion_runner.h"
#include "basic/runner_registry.h"
#include "idle_param/idle_param.h"

namespace runner {

class IdleRunner : public MotionRunner {
 public:
  IdleRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : MotionRunner(name, data_store) {}
  ~IdleRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;
  void SetupContext() override;
  void TeardownContext() override;

  void Log();

 private:
  static constexpr int kNumTryExit = 1000;

  int iter_ = 0;
  int num_try_exit_ = kNumTryExit;

  std::shared_ptr<data::IdleParam> param_;
};

}  // namespace runner

REGISTER_RUNNER(IdleRunner, "idle_runner", kMotion)
