#ifndef RUNNER_BASIC_INCLUDE_BASIC_BASIC_RUNNER_H_
#define RUNNER_BASIC_INCLUDE_BASIC_BASIC_RUNNER_H_

#include <memory>

#include "data_store/data_store.h"

namespace runner {

enum class RunnerState {
  kRunning = 0,
  kTryExit,
  kFault,
};

enum class TransitionState {
  // Attempt to transition/exit completed successfully
  kCompleted = 0,
  // Currently attempting to transition/exit
  kTrying,
  // Attempt to transition/exit failed
  kFailed,
  // Restore to running state
  kRestoreRunning,
};

class BasicRunner {
 public:
  BasicRunner() = default;
  BasicRunner(const std::shared_ptr<data::DataStore>& data_store);
  BasicRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store);
  BasicRunner(const BasicRunner&) = delete;
  BasicRunner(BasicRunner&&) = delete;
  BasicRunner& operator=(const BasicRunner&) = delete;
  virtual ~BasicRunner() { End(); }
  virtual bool Initialize();
  virtual bool Enter();
  virtual void Run();
  virtual TransitionState TryExit();
  virtual bool Exit();
  virtual void End();

  void SetRunnerState(RunnerState state) { runner_state_ = state; }
  RunnerState GetRunnerState() const { return runner_state_; }
  void SetPreviousMotion(std::string_view name) { previous_motion_ = name; }
  void SetParamTag(std::string_view tag) { param_tag_ = tag.data(); }
  void SetRunnerPeriod(double period) { runner_period_ = period; }
  double GetRunnerPeriod() const { return runner_period_; }
  void SetTerminate(bool terminate);
  bool GetTerminate() const;

  const std::string& name() const { return name_; }

 protected:
  std::shared_ptr<data::DataStore> data_store_;
  std::shared_ptr<data::ModelParam> model_param_;
  std::string param_tag_ = "";
  double runner_period_ = 0.0;

 private:
  RunnerState runner_state_ = RunnerState::kRunning;
  std::string previous_motion_;
  std::string name_ = "runner";
  bool terminate_ = false;
};
}  // namespace runner

#endif  // RUNNER_BASIC_INCLUDE_BASIC_BASIC_RUNNER_H_
