#ifndef BASIC_MOTION_RUNNER_H
#define BASIC_MOTION_RUNNER_H

#include "basic/basic_runner.h"
#include "control_output/control_output.h"

namespace runner {
class MotionRunner : public BasicRunner {
 public:
  MotionRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : BasicRunner(name, data_store) {
    output_.Resize(data_store_->model_param->num_total_joints);
  }

  ~MotionRunner() override = default;

  virtual void SetupContext() = 0;
  virtual void TeardownContext() = 0;

  // 输出缓存接口
  void SetOutput(const data::ControlOutput& output) { output_ = output; }
  const data::ControlOutput& GetOutput() const { return output_; }
  data::ControlOutput& GetMutableOutput() { return output_; }

 protected:
  data::ControlOutput output_;
};

}  // namespace runner

#endif  // BASIC_MOTION_RUNNER_H
