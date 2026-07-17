#pragma once

#include "basic/basic_runner.h"
#include "basic/runner_registry.h"

#ifdef __BUILD_ROS2
#  include "ros2_node/ros2_node_interface.h"
#endif

namespace runner {

class Ros2BridgeRunner : public BasicRunner {
 public:
  Ros2BridgeRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store)
      : BasicRunner(name, data_store) {
    param_ = data::ParamManager::create<data::Ros2BridgeParam>();
  }
  ~Ros2BridgeRunner() = default;

  bool Enter() override;
  void Run() override;
  TransitionState TryExit() override;
  bool Exit() override;
  void End() override;

  void Log();

 private:
  int iter_ = 0;
  std::shared_ptr<data::Ros2BridgeParam> param_;
#ifdef __BUILD_ROS2
  std::shared_ptr<ros2::Ros2NodeInterface> ros2_node_interface_;
#endif
};
}  // namespace runner

REGISTER_RUNNER(Ros2BridgeRunner, "ros2_bridge_runner", kResident)
