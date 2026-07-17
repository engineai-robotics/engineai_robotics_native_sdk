#include "ros2_bridge/ros2_bridge_runner.h"

#include <glog/logging.h>
#include <iostream>

namespace runner {

bool Ros2BridgeRunner::Enter() {
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::Ros2BridgeParam>(param_tag_);
  }

#ifdef __BUILD_ROS2
  ros2_node_interface_ = std::make_shared<ros2::Ros2NodeInterface>(data_store_, param_);
#endif

  return true;
}

void Ros2BridgeRunner::Run() {
#ifdef __BUILD_ROS2
  if (!GetTerminate()) {
    ros2_node_interface_->Run();
  }
#endif
}

TransitionState Ros2BridgeRunner::TryExit() { return TransitionState::kCompleted; }

bool Ros2BridgeRunner::Exit() { return true; }

void Ros2BridgeRunner::End() {
  BasicRunner::End();
#ifdef __BUILD_ROS2
  if (ros2_node_interface_ != nullptr) {
    ros2_node_interface_->Stop();
    ros2_node_interface_.reset();
  }
#endif
}
}  // namespace runner
