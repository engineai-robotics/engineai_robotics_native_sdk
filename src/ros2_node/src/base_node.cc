#include "ros2_node/base_node.h"

#include <glog/logging.h>
#include <chrono>

#include "math/constants.h"

namespace ros2 {

LogicNode::LogicNode(const std::shared_ptr<data::DataStore>& data_store, const std::string& param_tag,
                     const std::string& node_name)
    : BaseNode(data_store, param_tag, node_name) {
  if (param_->enable.has_value()) {
    SetEnabled(param_->enable.value());
  }
}

void LogicNode::Start() {
  // 懒加载：如果节点未初始化，则先初始化
  if (!is_initialized_) {
    if (!Init()) {
      return;
    }
    is_initialized_ = true;
  }

  if (!timer_) {
    if (!param_->period.has_value()) {
      return;
    }

    int period = param_->period.value() * math::kSecondToMilliSecond;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&LogicNode::TimerCallback, this));
  }
  is_enabled_ = true;
  if (timer_) {
    timer_->reset();
  }
}

void LogicNode::Stop() {
  if (is_enabled_) {
    if (timer_) {
      timer_->cancel();
    }
    is_enabled_ = false;
  }
}

}  // namespace ros2
