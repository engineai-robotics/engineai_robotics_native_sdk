#include "ros2_node/ros2_node_interface.h"

#include <glog/logging.h>

namespace ros2 {

Ros2NodeInterface::Ros2NodeInterface(const std::shared_ptr<data::DataStore>& data_store,
                                     const std::shared_ptr<data::Ros2BridgeParam>& param) {
  if (!rclcpp::ok()) {
    rclcpp::InitOptions op;
    op.shutdown_on_signal = true;
    rclcpp::init(0, nullptr, op, rclcpp::SignalHandlerOptions::None);
    LOG(INFO) << "ROS2 initialized";
  }
  ros2_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  manager_node_ = std::make_shared<ManagerNode>(data_store, param->GetTag());
  ros2_executor_->add_node(manager_node_);

  for (const auto& node_name : param->activated_node) {
    auto node_ptr = manager_node_->CreateLogicNode(data_store, param->GetTag(), node_name);
    if (!node_ptr) {
      LOG(ERROR) << "Failed to create " << node_name;
      continue;
    }

    auto ros2_node_ptr = std::static_pointer_cast<rclcpp::Node>(node_ptr);
    if (!ros2_node_ptr) {
      LOG(ERROR) << "Failed to cast to rclcpp::Node for " << node_name;
      continue;
    }

    ros2_executor_->add_node(ros2_node_ptr);
    LOG(INFO) << "Added " << node_name << " to ros2 executor";
  }
}

void Ros2NodeInterface::Run() const {
  if (ros2_executor_) {
    ros2_executor_->spin();
  } else {
    LOG(ERROR) << "Cannot run ros2 executor";
  }
}

void Ros2NodeInterface::Stop() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
    if (!rclcpp::ok()) {
      LOG(INFO) << "ROS2 shutdown successful";
    } else {
      LOG(ERROR) << "ROS2 shutdown failed";
    }
  } else {
    LOG(WARNING) << "ROS2 already shutdown";
  }
  if (ros2_executor_) {
    ros2_executor_.reset();
  }
}
}  // namespace ros2
