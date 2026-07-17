#include "ros2_node/manager_node.h"

namespace ros2 {

bool ManagerNode::CreateSubscription() {
  if (!param_->subscribe_topics.has_value()) {
    return false;
  }
  const auto& subscribe_topics = param_->subscribe_topics.value();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  try {
    if (subscribe_topics.contains("node_control")) {
      node_control_subscriber_ = this->create_subscription<interface_protocol::msg::NodeControl>(
          subscribe_topics.at("node_control"), qos,
          std::bind(&ManagerNode::LogicNodeControlCallback, this, std::placeholders::_1));
      LOG(INFO) << "Create " << subscribe_topics.at("node_control") << " subscription success";
    }

    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Error creating subscriptions: " << e.what();
    return false;
  }
}

std::shared_ptr<LogicNode> ManagerNode::CreateLogicNode(const std::shared_ptr<data::DataStore>& data_store,
                                                        const std::string& param_tag, const std::string& node_name) {
  {
    std::shared_lock lock_read(logic_node_map_mutex_);
    if (logic_node_map_.contains(node_name)) {
      return logic_node_map_.at(node_name);
    }
  }

  if (!creators_cache_.contains(node_name)) {
    LOG(ERROR) << node_name << " is not registered in creators";
    return nullptr;
  }

  auto node_ptr = creators_cache_.at(node_name)(data_store, param_tag);
  if (node_ptr) {
    std::unique_lock lock_write(logic_node_map_mutex_);
    logic_node_map_.emplace(node_name, node_ptr);
    if (node_ptr->IsEnabled()) {
      node_ptr->Start();
    }

    return node_ptr;
  } else {
    LOG(ERROR) << "Creator function returned nullptr for logic node '" << node_name << "' with type tag '" << node_name
               << "'.";
  }

  return nullptr;
}

bool ManagerNode::LogicNodeControlCallback(const interface_protocol::msg::NodeControl::SharedPtr msg) {
  if (!msg) {
    LOG(ERROR) << "LogicNodeControlCallback received nullptr";
    return false;
  }

  std::shared_ptr<LogicNode> node;
  {
    std::shared_lock lock_read(logic_node_map_mutex_);
    if (auto it = logic_node_map_.find(msg->node_name); it != logic_node_map_.end()) {
      node = it->second;
    }
  }

  if (!node) {
    LOG(ERROR) << "LogicNodeControlCallback received unknown node name: " << msg->node_name;
    return false;
  }
  LOG(INFO) << "Node control received node name: " << msg->node_name
            << " command: " << (msg->command ? "start" : "stop");
  msg->command ? node->Start() : node->Stop();
  return true;
}

}  // namespace ros2
