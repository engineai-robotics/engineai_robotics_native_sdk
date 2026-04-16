#pragma once

#include "interface_protocol/msg/node_control.hpp"
#include "ros2_node/base_node.h"

namespace ros2 {

class ManagerNode final : public BaseNode {
 public:
  ManagerNode(const std::shared_ptr<data::DataStore>& data_store, const std::string& param_tag,
              const std::string& node_name = "manager_node")
      : BaseNode(data_store, param_tag, node_name), creators_cache_(LogicNode::GetLogicNodeCreators()) {
    CreateSubscription();
    LOG(INFO) << "ManagerNode created successfully";
  }

  ~ManagerNode() = default;

  bool LogicNodeControlCallback(const interface_protocol::msg::NodeControl::SharedPtr msg);
  std::shared_ptr<LogicNode> CreateLogicNode(const std::shared_ptr<data::DataStore>& data_store,
                                             const std::string& param_tag, const std::string& node_name);

 private:
  bool CreateSubscription();

  rclcpp::Subscription<interface_protocol::msg::NodeControl>::SharedPtr node_control_subscriber_;

  std::shared_mutex logic_node_map_mutex_;
  std::map<std::string, std::shared_ptr<LogicNode>> logic_node_map_;
  const std::map<std::string, LogicNodeCreatorType>& creators_cache_;
};

}  // namespace ros2
