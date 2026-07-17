#pragma once

#include <rclcpp/rclcpp.hpp>
#include "data_store/data_store.h"
#include "ros2_bridge_param/ros2_bridge_param.h"
namespace ros2 {

class LogicNode;
using LogicNodeCreatorType =
    std::function<std::shared_ptr<LogicNode>(const std::shared_ptr<data::DataStore>&, const std::string& param_tag)>;

class BaseNode : public rclcpp::Node {
 public:
  BaseNode(const std::shared_ptr<data::DataStore>& data_store, const std::string& param_tag,
           const std::string& node_name = "base_node")
      : rclcpp::Node(node_name),
        data_store_(data_store),
        param_(std::make_unique<data::Ros2NodeParam>(node_name, param_tag)),
        node_name_(node_name) {}
  std::string GetNodeName() const { return node_name_; }
  virtual ~BaseNode() = default;

 protected:
  std::shared_ptr<data::DataStore> data_store_;
  std::unique_ptr<data::Ros2NodeParam> param_;
  std::string node_name_;
};

class LogicNode : public BaseNode {
 public:
  LogicNode(const std::shared_ptr<data::DataStore>& data_store, const std::string& param_tag,
            const std::string& node_name = "custom_node");

  virtual ~LogicNode() = default;
  virtual bool Init() { return true; }
  virtual void Start();
  virtual void Stop();
  bool IsInitialized() const { return is_initialized_; }
  bool IsEnabled() const { return is_enabled_; }
  void SetEnabled(bool enabled) { is_enabled_ = enabled; }
  virtual void TimerCallback() {}

  static std::map<std::string, LogicNodeCreatorType>& GetLogicNodeCreators() {
    static std::map<std::string, LogicNodeCreatorType> logic_node_creators_map;
    return logic_node_creators_map;
  }

 protected:
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_initialized_ = false;
  bool is_enabled_ = true;
};

#define REGISTER_LOGIC_NODE_TYPE(NodeClassName, RegisteredTypeTag)             \
  static_assert(std::is_base_of<ros2::LogicNode, NodeClassName>::value,        \
                #NodeClassName " must inherit from ros2::LogicNode.");         \
  inline static bool _##NodeClassName##_registered = []() {                    \
    ros2::LogicNode::GetLogicNodeCreators()[RegisteredTypeTag] =               \
        [](const std::shared_ptr<data::DataStore>& ds,                         \
           const std::string& param_tag) -> std::shared_ptr<ros2::LogicNode> { \
      return std::make_shared<NodeClassName>(ds, param_tag);                   \
    };                                                                         \
    return true;                                                               \
  }();

}  // namespace ros2
