#ifndef ROS2_NODE_INCLUDE_ROS2_NODE_ROS2_NODE_INTERFACE_H_
#define ROS2_NODE_INCLUDE_ROS2_NODE_ROS2_NODE_INTERFACE_H_

#include <memory>

#include "data_store/data_store.h"

#include "ros2_node/manager_node.h"

namespace ros2 {

class Ros2NodeInterface {
 public:
  Ros2NodeInterface(const std::shared_ptr<data::DataStore>& data_store,
                    const std::shared_ptr<data::Ros2BridgeParam>& param);
  ~Ros2NodeInterface() = default;

  void Run() const;
  void Stop();

 private:
  std::string node_name_ = "";
  rclcpp::executors::MultiThreadedExecutor::SharedPtr ros2_executor_;

  std::shared_ptr<ManagerNode> manager_node_;
};
}  // namespace ros2

#endif  // ROS2_NODE_INCLUDE_ROS2_NODE_ROS2_NODE_INTERFACE_H_
