#pragma once

#include "ros2_node/base_node.h"

#include "gamepad_info/gamepad_info.h"
#include "interface_protocol/msg/gamepad_keys.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/led_control.hpp"
#include "interface_protocol/msg/motor_debug.hpp"
#include "interface_protocol/msg/power_info.hpp"
#include "motor_debug/motor_debug.h"
#include "power_info/power_info.h"
#include "variant_store/variant_store.h"

namespace ros2 {
class HardwareInterfaceNode final : public LogicNode {
 public:
  HardwareInterfaceNode(const std::shared_ptr<data::DataStore>& data_store, const std::string& param_tag,
                        const std::string& node_name = "hardware_interface_node")
      : LogicNode(data_store, param_tag, node_name) {}
  ~HardwareInterfaceNode() = default;

 private:
  bool Init() override;
  void Start() override;
  void Stop() override;
  void TimerCallback() override;

  bool CreateTimer();
  bool CreatePublisher();
  bool CreateSubscription();

  void PublishImuInfo();
  void PublishGamepadKeys();
  void PublishMotorDebug();
  void PublishPowerInfo();
  void PublishMotorState();
  void PublishMotorCommand();
  void PublishJointState();
  void PublishJointCommandFeedback();

  void LedControlCallback(const interface_protocol::msg::LedControl::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<interface_protocol::msg::ImuInfo>::SharedPtr imu_pub_;
  rclcpp::Publisher<interface_protocol::msg::GamepadKeys>::SharedPtr gamepad_pub_;
  rclcpp::Publisher<interface_protocol::msg::MotorDebug>::SharedPtr motor_debug_pub_;
  rclcpp::Publisher<interface_protocol::msg::PowerInfo>::SharedPtr power_info_pub_;
  rclcpp::Publisher<interface_protocol::msg::JointState>::SharedPtr motor_state_publisher_;
  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr motor_command_publisher_;
  rclcpp::Publisher<interface_protocol::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_command_feedback_pub_;

  // Subscription
  rclcpp::Subscription<interface_protocol::msg::LedControl>::SharedPtr led_control_sub_;

  // Message
  std::shared_ptr<interface_protocol::msg::ImuInfo> imu_msg_;
  std::shared_ptr<interface_protocol::msg::GamepadKeys> gamepad_msg_;
  std::shared_ptr<interface_protocol::msg::MotorDebug> motor_debug_msg_;
  std::shared_ptr<interface_protocol::msg::PowerInfo> power_info_msg_;
  interface_protocol::msg::JointState motor_state_;
  interface_protocol::msg::JointCommand motor_command_;
  std::shared_ptr<interface_protocol::msg::JointState> joint_state_msg_;
  std::shared_ptr<interface_protocol::msg::JointCommand> joint_command_feedback_msg_;

  // Timer
  rclcpp::TimerBase::SharedPtr gamepad_timer_;
  rclcpp::TimerBase::SharedPtr motor_debug_timer_;
  rclcpp::TimerBase::SharedPtr power_info_timer_;

  // Variables
  data::ImuInfo imu_info_;

  // Subscribers
  data::Subscriber<data::GamepadInfo> gamepad_subscriber_;
  data::Subscriber<data::MotorDebug> motor_debug_subscriber_;
  data::Subscriber<data::PowerInfo> power_info_subscriber_;
};

REGISTER_LOGIC_NODE_TYPE(HardwareInterfaceNode, "hardware_interface_node");
}  // namespace ros2
