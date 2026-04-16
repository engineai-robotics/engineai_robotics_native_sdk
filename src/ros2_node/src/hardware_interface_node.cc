#include "ros2_node/hardware_interface_node.h"
#include "math/constants.h"
#include "ros2_node/ros2_message_convertor.h"
#include "variant_store/variant_store.h"

namespace ros2 {
bool HardwareInterfaceNode::Init() {
  if (!LogicNode::Init()) {
    return false;
  }

  auto& variant_store = data::VariantStore::GetInstance();
  gamepad_subscriber_ = variant_store.CreateSubscriber<data::GamepadInfo>("hardware/gamepad_info");
  motor_debug_subscriber_ = variant_store.CreateSubscriber<data::MotorDebug>("hardware/motor_debug");
  power_info_subscriber_ = variant_store.CreateSubscriber<data::PowerInfo>("hardware/power_info");

  if (!CreatePublisher() || !CreateSubscription()) {
    return false;
  }

  if (!CreateTimer()) {
    return false;
  }

  return true;
}

void HardwareInterfaceNode::Start() {
  // 调用父类的Start方法
  LogicNode::Start();

  // 启动已存在的定时器
  if (timer_) {
    timer_->reset();
  }
  if (gamepad_timer_) {
    gamepad_timer_->reset();
  }

  if (motor_debug_timer_) {
    motor_debug_timer_->reset();
  }

  if (power_info_timer_) {
    power_info_timer_->reset();
  }
}

void HardwareInterfaceNode::Stop() {
  LogicNode::Stop();

  if (gamepad_timer_) {
    gamepad_timer_->cancel();
  }

  if (motor_debug_timer_) {
    motor_debug_timer_->cancel();
  }

  if (power_info_timer_) {
    power_info_timer_->cancel();
  }
}

void HardwareInterfaceNode::TimerCallback() {
  PublishImuInfo();
  PublishMotorState();
  PublishMotorCommand();
  PublishJointState();
  PublishJointCommandFeedback();
}

bool HardwareInterfaceNode::CreateTimer() {  // 检查参数是否存在
  if (!param_->mapping_periods.has_value()) {
    return false;
  }

  const auto& mapping_periods = param_->mapping_periods.value();

  try {
    if (mapping_periods.contains("gamepad_keys") && !gamepad_timer_) {
      gamepad_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(mapping_periods.at("gamepad_keys") * math::kSecondToMilliSecond)),
          std::bind(&HardwareInterfaceNode::PublishGamepadKeys, this));
      LOG(INFO) << "Create gamepad_keys timer success";
    }

    if (mapping_periods.contains("motor_debug") && !motor_debug_timer_) {
      motor_debug_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(mapping_periods.at("motor_debug") * math::kSecondToMilliSecond)),
          std::bind(&HardwareInterfaceNode::PublishMotorDebug, this));
      LOG(INFO) << "Create motor_debug timer success";
    }

    if (mapping_periods.contains("power_info") && !power_info_timer_) {
      power_info_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(mapping_periods.at("power_info") * math::kSecondToMilliSecond)),
          std::bind(&HardwareInterfaceNode::PublishPowerInfo, this));
      LOG(INFO) << "Create power_info timer success";
    }

    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Error creating timers: " << e.what();
    return false;
  }
}

bool HardwareInterfaceNode::CreatePublisher() {
  if (!param_->publish_topics.has_value()) {
    return false;
  }

  const auto& publish_topics = param_->publish_topics.value();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();
  auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  imu_msg_ = std::make_shared<interface_protocol::msg::ImuInfo>();
  gamepad_msg_ = std::make_shared<interface_protocol::msg::GamepadKeys>();
  motor_debug_msg_ = std::make_shared<interface_protocol::msg::MotorDebug>();
  power_info_msg_ = std::make_shared<interface_protocol::msg::PowerInfo>();
  joint_state_msg_ = std::make_shared<interface_protocol::msg::JointState>();
  joint_command_feedback_msg_ = std::make_shared<interface_protocol::msg::JointCommand>();

  try {
    if (publish_topics.contains("imu_info")) {
      imu_pub_ = this->create_publisher<interface_protocol::msg::ImuInfo>(publish_topics.at("imu_info"), qos);
      LOG(INFO) << "Create " << publish_topics.at("imu_info") << " publisher success";
    }
    if (publish_topics.contains("gamepad_keys")) {
      gamepad_pub_ =
          this->create_publisher<interface_protocol::msg::GamepadKeys>(publish_topics.at("gamepad_keys"), qos);
      LOG(INFO) << "Create " << publish_topics.at("gamepad_keys") << " publisher success";
    }
    if (publish_topics.contains("motor_debug")) {
      motor_debug_pub_ =
          this->create_publisher<interface_protocol::msg::MotorDebug>(publish_topics.at("motor_debug"), qos_reliable);
      LOG(INFO) << "Create " << publish_topics.at("motor_debug") << " publisher success";
    }
    if (publish_topics.contains("power_info")) {
      power_info_pub_ =
          this->create_publisher<interface_protocol::msg::PowerInfo>(publish_topics.at("power_info"), qos);
      LOG(INFO) << "Create " << publish_topics.at("power_info") << " publisher success";
    }
    if (publish_topics.contains("motor_state")) {
      motor_state_publisher_ =
          this->create_publisher<interface_protocol::msg::JointState>(publish_topics.at("motor_state"), qos);
      LOG(INFO) << "Create " << publish_topics.at("motor_state") << " publisher success";
    }
    if (publish_topics.contains("motor_command")) {
      motor_command_publisher_ =
          this->create_publisher<interface_protocol::msg::JointCommand>(publish_topics.at("motor_command"), qos);
      LOG(INFO) << "Create " << publish_topics.at("motor_command") << " publisher success";
    }
    if (publish_topics.contains("joint_state")) {
      joint_state_pub_ =
          this->create_publisher<interface_protocol::msg::JointState>(publish_topics.at("joint_state"), qos);
      LOG(INFO) << "Create " << publish_topics.at("joint_state") << " publisher success";
    }
    if (publish_topics.contains("joint_command_feedback")) {
      joint_command_feedback_pub_ = this->create_publisher<interface_protocol::msg::JointCommand>(
          publish_topics.at("joint_command_feedback"), qos);
      LOG(INFO) << "Create " << publish_topics.at("joint_command_feedback") << " publisher success";
    }
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Error creating publishers: " << e.what();
    return false;
  }
}

bool HardwareInterfaceNode::CreateSubscription() {
  if (!param_->subscribe_topics.has_value()) {
    return false;
  }
  const auto& subscribe_topics = param_->subscribe_topics.value();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  try {
    if (subscribe_topics.contains("led_control")) {
      led_control_sub_ = this->create_subscription<interface_protocol::msg::LedControl>(
          subscribe_topics.at("led_control"), qos,
          std::bind(&HardwareInterfaceNode::LedControlCallback, this, std::placeholders::_1));
      LOG(INFO) << "Create " << subscribe_topics.at("led_control") << " subscription success";
    }

    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Error creating subscriptions: " << e.what();
    return false;
  }
}

void HardwareInterfaceNode::LedControlCallback(const interface_protocol::msg::LedControl::SharedPtr msg) {
  const auto color = static_cast<data::LedColor>(msg->color);
  data_store_->led_control_request.Set(data::LedMode::CUSTOM, color);
}

void HardwareInterfaceNode::PublishImuInfo() {
  if (!imu_pub_) {
    return;
  }

  // Gets current imu info
  const auto& imu_data = data_store_->imu_info.Get();
  ToRos2Msg(*imu_data, *imu_msg_);
  imu_pub_->publish(*imu_msg_);
}

void HardwareInterfaceNode::PublishGamepadKeys() {
  if (!gamepad_pub_) {
    return;
  }

  // 使用 VariantStore 中的硬件手柄原始数据发布，而非叠加后的 data_store->gamepad_info
  auto gamepad_info = gamepad_subscriber_.Get();

  gamepad_msg_->header.stamp = rclcpp::Clock().now();
  gamepad_msg_->hardware_connected = gamepad_info->hardware_connected;

  // 填充数字量状态
  std::array<int32_t, 12> digital_states = {
      static_cast<int32_t>(gamepad_info->LB),           // LB
      static_cast<int32_t>(gamepad_info->RB),           // RB
      static_cast<int32_t>(gamepad_info->A),            // A
      static_cast<int32_t>(gamepad_info->B),            // B
      static_cast<int32_t>(gamepad_info->X),            // X
      static_cast<int32_t>(gamepad_info->Y),            // Y
      static_cast<int32_t>(gamepad_info->BACK),         // BACK
      static_cast<int32_t>(gamepad_info->START),        // START
      static_cast<int32_t>(gamepad_info->CROSS_X > 0),  // CROSS_X_UP
      static_cast<int32_t>(gamepad_info->CROSS_X < 0),  // CROSS_X_DOWN
      static_cast<int32_t>(gamepad_info->CROSS_Y < 0),  // CROSS_Y_LEFT
      static_cast<int32_t>(gamepad_info->CROSS_Y > 0)   // CROSS_Y_RIGHT
  };
  gamepad_msg_->digital_states = digital_states;

  // 填充模拟量状态（直接使用驱动层已经归一化的值）
  std::array<double, 6> analog_states = {
      gamepad_info->LT,            // LT
      gamepad_info->RT,            // RT
      gamepad_info->LeftStick_X,   // LeftStick_X
      gamepad_info->LeftStick_Y,   // LeftStick_Y
      gamepad_info->RightStick_X,  // RightStick_X
      gamepad_info->RightStick_Y   // RightStick_Y
  };
  gamepad_msg_->analog_states = analog_states;

  gamepad_pub_->publish(*gamepad_msg_);
}

void HardwareInterfaceNode::PublishMotorDebug() {
  if (!motor_debug_pub_) {
    return;
  }

  // Gets motor debug data from variant store
  auto motor_debug_data = motor_debug_subscriber_.Get();
  if (!motor_debug_data || motor_debug_data->Size() == 0) {
    return;
  }

  // Converts data to ROS2 message
  motor_debug_msg_->mos_temperature.assign(motor_debug_data->mos_temperature.begin(),
                                           motor_debug_data->mos_temperature.end());
  motor_debug_msg_->motor_temperature.assign(motor_debug_data->motor_temperature.begin(),
                                             motor_debug_data->motor_temperature.end());
  motor_debug_msg_->voltage.assign(motor_debug_data->voltage.begin(), motor_debug_data->voltage.end());
  motor_debug_msg_->current.assign(motor_debug_data->current.begin(), motor_debug_data->current.end());
  motor_debug_msg_->error_code.assign(motor_debug_data->error_code.begin(), motor_debug_data->error_code.end());
  motor_debug_msg_->offline.assign(motor_debug_data->offline.begin(), motor_debug_data->offline.end());
  motor_debug_msg_->enable.assign(motor_debug_data->enable.begin(), motor_debug_data->enable.end());

  motor_debug_pub_->publish(*motor_debug_msg_);
}

void HardwareInterfaceNode::PublishPowerInfo() {
  if (!power_info_pub_) {
    return;
  }

  // Gets power info data from variant store
  auto power_info_data = power_info_subscriber_.Get();
  if (!power_info_data) {
    return;
  }

  // Converts data to ROS2 message
  power_info_msg_->enable = power_info_data->enable;
  power_info_msg_->percentage = power_info_data->percentage;
  power_info_msg_->voltage = power_info_data->voltage;
  power_info_msg_->current = power_info_data->current;
  power_info_msg_->current_limit = power_info_data->current_limit;
  power_info_msg_->error_code = power_info_data->error_code;

  power_info_pub_->publish(*power_info_msg_);
}

void HardwareInterfaceNode::PublishMotorState() {
  if (!motor_state_publisher_) {
    return;
  }

  Eigen::VectorXd q, qd, tau;
  data_store_->motor_info.GetState(data::JointInfoType::kPosition, q);
  data_store_->motor_info.GetState(data::JointInfoType::kVelocity, qd);
  data_store_->motor_info.GetState(data::JointInfoType::kTorque, tau);
  ToRos2Msg(q, qd, tau, motor_state_);
  motor_state_.header.stamp = this->now();
  motor_state_publisher_->publish(motor_state_);
}

void HardwareInterfaceNode::PublishMotorCommand() {
  if (!motor_command_publisher_) {
    return;
  }

  Eigen::VectorXd q_cmd, qd_cmd, tau_ff_cmd, tau_cmd, kp_cmd, kd_cmd;
  data_store_->motor_info.GetCommand(data::JointInfoType::kPosition, q_cmd);
  data_store_->motor_info.GetCommand(data::JointInfoType::kVelocity, qd_cmd);
  data_store_->motor_info.GetCommand(data::JointInfoType::kFeedForwardTorque, tau_ff_cmd);
  data_store_->motor_info.GetCommand(data::JointInfoType::kTorque, tau_cmd);
  data_store_->motor_info.GetCommand(data::JointInfoType::kStiffness, kp_cmd);
  data_store_->motor_info.GetCommand(data::JointInfoType::kDamping, kd_cmd);
  ToRos2Msg(q_cmd, qd_cmd, tau_ff_cmd, tau_cmd, kp_cmd, kd_cmd, motor_command_);
  motor_command_.header.stamp = this->now();
  motor_command_publisher_->publish(motor_command_);
}

void HardwareInterfaceNode::PublishJointState() {
  if (!joint_state_pub_) {
    return;
  }

  // Gets joint count from model parameter
  int num_joints = data_store_->model_param->num_total_joints;

  // Resizes message vectors to correct size
  joint_state_msg_->position.resize(num_joints);
  joint_state_msg_->velocity.resize(num_joints);
  joint_state_msg_->torque.resize(num_joints);

  // Gets joint state data using pre-sized Eigen vectors
  Eigen::VectorXd position(num_joints);
  Eigen::VectorXd velocity(num_joints);
  Eigen::VectorXd torque(num_joints);

  data_store_->joint_info.GetState(data::JointInfoType::kPosition, position);
  data_store_->joint_info.GetState(data::JointInfoType::kVelocity, velocity);
  data_store_->joint_info.GetState(data::JointInfoType::kTorque, torque);

  // Efficiently copies data using Eigen assignment to mapped vectors
  Eigen::Map<Eigen::VectorXd>(joint_state_msg_->position.data(), num_joints) = position;
  Eigen::Map<Eigen::VectorXd>(joint_state_msg_->velocity.data(), num_joints) = velocity;
  Eigen::Map<Eigen::VectorXd>(joint_state_msg_->torque.data(), num_joints) = torque;

  joint_state_pub_->publish(*joint_state_msg_);
}

void HardwareInterfaceNode::PublishJointCommandFeedback() {
  if (!joint_command_feedback_pub_) {
    return;
  }

  // Gets joint count from model parameter
  int num_joints = data_store_->model_param->num_total_joints;

  // Resizes message vectors to correct size
  joint_command_feedback_msg_->position.resize(num_joints);
  joint_command_feedback_msg_->velocity.resize(num_joints);
  joint_command_feedback_msg_->feed_forward_torque.resize(num_joints);
  joint_command_feedback_msg_->torque.resize(num_joints);
  joint_command_feedback_msg_->stiffness.resize(num_joints);
  joint_command_feedback_msg_->damping.resize(num_joints);

  // Gets joint command data using pre-sized Eigen vectors
  Eigen::VectorXd q_cmd(num_joints);
  Eigen::VectorXd qd_cmd(num_joints);
  Eigen::VectorXd tau_ff_cmd(num_joints);
  Eigen::VectorXd tau_cmd(num_joints);
  Eigen::VectorXd kp_cmd(num_joints);
  Eigen::VectorXd kd_cmd(num_joints);

  data_store_->joint_info.GetCommand(data::JointInfoType::kPosition, q_cmd);
  data_store_->joint_info.GetCommand(data::JointInfoType::kVelocity, qd_cmd);
  data_store_->joint_info.GetCommand(data::JointInfoType::kFeedForwardTorque, tau_ff_cmd);
  data_store_->joint_info.GetCommand(data::JointInfoType::kTorque, tau_cmd);
  data_store_->joint_info.GetCommand(data::JointInfoType::kStiffness, kp_cmd);
  data_store_->joint_info.GetCommand(data::JointInfoType::kDamping, kd_cmd);

  // Efficiently copies data using Eigen assignment to mapped vectors
  Eigen::Map<Eigen::VectorXd>(joint_command_feedback_msg_->position.data(), num_joints) = q_cmd;
  Eigen::Map<Eigen::VectorXd>(joint_command_feedback_msg_->velocity.data(), num_joints) = qd_cmd;
  Eigen::Map<Eigen::VectorXd>(joint_command_feedback_msg_->feed_forward_torque.data(), num_joints) = tau_ff_cmd;
  Eigen::Map<Eigen::VectorXd>(joint_command_feedback_msg_->torque.data(), num_joints) = tau_cmd;
  Eigen::Map<Eigen::VectorXd>(joint_command_feedback_msg_->stiffness.data(), num_joints) = kp_cmd;
  Eigen::Map<Eigen::VectorXd>(joint_command_feedback_msg_->damping.data(), num_joints) = kd_cmd;

  // Sets header and publishes message
  joint_command_feedback_msg_->header.stamp = this->now();
  joint_command_feedback_pub_->publish(*joint_command_feedback_msg_);
}

}  // namespace ros2
