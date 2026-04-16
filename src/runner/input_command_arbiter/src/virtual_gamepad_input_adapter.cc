#include "input_command_arbiter/virtual_gamepad_input_adapter.h"

#include "lcm_param/lcm_param.h"

namespace runner {

VirtualGamepadInputAdapter::VirtualGamepadInputAdapter(std::string name,
                                                       const std::shared_ptr<data::DataStore>& data_store)
    : BaseInputAdapter(std::move(name), data_store) {
  Init();
}

void VirtualGamepadInputAdapter::Init() {
  virtual_gamepad_publisher_ =
      data::VariantStore::GetInstance().CreatePublisher<data::GamepadInfo>("virtual/gamepad_info");
  ResetVirtualInput();
  last_input_time_ = std::chrono::steady_clock::now();
  last_task_state_publish_time_ = last_input_time_;

  InitConnection();
  InitSubscription();
}

void VirtualGamepadInputAdapter::Run() {
  if (!EnsureConnection()) {
    return;
  }

  int handle_result = 0;
  do {
    handle_result = lcm_->handleTimeout(0);
  } while (handle_result > 0);

  if (handle_result < 0) {
    SetLastError("Virtual gamepad LCM handleTimeout failed. Resetting adapter connection.");
    lcm_.reset();
    subscription_initialized_ = false;
    ResetVirtualInput();
    virtual_gamepad_publisher_.Publish(virtual_input_);
    return;
  }

  ClearLastError();
  UpdateAvailability();

  auto now = std::chrono::steady_clock::now();
  if (now - last_task_state_publish_time_ >= kTaskStatePublishPeriod) {
    PublishTaskState();
    last_task_state_publish_time_ = now;
  }

  Log();
}

void VirtualGamepadInputAdapter::Process(data::GamepadInfo& input) {
  input = virtual_input_;
}

bool VirtualGamepadInputAdapter::IsActive() const {
  return virtual_available_;
}

void VirtualGamepadInputAdapter::Log() const {
  if (!virtual_available_) {
    return;
  }

  VLOG(1) << "Virtual gamepad active, combined_key_value=" << virtual_input_.combined_key_value
          << ", sticks=(" << virtual_input_.LeftStick_X << ", " << virtual_input_.LeftStick_Y << ", "
          << virtual_input_.RightStick_X << ", " << virtual_input_.RightStick_Y << ")";
}

bool VirtualGamepadInputAdapter::EnsureConnection() {
  if (lcm_ && subscription_initialized_) {
    return true;
  }

  reconnect_attempt_count_++;
  if (reconnect_attempt_count_ != 1 && reconnect_attempt_count_ % kReconnectIntervalCount != 0) {
    return false;
  }

  InitConnection();
  InitSubscription();
  return lcm_ && subscription_initialized_;
}

void VirtualGamepadInputAdapter::InitConnection() {
  param_ = data::ParamManager::create<data::LcmParam>();
  const int ttl = param_->multicast ? param_->ttl : 0;
  const std::string url = "udpm://" + param_->ip_port + "?ttl=" + std::to_string(ttl);

  auto lcm = std::make_shared<lcm::LCM>(url);
  if (!lcm->good()) {
    SetLastError("Failed to initialize virtual gamepad LCM on URL: " + url);
    lcm_.reset();
    subscription_initialized_ = false;
    return;
  }

  ClearLastError();
  LOG(INFO) << "Virtual gamepad adapter LCM URL: " << url;
  lcm_ = std::move(lcm);
}

void VirtualGamepadInputAdapter::InitSubscription() {
  if (!lcm_ || subscription_initialized_) {
    return;
  }

  lcm_->subscribe("virtual_gamepad/gamepad_keys", &VirtualGamepadInputAdapter::HandleUpdateGamepadKeys, this);
  subscription_initialized_ = true;
}

void VirtualGamepadInputAdapter::ResetVirtualInput() {
  virtual_input_.Reset();
  virtual_available_ = false;
}

void VirtualGamepadInputAdapter::PublishTaskState() {
  if (!lcm_) {
    return;
  }

  const auto current_task_name = data_store_->current_motion_task_name.Get();
  if (!current_task_name) {
    return;
  }

  data::TaskState task_state;
  task_state.current_motion_task_name = *current_task_name;
  lcm_->publish("task_state", &task_state);
}

void VirtualGamepadInputAdapter::UpdateAvailability() {
  if (!virtual_available_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_input_time_ <= kInputTimeout) {
    return;
  }

  LOG(INFO) << "Virtual gamepad input timed out. Releasing adapter control.";
  ResetVirtualInput();
  virtual_gamepad_publisher_.Publish(virtual_input_);
}

int VirtualGamepadInputAdapter::UpdateKeyValue(const data::GamepadKeys& msg) const {
  int value = 0;

  for (int i = 0; i < data::GamepadTool::kKeyString.size(); ++i) {
    if (msg.digital_states[i]) {
      value |= (1 << i);
    }
  }

  return value;
}

void VirtualGamepadInputAdapter::HandleUpdateGamepadKeys(const lcm::ReceiveBuffer*,
                                                         const std::string&,
                                                         const data::GamepadKeys* msg) {
  const bool was_available = virtual_available_;

  virtual_input_.LB = msg->digital_states[0];
  virtual_input_.RB = msg->digital_states[1];
  virtual_input_.A = msg->digital_states[2];
  virtual_input_.B = msg->digital_states[3];
  virtual_input_.X = msg->digital_states[4];
  virtual_input_.Y = msg->digital_states[5];
  virtual_input_.BACK = msg->digital_states[6];
  virtual_input_.START = msg->digital_states[7];

  virtual_input_.CROSS_X = 0;
  if (msg->digital_states[8]) virtual_input_.CROSS_X = 1;
  if (msg->digital_states[9]) virtual_input_.CROSS_X = -1;

  virtual_input_.CROSS_Y = 0;
  if (msg->digital_states[10]) virtual_input_.CROSS_Y = 1;
  if (msg->digital_states[11]) virtual_input_.CROSS_Y = -1;

  virtual_input_.LT = msg->analog_states[0];
  virtual_input_.RT = msg->analog_states[1];
  virtual_input_.LeftStick_X = msg->analog_states[2];
  virtual_input_.LeftStick_Y = -msg->analog_states[3];
  virtual_input_.RightStick_X = msg->analog_states[4];
  virtual_input_.RightStick_Y = -msg->analog_states[5];

  virtual_input_.combined_key_value = UpdateKeyValue(*msg);
  virtual_input_.hardware_connected = false;

  last_input_time_ = std::chrono::steady_clock::now();
  virtual_available_ = true;
  if (!was_available) {
    LOG(INFO) << "Virtual gamepad input connected.";
  }
  virtual_gamepad_publisher_.Publish(virtual_input_);
}

}  // namespace runner
