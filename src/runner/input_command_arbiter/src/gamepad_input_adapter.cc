#include "input_command_arbiter/gamepad_input_adapter.h"

namespace runner {

GamepadInputAdapter::GamepadInputAdapter(std::string name, const std::shared_ptr<data::DataStore>& data_store)
    : BaseInputAdapter(name, data_store) {
  Init();
}

void GamepadInputAdapter::Init() {
  hardware_gamepad_publisher_ =
      data::VariantStore::GetInstance().CreatePublisher<data::GamepadInfo>("hardware/gamepad_info");
  ResetDriverState();
  CheckDeviceConnected();
}

void GamepadInputAdapter::Run() {
  // Refresh connection state and read the latest hardware sample.
  hardware_available_ = false;

  if (CheckDeviceConnected() == false) {
    // Publish a disconnected state immediately so downstream ROS2 consumers
    // observe the hardware loss without delay.
    hardware_input_.Reset();
    hardware_gamepad_publisher_.Publish(hardware_input_);
    return;
  }

  ReadHardwareInput();
  hardware_available_ = true;
  hardware_input_.hardware_connected = true;
  ClearLastError();
  hardware_gamepad_publisher_.Publish(hardware_input_);
  Log();
}

void GamepadInputAdapter::Process(data::GamepadInfo& input) {
  // Hardware input currently replaces the aggregated command as a whole.
  input = hardware_input_;
}

bool GamepadInputAdapter::IsActive() const {
  // This adapter participates only while valid hardware data is available.
  return hardware_available_;
}

void GamepadInputAdapter::ReadHardwareInput() {
  // Snapshot the latest driver state into the shared GamepadInfo structure.
  hardware_input_.combined_key_value = UpdateKeyValue();
  hardware_input_.LB = gamepad_driver_.logitech_data_res.LB;
  hardware_input_.RB = gamepad_driver_.logitech_data_res.RB;
  hardware_input_.A = gamepad_driver_.logitech_data_res.A;
  hardware_input_.B = gamepad_driver_.logitech_data_res.B;
  hardware_input_.X = gamepad_driver_.logitech_data_res.X;
  hardware_input_.Y = gamepad_driver_.logitech_data_res.Y;
  hardware_input_.BACK = gamepad_driver_.logitech_data_res.BACK;
  hardware_input_.START = gamepad_driver_.logitech_data_res.START;
  hardware_input_.CROSS_X = gamepad_driver_.logitech_data_res.CROSS_X;
  hardware_input_.CROSS_Y = gamepad_driver_.logitech_data_res.CROSS_Y;

  hardware_input_.LT = gamepad_driver_.logitech_data_res.LT;
  hardware_input_.RT = gamepad_driver_.logitech_data_res.RT;

  hardware_input_.LeftStick_X = gamepad_driver_.logitech_data_res.leftStickXAnalog;
  hardware_input_.LeftStick_Y = gamepad_driver_.logitech_data_res.leftStickYAnalog;
  hardware_input_.RightStick_X = gamepad_driver_.logitech_data_res.rightStickXAnalog;
  hardware_input_.RightStick_Y = gamepad_driver_.logitech_data_res.rightStickYAnalog;
}

void GamepadInputAdapter::ResetDriverState() {
  gamepad_driver_.Zeros();
  hardware_input_.Reset();

  driver_initialized_ = false;
  failed_count_ = 0;
  hardware_available_ = false;
}

bool GamepadInputAdapter::CheckDeviceConnected() {
  static constexpr uint32_t kDriverRetryIntervalCount = 200;

  if (!driver_initialized_) {
    failed_count_++;

    if (failed_count_ % kDriverRetryIntervalCount != 0) {
      return false;
    }

    if (gamepad_driver_.Init() != EXIT_SUCCESS) {
      failed_count_ = 0;
      SetLastError("Failed to initialize gamepad driver. Will retry later.");
      return false;
    }

    driver_initialized_ = true;
    failed_count_ = 0;
    LOG(INFO) << "Gamepad driver initialized successfully.";
  }

  if (gamepad_driver_.ListenInput() != EXIT_SUCCESS) {
    SetLastError("Failed to listen to gamepad input. Resetting driver.");
    ResetDriverState();
    return false;
  }

  ClearLastError();
  return true;
}

int GamepadInputAdapter::UpdateKeyValue() {
  int value = 0;

  if (gamepad_driver_.logitech_data_res.LB) {
    value |= gamepad_tool_.KeyStringToValue("LB");
  }
  if (gamepad_driver_.logitech_data_res.RB) {
    value |= gamepad_tool_.KeyStringToValue("RB");
  }
  if (gamepad_driver_.logitech_data_res.A) {
    value |= gamepad_tool_.KeyStringToValue("A");
  }
  if (gamepad_driver_.logitech_data_res.B) {
    value |= gamepad_tool_.KeyStringToValue("B");
  }
  if (gamepad_driver_.logitech_data_res.X) {
    value |= gamepad_tool_.KeyStringToValue("X");
  }
  if (gamepad_driver_.logitech_data_res.Y) {
    value |= gamepad_tool_.KeyStringToValue("Y");
  }
  if (gamepad_driver_.logitech_data_res.BACK) {
    value |= gamepad_tool_.KeyStringToValue("BACK");
  }
  if (gamepad_driver_.logitech_data_res.START) {
    value |= gamepad_tool_.KeyStringToValue("START");
  }
  if (gamepad_driver_.logitech_data_res.CROSS_X > 0) {
    value |= gamepad_tool_.KeyStringToValue("CROSS_X_UP");
  }
  if (gamepad_driver_.logitech_data_res.CROSS_X < 0) {
    value |= gamepad_tool_.KeyStringToValue("CROSS_X_DOWN");
  }
  if (gamepad_driver_.logitech_data_res.CROSS_Y > 0) {
    value |= gamepad_tool_.KeyStringToValue("CROSS_Y_LEFT");
  }
  if (gamepad_driver_.logitech_data_res.CROSS_Y < 0) {
    value |= gamepad_tool_.KeyStringToValue("CROSS_Y_RIGHT");
  }

  return value;
}

}  // namespace runner
