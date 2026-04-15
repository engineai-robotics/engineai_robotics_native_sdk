#pragma once

#include "gamepad/gamepad_driver.h"
#include "input_command_arbiter/base_input_adapter.h"
#include "variant_store/variant_store.h"

namespace runner {

// Adapter that converts raw hardware gamepad state into the shared input format.
class GamepadInputAdapter : public BaseInputAdapter {
 public:
  explicit GamepadInputAdapter(std::string name, const std::shared_ptr<data::DataStore>& data_store);
  ~GamepadInputAdapter() = default;

  void Init() override;
  void Run() override;

  // Chain-of-responsibility hooks.
  void Process(data::GamepadInfo& input) override;
  bool IsActive() const override;

 private:
  void ResetDriverState();
  bool CheckDeviceConnected();
  void ReadHardwareInput();

  int UpdateKeyValue();

  uint32_t failed_count_ = 0;
  bool driver_initialized_ = false;
  bool hardware_available_ = false;
  data::GamepadInfo hardware_input_;

  data::GamepadTool gamepad_tool_;
  hardware::LogitechGamepadDriver gamepad_driver_;
  data::Publisher<data::GamepadInfo> hardware_gamepad_publisher_;
};

}  // namespace runner
