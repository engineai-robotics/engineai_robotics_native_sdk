#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>

#include "input_command_arbiter/base_input_adapter.h"
#include "lcm_data/GamepadKeys.hpp"
#include "lcm_data/TaskState.hpp"
#include "variant_store/variant_store.h"

namespace data {
class LcmParam;
}

namespace runner {

// Adapter that converts LCM virtual gamepad messages into the shared input format.
class VirtualGamepadInputAdapter : public BaseInputAdapter {
 public:
  explicit VirtualGamepadInputAdapter(std::string name, const std::shared_ptr<data::DataStore>& data_store);
  ~VirtualGamepadInputAdapter() = default;

  void Init() override;
  void Run() override;
  void Process(data::GamepadInfo& input) override;
  bool IsActive() const override;
  void Log() const override;

 private:
  bool EnsureConnection();
  void InitConnection();
  void InitSubscription();
  void ResetVirtualInput();
  void PublishTaskState();
  void UpdateAvailability();
  int UpdateKeyValue(const data::GamepadKeys& msg) const;

  void HandleUpdateGamepadKeys(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                               const data::GamepadKeys* msg);

  std::shared_ptr<data::LcmParam> param_;
  std::shared_ptr<lcm::LCM> lcm_;

  data::GamepadInfo virtual_input_;
  data::Publisher<data::GamepadInfo> virtual_gamepad_publisher_;

  bool subscription_initialized_ = false;
  bool virtual_available_ = false;
  uint32_t reconnect_attempt_count_ = 0;

  std::chrono::steady_clock::time_point last_input_time_;
  std::chrono::steady_clock::time_point last_task_state_publish_time_;

  static constexpr uint32_t kReconnectIntervalCount = 200;
  static constexpr std::chrono::milliseconds kInputTimeout{200};
  static constexpr std::chrono::milliseconds kTaskStatePublishPeriod{20};
};

}  // namespace runner
