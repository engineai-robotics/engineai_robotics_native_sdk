#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "basic/basic_runner.h"
#include "basic/runner_registry.h"
#include "input_command_arbiter/base_input_adapter.h"

namespace runner {

// Collects inputs from multiple sources and resolves the final command
// by applying adapters in ascending priority order.
class InputCommandArbiterRunner : public BasicRunner {
 public:
  InputCommandArbiterRunner(std::string_view name, const std::shared_ptr<data::DataStore>& data_store);
  ~InputCommandArbiterRunner() = default;

  void Run() override;

 private:
  // Hardware-exclusive sources (rc02, gamepad), highest priority first.
  void RegisterHardwareSource(const std::string& name, std::shared_ptr<BaseInputAdapter> adapter);

  // Override sources (virtual_gamepad), applied on top of the selected hardware source.
  void RegisterOverrideSource(const std::string& name, std::shared_ptr<BaseInputAdapter> adapter);

  std::vector<std::shared_ptr<BaseInputAdapter>> hardware_sources_;
  std::vector<std::shared_ptr<BaseInputAdapter>> override_sources_;
  int selected_hardware_idx_{-1};
};

}  // namespace runner

REGISTER_RUNNER(InputCommandArbiterRunner, "input_command_arbiter_runner", kResident)
