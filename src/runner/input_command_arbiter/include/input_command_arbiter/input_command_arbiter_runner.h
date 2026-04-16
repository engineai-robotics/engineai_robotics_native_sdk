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
  void RegisterInputSource(const std::string& name, std::shared_ptr<BaseInputAdapter> adapter);

  // Input sources ordered from low to high priority to form the chain.
  std::vector<std::shared_ptr<BaseInputAdapter>> input_sources_;
  std::string last_reported_error_;
};

}  // namespace runner

REGISTER_RUNNER(InputCommandArbiterRunner, "input_command_arbiter_runner", kResident)
