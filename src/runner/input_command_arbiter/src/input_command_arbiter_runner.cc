#include "input_command_arbiter/input_command_arbiter_runner.h"
#include "input_command_arbiter/gamepad_input_adapter.h"

namespace runner {

InputCommandArbiterRunner::InputCommandArbiterRunner(std::string_view name,
                                                     const std::shared_ptr<data::DataStore>& data_store)
    : BasicRunner(name, data_store) {
  // Register input sources from low to high priority.
  RegisterInputSource("gamepad", std::make_shared<GamepadInputAdapter>("gamepad", data_store));

  // Initialize all registered sources once during runner construction.
  for (auto& source : input_sources_) {
    source->Init();
  }
}

void InputCommandArbiterRunner::Run() {
  // Start from an empty command and let active sources update it in order.
  data::GamepadInfo result;
  result.Reset();

  // Later sources can override earlier ones because they have higher priority.
  for (auto& source : input_sources_) {
    source->Run();
    if (source->IsActive()) {
      source->Process(result);
    }
  }

  // Publish the final arbitrated input to the shared data store.
  data_store_->gamepad_info.Set(result);
}

void InputCommandArbiterRunner::RegisterInputSource(const std::string& name,
                                                    std::shared_ptr<BaseInputAdapter> adapter) {
  input_sources_.emplace_back(std::move(adapter));
  LOG(INFO) << "Registered input source: " << name;
}

}  // namespace runner
