#include "input_command_arbiter/input_command_arbiter_runner.h"
#include "input_command_arbiter/gamepad_input_adapter.h"
#include "input_command_arbiter/virtual_gamepad_input_adapter.h"

namespace runner {

InputCommandArbiterRunner::InputCommandArbiterRunner(std::string_view name,
                                                     const std::shared_ptr<data::DataStore>& data_store)
    : BasicRunner(name, data_store) {
  // Register input sources from low to high priority.
  RegisterInputSource("gamepad", std::make_shared<GamepadInputAdapter>("gamepad", data_store));
  RegisterInputSource("virtual_gamepad", std::make_shared<VirtualGamepadInputAdapter>("virtual_gamepad", data_store));
}

void InputCommandArbiterRunner::Run() {
  // Start from an empty command and let active sources update it in order.
  data::GamepadInfo result;
  result.Reset();
  bool has_active_source = false;
  std::string selected_error;

  // Later sources can override earlier ones because they have higher priority.
  for (auto& source : input_sources_) {
    source->Run();
    if (source->IsActive()) {
      has_active_source = true;
      source->Process(result);
      continue;
    }

    if (selected_error.empty()) {
      const auto& error = source->GetLastError();
      if (!error.empty()) {
        selected_error = source->GetName() + ": " + error;
      }
    }
  }

  if (has_active_source || selected_error.empty()) {
    last_reported_error_.clear();
  } else if (last_reported_error_ != selected_error) {
    LOG(ERROR) << selected_error;
    last_reported_error_ = selected_error;
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
