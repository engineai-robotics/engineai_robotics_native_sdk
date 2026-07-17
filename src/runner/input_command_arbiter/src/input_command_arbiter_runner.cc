#include "input_command_arbiter/input_command_arbiter_runner.h"
#include "input_command_arbiter/gamepad_input_adapter.h"
#include "input_command_arbiter/rc02_input_adapter.h"
#include "input_command_arbiter/virtual_gamepad_input_adapter.h"

#include <glog/logging.h>

namespace runner {

InputCommandArbiterRunner::InputCommandArbiterRunner(std::string_view name,
                                                     const std::shared_ptr<data::DataStore>& data_store)
    : BasicRunner(name, data_store) {
  RegisterHardwareSource("rc02", std::make_shared<Rc02InputAdapter>("rc02", data_store));
  RegisterHardwareSource("gamepad", std::make_shared<GamepadInputAdapter>("gamepad", data_store));
  RegisterOverrideSource("virtual_gamepad",
                         std::make_shared<VirtualGamepadInputAdapter>("virtual_gamepad", data_store));

  selected_hardware_idx_ = -1;
  for (int i = 0; i < static_cast<int>(hardware_sources_.size()); ++i) {
    if (!hardware_sources_[i]->Init()) {
      LOG(WARNING) << "InputCommandArbiterRunner: Init failed for hardware source '"
                   << hardware_sources_[i]->GetName() << "'.";
      continue;
    }
    LOG(INFO) << "InputCommandArbiterRunner: hardware source '" << hardware_sources_[i]->GetName()
              << "' Init succeeded at index " << i << ".";
    selected_hardware_idx_ = i;
    break;
  }

  for (auto& source : override_sources_) {
    if (!source->Init()) {
      LOG(WARNING) << "InputCommandArbiterRunner: Init failed for override source '" << source->GetName() << "'.";
    }
  }
}

void InputCommandArbiterRunner::Run() {
  data::GamepadInfo result;
  result.Reset();

  if (selected_hardware_idx_ < 0) {
    for (int i = 0; i < static_cast<int>(hardware_sources_.size()); ++i) {
      if (hardware_sources_[i]->Run() == InputAdapterStatus::NORMAL && selected_hardware_idx_ < 0) {
        selected_hardware_idx_ = i;
      }
      if (hardware_sources_[i]->IsActive()) {
        hardware_sources_[i]->Process(result);
        LOG(INFO) << "Hardware source locked: " << hardware_sources_[i]->GetName();
        break;
      }
    }
  } else {
    auto& selected = hardware_sources_[selected_hardware_idx_];
    static_cast<void>(selected->Run());
    if (selected->IsActive()) {
      selected->Process(result);
    }
  }

  for (auto& source : override_sources_) {
    static_cast<void>(source->Run());
    if (source->IsActive()) {
      source->Process(result);
    }
  }

  data_store_->gamepad_info.Set(result);
}

void InputCommandArbiterRunner::RegisterHardwareSource(const std::string& name,
                                                     std::shared_ptr<BaseInputAdapter> adapter) {
  hardware_sources_.emplace_back(std::move(adapter));
  LOG(INFO) << "Registered hardware source: " << name;
}

void InputCommandArbiterRunner::RegisterOverrideSource(const std::string& name,
                                                       std::shared_ptr<BaseInputAdapter> adapter) {
  override_sources_.emplace_back(std::move(adapter));
  LOG(INFO) << "Registered override source: " << name;
}

}  // namespace runner
