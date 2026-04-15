#include "input_command_arbiter/base_input_adapter.h"

namespace runner {

void BaseInputAdapter::Log() const {
  // Keep adapter logging lightweight; detailed state belongs to subclasses.
  VLOG(1) << name_ << " adapter status";
}

bool BaseInputAdapter::IsRetaining() const {
  // Fast path when retention is not enabled.
  if (!is_retaining_) {
    return false;
  }

  // Retention expires after the configured duration.
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration<double>(now - retain_start_time_).count();
  return elapsed < retain_duration_;
}

void BaseInputAdapter::StopRetaining() {
  // Clear any active retention window immediately.
  is_retaining_ = false;
  retain_duration_ = 0.0;
}

bool BaseInputAdapter::RetainInputCommand(double duration) {
  // Enable temporary command retention for the requested duration.
  if (duration <= 0.0) {
    return false;
  }

  is_retaining_ = true;
  retain_start_time_ = std::chrono::steady_clock::now();
  retain_duration_ = duration;
  return true;
}

}  // namespace runner
