#include "input_command_arbiter/rc02_input_adapter.h"

#include <glog/logging.h>

namespace runner {

namespace {

constexpr double kRc02InputRetainDuration = 0.3;

}  // namespace

// =============================================================================
// 生命周期
// =============================================================================

Rc02InputAdapter::Rc02InputAdapter(std::string name, const std::shared_ptr<data::DataStore>& data_store)
    : BaseInputAdapter(name, data_store) {}

bool Rc02InputAdapter::Init() {
  hardware_rc02_publisher_ =
      data::VariantStore::GetInstance().CreatePublisher<data::GamepadInfo>("hardware/gamepad_info");
  failed_count_ = 0;
  driver_initialized_ = false;
  rc02_data_available_ = false;
  rc02_input_.Reset();

  // Align with CheckRc02Connected's retry modulus so the first Init call actually attempts serial init.
  static constexpr uint32_t kDriverRetryIntervalCount = 200;
  failed_count_ = kDriverRetryIntervalCount - 1;
  if (!TryReconnectRc02()) return false;

  LOG(INFO) << "Rc02InputAdapter initialized";
  return true;
}

InputAdapterStatus Rc02InputAdapter::Run() {
  rc02_data_available_ = false;
  if (!CheckRc02Connected()) {
    driver_initialized_ = false;
    if (!TryReconnectRc02()) {
      rc02_input_.Reset();
      hardware_rc02_publisher_.Publish(rc02_input_);
      return InputAdapterStatus::LOST;
    }
  }

  const bool was_retaining = prev_rc02_data_available_;
  const bool got_valid_rc02_input = rc02_.SendMotionAndACK(0, 0);
  if (got_valid_rc02_input) {
    const hardware::Rc02InputData raw_input = rc02_.GetInputData();
    rc02_input_.Reset();
    GetKeyInputFromRc02Raw(raw_input);
    RetainInputCommand(kRc02InputRetainDuration);
    rc02_data_available_ = true;
  } else if (IsRetaining()) {
    rc02_data_available_ = true;
  } else if (was_retaining) {
    LOG(WARNING) << "Rc02InputAdapter: rc02 data reception timed out (no new data within retain duration "
                 << kRc02InputRetainDuration << "s).";
  }
  prev_rc02_data_available_ = rc02_data_available_;

  hardware_rc02_publisher_.Publish(rc02_input_);
  Log();
  return InputAdapterStatus::NORMAL;
}

void Rc02InputAdapter::Process(data::GamepadInfo& input) { input = rc02_input_; }

bool Rc02InputAdapter::IsActive() const { return rc02_data_available_; }

// =============================================================================
// 连接管理
// =============================================================================

bool Rc02InputAdapter::CheckRc02Connected() { return driver_initialized_ && rc02_.IsConnected(); }

bool Rc02InputAdapter::Rc02Init() {
  if (!rc02_.Init()) return false;
  if (!rc02_.GetRc02HardwareVersion()) return false;
  return true;
}

bool Rc02InputAdapter::TryReconnectRc02() {
  static constexpr uint32_t kDriverRetryIntervalCount = 200;

  failed_count_++;
  if (failed_count_ % kDriverRetryIntervalCount != 0) return false;

  if (!Rc02Init()) return false;

  driver_initialized_ = true;
  failed_count_ = 0;
  LOG(INFO) << "Rc02InputAdapter: RC02Driver initialized successfully.";
  return true;
}

// =============================================================================
// 输入解析
// =============================================================================

void Rc02InputAdapter::GetKeyInputFromRc02Raw(const hardware::Rc02InputData& raw) {
  rc02_input_.hardware_connected = true;

  // Map RC02 input to GamepadInfo (SELECT is intentionally ignored).
  rc02_input_.LB = raw.L1;
  rc02_input_.RB = raw.R1;
  rc02_input_.A = raw.A;
  rc02_input_.B = raw.B;
  rc02_input_.X = raw.X;
  rc02_input_.Y = raw.Y;
  rc02_input_.BACK = raw.BACK;
  rc02_input_.START = raw.START;
  rc02_input_.LT = raw.L2;
  rc02_input_.RT = raw.R2;
  rc02_input_.LeftStick_X = raw.LeftStick_ud;
  rc02_input_.LeftStick_Y = raw.LeftStick_lr;
  rc02_input_.RightStick_X = raw.RightStick_ud;
  rc02_input_.RightStick_Y = raw.RightStick_lr;

  if (raw.CROSS_UP)
    rc02_input_.CROSS_X = 1;
  else if (raw.CROSS_DOWN)
    rc02_input_.CROSS_X = -1;
  else
    rc02_input_.CROSS_X = 0;

  if (raw.CROSS_LEFT)
    rc02_input_.CROSS_Y = 1;
  else if (raw.CROSS_RIGHT)
    rc02_input_.CROSS_Y = -1;
  else
    rc02_input_.CROSS_Y = 0;

  int key_value = 0;
  if (raw.L1) key_value |= gamepad_tool_.KeyStringToValue("LB");
  if (raw.R1) key_value |= gamepad_tool_.KeyStringToValue("RB");
  if (raw.A) key_value |= gamepad_tool_.KeyStringToValue("A");
  if (raw.B) key_value |= gamepad_tool_.KeyStringToValue("B");
  if (raw.X) key_value |= gamepad_tool_.KeyStringToValue("X");
  if (raw.Y) key_value |= gamepad_tool_.KeyStringToValue("Y");
  if (raw.BACK) key_value |= gamepad_tool_.KeyStringToValue("BACK");
  if (raw.START) key_value |= gamepad_tool_.KeyStringToValue("START");
  if (raw.CROSS_UP) key_value |= gamepad_tool_.KeyStringToValue("CROSS_X_UP");
  if (raw.CROSS_DOWN) key_value |= gamepad_tool_.KeyStringToValue("CROSS_X_DOWN");
  if (raw.CROSS_LEFT) key_value |= gamepad_tool_.KeyStringToValue("CROSS_Y_LEFT");
  if (raw.CROSS_RIGHT) key_value |= gamepad_tool_.KeyStringToValue("CROSS_Y_RIGHT");
  rc02_input_.combined_key_value = key_value;
}

}  // namespace runner
