#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "gamepad_info/gamepad_info.h"
#include "input_command_arbiter/base_input_adapter.h"
#include "rc02/rc02_driver.h"
#include "variant_store/variant_store.h"

namespace runner {

class Rc02InputAdapter : public BaseInputAdapter {
 public:
  // -------------------------------------------------------------------------
  // 生命周期
  // -------------------------------------------------------------------------
  explicit Rc02InputAdapter(std::string name, const std::shared_ptr<data::DataStore>& data_store);
  ~Rc02InputAdapter() = default;

  bool Init() override;
  InputAdapterStatus Run() override;

  // -------------------------------------------------------------------------
  // BaseInputAdapter 接口重写
  // -------------------------------------------------------------------------
  void Process(data::GamepadInfo& input) override;
  bool IsActive() const override;

 private:
  // -------------------------------------------------------------------------
  // 连接管理
  // -------------------------------------------------------------------------
  bool CheckRc02Connected();
  bool TryReconnectRc02();
  bool Rc02Init();

  // -------------------------------------------------------------------------
  // 输入解析
  // -------------------------------------------------------------------------
  // Parses normal gamepad fields such as buttons, triggers, sticks and d-pad.
  void GetKeyInputFromRc02Raw(const hardware::Rc02InputData& raw);

  // -------------------------------------------------------------------------
  // 成员变量
  // -------------------------------------------------------------------------

  // 硬件驱动与连接状态
  hardware::RC02Driver rc02_;
  bool rc02_data_available_ = false;
  bool prev_rc02_data_available_ = false;  // 上一帧是否有有效输入，用于跨帧检测数据中断
  uint32_t failed_count_ = 0;
  bool driver_initialized_ = false;

  // 输入数据
  data::GamepadInfo rc02_input_;
  data::GamepadTool gamepad_tool_;
  data::Publisher<data::GamepadInfo> hardware_rc02_publisher_;
};

}  // namespace runner
