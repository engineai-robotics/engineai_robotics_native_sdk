#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "rc02/analysis_data.h"
#include "serial/serial.h"

namespace hardware {

struct Rc02InputData {
  int LeftStickButton = 0, RightStickButton = 0, DigitalR2 = 0, L1 = 0, R1 = 0, DigitalL2 = 0;
  int A = 0, B = 0, X = 0, Y = 0, CROSS_UP = 0, CROSS_DOWN = 0, CROSS_LEFT = 0, CROSS_RIGHT = 0;
  int START = 0, SELECT = 0, BACK = 0;

  double L2 = 0.0, R2 = 0.0;
  double LeftStick_ud = 0.0, LeftStick_lr = 0.0;
  double RightStick_ud = 0.0, RightStick_lr = 0.0;

  // Mirrored from Rc02OriginData; values are produced from uint8_t so they
  // are always in [0, 255].
  int MotionCommand = 0;  // <- Rc02OriginData::motion_command
  int SkillCommand = 0;   // <- Rc02OriginData::skill_command
  int heartbeat = 0;      // <- Rc02OriginData::heartbeat
};

struct Rc02InitInfo {
  int8_t Product;
  int8_t Protocol_patch, Protocol_minor, Protocol_major;
  int8_t Motion_patch, Motion_minor, Motion_major;
  int8_t Hardware_patch, Hardware_minor, Hardware_major;
  int8_t Robot_rc02_hw;
};

class RC02Driver {
 public:
  // -------------------------------------------------------------------------
  // 生命周期
  // -------------------------------------------------------------------------
  explicit RC02Driver(std::string dev = "/dev/ttyJS", int baud = 2000000);
  ~RC02Driver();

  bool Init();
  void Close() noexcept;

  // -------------------------------------------------------------------------
  // 周期性收发：发送当前状态帧，然后读取并解析遥控指令
  // -------------------------------------------------------------------------
  bool SendMotionAndACK(uint8_t error_code, uint8_t current_motion);

  // -------------------------------------------------------------------------
  // 发送帧
  // -------------------------------------------------------------------------

  /// 发送状态帧：0xAA | len 0x05 | 0x0C | error_code | current_motion | 0 | 0 | CRC16
  bool SendMotionStatus(uint8_t error_code, uint8_t current_motion);

  /// 构建初始化信息（使用默认值），robot_model 写入 init_info.Product
  Rc02InitInfo BuildInitData(int8_t robot_model, std::string protocol_version = "1.0.1") const;

  /// 发送初始化帧：0xAA | len 0x33 | 0x0D | product | protocol | motion | hardware | CRC16
  bool SendInitData(const Rc02InitInfo& init_info);

  // -------------------------------------------------------------------------
  // 硬件版本
  // -------------------------------------------------------------------------

  /// 发送版本查询帧，成功时更新缓存的硬件版本
  bool GetRc02HardwareVersion(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

  /// 返回最近一次成功获取的硬件版本
  int8_t GetCachedRc02HardwareVersion() const { return last_hw_version_; }

  // -------------------------------------------------------------------------
  // 数据读取与连接状态
  // -------------------------------------------------------------------------
  const Rc02InputData& GetInputData();
  const Rc02OriginData& GetOriginData() const { return origin_; }

  bool IsConnected() const;

 private:
  // -------------------------------------------------------------------------
  // 内部实现
  // -------------------------------------------------------------------------
  void initSerial();
  void ParseInputData();
  void ResetConnectionState() noexcept;

  static bool IsBitSet(uint8_t byte, int bit_pos) { return (byte & (1u << bit_pos)) != 0; }
  static double NormalizeTrigger(uint8_t value);
  static double NormalizeStick(int8_t value);

  // -------------------------------------------------------------------------
  // 成员变量
  // -------------------------------------------------------------------------

  // 串口配置
  std::string dev_path_;
  int baudrate_ = 2000000;
  serial::Serial serial_;
  bool configured_ = false;

  // 解析后的数据
  Rc02OriginData origin_{};
  Rc02InputData input_{};

  // 硬件版本缓存
  int8_t last_hw_version_ = 5;

  // 接收缓冲与连接时间戳
  std::vector<uint8_t> rx_buffer_{};
  std::chrono::steady_clock::time_point last_good_rx_{};
  bool has_good_rx_ = false;
};

}  // namespace hardware
