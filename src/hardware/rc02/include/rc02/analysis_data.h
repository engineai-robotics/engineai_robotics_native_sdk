#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace hardware {

// RC02 common frame protocol:
// header(1B, 0xAA) + len(1B) + control(NB) + crc(2B)
constexpr uint8_t MODE_HEADER = 0;
constexpr uint8_t MODE_LENGTH = 1;
constexpr uint8_t MODE_DATA = 2;
constexpr uint8_t MODE_CRC_L = 3;  // relative index: MODE_DATA + len
constexpr uint8_t MODE_CRC_H = 4;  // relative index: MODE_DATA + len + 1

constexpr uint8_t RC02_HEADER_VALUE = 0xAA;
constexpr size_t RC02_FRAME_FIXED_SIZE = 4;  // header(1) + len(1) + crc(2)
constexpr size_t RC02_CRC_SIZE = 2;

// Host -> device status TX: length = 0x01 + N (N=4 -> 0x05); control = 0x0C + uint8[4] (only [0],[1] used).
constexpr uint8_t RC02_STATUS_TX_CMD = 0x0C;
constexpr uint8_t RC02_STATUS_TX_DATA_N = 4;
constexpr uint8_t RC02_STATUS_TX_LEN = static_cast<uint8_t>(0x01 + RC02_STATUS_TX_DATA_N);
constexpr size_t RC02_STATUS_TX_CONTROL_SIZE = 1 + RC02_STATUS_TX_DATA_N;
constexpr size_t RC02_STATUS_TX_FRAME_SIZE = 2 + RC02_STATUS_TX_CONTROL_SIZE + 2;  // header + len + control + crc

// Host -> device init TX: length = 0x01 + N (N=50 -> 0x33); control = 0x0D + uint8[50]
// (offsets 0..11 carry cmd/product/protocol/motion/hardware/robot_rc02_hw; the trailing
// 39 bytes are reserved. For current debug flow they are initialized to 9).
constexpr uint8_t RC02_INIT_TX_CMD = 0x0D;
constexpr uint8_t RC02_INIT_TX_DATA_N = 50;
constexpr uint8_t RC02_INIT_TX_LEN = static_cast<uint8_t>(0x01 + RC02_INIT_TX_DATA_N);
constexpr size_t RC02_INIT_TX_CONTROL_SIZE = 1 + RC02_INIT_TX_DATA_N;
constexpr size_t RC02_INIT_TX_FRAME_SIZE = 2 + RC02_INIT_TX_CONTROL_SIZE + 2;

// Host -> device hardware version query: length = 0x01; data = 0x04.
constexpr uint8_t RC02_HW_VERSION_CMD = 0x04;
constexpr uint8_t RC02_HW_VERSION_REQ_LEN = 0x01;
constexpr size_t RC02_HW_VERSION_REQ_FRAME_SIZE = 5;  // header + len + cmd + crc(2)

// Device -> host hardware version response: length = 0x05; data = 0x04 + char[4].
constexpr uint8_t RC02_HW_VERSION_RSP_LEN = 0x05;
constexpr size_t RC02_HW_VERSION_RSP_FRAME_SIZE = 9;  // header + len + cmd+ver4 + crc(2)

// Device -> host init ACK response: length = 0x01; data = 0x0D.
constexpr uint8_t RC02_INIT_ACK_RSP_LEN = 0x01;
constexpr size_t RC02_INIT_ACK_RSP_FRAME_SIZE = 5;  // header + len + cmd + crc(2)

// Offsets within init control block (cmd + 50-byte tail), aligned with protocol table:
// 0:cmd, 1:product, 2..4:protocol(patch/minor/major), 5..7:motion(patch/minor/major),
// 8..10:hardware(patch/minor/major), 11:robot_rc02_hw.
constexpr uint8_t INIT_CONTROL_OFF_CMD = 0;
constexpr uint8_t INIT_CONTROL_OFF_PRODUCT = 1;
constexpr uint8_t INIT_CONTROL_OFF_PROTOCOL_PATCH = 2;
constexpr uint8_t INIT_CONTROL_OFF_PROTOCOL_MINOR = 3;
constexpr uint8_t INIT_CONTROL_OFF_PROTOCOL_MAJOR = 4;
constexpr uint8_t INIT_CONTROL_OFF_MOTION_PATCH = 5;
constexpr uint8_t INIT_CONTROL_OFF_MOTION_MINOR = 6;
constexpr uint8_t INIT_CONTROL_OFF_MOTION_MAJOR = 7;
constexpr uint8_t INIT_CONTROL_OFF_HARDWARE_PATCH = 8;
constexpr uint8_t INIT_CONTROL_OFF_HARDWARE_MINOR = 9;
constexpr uint8_t INIT_CONTROL_OFF_HARDWARE_MAJOR = 10;
constexpr uint8_t INIT_CONTROL_OFF_ROBOT_RC02_HW = 11;

// RX control block: ID(1, 0x0B) + DATA(20). Length byte = 0x15 (21).
constexpr uint8_t RC02_CONTROL_ID = 0x0B;
constexpr size_t RC02_EXPECTED_CONTROL_SIZE = 21;

// Byte offsets within the control block (data[MODE_DATA] ..), i.e. after header + length.
constexpr uint8_t CONTROL_OFF_ID = 0;
constexpr uint8_t CONTROL_OFF_HEARTBEAT = 1;
constexpr uint8_t CONTROL_OFF_L2 = 2;
constexpr uint8_t CONTROL_OFF_R2 = 3;
constexpr uint8_t CONTROL_OFF_LEFT_STICK_UD = 4;
constexpr uint8_t CONTROL_OFF_LEFT_STICK_LR = 5;
constexpr uint8_t CONTROL_OFF_RIGHT_STICK_UD = 6;
constexpr uint8_t CONTROL_OFF_RIGHT_STICK_LR = 7;
constexpr uint8_t CONTROL_OFF_BUTTONS_1 = 8;
constexpr uint8_t CONTROL_OFF_BUTTONS_2 = 9;
constexpr uint8_t CONTROL_OFF_BUTTONS_3 = 10;
constexpr uint8_t CONTROL_OFF_MOTION_COMMAND = 11;
constexpr uint8_t CONTROL_OFF_SKILL_COMMAND = 12;
constexpr uint8_t CONTROL_OFF_RESERVED = 13;  // 8 bytes res[8]

struct Rc02OriginData {
  uint8_t control_id = 0;
  uint8_t heartbeat = 0;

  uint8_t l2 = 0;  // [0,255]
  uint8_t r2 = 0;  // [0,255]

  int8_t left_stick_ud = 0;   // [-127,127]
  int8_t left_stick_lr = 0;   // [-127,127]
  int8_t right_stick_ud = 0;  // [-127,127]
  int8_t right_stick_lr = 0;  // [-127,127]

  uint8_t buttons_1 = 0;
  uint8_t buttons_2 = 0;
  uint8_t buttons_3 = 0;

  uint8_t motion_command = 0;
  uint8_t skill_command = 0;

  uint8_t reserved[8]{};
};

enum class Rc02ParseStatus {
  kOk,
  kNeedMoreData,
  kSkipOneByte,
};

uint16_t LIB_CRC_Checksum(const uint8_t* pData, size_t size);
bool HasEffectiveRc02Input(const uint8_t* data, size_t size);
Rc02ParseStatus ParseRc02Frame(const uint8_t* data, size_t size, Rc02OriginData* out, size_t* consumed_size);
Rc02ParseStatus ParseRc02HardwareVersionResponse(const uint8_t* data, size_t size, int8_t* out_version,
                                                 size_t* consumed_size);
Rc02ParseStatus ParseRc02InitAck(const uint8_t* data, size_t size, size_t* consumed_size);

std::array<uint8_t, RC02_STATUS_TX_FRAME_SIZE> BuildRc02MotionStatusFrame(uint8_t error_code, uint8_t current_motion);

/// 5 groups aligned with Rc02InitInfo: product / protocol(patch,minor,major) /
/// motion(patch,minor,major) / hardware(patch,minor,major) / robot_rc02_hw.
std::array<uint8_t, RC02_INIT_TX_FRAME_SIZE> BuildRc02InitFrame(int8_t product,
                                                                const std::array<int8_t, 3>& protocol_version,
                                                                const std::array<int8_t, 3>& motion_version,
                                                                const std::array<int8_t, 3>& hardware_version,
                                                                int8_t robot_rc02_hw);
std::array<uint8_t, RC02_HW_VERSION_REQ_FRAME_SIZE> BuildRc02HardwareVersionRequestFrame();

}  // namespace hardware
