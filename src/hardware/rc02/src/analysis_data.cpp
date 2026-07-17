#include "rc02/analysis_data.h"

#include <algorithm>
#include <sstream>
#include <string>

#include <glog/logging.h>

namespace hardware {

namespace {

constexpr uint8_t kInactiveButtonByteValue = 0xFFu;
constexpr size_t kMinimumEffectiveInputFrameSize = MODE_DATA + CONTROL_OFF_SKILL_COMMAND + 1;

bool HasAnyNonZeroControlValue(const uint8_t* control, uint8_t begin_offset, uint8_t end_offset) {
  return std::any_of(control + begin_offset, control + end_offset + 1, [](uint8_t value) { return value != 0; });
}

bool HasTriggerOrStickInput(const uint8_t* control) {
  return HasAnyNonZeroControlValue(control, CONTROL_OFF_L2, CONTROL_OFF_RIGHT_STICK_LR);
}

bool HasButtonInput(const uint8_t* control) {
  return control[CONTROL_OFF_BUTTONS_1] != kInactiveButtonByteValue ||
         control[CONTROL_OFF_BUTTONS_2] != kInactiveButtonByteValue;
}

bool HasMotionOrSkillCommand(const uint8_t* control) {
  return HasAnyNonZeroControlValue(control, CONTROL_OFF_MOTION_COMMAND, CONTROL_OFF_SKILL_COMMAND);
}

}  // namespace

uint16_t LIB_CRC_Checksum(const uint8_t* pData, size_t size) {
  if (pData == nullptr || size == 0) {
    return 0xFFFF;
  }

  uint16_t res = 0;
  for (size_t i = 0; i < size; ++i) {
    res = static_cast<uint16_t>(res + pData[i]);
  }
  res = static_cast<uint16_t>(0xFFFFu - res);
  return res;
}

bool HasEffectiveRc02Input(const uint8_t* data, size_t size) {
  if (data == nullptr || size < kMinimumEffectiveInputFrameSize) {
    return false;
  }

  const uint8_t* control = data + MODE_DATA;
  return HasTriggerOrStickInput(control) || HasButtonInput(control) || HasMotionOrSkillCommand(control);
}

Rc02ParseStatus ParseRc02Frame(const uint8_t* data, size_t size, Rc02OriginData* out, size_t* consumed_size) {
  if (out == nullptr || consumed_size == nullptr || data == nullptr || size == 0) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  if (data[MODE_HEADER] != RC02_HEADER_VALUE) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  if (size < MODE_DATA) {
    return Rc02ParseStatus::kNeedMoreData;
  }

  const uint8_t control_len = data[MODE_LENGTH];
  if (control_len != RC02_EXPECTED_CONTROL_SIZE) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  const size_t frame_size = RC02_FRAME_FIXED_SIZE + static_cast<size_t>(control_len);
  if (size < frame_size) {
    return Rc02ParseStatus::kNeedMoreData;
  }

  const uint8_t* control = data + MODE_DATA;
  if (control[CONTROL_OFF_ID] != RC02_CONTROL_ID) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  const uint8_t crc_l = data[MODE_DATA + control_len];
  const uint8_t crc_h = data[MODE_DATA + control_len + 1];
  const uint16_t received_crc = static_cast<uint16_t>(crc_l | (static_cast<uint16_t>(crc_h) << 8));
  // CRC input stream includes: len_byte + control_bytes.
  // Pass pointer starting from the length byte position within the original frame storage.
  const uint16_t expected_crc = LIB_CRC_Checksum(data + MODE_LENGTH, static_cast<size_t>(1u + control_len));
  if (received_crc != expected_crc) {
    LOG_EVERY_N(WARNING, 200) << "ParseRc02Frame: CRC mismatch, received=0x" << std::hex << received_crc
                              << " expected=0x" << expected_crc << std::dec;
    return Rc02ParseStatus::kSkipOneByte;
  }

  out->control_id = control[CONTROL_OFF_ID];
  out->heartbeat = control[CONTROL_OFF_HEARTBEAT];
  out->l2 = control[CONTROL_OFF_L2];
  out->r2 = control[CONTROL_OFF_R2];
  out->left_stick_ud = static_cast<int8_t>(control[CONTROL_OFF_LEFT_STICK_UD]);
  out->left_stick_lr = static_cast<int8_t>(control[CONTROL_OFF_LEFT_STICK_LR]);
  out->right_stick_ud = static_cast<int8_t>(control[CONTROL_OFF_RIGHT_STICK_UD]);
  out->right_stick_lr = static_cast<int8_t>(control[CONTROL_OFF_RIGHT_STICK_LR]);
  out->buttons_1 = control[CONTROL_OFF_BUTTONS_1];
  out->buttons_2 = control[CONTROL_OFF_BUTTONS_2];
  out->buttons_3 = control[CONTROL_OFF_BUTTONS_3];
  out->motion_command = control[CONTROL_OFF_MOTION_COMMAND];
  out->skill_command = control[CONTROL_OFF_SKILL_COMMAND];
  for (size_t i = 0; i < 8; ++i) {
    out->reserved[i] = control[CONTROL_OFF_RESERVED + i];
  }

  *consumed_size = frame_size;
  return Rc02ParseStatus::kOk;
}

std::array<uint8_t, RC02_STATUS_TX_FRAME_SIZE> BuildRc02MotionStatusFrame(uint8_t error_code, uint8_t current_motion) {
  std::array<uint8_t, RC02_STATUS_TX_CONTROL_SIZE> control{};
  control[0] = RC02_STATUS_TX_CMD;
  control[1] = error_code;
  control[2] = current_motion;
  control[3] = 0;
  control[4] = 0;

  // CRC input stream includes: len_byte + control_bytes.
  std::array<uint8_t, 1 + RC02_STATUS_TX_CONTROL_SIZE> crc_input{};
  crc_input[0] = RC02_STATUS_TX_LEN;
  for (size_t i = 0; i < control.size(); ++i) {
    crc_input[1 + i] = control[i];
  }
  const uint16_t crc = LIB_CRC_Checksum(crc_input.data(), crc_input.size());

  std::array<uint8_t, RC02_STATUS_TX_FRAME_SIZE> frame{
      RC02_HEADER_VALUE,
      RC02_STATUS_TX_LEN,
      control[0],
      control[1],
      control[2],
      control[3],
      control[4],
      static_cast<uint8_t>(crc & 0xFFu),
      static_cast<uint8_t>((crc >> 8) & 0xFFu),
  };

  return frame;
}

std::array<uint8_t, RC02_INIT_TX_FRAME_SIZE> BuildRc02InitFrame(const int8_t product,
                                                                const std::array<int8_t, 3>& protocol_version,
                                                                const std::array<int8_t, 3>& motion_version,
                                                                const std::array<int8_t, 3>& hardware_version,
                                                                const int8_t robot_rc02_hw) {
  std::array<uint8_t, RC02_INIT_TX_CONTROL_SIZE> control{};
  // control.fill(9);  // Debug default: initialize all control bytes to 9 first.
  control[INIT_CONTROL_OFF_CMD] = RC02_INIT_TX_CMD;
  control[INIT_CONTROL_OFF_PRODUCT] = static_cast<uint8_t>(product);
  control[INIT_CONTROL_OFF_PROTOCOL_PATCH] = static_cast<uint8_t>(protocol_version[0]);
  control[INIT_CONTROL_OFF_PROTOCOL_MINOR] = static_cast<uint8_t>(protocol_version[1]);
  control[INIT_CONTROL_OFF_PROTOCOL_MAJOR] = static_cast<uint8_t>(protocol_version[2]);
  control[INIT_CONTROL_OFF_MOTION_PATCH] = static_cast<uint8_t>(motion_version[0]);
  control[INIT_CONTROL_OFF_MOTION_MINOR] = static_cast<uint8_t>(motion_version[1]);
  control[INIT_CONTROL_OFF_MOTION_MAJOR] = static_cast<uint8_t>(motion_version[2]);
  control[INIT_CONTROL_OFF_HARDWARE_PATCH] = static_cast<uint8_t>(hardware_version[0]);
  control[INIT_CONTROL_OFF_HARDWARE_MINOR] = static_cast<uint8_t>(hardware_version[1]);
  control[INIT_CONTROL_OFF_HARDWARE_MAJOR] = static_cast<uint8_t>(hardware_version[2]);
  control[INIT_CONTROL_OFF_ROBOT_RC02_HW] = static_cast<uint8_t>(robot_rc02_hw);

  // CRC input stream includes: len_byte + control_bytes.
  std::array<uint8_t, 1 + RC02_INIT_TX_CONTROL_SIZE> crc_input{};
  crc_input[0] = RC02_INIT_TX_LEN;
  for (size_t i = 0; i < control.size(); ++i) {
    crc_input[1 + i] = control[i];
  }
  const uint16_t crc = LIB_CRC_Checksum(crc_input.data(), crc_input.size());

  std::array<uint8_t, RC02_INIT_TX_FRAME_SIZE> frame{};
  frame[0] = RC02_HEADER_VALUE;
  frame[1] = RC02_INIT_TX_LEN;
  for (size_t i = 0; i < control.size(); ++i) {
    frame[2 + i] = control[i];
  }
  frame[2 + control.size()] = static_cast<uint8_t>(crc & 0xFFu);
  frame[2 + control.size() + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
  return frame;
}

std::array<uint8_t, RC02_HW_VERSION_REQ_FRAME_SIZE> BuildRc02HardwareVersionRequestFrame() {
  std::array<uint8_t, RC02_HW_VERSION_REQ_FRAME_SIZE> frame{};
  frame[MODE_HEADER] = RC02_HEADER_VALUE;
  frame[MODE_LENGTH] = RC02_HW_VERSION_REQ_LEN;
  frame[MODE_DATA] = RC02_HW_VERSION_CMD;

  const uint16_t crc = LIB_CRC_Checksum(frame.data() + MODE_LENGTH, 1 + RC02_HW_VERSION_REQ_LEN);
  frame[MODE_DATA + RC02_HW_VERSION_REQ_LEN] = static_cast<uint8_t>(crc & 0xFFu);
  frame[MODE_DATA + RC02_HW_VERSION_REQ_LEN + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFu);
  return frame;
}

Rc02ParseStatus ParseRc02InitAck(const uint8_t* data, size_t size, size_t* consumed_size) {
  if (consumed_size == nullptr || data == nullptr || size == 0) {
    return Rc02ParseStatus::kSkipOneByte;
  }
  if (data[MODE_HEADER] != RC02_HEADER_VALUE) {
    return Rc02ParseStatus::kSkipOneByte;
  }
  if (size < MODE_DATA) {
    return Rc02ParseStatus::kNeedMoreData;
  }
  const uint8_t len = data[MODE_LENGTH];
  if (len != RC02_INIT_ACK_RSP_LEN) {
    return Rc02ParseStatus::kSkipOneByte;
  }
  const size_t frame_size = RC02_FRAME_FIXED_SIZE + static_cast<size_t>(len);
  if (size < frame_size) {
    return Rc02ParseStatus::kNeedMoreData;
  }
  if (data[MODE_DATA] != RC02_INIT_TX_CMD) {
    return Rc02ParseStatus::kSkipOneByte;
  }
  const uint8_t crc_l = data[MODE_DATA + len];
  const uint8_t crc_h = data[MODE_DATA + len + 1];
  const uint16_t received_crc = static_cast<uint16_t>(crc_l | (static_cast<uint16_t>(crc_h) << 8));
  const uint16_t expected_crc = LIB_CRC_Checksum(data + MODE_LENGTH, 1 + static_cast<size_t>(len));
  if (received_crc != expected_crc) {
    LOG_EVERY_N(WARNING, 200) << "ParseRc02InitAck: CRC mismatch, received=0x" << std::hex << received_crc
                              << " expected=0x" << expected_crc << std::dec;
    return Rc02ParseStatus::kSkipOneByte;
  }
  *consumed_size = frame_size;
  return Rc02ParseStatus::kOk;
}

Rc02ParseStatus ParseRc02HardwareVersionResponse(const uint8_t* data, size_t size, int8_t* out_version,
                                                 size_t* consumed_size) {
  if (out_version == nullptr || consumed_size == nullptr || data == nullptr || size == 0) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  if (data[MODE_HEADER] != RC02_HEADER_VALUE) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  if (size < MODE_DATA) {
    return Rc02ParseStatus::kNeedMoreData;
  }

  const uint8_t len = data[MODE_LENGTH];
  if (len != RC02_HW_VERSION_RSP_LEN) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  const size_t frame_size = RC02_FRAME_FIXED_SIZE + static_cast<size_t>(len);
  if (size < frame_size) {
    return Rc02ParseStatus::kNeedMoreData;
  }

  if (data[MODE_DATA] != RC02_HW_VERSION_CMD) {
    return Rc02ParseStatus::kSkipOneByte;
  }

  const uint8_t crc_l = data[MODE_DATA + len];
  const uint8_t crc_h = data[MODE_DATA + len + 1];
  const uint16_t received_crc = static_cast<uint16_t>(crc_l | (static_cast<uint16_t>(crc_h) << 8));
  const uint16_t expected_crc = LIB_CRC_Checksum(data + MODE_LENGTH, 1 + len);
  if (received_crc != expected_crc) {
    LOG_EVERY_N(WARNING, 200) << "ParseRc02HardwareVersionResponse: CRC mismatch, received=0x" << std::hex
                              << received_crc << " expected=0x" << expected_crc << std::dec;
    return Rc02ParseStatus::kSkipOneByte;
  }

  // The 4-byte payload after the 0x04 cmd byte is a model-prefixed version
  // string, e.g. "T005" or "R012". The first byte is a model/series letter
  // and is ignored; only the trailing three bytes carry the numeric version
  // (e.g. "005" -> 5, "012" -> 12).
  const std::string ver(reinterpret_cast<const char*>(data + MODE_DATA + 2), 3);

  try {
    const int parsed = std::stoi(ver);
    if (parsed < -128 || parsed > 127) {
      return Rc02ParseStatus::kSkipOneByte;
    }
    *out_version = static_cast<int8_t>(parsed);
    *consumed_size = frame_size;
    return Rc02ParseStatus::kOk;
  } catch (...) {
    return Rc02ParseStatus::kSkipOneByte;
  }
}

}  // namespace hardware
