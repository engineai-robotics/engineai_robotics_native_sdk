#include "rc02/rc02_driver.h"

#include <glog/logging.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace {

constexpr int kStickAbsMax = 127;
constexpr int kTriggerMax = 255;

// Render a byte buffer as space-separated upper-case hex, e.g. "AA 01 04 ...".
std::string BytesToHex(const uint8_t* data, size_t size) {
  std::stringstream ss;
  ss << std::uppercase << std::hex << std::setfill('0');
  for (size_t i = 0; i < size; ++i) {
    if (i != 0) ss << ' ';
    ss << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  return ss.str();
}

bool ParseVersionString(const std::string& version, int8_t* major, int8_t* minor, int8_t* patch) {
  if (major == nullptr || minor == nullptr || patch == nullptr) return false;
  std::stringstream ss(version);
  int maj = 0, min = 0, pat = 0;
  char dot1 = '\0', dot2 = '\0';
  if (!(ss >> maj >> dot1 >> min >> dot2 >> pat) || dot1 != '.' || dot2 != '.') return false;
  if (maj < 0 || maj > 127 || min < 0 || min > 127 || pat < 0 || pat > 127) return false;
  *major = static_cast<int8_t>(maj);
  *minor = static_cast<int8_t>(min);
  *patch = static_cast<int8_t>(pat);
  return true;
}

bool ParseVersionFromEnv(const char* env_name, int8_t* major, int8_t* minor, int8_t* patch) {
  const char* raw = std::getenv(env_name);
  return raw != nullptr && ParseVersionString(std::string(raw), major, minor, patch);
}

}  // namespace

namespace hardware {

// =============================================================================
// 生命周期
// =============================================================================

RC02Driver::RC02Driver(std::string dev, int baud) : dev_path_(std::move(dev)), baudrate_(baud) {}

RC02Driver::~RC02Driver() { Close(); }

bool RC02Driver::Init() {
  Close();
  initSerial();
  return configured_;
}

void RC02Driver::Close() noexcept {
  if (serial_.isOpen()) {
    serial_.close();
  }
  ResetConnectionState();
  origin_ = {};
  input_ = {};
  rx_buffer_.clear();
}

// =============================================================================
// 周期性收发
// =============================================================================

bool RC02Driver::SendMotionAndACK(uint8_t error_code, uint8_t current_motion) {
  static uint8_t last_logged_current_motion = 0;
  static bool has_logged_current_motion = false;
  if (!has_logged_current_motion || last_logged_current_motion != current_motion) {
    LOG(INFO) << "RC02 current_motion changed: " << static_cast<int>(last_logged_current_motion) << " -> "
              << static_cast<int>(current_motion);
    last_logged_current_motion = current_motion;
    has_logged_current_motion = true;
  }

  if (!configured_ || !serial_.isOpen()) {
    if (!serial_.isOpen()) {
      ResetConnectionState();
    }
    LOG_EVERY_N(ERROR, 200) << "RC02 SendMotionAndACK pre-check failed: configured_=" << configured_
                            << ", serialOpen=" << serial_.isOpen() << ", error_code=" << static_cast<int>(error_code)
                            << ", current_motion=" << static_cast<int>(current_motion);
    return false;
  }

  if (!SendMotionStatus(error_code, current_motion)) return false;
  std::this_thread::sleep_for(std::chrono::microseconds(100));

  std::array<uint8_t, 1024> buffer{};
  bool got_any_good = false;

  try {
    const size_t avail = serial_.available();
    if (avail > 0) {
      const size_t read_size = std::min(avail, buffer.size());
      const size_t n = serial_.read(buffer.data(), read_size);
      if (n > 0) {
        rx_buffer_.insert(rx_buffer_.end(), buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(n));
      }
    }
  } catch (std::exception& e) {
    LOG_EVERY_N(ERROR, 200) << "RC02 serial read exception: " << e.what();
    ResetConnectionState();
    if (serial_.isOpen()) {
      try {
        serial_.close();
      } catch (...) {
      }
    }
    return false;
  }

  constexpr size_t kMaxRxBufferSize = 2048;
  if (rx_buffer_.size() > kMaxRxBufferSize) {
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - static_cast<std::ptrdiff_t>(kMaxRxBufferSize));
  }

  while (!rx_buffer_.empty()) {
    Rc02OriginData out{};
    size_t consumed = 0;
    const Rc02ParseStatus status = ParseRc02Frame(rx_buffer_.data(), rx_buffer_.size(), &out, &consumed);
    if (status == Rc02ParseStatus::kNeedMoreData) break;
    if (status == Rc02ParseStatus::kSkipOneByte) {
      LOG(WARNING) << "RC02 rx: skipping unexpected byte 0x" << std::uppercase << std::hex << std::setw(2)
                   << std::setfill('0') << static_cast<unsigned>(rx_buffer_.front()) << ".";
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }
    origin_ = out;
    last_good_rx_ = std::chrono::steady_clock::now();
    has_good_rx_ = true;
    got_any_good = true;
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(consumed));
  }

  return got_any_good;
}

// =============================================================================
// 发送帧
// =============================================================================

bool RC02Driver::SendMotionStatus(uint8_t error_code, uint8_t current_motion) {
  if (!configured_ || !serial_.isOpen()) {
    if (!serial_.isOpen()) {
      ResetConnectionState();
    }
    LOG_EVERY_N(ERROR, 200) << "RC02 serial is not ready, cannot send motion status.";
    return false;
  }

  const std::array<uint8_t, RC02_STATUS_TX_FRAME_SIZE> frame = BuildRc02MotionStatusFrame(error_code, current_motion);
  try {
    const size_t written = serial_.write(frame.data(), frame.size());
    if (written != frame.size()) {
      LOG_EVERY_N(ERROR, 200) << "RC02 status frame write incomplete: " << written << "/" << frame.size();
      ResetConnectionState();
      if (serial_.isOpen()) {
        try {
          serial_.close();
        } catch (...) {
        }
      }
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    LOG_EVERY_N(ERROR, 200) << "RC02 status frame write exception: " << e.what();
    ResetConnectionState();
    if (serial_.isOpen()) {
      try {
        serial_.close();
      } catch (...) {
      }
    }
    return false;
  }
}

Rc02InitInfo RC02Driver::BuildInitData(const int8_t robot_model, std::string protocol_version) const {
  Rc02InitInfo init_info{};

  init_info.Product = robot_model;
  init_info.Protocol_major = 0x01;
  init_info.Protocol_minor = 0x00;
  init_info.Protocol_patch = 0x01;
  if (!ParseVersionString(protocol_version, &init_info.Protocol_major, &init_info.Protocol_minor,
                          &init_info.Protocol_patch)) {
    LOG(WARNING) << "RC02 protocol version is invalid: '" << protocol_version
                 << "', fallback to default protocol version 1.0.1.";
  }

  init_info.Motion_major = 0x00;
  init_info.Motion_minor = 0x00;
  init_info.Motion_patch = 0x00;
  if (!ParseVersionFromEnv("ENGINEAI_ROBOTICS_VERSION", &init_info.Motion_major, &init_info.Motion_minor,
                           &init_info.Motion_patch)) {
    LOG_EVERY_N(WARNING, 200) << "ENGINEAI_ROBOTICS_VERSION is missing/invalid, fallback to default motion version: "
                              << static_cast<int>(0x00) << "." << static_cast<int>(0x00) << "."
                              << static_cast<int>(0x00);
  }

  // 当前未有独立硬件版本，暂时和运控版本保持一致
  init_info.Hardware_patch = init_info.Motion_patch;
  init_info.Hardware_minor = init_info.Motion_minor;
  init_info.Hardware_major = init_info.Motion_major;

  init_info.Robot_rc02_hw = GetCachedRc02HardwareVersion();

  LOG(INFO) << "RC02 init data prepared: "
            << "Product=" << static_cast<int>(init_info.Product)
            << ", Protocol(patch/minor/major)=" << static_cast<int>(init_info.Protocol_patch) << "/"
            << static_cast<int>(init_info.Protocol_minor) << "/" << static_cast<int>(init_info.Protocol_major)
            << ", Motion(patch/minor/major)=" << static_cast<int>(init_info.Motion_patch) << "/"
            << static_cast<int>(init_info.Motion_minor) << "/" << static_cast<int>(init_info.Motion_major)
            << ", Hardware(patch/minor/major)=" << static_cast<int>(init_info.Hardware_patch) << "/"
            << static_cast<int>(init_info.Hardware_minor) << "/" << static_cast<int>(init_info.Hardware_major)
            << ", Robot_rc02_hw=" << static_cast<int>(init_info.Robot_rc02_hw);
  return init_info;
}

bool RC02Driver::SendInitData(const Rc02InitInfo& init_info) {
  if (!configured_ || !serial_.isOpen()) {
    if (!serial_.isOpen()) {
      ResetConnectionState();
    }
    LOG_EVERY_N(ERROR, 200) << "RC02 serial is not ready, cannot send init data.";
    return false;
  }

  const std::array<int8_t, 3> protocol_version{init_info.Protocol_patch, init_info.Protocol_minor,
                                               init_info.Protocol_major};
  const std::array<int8_t, 3> motion_version{init_info.Motion_patch, init_info.Motion_minor, init_info.Motion_major};
  const std::array<int8_t, 3> hardware_version{init_info.Hardware_patch, init_info.Hardware_minor,
                                               init_info.Hardware_major};

  const std::array<uint8_t, RC02_INIT_TX_FRAME_SIZE> frame = BuildRc02InitFrame(
      init_info.Product, protocol_version, motion_version, hardware_version, init_info.Robot_rc02_hw);
  LOG(INFO) << "RC02 init frame (" << frame.size() << "B): " << BytesToHex(frame.data(), frame.size());

  try {
    const size_t written = serial_.write(frame.data(), frame.size());
    if (written != frame.size()) {
      LOG_EVERY_N(ERROR, 200) << "RC02 init frame write incomplete: " << written << "/" << frame.size();
      ResetConnectionState();
      if (serial_.isOpen()) {
        try {
          serial_.close();
        } catch (...) {
        }
      }
      return false;
    }
  } catch (const std::exception& e) {
    LOG_EVERY_N(ERROR, 200) << "RC02 init frame write exception: " << e.what();
    ResetConnectionState();
    if (serial_.isOpen()) {
      try {
        serial_.close();
      } catch (...) {
      }
    }
    return false;
  }

  // Wait for init ACK: header(AA) + len(01) + cmd(0D) + crc(2B), CRC validated by ParseRc02InitAck.
  constexpr auto kAckTimeout = std::chrono::milliseconds(500);

  std::vector<uint8_t> rx;
  rx.reserve(RC02_INIT_ACK_RSP_FRAME_SIZE);
  const auto deadline = std::chrono::steady_clock::now() + kAckTimeout;
  std::array<uint8_t, 16> tmp{};

  while (std::chrono::steady_clock::now() < deadline) {
    try {
      const size_t avail = serial_.available();
      if (avail > 0) {
        const size_t n = serial_.read(tmp.data(), std::min(avail, tmp.size()));
        if (n > 0) {
          rx.insert(rx.end(), tmp.begin(), tmp.begin() + static_cast<std::ptrdiff_t>(n));
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    } catch (const std::exception& e) {
      LOG(ERROR) << "RC02 init ACK read exception: " << e.what();
      ResetConnectionState();
      if (serial_.isOpen()) {
        try {
          serial_.close();
        } catch (...) {
        }
      }
      return false;
    }

    for (size_t i = 0; i < rx.size(); ++i) {
      size_t consumed = 0;
      const Rc02ParseStatus status = ParseRc02InitAck(rx.data() + i, rx.size() - i, &consumed);
      if (status == Rc02ParseStatus::kNeedMoreData) break;
      if (status == Rc02ParseStatus::kSkipOneByte) continue;
      LOG(INFO) << "RC02 init ACK received: " << BytesToHex(rx.data() + i, consumed);
      return true;
    }
  }

  LOG(WARNING) << "RC02 init ACK timeout or mismatch. rx (" << rx.size() << "B): " << BytesToHex(rx.data(), rx.size());
  return false;
}

// =============================================================================
// 硬件版本
// =============================================================================

bool RC02Driver::GetRc02HardwareVersion(std::chrono::milliseconds timeout) {
  if (!configured_ || !serial_.isOpen()) {
    LOG_EVERY_N(ERROR, 200) << "RC02 serial is not ready, cannot query hardware version.";
    return false;
  }

  const std::array<uint8_t, RC02_HW_VERSION_REQ_FRAME_SIZE> req = BuildRc02HardwareVersionRequestFrame();
  LOG(INFO) << "RC02 version req (" << req.size() << "B): " << BytesToHex(req.data(), req.size());

  try {
    const size_t written = serial_.write(req.data(), req.size());
    if (written != req.size()) {
      LOG_EVERY_N(ERROR, 200) << "RC02 version request write incomplete: " << written << "/" << req.size();
      return false;
    }
  } catch (const std::exception& e) {
    LOG_EVERY_N(ERROR, 200) << "RC02 version request write exception: " << e.what();
    ResetConnectionState();
    if (serial_.isOpen()) {
      try {
        serial_.close();
      } catch (...) {
      }
    }
    return false;
  }

  std::vector<uint8_t> rx;
  rx.reserve(32);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::array<uint8_t, 32> tmp{};

  while (std::chrono::steady_clock::now() < deadline) {
    try {
      const size_t avail = serial_.available();
      if (avail > 0) {
        const size_t read_size = std::min(avail, tmp.size());
        const size_t n = serial_.read(tmp.data(), read_size);
        if (n > 0) {
          rx.insert(rx.end(), tmp.begin(), tmp.begin() + static_cast<std::ptrdiff_t>(n));
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    } catch (const std::exception& e) {
      LOG_EVERY_N(ERROR, 200) << "RC02 version response read exception: " << e.what();
      ResetConnectionState();
      if (serial_.isOpen()) {
        try {
          serial_.close();
        } catch (...) {
        }
      }
      return false;
    }

    for (size_t i = 0; i < rx.size(); ++i) {
      int8_t version = 0;
      size_t consumed = 0;
      const Rc02ParseStatus status =
          ParseRc02HardwareVersionResponse(rx.data() + i, rx.size() - i, &version, &consumed);
      if (status == Rc02ParseStatus::kNeedMoreData) break;
      if (status == Rc02ParseStatus::kSkipOneByte) continue;
      last_hw_version_ = version;
      LOG(INFO) << "RC02 version rx (" << rx.size() << "B): " << BytesToHex(rx.data(), rx.size())
                << " | parsed version=" << static_cast<int>(version) << " (consumed " << consumed << "B at offset " << i
                << ")";
      return true;
    }

    if (rx.size() > 128) {
      rx.erase(rx.begin(), rx.end() - 32);
    }
  }

  LOG_EVERY_N(WARNING, 100) << "RC02 version response timeout. rx (" << rx.size()
                            << "B): " << BytesToHex(rx.data(), rx.size());
  return false;
}

// =============================================================================
// 数据读取与连接状态
// =============================================================================

const Rc02InputData& RC02Driver::GetInputData() {
  ParseInputData();
  return input_;
}

bool RC02Driver::IsConnected() const { return configured_ && serial_.isOpen(); }

// =============================================================================
// 内部实现
// =============================================================================

void RC02Driver::ResetConnectionState() noexcept {
  configured_ = false;
  has_good_rx_ = false;
}

void RC02Driver::initSerial() {
  configured_ = false;
  try {
    serial_.setPort(dev_path_);
    serial_.setBaudrate(static_cast<uint32_t>(baudrate_));
    serial_.setBytesize(serial::eightbits);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout(serial::Timeout::max(), 2, 0, 1000, 0);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (serial::IOException& e) {
    LOG_EVERY_N(WARNING, 200) << "Unable to open RC02 serial port: " << dev_path_ << ", reason: " << e.what();
    return;
  } catch (std::exception& e) {
    LOG(ERROR) << "RC02 serial init exception: " << e.what();
    return;
  }
  if (serial_.isOpen()) {
    configured_ = true;
    LOG(INFO) << "RC02 serial port opened: " << dev_path_ << " @ " << baudrate_ << " baud.";
  }
}

double RC02Driver::NormalizeTrigger(uint8_t value) {
  return static_cast<double>(value) / static_cast<double>(kTriggerMax);
}

double RC02Driver::NormalizeStick(int8_t value) {
  const int clamped = std::clamp(static_cast<int>(value), -kStickAbsMax, kStickAbsMax);
  return static_cast<double>(clamped) / static_cast<double>(kStickAbsMax);
}

void RC02Driver::ParseInputData() {
  input_ = {};

  input_.RightStickButton = !IsBitSet(origin_.buttons_1, 7);
  input_.LeftStickButton = !IsBitSet(origin_.buttons_1, 6);
  input_.DigitalR2 = !IsBitSet(origin_.buttons_1, 3);
  input_.L1 = !IsBitSet(origin_.buttons_1, 2);
  input_.R1 = !IsBitSet(origin_.buttons_1, 1);
  input_.DigitalL2 = !IsBitSet(origin_.buttons_1, 0);

  input_.A = !IsBitSet(origin_.buttons_2, 7);
  input_.B = !IsBitSet(origin_.buttons_2, 6);
  input_.X = !IsBitSet(origin_.buttons_2, 5);
  input_.Y = !IsBitSet(origin_.buttons_2, 4);
  input_.CROSS_DOWN = !IsBitSet(origin_.buttons_2, 3);
  input_.CROSS_RIGHT = !IsBitSet(origin_.buttons_2, 2);
  input_.CROSS_LEFT = !IsBitSet(origin_.buttons_2, 1);
  input_.CROSS_UP = !IsBitSet(origin_.buttons_2, 0);

  // Physical buttons 1: bit[5] start, bit[4] back.
  input_.START = !IsBitSet(origin_.buttons_1, 5);
  input_.BACK = !IsBitSet(origin_.buttons_1, 4);

  input_.L2 = !IsBitSet(origin_.buttons_1, 0);
  input_.R2 = !IsBitSet(origin_.buttons_1, 3);

  input_.LeftStick_ud = -NormalizeStick(origin_.left_stick_ud);
  input_.LeftStick_lr = -NormalizeStick(origin_.left_stick_lr);
  input_.RightStick_ud = -NormalizeStick(origin_.right_stick_ud);
  input_.RightStick_lr = -NormalizeStick(origin_.right_stick_lr);

  input_.heartbeat = static_cast<int>(origin_.heartbeat);
  //input_.MotionCommand = static_cast<int>(origin_.motion_command);
  //input_.SkillCommand = static_cast<int>(origin_.skill_command);
}

}  // namespace hardware
