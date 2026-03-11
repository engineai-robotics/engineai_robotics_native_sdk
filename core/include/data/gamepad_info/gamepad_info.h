#ifndef DATA_GAMEPAD_INFO_GAMEPAD_INFO_H_
#define DATA_GAMEPAD_INFO_GAMEPAD_INFO_H_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace data {

// LB: pressed as 1, released as 0
// RB: pressed as 1, released as 0
// A: pressed as 1, released as 0
// B: pressed as 1, released as 0
// X: pressed as 1, released as 0
// Y: pressed as 1, released as 0
// Back: pressed as 1, released as 0
// Start: pressed as 1, released as 0
// CROSS_X: up pressed as 1, down pressed as -1, released as 0
// CROSS_Y: left pressed as 1, right pressed as -1, released as 0

// LT: pressed from 0 to 1, released as 0
// RT: pressed from 0 to 1, released as 0
// LeftStick_X: up from 0 to 1, down from 0 to -1, middle as 0
// LeftStick_Y: left from 0 to 1, right from 0 to -1, middle as 0
// RightStick_X: up from 0 to 1, down from 0 to -1, middle as 0
// RightStick_Y: left from 0 to 1, right from 0 to -1, middle as 0
class GamepadInfo {
 public:
  GamepadInfo() { Reset(); }
  ~GamepadInfo() = default;

  int LB;
  int RB;
  int A;
  int B;
  int X;
  int Y;
  int BACK;
  int START;
  int CROSS_X;
  int CROSS_Y;

  double LT;
  double RT;
  double LeftStick_X;
  double LeftStick_Y;
  double RightStick_X;
  double RightStick_Y;

  int combined_key_value = 0;

  bool hardware_connected = false;

  bool enforce_command = false;
  void Reset();
};

class GamepadTool {
 public:
  GamepadTool() { SetKeyValue(); }
  ~GamepadTool() = default;

  void SetKeyValue();
  int KeyStringToValue(std::string_view key) { return key_string_to_value_.at(key.data()); }
  int CalcCombinedKeyValue(const std::vector<std::string>& key);
  data::GamepadInfo ConstructGamepadInfo(const std::vector<std::string>& key);

  static constexpr const auto kKeyString = std::to_array<std::string_view>({
      "LB",
      "RB",
      "A",
      "B",
      "X",
      "Y",
      "BACK",
      "START",
      "CROSS_X_UP",
      "CROSS_X_DOWN",
      "CROSS_Y_LEFT",
      "CROSS_Y_RIGHT",
  });

  using IntMember = int data::GamepadInfo::*;
  static const inline std::unordered_map<std::string, std::pair<IntMember, int>> kKeyToMember = {
      {"LB", {&data::GamepadInfo::LB, 1}},
      {"RB", {&data::GamepadInfo::RB, 1}},
      {"A", {&data::GamepadInfo::A, 1}},
      {"B", {&data::GamepadInfo::B, 1}},
      {"X", {&data::GamepadInfo::X, 1}},
      {"Y", {&data::GamepadInfo::Y, 1}},
      {"BACK", {&data::GamepadInfo::BACK, 1}},
      {"START", {&data::GamepadInfo::START, 1}},
      {"CROSS_X_UP", {&data::GamepadInfo::CROSS_X, 1}},
      {"CROSS_X_DOWN", {&data::GamepadInfo::CROSS_X, -1}},
      {"CROSS_Y_LEFT", {&data::GamepadInfo::CROSS_Y, 1}},
      {"CROSS_Y_RIGHT", {&data::GamepadInfo::CROSS_Y, -1}},
  };

 private:
  std::unordered_map<std::string, int> key_string_to_value_;
};
}  // namespace data
#endif  // DATA_GAMEPAD_INFO_GAMEPAD_INFO_H_
