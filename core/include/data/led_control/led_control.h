#pragma once

#include <array>
#include <cstddef>
#include <mutex>
#include <shared_mutex>

namespace data {

enum class LedColor : unsigned char {
  UNDEFINED = 0x0,
  BLINK_RED = 0x1,
  BLINK_GREEN = 0x2,
  BLINK_BLUE = 0x3,
  BLINK_WHITE = 0x4,
  CONSTANT_ON_WHITE = 0x5,
  CONSTANT_ON_GREEN = 0x6,
  BREATHE_WHITE = 0x7,
  WATER_WHITE = 0x8,
  BREATHE_RED = 0x9,
  BLINK_ORANGE = 0xa,
  CONSTANT_ON_ORANGE = 0xb,
  COUNT,
};

enum class LedMode : unsigned char {
  ERROR = 0x0,
  CUSTOM = 0x1,
  DEBUG = 0x2,
  NORMAL = 0x3,
  COUNT,
};

static constexpr std::size_t kModeCount = static_cast<std::size_t>(LedMode::COUNT);

/*
  * LedControl 负责统一管理各模块的 LED 颜色请求。
  * 不同模块可通过 Set() 方法为特定 LedMode 设置 LED 颜色，或通过 Clear() 方法清除颜色设置。

  * 最终 LED 显示颜色按模式优先级确定：
  *   ERROR > CUSTOM > DEBUG > NORMAL
  * 高优先级请求会覆盖低优先级请求；若某 LedMode 被清除（设置为 UNDEFINED），则不参与颜色计算。
  * 当没有更高优先级的请求时，NORMAL 模式会回退到默认颜色 CONSTANT_ON_WHITE。
*/
class LedControl {
 public:
  LedControl() { Reset(); }
  ~LedControl() = default;

  void Reset() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    requests_.fill(LedColor::UNDEFINED);
    requests_[Index(LedMode::NORMAL)] = LedColor::CONSTANT_ON_WHITE;
  }

  void Set(LedMode mode, LedColor color) {
    if (color == LedColor::UNDEFINED) {
      Clear(mode);
      return;
    }

    std::unique_lock<std::shared_mutex> lock(mutex_);
    requests_[Index(mode)] = color;
  }

  std::array<LedColor, kModeCount> Get() {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return requests_;
  }

  void Clear(LedMode mode) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (mode == LedMode::NORMAL) {
      requests_[Index(mode)] = LedColor::CONSTANT_ON_WHITE;
      return;
    }

    requests_[Index(mode)] = LedColor::UNDEFINED;
  }

 private:
  static constexpr std::size_t Index(LedMode mode) { return static_cast<std::size_t>(mode); }

  mutable std::shared_mutex mutex_;
  /* requests_ 按 LedMode 枚举顺序存储各模式的颜色请求；下标越小优先级越高。
   * 遍历时遇到的首个非 UNDEFINED 颜色决定最终 LED 显示，处理逻辑位于 LedController::ProcessLedControl()。
   * 示例：
   *   {UNDEFINED, BLINK_GREEN, UNDEFINED, CONSTANT_ON_WHITE}      -> BLINK_GREEN
   *   {UNDEFINED, UNDEFINED, UNDEFINED, CONSTANT_ON_WHITE}        -> CONSTANT_ON_WHITE
   *   {BLINK_RED, BLINK_GREEN, BLINK_ORANGE, CONSTANT_ON_WHITE}   -> BLINK_RED
   */
  std::array<LedColor, kModeCount> requests_;
};
}  // namespace data
