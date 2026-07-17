#ifndef COMMON_TOOL_INCLUDE_TOOL_TIMESTAMP_GENERATOR_H_
#define COMMON_TOOL_INCLUDE_TOOL_TIMESTAMP_GENERATOR_H_

#include <atomic>
#include <chrono>

namespace common {

class TimestampGenerator {
 public:
  ~TimestampGenerator() = default;
  TimestampGenerator(const TimestampGenerator&) = delete;
  TimestampGenerator& operator=(const TimestampGenerator&) = delete;
  static TimestampGenerator& GetInstance() {
    static TimestampGenerator instance;
    return instance;
  }

  void SetReferenceTime() { reference_time_ = std::chrono::system_clock::now(); }

  double GetTimestampInMilliseconds() {
    if (!initialized_.load()) {
      reference_time_ = std::chrono::system_clock::now();
      initialized_.store(true);
    }
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(now - reference_time_);
    return duration.count();
  }

 private:
  TimestampGenerator() = default;

  std::atomic<bool> initialized_{false};
  std::chrono::time_point<std::chrono::system_clock> reference_time_;
};
}  // namespace common

#endif  // COMMON_TOOL_INCLUDE_TOOL_TIMESTAMP_GENERATOR_H_
