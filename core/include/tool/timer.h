#pragma once

#include <assert.h>
#include <stdint.h>
#include <time.h>

namespace common {

class Timer {
 public:
  explicit Timer() { StartTimer(); }

  void StartTimer() { clock_gettime(CLOCK_MONOTONIC, &start_time_); }

  double GetElapsedSeconds() { return (double)GetElapsedNanoseconds() * kNanosecondPerSecond; }

  double GetElapsedMilliseconds() { return (double)GetElapsedNanoseconds() * kNanosecondPerMillisecond; }

  double GetElapsedMicroseconds() { return (double)GetElapsedNanoseconds() * kNanosecondPerMicrosecond; }

  int64_t GetElapsedNanoseconds() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - start_time_.tv_nsec) + kSecondPerNanosecond * (now.tv_sec - start_time_.tv_sec);
  }

 private:
  static constexpr double kNanosecondPerSecond = 1.e-9;
  static constexpr double kNanosecondPerMillisecond = 1.e-6;
  static constexpr double kNanosecondPerMicrosecond = 1.e-3;
  static constexpr int64_t kSecondPerNanosecond = 1000000000;

  struct timespec start_time_;
};
}  // namespace common
