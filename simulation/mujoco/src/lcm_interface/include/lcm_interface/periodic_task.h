#ifndef LCM_INTERFACE_INCLUDE_BASE_PERIODIC_TASK_H_
#define LCM_INTERFACE_INCLUDE_BASE_PERIODIC_TASK_H_

#include <atomic>
#include <chrono>
#include <exception>
#include <string>
#include <thread>

#include <glog/logging.h>

namespace lcm_interface {

class PeriodicTask {
 public:
  PeriodicTask(const std::string& name, float period) : name_(name), period_(period) {}

  virtual ~PeriodicTask() { TaskStop(); }

  void TaskCreate() {
    if (thread_running_.exchange(true)) {
      LOG(INFO) << "[PeriodicTask] Tried to create [" << name_ << "] but it was already running";
      return;
    }

    task_running_.store(false);
    thread_ = std::thread([this]() { LoopFunction(); });
  }

  void TaskStart() { task_running_.store(true); }

  void TaskStop() {
    if (!thread_running_.exchange(false)) {
      return;
    }

    if (thread_.joinable()) {
      LOG(INFO) << "[PeriodicTask] Waiting for " << name_ << " to stop...";
      thread_.join();
      LOG(INFO) << "[PeriodicTask] Done";
    }
  }

  virtual void TaskInit() = 0;
  virtual void TaskRun() = 0;

  void SetPeriod(float period) { period_.store(period); }

  float GetPeriod() const { return period_.load(); }

 private:
  void LoopFunction() {
    auto next_wakeup = std::chrono::steady_clock::now();

    LOG(INFO) << "[PeriodicTask] Start [" << name_ << "] with period (" << period_.load() << " s)";

    while (thread_running_.load()) {
      if (task_running_.load()) {
        try {
          TaskRun();
        } catch (const std::exception& e) {
          LOG(ERROR) << "Exception during " << name_ << " loop function: " << e.what();
          thread_running_.store(false);
          break;
        }
      }

      const float period = period_.load();
      if (period <= 0.0f) {
        std::this_thread::yield();
        continue;
      }

      const auto period_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period));
      next_wakeup += period_ns;
      const auto now = std::chrono::steady_clock::now();
      if (next_wakeup < now) {
        next_wakeup = now;
      }
      std::this_thread::sleep_until(next_wakeup);
    }
    LOG(INFO) << "[PeriodicTask] " << name_ << " has stopped";
  }

  std::string name_;
  std::thread thread_;
  std::atomic<float> period_{0};

  std::atomic<bool> thread_running_{false};
  std::atomic<bool> task_running_{false};
};

}  // namespace lcm_interface

#endif  // LCM_INTERFACE_INCLUDE_BASE_PERIODIC_TASK_H_
