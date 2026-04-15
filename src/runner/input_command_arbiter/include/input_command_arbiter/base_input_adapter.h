#pragma once

#include <chrono>
#include "data_store/data_store.h"

namespace runner {

// Common interface for any input source that can participate in arbitration.
class BaseInputAdapter {
 public:
  explicit BaseInputAdapter(std::string name, const std::shared_ptr<data::DataStore>& data_store)
      : name_(name), data_store_(data_store) {}
  virtual ~BaseInputAdapter() = default;

  virtual void Init() = 0;
  virtual void Run() = 0;

  // Chain-of-responsibility entry point. Implementations may overwrite or
  // merge data into the aggregated input.
  virtual void Process(data::GamepadInfo& input) = 0;

  // Returns whether this source should be applied in the current cycle.
  virtual bool IsActive() const = 0;

  virtual const std::string& GetName() const { return name_; }

  virtual void Log() const;
  virtual bool IsRetaining() const;
  virtual bool RetainInputCommand(double duration);
  virtual void StopRetaining();

 protected:
  std::string name_;
  std::shared_ptr<data::DataStore> data_store_;

  // Retention state
  bool is_retaining_{false};
  double retain_duration_{0.0};
  std::chrono::steady_clock::time_point retain_start_time_;
};

}  // namespace runner
