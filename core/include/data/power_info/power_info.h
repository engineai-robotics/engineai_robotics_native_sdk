#pragma once

namespace data {

class PowerInfo {
 public:
  PowerInfo();
  ~PowerInfo();
  bool enable;
  float percentage;
  float voltage;
  float current;
  float current_limit;
  int error_code;
};

}  // namespace data