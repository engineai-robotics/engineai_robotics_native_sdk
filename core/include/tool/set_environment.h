#pragma once

#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>

namespace common {

class SetEnvironment {
 public:
  SetEnvironment(const std::string& env_str, const std::string& env_path, int replace = 1) {
    if (!setenv(env_str.c_str(), env_path.c_str(), /*overwrite*/ replace)) {
      std::cout << "[INFO] Set [" << env_str << "] as [" << env_path << "] successfully." << std::endl;
    } else {
      std::cerr << "[ERROR] Set [" << env_str << "] as [" << env_path << "] failed." << std::endl;
    }

    env_[env_str] = env_path;
  }
  ~SetEnvironment() = default;

  std::string GetEnvironment(const std::string& env_str) { return env_.at(env_str); }

 private:
  std::unordered_map<std::string, std::string> env_;
};
}  // namespace common
