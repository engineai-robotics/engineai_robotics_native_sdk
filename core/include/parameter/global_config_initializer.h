#pragma once

#include <string>

namespace common {

bool RobotSelector(const std::string& config_dir, std::string& robot_name);
bool InitializeGlobalConfigPath(const std::string& default_robot_name = "");
bool IsInMujoco();

class GlobalPathManager {
 public:
  ~GlobalPathManager() = default;
  GlobalPathManager(const GlobalPathManager&) = delete;
  GlobalPathManager& operator=(const GlobalPathManager&) = delete;
  static GlobalPathManager& GetInstance() {
    static GlobalPathManager instance;
    return instance;
  }

  void SetRobotName(const std::string& name) { this->robot_name_ = name; }
  void SetConfigPath(const std::string& path) { this->config_path_ = path; }
  void SetResourcePath(const std::string& path) { this->resource_path_ = path; }
  void SetUrdfPath(const std::string& path) { this->urdf_path_ = path; }

  std::string GetRobotName() const { return this->robot_name_; }
  std::string GetConfigPath() const { return this->config_path_; }
  std::string GetResourcePath() const { return this->resource_path_; }
  std::string GetUrdfPath() const { return this->urdf_path_; }

 private:
  GlobalPathManager() = default;
  std::string robot_name_;
  std::string config_path_;
  std::string resource_path_;
  std::string urdf_path_;
};
}  // namespace common
