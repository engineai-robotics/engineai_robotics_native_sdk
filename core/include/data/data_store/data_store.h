#ifndef DATA_DATA_STORE_DATA_STORE_H_
#define DATA_DATA_STORE_DATA_STORE_H_

#include <glog/logging.h>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "contact_signal_info/contact_signal_info.h"
#include "gamepad_info/gamepad_info.h"
#include "guarded_data/guarded_data.h"
#include "imu_info/imu_info.h"
#include "joint_info/joint_info.h"
#include "joint_override_command/joint_override_command.h"
#include "led_control/led_control.h"
#include "link_info/link_info.h"
#include "model_param/model_param.h"
namespace data {

class DataStore {
 public:
  DataStore();
  ~DataStore() = default;

  std::shared_ptr<ModelParam> model_param;

  GuardedData<GamepadInfo> gamepad_info;
  GuardedData<ImuInfo> imu_info;
  JointInfo motor_info{model_param};
  JointInfo joint_info{model_param};

  GuardedData<LinkInfo> imu_state_in_world;
  GuardedData<LinkInfo> base_state_in_world;
  GuardedData<LinkInfo> com_state_in_world;
  GuardedData<LinkInfo> simulated_base_state_in_world;

  std::unordered_map<std::string, GuardedData<LinkInfo>> end_state_in_world;
  std::unordered_map<std::string, GuardedData<LinkInfo>> end_state_in_base;

  std::shared_ptr<ContactSignalInfo> reference_contact_signal_info;

  GuardedData<std::string> current_motion_task_name;

  GuardedData<JointOverrideCommand> joint_override_command;
  GuardedData<std::unordered_set<int>> joint_override_allowed_indices;
  LedControl led_control_request;
  std::atomic<bool> parallel_by_classic_parser{true};
  std::atomic<bool> enable_or_disable_motor{false};
  std::atomic<bool> enabled_motor_state{false};
};
}  // namespace data

#endif  // DATA_DATA_STORE_DATA_STORE_H_
