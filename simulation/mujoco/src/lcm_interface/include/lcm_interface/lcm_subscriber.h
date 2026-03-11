#ifndef LCM_INTERFACE_INCLUDE_LCM_LCM_INTERFACE_LCM_SUBSCRIBER_H_
#define LCM_INTERFACE_INCLUDE_LCM_LCM_INTERFACE_LCM_SUBSCRIBER_H_

#include <memory>

#include <glog/logging.h>
#include <lcm/lcm-cpp.hpp>

#include "lcm_data/SimCommand.hpp"
#include "lcm_interface/lcm_data_store.h"
#include "lcm_interface/periodic_task.h"
#include "lcm_param/lcm_param.h"

class LcmSubscriber : public lcm_interface::PeriodicTask {
 public:
  LcmSubscriber(const data::LcmParam& param, const std::shared_ptr<LcmDataStore>& lcm_data_store);
  ~LcmSubscriber() override = default;

  void TaskInit() override;
  void TaskRun() override;

  void HandleSimCommand(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const data::SimCommand* msg);

 private:
  std::shared_ptr<LcmDataStore> lcm_data_store_;
  SimCommand sim_command_;

  std::shared_ptr<lcm::LCM> lcm_;
  std::string channel_ = "sim_command";

  int iter_ = 0;
  static constexpr int kDelayIterations = 100;
};

#endif  // LCM_INTERFACE_INCLUDE_LCM_LCM_INTERFACE_LCM_SUBSCRIBER_H_
