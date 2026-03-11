#ifndef LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_PUBLISHER_H_
#define LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_PUBLISHER_H_

#include <memory>

#include <glog/logging.h>
#include <lcm/lcm-cpp.hpp>

#include "lcm_data/SimState.hpp"
#include "lcm_interface/lcm_data_store.h"
#include "lcm_interface/periodic_task.h"
#include "lcm_param/lcm_param.h"

class LcmPublisher : public lcm_interface::PeriodicTask {
 public:
  LcmPublisher(const data::LcmParam& param, const std::shared_ptr<LcmDataStore>& lcm_data_store);
  ~LcmPublisher() override = default;

  void TaskInit() override;
  void TaskRun() override;

 private:
  std::shared_ptr<LcmDataStore> lcm_data_store_;
  SimState sim_state_;

  std::shared_ptr<lcm::LCM> lcm_;
  std::string channel_ = "sim_state";
};

#endif  // LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_PUBLISHER_H_
