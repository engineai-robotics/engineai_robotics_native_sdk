#pragma once

#include "basic_param/basic_param.h"

namespace data {

class IdleParam : public BasicParam {
 public:
  IdleParam(std::string_view tag = "idle") : BasicParam(tag){};

  DEFINE_PARAM_SCOPE(scope_);
  std::optional<bool> LOAD_PARAM(pre_motion_cmd_enable);
  std::optional<std::vector<Eigen::VectorXi>> LOAD_PARAM(pre_cmd_enable_joints);
  std::optional<std::vector<Eigen::VectorXd>> LOAD_PARAM(stiffness);
  std::optional<std::vector<Eigen::VectorXd>> LOAD_PARAM(damping);

  void Update() override {
    LOAD_PARAM(pre_motion_cmd_enable);
    LOAD_PARAM(pre_cmd_enable_joints);
    LOAD_PARAM(stiffness);
    LOAD_PARAM(damping);
  }
};

}  // namespace data
