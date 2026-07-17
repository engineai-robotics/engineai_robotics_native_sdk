#include "pd_stand_param/pd_stand_param.h"
#include "parameter/parameter_loader.h"
namespace data {

void PdStandParam::Check() {
  auto model_param_scope_ = FindScope("model");
  if (model_param_scope_.empty()) {
    throw std::runtime_error("model parameter scope not found");
  }
  auto model_param_node = common::GetGlobalConfigTree().FindNode(model_param_scope_);

  auto limbs_num = model_param_node["limbs"].size();
  if (limbs_num != desired_joint_position.size() || limbs_num != stiffness.size() || limbs_num != damping.size()) {
    throw std::runtime_error("limbs number not match");
  }

  for (size_t i = 0; i < limbs_num; i++) {
    auto joint_num = model_param_node["limbs"][i]["joints"].size();
    if (desired_joint_position[i].size() != joint_num || stiffness[i].size() != joint_num ||
        damping[i].size() != joint_num) {
      throw std::runtime_error("joint number not match");
    }
  }
}

}  // namespace data
