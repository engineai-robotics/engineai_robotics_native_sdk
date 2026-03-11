#ifndef DATA__PARAM_PD_STAND_PARAM_PD_STAND_PARAM_H_
#define DATA__PARAM_PD_STAND_PARAM_PD_STAND_PARAM_H_

#include "basic_param/basic_param.h"

namespace data {

class PdStandParam : public BasicParam {
 public:
  PdStandParam(std::string_view tag = "pd_stand") : BasicParam(tag){};

  DEFINE_PARAM_SCOPE(scope_);
  std::vector<Eigen::VectorXd> LOAD_PARAM(desired_joint_position);
  std::vector<Eigen::VectorXd> LOAD_PARAM(stiffness);
  std::vector<Eigen::VectorXd> LOAD_PARAM(damping);
  double LOAD_PARAM(duration);
  std::optional<double> LOAD_PARAM(initial_joint_position_bias_threshold);
  std::optional<bool> LOAD_PARAM(auto_transition);

  void Update() override {
    LOAD_PARAM(desired_joint_position);
    LOAD_PARAM(stiffness);
    LOAD_PARAM(damping);
    LOAD_PARAM(duration);
    LOAD_PARAM(initial_joint_position_bias_threshold);
    LOAD_PARAM(auto_transition);
  }

  void Check() override;
};
}  // namespace data

#endif  // DATA__PARAM_PD_STAND_PARAM_PD_STAND_PARAM_H_
