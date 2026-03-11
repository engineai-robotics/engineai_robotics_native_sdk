#pragma once

#include "basic_param/basic_param.h"

namespace data {

class PassiveParam : public BasicParam {
 public:
  PassiveParam(std::string_view tag = "passive") : BasicParam(tag){};

  DEFINE_PARAM_SCOPE(scope_);
  std::optional<std::vector<Eigen::VectorXd>> LOAD_PARAM(damping);

  void Update() override { LOAD_PARAM(damping); }
};
}  // namespace data
