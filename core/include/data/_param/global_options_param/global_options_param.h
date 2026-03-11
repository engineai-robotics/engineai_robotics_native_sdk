#pragma once

#include "basic_param/basic_param.h"

namespace data {

class GlobalOptionsParam : public BasicParam {
 public:
  GlobalOptionsParam(std::string_view tag = "global_options") : BasicParam(tag){};

  DEFINE_PARAM_SCOPE(scope_);
  bool LOAD_PARAM(strict_motion_check);

  void Update() override { LOAD_PARAM(strict_motion_check); }
};
}  // namespace data
