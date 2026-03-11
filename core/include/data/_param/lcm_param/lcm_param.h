#ifndef DATA__PARAM_LCM_PARAM_LCM_PARAM_H_
#define DATA__PARAM_LCM_PARAM_LCM_PARAM_H_

#include "basic_param/basic_param.h"

namespace data {

class LcmParam : public BasicParam {
 public:
  LcmParam(std::string_view tag = "lcm") : BasicParam(tag){};

  DEFINE_PARAM_SCOPE(scope_);
  bool LOAD_PARAM(multicast);
  std::string LOAD_PARAM(ip_port);
  int LOAD_PARAM(ttl);

  void Update() {
    LOAD_PARAM(multicast);
    LOAD_PARAM(ip_port);
    LOAD_PARAM(ttl);
  }
};
}  // namespace data

#endif  // DATA__PARAM_LCM_PARAM_LCM_PARAM_H_
