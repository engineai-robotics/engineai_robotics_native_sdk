#pragma once

#include "basic_param/basic_param.h"
#include "contact_info/contact_info.h"
#include "parameter/global_config_initializer.h"
#include "tool/string_join.h"

namespace data {

class ContactParam : public BasicParam {
 public:
  ContactParam(std::string_view name, std::string_view tag = "dynamic_wbc_test") : BasicParam(tag), name_(name) {
    full_scope_ = common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), GetScope());
    Reset();
    Update();
  };

  void Reset();
  virtual void Update() override;

  std::vector<data::ContactInfo> contact_info;

 private:
  std::string name_ = "";
  std::string full_scope_ = "";
};

}  // namespace data

namespace YAML {

template <>
struct convert<data::ContactInfo> {
  static bool decode(const Node& node, data::ContactInfo& param);
};
}  // namespace YAML
