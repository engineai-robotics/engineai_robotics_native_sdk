#pragma once

#include <optional>

#include "basic_param/basic_param.h"
#include "parameter/global_config_initializer.h"
#include "parameter/parameter_loader.h"
#include "tool/string_join.h"

namespace data {

class Ros2NodeParam : public BasicParam {
 public:
  Ros2NodeParam(std::string_view name = "", std::string_view tag = "ros2_bridge") : BasicParam(tag, "", name){};
  ~Ros2NodeParam() = default;
  DEFINE_PARAM_SCOPE_AND_FIELD(scope_, field_);
  std::optional<double> LOAD_NESTED_FIELD_PARAM(period);
  std::optional<std::string> LOAD_NESTED_FIELD_PARAM(topic);
  std::optional<std::map<std::string, double>> LOAD_NESTED_FIELD_PARAM(mapping_periods);
  std::optional<std::map<std::string, std::string>> LOAD_NESTED_FIELD_PARAM(publish_topics);
  std::optional<std::map<std::string, std::string>> LOAD_NESTED_FIELD_PARAM(subscribe_topics);
  std::optional<std::set<std::string>> LOAD_NESTED_FIELD_PARAM(motion_switch_white_list);
  std::optional<bool> LOAD_NESTED_FIELD_PARAM(enable);
};

class Ros2BridgeParam : public BasicParam {
 public:
  Ros2BridgeParam(std::string_view tag = "ros2_bridge") : BasicParam(tag){};

  DEFINE_PARAM_SCOPE(scope_);
  std::vector<std::string> LOAD_PARAM(activated_node);
};

}  // namespace data
