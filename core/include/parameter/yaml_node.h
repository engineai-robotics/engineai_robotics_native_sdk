#ifndef COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_NODE_H_
#define COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_NODE_H_

#include <map>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace common {

class YamlNode {
 public:
  YamlNode& operator[](std::string_view path);
  void Merge(const YAML::Node& node);

  YAML::Node& value() { return value_; }
  const YAML::Node& value() const { return value_; }
  const std::map<std::string, YamlNode>& children() const { return children_; }

 private:
  YAML::Node value_;
  std::map<std::string, YamlNode> children_;
};
}  // namespace common

#endif  // COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_NODE_H_
