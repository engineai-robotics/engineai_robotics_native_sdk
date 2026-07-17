#ifndef DATA__PARAM_BASIC_PARAM_BASIC_PARAM_IMPL_H_
#define DATA__PARAM_BASIC_PARAM_BASIC_PARAM_IMPL_H_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include "parameter/parameter_loader.h"

namespace data {

class BasicParam {
 public:
  BasicParam() = default;
  BasicParam(std::string_view tag, std::string_view custom_scope = "", std::string_view custom_field = "");
  virtual ~BasicParam() = default;

  virtual void Update() {}
  virtual void Log() {}
  virtual void Check() {}
  const std::string& GetTag() const;
  const std::string& GetScope() const;
  const std::string& GetField() const;
  const std::string& GetAccess() const;

  void ParseInheritConfig(const YAML::Node& configs, std::map<std::string, YAML::Node>& merged_configs);
  std::map<std::string, YAML::Node> ParseMode(std::string_view tag);
  static const std::string FindScope(const std::string& tag);
  static void BuildScopeMap(const std::map<std::string, YAML::Node>& configs);

 protected:
  friend class Manager;
  std::string tag_ = "";
  std::string scope_ = "";
  std::string field_ = "";
  std::string access_ = "read-write";

  static std::string active_mode_;
  static std::map<std::string, std::string> tag_map_scope_;
};

}  // namespace data

#endif  // DATA__PARAM_BASIC_PARAM_BASIC_PARAM_IMPL_H_
