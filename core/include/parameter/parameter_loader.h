#ifndef COMMON_PARAMETER_INCLUDE_PARAMETER_PARAMETER_LOADER_H_
#define COMMON_PARAMETER_INCLUDE_PARAMETER_PARAMETER_LOADER_H_

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include "parameter/yaml_convertor.h"
#include "parameter/yaml_reader.h"

namespace common {

// |root_path| is the path to the root config directory.
// It can be absolute or relative to the current working directory.
void SetGlobalConfigPath(const std::string& root_path);
// |overlay_file| is the path to the overlay file.
// It can be absolute or relative to the current working directory.
void UpdateGlobalConfigPath(const std::string& overlay_file);
YamlReader& GetGlobalConfigTree();

// TODO: Mark these as internal namespace
// Supports YAML tag "!deg" of conversion from degree to radian.
template <typename T, typename Enable = void>
struct YamlDegreesSupport {
  static constexpr bool value = false;
};

template <typename T>
struct YamlDegreesSupport<T, std::enable_if_t<std::is_floating_point_v<T>>> {
  static constexpr bool value = true;
};

// TODO: Mark this as internal namespace
template <typename T>
T ParseYamlAs(const YAML::Node& node, std::string_view scope, std::string_view name) {
  try {
    T value = node.as<T>();
    if constexpr (YamlDegreesSupport<T>::value) {
      if (node.Tag() == "!deg") {
        return value * M_PI / 180;
      }
    }
    return value;
  } catch (const YAML::Exception& e) {
    // Will not return std::nullopt for invalid data.
    std::string msg = fmt::format("Failed to parse scope/name: {}/{}, error: {}", scope, name, e.msg);
    throw YAML::Exception(e.mark, msg);
  }
}

// Design note: This uses partial specialized structs instead of if constexpr to
// differentiate std::optional behavior at compile-time, because in if constexpr
// a is_specialization_of construct is used to detect if a type T is a
// specialization of std::optional, but SWIG cannot handle variadic template
// used in is_specialization_of.

template <typename T>
struct ScopedParameterGetter {
  // Returns the parsed value of the parameter.
  // `T`:
  //   For floating point types or Eigen types, supports conversion from degree to radian with "!deg" tag.
  //   For std::optional<U>, see the specialization of ScopedParameterGetter<std::optional<T>>.
  // Throws: std::invalid_argument when the value is null, even if `T` is std::string.
  // Throws: YAML::Exception when the value cannot be parsed into `T`.
  // The parameter's name ignores underscore suffix in |name|.
  static T Get(std::string_view scope, std::string_view name) {
    if (name.back() == '_') {
      name.remove_suffix(1);
    }
    const YAML::Node& node = GetGlobalConfigTree().FindNode(scope, name);

    // yaml-cpp returns std::string("null") for YAML::Node().as<std::string>().
    // Avoids this code path by checking it before entering yaml-cpp parsing.
    if (node.IsNull() || !node.IsDefined()) {
      std::string msg = fmt::format("Failed to parse null value [{}] in scope [{}]", name, scope);
      throw std::invalid_argument(msg);
    }

    return ParseYamlAs<std::remove_const_t<T>>(node, scope, name);
  }

  // Loads parameter from field
  static T GetFieldParam(std::string_view scope, std::string_view field, std::string_view name) {
    if (name.back() == '_') {
      name.remove_suffix(1);
    }

    YAML::Node root_node = GetGlobalConfigTree().FindNode(scope);
    YAML::Node field_node = root_node[std::string(field)];
    if (!field_node.IsDefined() || field_node.IsNull()) {
      std::string msg = fmt::format("Failed to find field [{}] in scope [{}]", field, scope);
      throw std::invalid_argument(msg);
    }

    YAML::Node param_node = field_node[std::string(name)];
    if (!param_node.IsDefined() || param_node.IsNull()) {
      std::string msg = fmt::format("Failed to find parameter [{}] in scope [{}]/field [{}]", name, scope, field);
      throw std::invalid_argument(msg);
    }

    return ParseYamlAs<std::remove_const_t<T>>(param_node, fmt::format("{}.{}", scope, field), name);
  }
};

template <typename T>
struct ScopedParameterGetter<std::optional<T>> {
  // Returns the parsed value of the parameter, or std::nullopt if the parameter has null value.
  // `T`:
  //   For floating point types or Eigen types, supports conversion from degree to radian with "!deg" tag.
  // Throws: YAML::Exception when the value cannot be parsed into `T`.
  // The parameter's name ignores underscore suffix in |name|.
  static std::optional<T> Get(std::string_view scope, std::string_view name) {
    if (name.back() == '_') {
      name.remove_suffix(1);
    }
    const YAML::Node& node = GetGlobalConfigTree().FindNode(scope, name, true);

    if (node.IsNull() || !node.IsDefined()) {
      return std::nullopt;
    }

    return ParseYamlAs<std::remove_const_t<T>>(node, scope, name);
  }

  static std::optional<T> GetFieldParam(std::string_view scope, std::string_view field, std::string_view name) {
    if (name.back() == '_') {
      name.remove_suffix(1);
    }

    // Gets the node specified by scope
    YAML::Node root_node = GetGlobalConfigTree().FindNode(scope, true);

    // Uses the bracket operator to access the nested structure: root_node[field][name]
    YAML::Node field_node = root_node[std::string(field)];
    if (!field_node.IsDefined() || field_node.IsNull()) {
      return std::nullopt;
    }

    YAML::Node param_node = field_node[std::string(name)];
    if (!param_node.IsDefined() || param_node.IsNull()) {
      return std::nullopt;
    }

    return ParseYamlAs<std::remove_const_t<T>>(param_node, fmt::format("{}.{}", scope, field), name);
  }
};

template <typename T>
struct ScopedParameterGetterWithDefault {
  // Returns `defvalue` if the parameter has null value.
  // Throws: YAML::Exception for non-null invalid value.
  // The parameter's name ignores underscore suffix in |name|.
  static T Get(std::string_view scope, std::string_view name, const T& defvalue) {
    if (scope.data() == nullptr) {
      return defvalue;
    }
    if (name.back() == '_') {
      name.remove_suffix(1);
    }
    const YAML::Node& node = GetGlobalConfigTree().FindNode(scope, name, true);

    if (node.IsNull() || !node.IsDefined()) {
      return defvalue;
    }

    return ParseYamlAs<std::remove_const_t<T>>(node, scope, name);
  }
};

template <typename T>
struct ScopedParameterGetterWithDefault<std::optional<T>> {
  // It is a compile time error for std::optional<T> parameters with specified default value.
  static std::optional<T> Get(std::string_view scope, std::string_view name, const T& defvalue) = delete;
};

// Defines the "parameter scope" of this C++ scope. Must precede all scoped parameters.
#define DEFINE_PARAM_SCOPE(str) std::string_view base_scoped_parameter_internal_scope_ = str

// Defines both parameter scope and field. Must precede all scoped parameters with field access.
#define DEFINE_PARAM_SCOPE_AND_FIELD(scope, field)                \
  std::string_view base_scoped_parameter_internal_scope_ = scope; \
  std::string_view base_scoped_parameter_internal_field_ = field

// Initializes this variable from locally defined config-scope.
#define LOAD_PARAM(var) \
  var = common::ScopedParameterGetter<decltype(var)>::Get(base_scoped_parameter_internal_scope_, #var)

// Initializes this variable from locally defined config-scope with default value.
#define LOAD_PARAM_DEFAULT(var, def) \
  var = common::ScopedParameterGetterWithDefault<decltype(var)>::Get(base_scoped_parameter_internal_scope_, #var, def)

// Initializes this variable from a field within the defined config-scope using nested access.
// This macro uses GetFieldParam to directly access YAML nodes with brackets: scope[field][var]
#define LOAD_NESTED_FIELD_PARAM(var)                                                                       \
  var = common::ScopedParameterGetter<decltype(var)>::GetFieldParam(base_scoped_parameter_internal_scope_, \
                                                                    base_scoped_parameter_internal_field_, #var)

// Initializes this variable from macro argument defined config-scope.
#define CUSTOM_LOAD_PARAM(scope, var) var = common::ScopedParameterGetter<decltype(var)>::Get(scope, #var)

// Initializes this variable from macro argument defined config-scope.
#define CUSTOM_LOAD_PARAM_DEFAULT(scope, var, def) \
  var = common::ScopedParameterGetterWithDefault<decltype(var)>::Get(scope, #var, def)

}  // namespace common
#endif  // COMMON_PARAMETER_INCLUDE_PARAMETER_PARAMETER_LOADER_H_
