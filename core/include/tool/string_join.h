#ifndef COMMON_TOOL_INCLUDE_TOOL_STRING_JOIN_H_
#define COMMON_TOOL_INCLUDE_TOOL_STRING_JOIN_H_

#include <string>

namespace common {

template <typename Separator>
void PathJoin(std::string& result, const Separator& sep, const std::string& last) {
  if (!result.empty()) {
    result += sep;
  }
  result += last;
}

template <typename Separator, typename... Args>
void PathJoin(std::string& result, const Separator& sep, const std::string& first, const Args&... args) {
  if (result.empty()) {
    result += first;
  } else {
    result += sep + first;
  }
  PathJoin(result, sep, args...);
}

template <typename Separator, typename... Args>
std::string StringJoin(const Separator& sep, const Args&... args) {
  std::string result;
  PathJoin(result, sep, args...);
  return result;
}

template <typename... Args>
std::string PathJoin(Args&&... args) {
  return StringJoin("/", std::forward<Args>(args)...);
}
}  // namespace common

#endif  // COMMON_TOOL_INCLUDE_TOOL_STRING_JOIN_H_
