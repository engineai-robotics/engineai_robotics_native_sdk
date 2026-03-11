#ifndef COMMON_TOOL_INCLUDE_TOOL_STRING_SPLIT_H_
#define COMMON_TOOL_INCLUDE_TOOL_STRING_SPLIT_H_

#include <string_view>
#include <vector>

namespace common {

// Splits string into nonempty components.
// Duplicate delimiters are combined. Delimiters at the beginning and end are ignored.
// Examples:
//   Input "//" with delimiter '/' will return an empty vector.
//   Input "/a//b/" with delimiter '/' will return "a" and "b".
std::vector<std::string_view> StringSplit(std::string_view str, char delimiter);
}  // namespace common

#endif  // COMMON_TOOL_INCLUDE_TOOL_STRING_SPLIT_H_
