#ifndef MATH_HASH_H_
#define MATH_HASH_H_

namespace math {
constexpr uint32_t String2Int(std::string_view str) {
  uint32_t hash = 5381;
  for (char c : str) {
    hash = (hash << 5) + hash + c;
  }

  return hash;
}
}  // namespace math

#endif
