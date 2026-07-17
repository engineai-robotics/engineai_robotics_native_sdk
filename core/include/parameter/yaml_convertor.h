#ifndef COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_CONVERTOR_H_
#define COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_CONVERTOR_H_
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace YAML {

template <typename Scalar, int Size, int Options>
struct convert<Eigen::Matrix<Scalar, Size, 1, Options>> {
  static bool decode(const YAML::Node& node, Eigen::Matrix<Scalar, Size, 1, Options>& vec) {
    std::vector<Scalar> data;
    // Node should be a sequence for a vector
    if (!node.IsSequence()) {
      return false;
    }

    int size = node.size();
    data.reserve(size);
    for (const auto& element : node) {
      data.push_back(element.as<Scalar>());
    }

    // Size of vector should match number of elements
    if (size != data.size()) {
      return false;
    }

    vec = Eigen::Map<const Eigen::Matrix<Scalar, Size, 1, Options>>(data.data(), size);
    return true;
  }
};

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct convert<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
  static bool decode(const YAML::Node& node, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& mat) {
    std::vector<Scalar> data;
    // Node should be a sequence for a matrix
    if (!node.IsSequence()) {
      return false;
    }

    int rows = node.size();
    int cols = 0;
    for (int i = 0; i < rows; ++i) {
      // Each row should be a sequence
      if (!node[i].IsSequence()) {
        return false;
      }
      if (i == 0) {
        cols = node[i].size();
      } else if (node[i].size() != cols) {
        // All rows should have the same number of elements
        return false;
      }
      for (const auto& element : node[i]) {
        data.push_back(element.as<Scalar>());
      }
    }

    // Total number of elements should match rows * cols
    if (rows * cols != data.size()) {
      return false;
    }

    mat =
        Eigen::Map<const Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor, MaxRows, MaxCols>>(data.data(), rows, cols);
    return true;
  }
};

template <typename Scalar, int Size, int Options>
struct convert<std::vector<Eigen::Matrix<Scalar, Size, 1, Options>>> {
  static bool decode(const YAML::Node& node, std::vector<Eigen::Matrix<Scalar, Size, 1, Options>>& vec_list) {
    if (!node.IsSequence()) {
      return false;
    }

    vec_list.clear();
    for (const auto& element : node) {
      Eigen::Matrix<Scalar, Size, 1, Options> vec;
      if (!convert<Eigen::Matrix<Scalar, Size, 1, Options>>::decode(element, vec)) {
        return false;
      }
      vec_list.push_back(vec);
    }
    return true;
  }
};

template <typename T>
struct convert<std::set<T>> {
  static bool decode(const Node& node, std::set<T>& out) {
    if (!node.IsSequence()) return false;
    out.clear();
    for (const auto& elem : node) {
      out.insert(elem.as<T>());
    }
    return true;
  }
};

}  // namespace YAML
#endif  // COMMON_PARAMETER_INCLUDE_PARAMETER_YAML_CONVERTOR_H_
