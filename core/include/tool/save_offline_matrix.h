#ifndef COMMON_TOOL_INCLUDE_TOOL_SAVE_OFFLINE_MATRIX_H_
#define COMMON_TOOL_INCLUDE_TOOL_SAVE_OFFLINE_MATRIX_H_

#include <glog/logging.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>

namespace common {

template <typename T>
class SaveOfflineMatrix {
 public:
  SaveOfflineMatrix(const Eigen::MatrixX<T>& matrix, const std::string& path, const std::string& delimiter = " ") {
    std::ofstream file(path);
    if (file.is_open()) {
      for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
          file << matrix(i, j) << delimiter;
        }
        file << "\n";
      }
      file.close();
      LOG(INFO) << " Save offline matrix to [" << path << "].";
    } else {
      LOG(ERROR) << "Failed to open file [" << path << "].";
    }
  }

  ~SaveOfflineMatrix() = default;
};
}  // namespace common

#endif  // COMMON_TOOL_INCLUDE_TOOL_SAVE_OFFLINE_MATRIX_H_
