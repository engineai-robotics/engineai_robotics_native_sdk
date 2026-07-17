#include "rl_mimic_trajectory_example/csv_loader.h"
#include <glog/logging.h>
#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include "parameter/global_config_initializer.h"
#include "rl_mimic_trajectory_example/npy_loader.h"
#include "rl_mimic_trajectory_example_param/rl_mimic_trajectory_example_param.h"
#include "tool/file_reader.h"
#include "tool/string_join.h"

namespace rl_mimic_trajectory {

// 简单csv读取和插值实现
Eigen::MatrixXd CsvLoader::LoadAndInterpolateJointTrajectory(const std::string& csv_path,
                                                             const std::vector<std::string>& joint_names, double src_dt,
                                                             double target_dt, int start_frame, int end_frame) {
  std::stringstream file_stream;

  // 使用FileReader读取文件
  if (!common::FileReader::ReadFileToStream(csv_path, file_stream)) {
    LOG(ERROR) << "Failed to read csv file: " << csv_path;
    throw std::runtime_error("Failed to read csv file: " + csv_path);
  }

  std::string line;
  std::vector<std::string> headers;
  // 读取表头
  if (std::getline(file_stream, line)) {
    std::stringstream ss(line);
    std::string col;
    while (std::getline(ss, col, ',')) {
      headers.push_back(col);
    }
  }
  // 建立关节名到列索引的映射
  std::map<std::string, int> joint_col_idx;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    auto it = std::find(headers.begin(), headers.end(), joint_names[i]);
    if (it == headers.end()) {
      throw std::runtime_error("Joint name not found in csv: " + joint_names[i]);
    }
    joint_col_idx[joint_names[i]] = std::distance(headers.begin(), it);
  }

  // 读取所有帧
  std::vector<std::vector<double>> raw_traj;
  while (std::getline(file_stream, line)) {
    std::stringstream ss(line);
    std::string val;
    std::vector<std::string> all_vals;

    // 读取整行的所有值
    while (std::getline(ss, val, ',')) {
      all_vals.push_back(val);
    }

    // 检查是否有足够的列
    if (all_vals.size() < headers.size()) {
      LOG(WARNING) << "Skipping incomplete row with " << all_vals.size() << " columns, expected " << headers.size();
      continue;
    }

    // 按joint_names的顺序提取对应的值
    std::vector<double> row;
    for (const auto& joint_name : joint_names) {
      auto it = joint_col_idx.find(joint_name);
      if (it != joint_col_idx.end() && it->second < (int)all_vals.size()) {
        try {
          row.push_back(std::stod(all_vals[it->second]));
        } catch (const std::exception& e) {
          LOG(ERROR) << "Failed to parse value for joint " << joint_name << ": " << all_vals[it->second];
          throw std::runtime_error("Invalid numeric value in CSV for joint: " + joint_name);
        }
      } else {
        LOG(ERROR) << "Column index out of range for joint " << joint_name;
        throw std::runtime_error("Column index out of range for joint: " + joint_name);
      }
    }

    if ((int)row.size() == (int)joint_names.size()) {
      raw_traj.push_back(row);
    } else {
      LOG(WARNING) << "Skipping row with incorrect number of joint values: " << row.size() << " vs "
                   << joint_names.size();
    }
  }
  size_t src_N = raw_traj.size();
  size_t num_joints = joint_names.size();
  if (src_N == 0) {
    throw std::runtime_error("No data rows found in csv: " + csv_path);
  }

  // 截取traj_frame区间，支持负数索引
  // 负数索引说明：
  //   -1: 最后一行 (src_N-1)
  //   -2: 倒数第二行 (src_N-2)
  //   -n: 倒数第n行 (src_N-n)
  int start_idx, end_idx;

  // 处理负数索引
  if (start_frame < 0) {
    start_idx = std::max(0, (int)src_N + start_frame);  // -1 -> src_N-1, -2 -> src_N-2, etc.
  } else {
    start_idx = std::min((int)src_N - 1, start_frame);
  }

  if (end_frame < 0) {
    end_idx = std::max(0, (int)src_N + end_frame);  // -1 -> src_N-1, -2 -> src_N-2, etc.
  } else {
    end_idx = std::min((int)src_N - 1, end_frame);
  }

  if (start_idx > end_idx) {
    throw std::runtime_error("Invalid traj_frame range: start > end");
  }
  std::vector<std::vector<double>> clipped_traj(raw_traj.begin() + start_idx, raw_traj.begin() + end_idx + 1);

  // 插值
  size_t clipped_N = clipped_traj.size();
  double total_time = (clipped_N - 1) * src_dt;
  size_t target_N = static_cast<size_t>(std::round(total_time / target_dt)) + 1;
  Eigen::MatrixXd interp_traj(target_N, num_joints);
  for (size_t j = 0; j < num_joints; ++j) {
    std::vector<double> src_vals(clipped_N);
    for (size_t i = 0; i < clipped_N; ++i) {
      src_vals[i] = clipped_traj[i][j];
    }
    for (size_t t = 0; t < target_N; ++t) {
      double tgt_time = t * target_dt;
      double src_idx_f = tgt_time / src_dt;
      size_t idx0 = static_cast<size_t>(std::floor(src_idx_f));
      size_t idx1 = std::min(idx0 + 1, clipped_N - 1);
      double alpha = src_idx_f - idx0;
      double v0 = src_vals[idx0];
      double v1 = src_vals[idx1];
      interp_traj(t, j) = (1 - alpha) * v0 + alpha * v1;
    }
  }
  return interp_traj;
}

Eigen::MatrixXd CsvLoader::LoadProfileTrajectory(const data::MotionStateProfile& profile,
                                                 const data::RlMinicTrajectoryParam& param, double runner_period) {
  // 拼接文件路径
  std::string file_path = common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), profile.data_path);

  // 检查文件扩展名，决定使用哪个加载器
  std::string extension;
  size_t dot_pos = file_path.find_last_of('.');
  if (dot_pos != std::string::npos) {
    extension = file_path.substr(dot_pos);
  }

  // 根据文件扩展名选择加载器
  if (extension == ".npy") {
    LOG(INFO) << "Loading NPY trajectory: " << file_path;
    return NpyLoader::LoadProfileTrajectory(profile, param, runner_period);
  } else if (extension == ".csv") {
    LOG(INFO) << "Loading CSV trajectory: " << file_path;
    const auto& joint_names = param.active_joint_names;
    double src_dt = profile.csv_dt;
    double target_dt = runner_period;

    int start_frame = 0, end_frame = -1;
    if (!profile.traj_frame.empty()) {
      start_frame = profile.traj_frame[0];
      end_frame = profile.traj_frame.size() > 1 ? profile.traj_frame[1] : start_frame;
    }
    Eigen::MatrixXd interp_traj =
        LoadAndInterpolateJointTrajectory(file_path, joint_names, src_dt, target_dt, start_frame, end_frame);
    return interp_traj;
  } else {
    LOG(ERROR) << "Unsupported file format: " << extension << " (file: " << file_path << ")";
    throw std::runtime_error("Unsupported trajectory file format. Only .csv and .npy are supported.");
  }
}

}  // namespace rl_mimic_trajectory
