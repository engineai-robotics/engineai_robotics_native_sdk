#include "rl_mimic_trajectory_example/npy_loader.h"
#include <glog/logging.h>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include "parameter/global_config_initializer.h"
#include "rl_mimic_trajectory_example_param/rl_mimic_trajectory_example_param.h"
#include "tool/file_reader.h"
#include "tool/string_join.h"

namespace rl_mimic_trajectory {

// NPY file format parser
// Supports NPY version 1.0 format with float32 data
struct NpyHeader {
  char magic[6];
  uint8_t major_version;
  uint8_t minor_version;
  uint16_t header_len;
};

// Parse NPY header and extract shape information, dtype, and fortran_order
bool ParseNpyHeader(std::istream& stream, std::vector<size_t>& shape, size_t& data_offset, std::string& dtype,
                    bool& fortran_order) {
  NpyHeader header;
  stream.read(reinterpret_cast<char*>(&header), 10);

  if (std::strncmp(header.magic, "\x93NUMPY", 6) != 0) {
    LOG(ERROR) << "Invalid NPY magic number";
    return false;
  }

  if (header.major_version != 1) {
    LOG(ERROR) << "Unsupported NPY version: " << (int)header.major_version;
    return false;
  }

  // Read header dictionary
  std::vector<char> header_dict(header.header_len);
  stream.read(header_dict.data(), header.header_len);
  std::string header_str(header_dict.begin(), header_dict.end());

  // Parse fortran_order from header string
  fortran_order = false;  // Default to C order
  size_t fortran_pos = header_str.find("'fortran_order':");
  if (fortran_pos != std::string::npos) {
    size_t true_pos = header_str.find("True", fortran_pos);
    if (true_pos != std::string::npos && true_pos < fortran_pos + 30) {
      fortran_order = true;
    }
  }

  // Parse dtype from header string
  // Format: "{'descr': '<f4', 'fortran_order': False, 'shape': (N, 34), }"
  // dtype can be '<f2' (float16), '<f4' (float32), '<f8' (float64), etc.
  size_t descr_pos = header_str.find("'descr':");
  if (descr_pos != std::string::npos) {
    size_t quote_start = header_str.find('\'', descr_pos + 8);
    size_t quote_end = header_str.find('\'', quote_start + 1);
    if (quote_start != std::string::npos && quote_end != std::string::npos) {
      dtype = header_str.substr(quote_start + 1, quote_end - quote_start - 1);
    }
  }

  // Parse shape from header string
  size_t shape_pos = header_str.find("'shape':");
  if (shape_pos == std::string::npos) {
    LOG(ERROR) << "Could not find shape in NPY header";
    return false;
  }

  size_t tuple_start = header_str.find('(', shape_pos);
  size_t tuple_end = header_str.find(')', tuple_start);
  if (tuple_start == std::string::npos || tuple_end == std::string::npos) {
    LOG(ERROR) << "Could not parse shape tuple";
    return false;
  }

  std::string shape_str = header_str.substr(tuple_start + 1, tuple_end - tuple_start - 1);
  std::stringstream ss(shape_str);
  std::string token;

  while (std::getline(ss, token, ',')) {
    // Trim whitespace
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (!token.empty()) {
      shape.push_back(std::stoull(token));
    }
  }

  data_offset = 10 + header.header_len;
  return true;
}

Eigen::MatrixXd NpyLoader::LoadAndInterpolateJointTrajectory(const std::string& npy_path,
                                                             const std::vector<std::string>& joint_names, double src_dt,
                                                             double target_dt, int start_frame, int end_frame) {
  std::stringstream file_stream;
  if (!common::FileReader::ReadFileToStream(npy_path, file_stream)) {
    LOG(ERROR) << "Failed to read NPY file: " << npy_path;
    throw std::runtime_error("Failed to read NPY file: " + npy_path);
  }

  // Parse NPY header
  std::vector<size_t> shape;
  size_t data_offset;
  std::string dtype;
  bool fortran_order;
  if (!ParseNpyHeader(file_stream, shape, data_offset, dtype, fortran_order)) {
    throw std::runtime_error("Failed to parse NPY header: " + npy_path);
  }

  LOG(INFO) << "NPY file dtype: " << dtype << ", shape: [" << shape[0] << ", " << shape[1]
            << "], fortran_order: " << (fortran_order ? "True" : "False");

  if (shape.size() != 2) {
    LOG(ERROR) << "Expected 2D array, got " << shape.size() << "D";
    throw std::runtime_error("Invalid NPY shape");
  }

  size_t num_frames = shape[0];
  size_t num_cols = shape[1];

  // NPY format: [base_pos(3), base_quat(4), joints(25), contacts(2 or 4), unused(0 or 4)]
  // Support multiple formats:
  // - 34 columns: base_pos(3) + base_quat(4) + joint_pos(25) + contacts(2)
  // - 36 columns: base_pos(3) + base_quat(4) + joint_pos(25) + contacts(4)
  // - 40 columns: base_pos(3) + base_quat(4) + joint_pos(25) + contacts(4) + unused(4)
  if (num_cols != 34 && num_cols != 36 && num_cols != 40) {
    LOG(ERROR) << "Expected 34, 36, or 40 columns in NPY file, got " << num_cols
               << ". Supported formats: 34 cols (2 contacts), 36 cols (4 contacts), or 40 cols (4 contacts + 4 unused)";
    return Eigen::MatrixXd();
  }

  // Read all data based on dtype
  std::vector<double> data(num_frames * num_cols);

  if (dtype == "<f2" || dtype == ">f2") {
    // Float16 format - need to read as uint16 and convert
    std::vector<uint16_t> raw_data(num_frames * num_cols);
    file_stream.read(reinterpret_cast<char*>(raw_data.data()), num_frames * num_cols * sizeof(uint16_t));

    // Convert float16 to float32/double
    // Float16 format: 1 sign bit, 5 exponent bits, 10 mantissa bits
    for (size_t i = 0; i < raw_data.size(); ++i) {
      uint16_t f16 = raw_data[i];

      // Debug first few values
      if (i >= 7 && i < 12) {
        LOG(INFO) << "[F16_CONVERT] data[" << i << "] raw_u16=0x" << std::hex << f16 << std::dec;
      }

      // Extract sign, exponent, mantissa
      uint32_t sign = (f16 >> 15) & 0x1;
      uint32_t exp = (f16 >> 10) & 0x1F;
      uint32_t mantissa = f16 & 0x3FF;

      // Convert to float32
      uint32_t f32;
      if (exp == 0) {
        if (mantissa == 0) {
          // Zero
          f32 = sign << 31;
        } else {
          // Denormalized number
          exp = 0;
          while ((mantissa & 0x400) == 0) {
            mantissa <<= 1;
            exp++;
          }
          mantissa &= 0x3FF;
          f32 = (sign << 31) | ((127 - 15 - exp) << 23) | (mantissa << 13);
        }
      } else if (exp == 0x1F) {
        // Inf or NaN
        f32 = (sign << 31) | (0xFF << 23) | (mantissa << 13);
      } else {
        // Normalized number
        f32 = (sign << 31) | ((exp - 15 + 127) << 23) | (mantissa << 13);
      }

      float f_val;
      std::memcpy(&f_val, &f32, sizeof(float));
      data[i] = static_cast<double>(f_val);
    }
  } else if (dtype == "<f4" || dtype == ">f4") {
    // Float32 format
    std::vector<float> raw_data(num_frames * num_cols);
    file_stream.read(reinterpret_cast<char*>(raw_data.data()), num_frames * num_cols * sizeof(float));
    for (size_t i = 0; i < raw_data.size(); ++i) {
      data[i] = static_cast<double>(raw_data[i]);
    }
  } else if (dtype == "<f8" || dtype == ">f8") {
    // Float64 format
    file_stream.read(reinterpret_cast<char*>(data.data()), num_frames * num_cols * sizeof(double));
  } else {
    LOG(ERROR) << "Unsupported NPY dtype: " << dtype;
    throw std::runtime_error("Unsupported NPY dtype: " + dtype);
  }

  // Extract joint positions (columns 7-31, 25 joints)
  // Joint order in NPY matches the expected joint_names order
  const size_t joint_start_col = 7;
  const size_t num_joints_in_npy = 25;

  if (joint_names.size() != num_joints_in_npy) {
    LOG(ERROR) << "Joint names size mismatch: expected " << num_joints_in_npy << ", got " << joint_names.size();
    throw std::runtime_error("Joint names size mismatch");
  }

  // Build trajectory matrix [num_frames, num_joints]
  std::vector<std::vector<double>> raw_traj(num_frames, std::vector<double>(num_joints_in_npy));
  for (size_t i = 0; i < num_frames; ++i) {
    for (size_t j = 0; j < num_joints_in_npy; ++j) {
      size_t col_idx = joint_start_col + j;
      size_t data_idx;
      if (fortran_order) {
        // Fortran order (column-major): index = col * num_rows + row
        data_idx = col_idx * num_frames + i;
      } else {
        // C order (row-major): index = row * num_cols + col
        data_idx = i * num_cols + col_idx;
      }
      raw_traj[i][j] = data[data_idx];
    }
  }

  // Debug: verify raw extraction
  if (num_frames > 0) {
    LOG(INFO) << "[NPY_EXTRACT] Raw frame 0 [0-4]: [" << raw_traj[0][0] << ", " << raw_traj[0][1] << ", "
              << raw_traj[0][2] << ", " << raw_traj[0][3] << ", " << raw_traj[0][4] << "]";
  }

  // Handle frame clipping with negative index support
  int start_idx, end_idx;

  if (start_frame < 0) {
    start_idx = std::max(0, (int)num_frames + start_frame);
  } else {
    start_idx = std::min((int)num_frames - 1, start_frame);
  }

  if (end_frame < 0) {
    end_idx = std::max(0, (int)num_frames + end_frame);
  } else {
    end_idx = std::min((int)num_frames - 1, end_frame);
  }

  if (start_idx > end_idx) {
    throw std::runtime_error("Invalid traj_frame range: start > end");
  }

  std::vector<std::vector<double>> clipped_traj(raw_traj.begin() + start_idx, raw_traj.begin() + end_idx + 1);

  // Debug: verify clipping
  LOG(INFO) << "[NPY_CLIP] Clipping from " << start_idx << " to " << end_idx << ", clipped_N=" << clipped_traj.size();
  if (!clipped_traj.empty()) {
    LOG(INFO) << "[NPY_CLIP] Clipped frame 0 [0-4]: [" << clipped_traj[0][0] << ", " << clipped_traj[0][1] << ", "
              << clipped_traj[0][2] << ", " << clipped_traj[0][3] << ", " << clipped_traj[0][4] << "]";
  }

  // Interpolation
  size_t clipped_N = clipped_traj.size();
  double total_time = (clipped_N - 1) * src_dt;
  size_t target_N = static_cast<size_t>(std::round(total_time / target_dt)) + 1;
  Eigen::MatrixXd interp_traj(target_N, num_joints_in_npy);

  LOG(INFO) << "[NPY_INTERP] Interpolating from " << clipped_N << " frames to " << target_N << " frames";

  for (size_t j = 0; j < num_joints_in_npy; ++j) {
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

  // Debug: verify interpolation result
  if (target_N > 0) {
    LOG(INFO) << "[NPY_INTERP] Result frame 0 [0-4]: [" << interp_traj(0, 0) << ", " << interp_traj(0, 1) << ", "
              << interp_traj(0, 2) << ", " << interp_traj(0, 3) << ", " << interp_traj(0, 4) << "]";
  }

  return interp_traj;
}

Eigen::MatrixXd NpyLoader::LoadProfileTrajectory(const data::MotionStateProfile& profile,
                                                 const data::RlMinicTrajectoryParam& param, double runner_period) {
  std::string npy_path = common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), profile.data_path);
  const auto& joint_names = param.active_joint_names;
  double src_dt = profile.csv_dt;
  double target_dt = runner_period;

  int start_frame = 0, end_frame = -1;
  if (!profile.traj_frame.empty()) {
    start_frame = profile.traj_frame[0];
    end_frame = profile.traj_frame.size() > 1 ? profile.traj_frame[1] : start_frame;
  }

  Eigen::MatrixXd interp_traj =
      LoadAndInterpolateJointTrajectory(npy_path, joint_names, src_dt, target_dt, start_frame, end_frame);

  // Debug: verify loaded data
  if (interp_traj.rows() > 0) {
    LOG(INFO) << "[NPY_DEBUG] Loaded trajectory shape: [" << interp_traj.rows() << ", " << interp_traj.cols() << "]";
    LOG(INFO) << "[NPY_DEBUG] First frame [0-4]: [" << interp_traj.row(0).head(5).transpose() << "]";
    if (interp_traj.rows() > 1) {
      LOG(INFO) << "[NPY_DEBUG] Second frame [0-4]: [" << interp_traj.row(1).head(5).transpose() << "]";
    }
  }

  return interp_traj;
}

}  // namespace rl_mimic_trajectory
