#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "rl_mimic_trajectory_example_param/rl_mimic_trajectory_example_param.h"

namespace rl_mimic_trajectory {

class NpyLoader {
 public:
  // Load NPY file and interpolate, returns [N, num_joints] matrix
  static Eigen::MatrixXd LoadAndInterpolateJointTrajectory(const std::string& npy_path,
                                                           const std::vector<std::string>& joint_names, double src_dt,
                                                           double target_dt, int start_frame, int end_frame);

  static Eigen::MatrixXd LoadProfileTrajectory(const data::MotionStateProfile& profile,
                                               const data::RlMinicTrajectoryParam& param, double runner_period = 0.02);
};

}  // namespace rl_mimic_trajectory
