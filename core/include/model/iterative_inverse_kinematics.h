#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "iterative_inverse_kinematics_param/iterative_inverse_kinematics_param.h"
#include "model/pinocchio_interface.h"
#include "model_param/model_param.h"

namespace model {

class IterativeInverseKinematics {
 public:
  IterativeInverseKinematics(const std::shared_ptr<data::ModelParam>& model_param);
  ~IterativeInverseKinematics() = default;

  // Calculates the joint angles for the end effector frame in the base frame by jacobian-based iterative algorithm
  bool CalculateByPoseInBase(const std::string& end_effector_frame, const Eigen::Vector3d& position,
                             const Eigen::Quaterniond& quaternion, const Eigen::VectorXd& q_init,
                             Eigen::VectorXd& q_ik);

 private:
  std::shared_ptr<data::ModelParam> model_param_;
  std::shared_ptr<data::IterativeInverseKinematicsParam> iterative_ik_param_;

  std::unique_ptr<model::PinocchioInterface> fixed_base_model_interface_;
};
}  // namespace model
