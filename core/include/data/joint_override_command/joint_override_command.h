#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace data {
class JointOverrideCommand {
 public:
  JointOverrideCommand() { Reset(); }
  ~JointOverrideCommand() = default;

  void Reset() {
    weight = 0.0;
    joint_indices.setZero();
    position.setZero();
    velocity.setZero();
    feed_forward_torque.setZero();
    torque.setZero();
    stiffness.setZero();
    damping.setZero();
  }

  bool IsEnable() const noexcept { return std::fpclassify(weight) != FP_ZERO; }

  double weight;
  Eigen::VectorXi joint_indices;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd feed_forward_torque;
  Eigen::VectorXd torque;
  Eigen::VectorXd stiffness;
  Eigen::VectorXd damping;
};
}  // namespace data
