#ifndef DATA_IMU_INFO_IMU_INFO_H_
#define DATA_IMU_INFO_IMU_INFO_H_

#include <Eigen/Dense>
#include <atomic>
#include "guarded_data/guarded_data.h"

namespace data {

class ImuInfo {
 public:
  ImuInfo() { Reset(); }
  ~ImuInfo() = default;

  void Reset() {
    quaternion = Eigen::Quaterniond::Identity();
    rpy = Eigen::Vector3d::Zero();
    linear_acceleration = Eigen::Vector3d::Zero();
    angular_velocity = Eigen::Vector3d::Zero();
  }

  Eigen::Quaterniond quaternion;
  Eigen::Vector3d rpy;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};
}  // namespace data

#endif  // DATA_IMU_INFO_IMU_INFO_H_
