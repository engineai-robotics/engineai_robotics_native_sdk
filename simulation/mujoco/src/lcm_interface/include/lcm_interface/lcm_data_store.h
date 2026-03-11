#ifndef LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_DATA_STORE_H_
#define LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_DATA_STORE_H_

#include <Eigen/Dense>
#include <vector>

#include "guarded_data/guarded_data.h"

class SimCommand {
 public:
  SimCommand() = default;
  SimCommand(int num_joints) { Reset(num_joints); }
  ~SimCommand() = default;

  void Reset(int num_joints) {
    q = Eigen::VectorXd::Zero(num_joints);
    qd = Eigen::VectorXd::Zero(num_joints);
    tau_ff = Eigen::VectorXd::Zero(num_joints);
    kp = Eigen::VectorXd::Zero(num_joints);
    kd = Eigen::VectorXd::Zero(num_joints);
  }

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd tau_ff;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};

class SimState {
 public:
  SimState() = default;
  SimState(int num_joints, int num_contacts, int num_single_contact_dimensions) {
    Reset(num_joints, num_contacts, num_single_contact_dimensions);
  }
  ~SimState() = default;

  void Reset(int num_joints, int num_contacts, int num_single_contact_dimensions) {
    q = Eigen::VectorXd::Zero(num_joints);
    qd = Eigen::VectorXd::Zero(num_joints);
    tau = Eigen::VectorXd::Zero(num_joints);
    contact_force = std::vector<Eigen::VectorXd>(num_contacts, Eigen::VectorXd::Zero(num_single_contact_dimensions));

    imu_sensor_quaternion = Eigen::Quaterniond::Identity();
    imu_sensor_linear_acceleration = Eigen::Vector3d::Zero();
    imu_sensor_angular_velocity = Eigen::Vector3d::Zero();

    imu_link_quaternion = Eigen::Quaterniond::Identity();
    imu_link_position = Eigen::Vector3d::Zero();
    imu_link_linear_velocity = Eigen::Vector3d::Zero();
    imu_link_angular_velocity = Eigen::Vector3d::Zero();

    base_link_quaternion = Eigen::Quaterniond::Identity();
    base_link_position = Eigen::Vector3d::Zero();
    base_link_linear_velocity = Eigen::Vector3d::Zero();
    base_link_angular_velocity = Eigen::Vector3d::Zero();
  }

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd tau;

  Eigen::Quaterniond imu_sensor_quaternion;
  Eigen::Vector3d imu_sensor_linear_acceleration;
  Eigen::Vector3d imu_sensor_angular_velocity;

  Eigen::Quaterniond imu_link_quaternion;
  Eigen::Vector3d imu_link_position;
  Eigen::Vector3d imu_link_linear_velocity;
  Eigen::Vector3d imu_link_angular_velocity;

  Eigen::Quaterniond base_link_quaternion;
  Eigen::Vector3d base_link_position;
  Eigen::Vector3d base_link_linear_velocity;
  Eigen::Vector3d base_link_angular_velocity;

  std::vector<Eigen::VectorXd> contact_force;
};

class LcmDataStore {
 public:
  LcmDataStore(int num_joints, int num_contacts, int num_single_contact_dimensions)
      : num_joints(num_joints),
        num_contacts(num_contacts),
        num_single_contact_dimensions(num_single_contact_dimensions) {
    sim_command.Set(SimCommand(num_joints));
    sim_state.Set(SimState(num_joints, num_contacts, num_single_contact_dimensions));
  }

  data::GuardedData<SimCommand> sim_command;
  data::GuardedData<SimState> sim_state;

  int num_joints = 0;
  int num_contacts = 0;
  int num_single_contact_dimensions = 0;
};

#endif  // LCM_INTERFACE_INCLUDE_LCM_INTERFACE_LCM_DATA_STORE_H_
