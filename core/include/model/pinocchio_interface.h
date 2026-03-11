#pragma once

#include <memory>
#include <string>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace model {

enum class FloatingBaseType {
  kFixed = 0,
  kFreeByQuaternion,
  kFreeByEulerZyx,
  kPlanarBySagittal,
  kXyzPitch,
};

static const std::unordered_map<std::string, FloatingBaseType> floating_base_type_map = {
    {"fixed", FloatingBaseType::kFixed},
    {"free_by_quaternion", FloatingBaseType::kFreeByQuaternion},
    {"free_by_euler_zyx", FloatingBaseType::kFreeByEulerZyx},
    {"planar_by_sagittal", FloatingBaseType::kPlanarBySagittal},
    {"xyz_pitch", FloatingBaseType::kXyzPitch},
};

class PinocchioInterface {
 public:
  PinocchioInterface() = default;
  PinocchioInterface(const std::string& urdf_file_path, FloatingBaseType floating_base_type, bool verbose = false);
  ~PinocchioInterface() = default;

  bool ReduceModel(const std::vector<std::string>& reduced_joint_names, const Eigen::VectorXd& q);
  bool IsModelReduced() const { return is_reduced_; }
  const Eigen::VectorXi& GetReducedQIndices() { return reduced_q_indices_; }
  const Eigen::VectorXi& GetReducedQdIndices() { return reduced_qd_indices_; }
  const Eigen::VectorXi& GetReducedIndicesInJointPosition() { return reduced_indices_in_joint_position_; }
  const Eigen::VectorXi& GetReducedIndicesInJointVelocity() { return reduced_indices_in_joint_velocity_; }
  const Eigen::VectorXi& GetRemainingQIndices() { return remaining_q_indices_; }
  const Eigen::VectorXi& GetRemainingQdIndices() { return remaining_qd_indices_; }
  const Eigen::VectorXi& GetRemainingIndicesInJointPosition() { return remaining_indices_in_joint_position_; }
  const Eigen::VectorXi& GetRemainingIndicesInJointVelocity() { return remaining_indices_in_joint_velocity_; }

  int AddFrameAtRunTime(const std::string& added_frame, const std::string& parent_frame,
                        const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
  void ResetData();

  bool ValidFrame(const std::string& frame_name) const;
  bool ValidJointName(const std::string& joint_name) const;
  int GetFrameId(const std::string& frame_name) const;
  int GetJointId(const std::string& joint_name) const;
  int GetJointIndexInQ(const std::string& joint_name) const;
  int GetJointIndexInQd(const std::string& joint_name) const;
  int GetQDimensions() const { return model_.nq; }
  int GetQdDimensions() const { return model_.nv; }
  int GetQddDimensions() const { return model_.nv; }
  int GetJointDimensions() const { return num_joint_dimensions_; }
  FloatingBaseType GetFloatingBaseType() const { return floating_base_type_; }

  void SetQ(const Eigen::VectorXd& q);
  void SetQd(const Eigen::VectorXd& qd);
  void SetQdd(const Eigen::VectorXd& qdd);
  void SetJointPosition(const Eigen::VectorXd& q_joint);
  void SetJointVelocity(const Eigen::VectorXd& qd_joint);
  void SetJointAcceleration(const Eigen::VectorXd& qdd_joint);
  void SetJointPosition(const std::string& joint_name, double position);
  void SetJointVelocity(const std::string& joint_name, double velocity);
  void SetJointAcceleration(const std::string& joint_name, double acceleration);
  void SetBasePosition(const Eigen::Vector3d& position);
  void SetBaseRotation(const Eigen::Quaterniond& quaternion);
  void SetBaseRotationEulerZyx(const Eigen::Vector3d& euler_zyx);
  void SetBaseLinearVelocity(const Eigen::Vector3d& linear_velocity);
  void SetBaseAngularVelocity(const Eigen::Vector3d& angular_velocity);
  void SetBaseLinearAcceleration(const Eigen::Vector3d& linear_acceleration);
  void SetBaseAngularAcceleration(const Eigen::Vector3d& angular_acceleration);
  void SetLinkInertia(const std::string& joint_name, double mass, const Eigen::Vector3d& com_position,
                      const Eigen::Matrix3d& inertia);
  void SetBaseInertia(double mass, const Eigen::Vector3d& com_position, const Eigen::Matrix3d& inertia);

  const Eigen::VectorXd& GetQ() const { return q_; }
  const Eigen::VectorXd& GetQd() const { return qd_; }
  const Eigen::VectorXd& GetQdd() const { return qdd_; }
  Eigen::VectorXd& GetJointPosition(Eigen::VectorXd& q_joint) const;
  Eigen::VectorXd& GetJointVelocity(Eigen::VectorXd& qd_joint) const;
  Eigen::VectorXd& GetJointAcceleration(Eigen::VectorXd& qdd_joint) const;
  double GetJointPosition(const std::string& joint_name) const { return q_(GetJointIndexInQ(joint_name)); }
  double GetJointVelocity(const std::string& joint_name) const { return qd_(GetJointIndexInQ(joint_name)); }
  double GetJointAcceleration(const std::string& joint_name) const { return qdd_(GetJointIndexInQ(joint_name)); }
  Eigen::VectorXd& GetJointPositionLowerLimit(Eigen::VectorXd& limit) const;
  Eigen::VectorXd& GetJointPositionUpperLimit(Eigen::VectorXd& limit) const;
  Eigen::VectorXd& GetJointVelocityLimit(Eigen::VectorXd& limit) const;
  Eigen::VectorXd& GetJointTorqueLimit(Eigen::VectorXd& limit) const;

  Eigen::Vector3d& GetBasePosition(Eigen::Vector3d& position) const;
  Eigen::Quaterniond& GetBaseRotation(Eigen::Quaterniond& quaternion) const;
  Eigen::Vector3d& GetBaseRotationEulerZyx(Eigen::Vector3d& euler_zyx) const;
  Eigen::Vector3d& GetBaseLinearVelocity(Eigen::Vector3d& linear_velocity) const;
  Eigen::Vector3d& GetBaseAngularVelocity(Eigen::Vector3d& angular_velocity) const;
  Eigen::Vector3d& GetBaseLinearAcceleration(Eigen::Vector3d& linear_acceleration) const;
  Eigen::Vector3d& GetBaseAngularAcceleration(Eigen::Vector3d& angular_acceleration) const;

  Eigen::Vector3d& GetFramePosition(const std::string& frame_name, Eigen::Vector3d& position) const;
  Eigen::Matrix3d& GetFrameRotation(const std::string& frame_name, Eigen::Matrix3d& rotation) const;
  Eigen::Quaterniond& GetFrameRotation(const std::string& frame_name, Eigen::Quaterniond& quaternion) const;
  Eigen::Vector3d& GetFrameLinearVelocity(const std::string& frame_name, Eigen::Vector3d& linear_velocity) const;
  Eigen::Vector3d& GetFrameAngularVelocity(const std::string& frame_name, Eigen::Vector3d& angular_velocity) const;
  Eigen::Vector3d& GetFrameLinearAcceleration(const std::string& frame_name, Eigen::Vector3d& linear_acc) const;
  Eigen::Vector3d& GetFrameAngularAcceleration(const std::string& frame_name, Eigen::Vector3d& angular_acc) const;
  Eigen::Matrix3Xd& GetFramePositionJacobian(const std::string& frame_name, Eigen::Matrix3Xd& jacobian);
  Eigen::Matrix3Xd& GetFrameRotationJacobian(const std::string& frame_name, Eigen::Matrix3Xd& jacobian);
  Eigen::MatrixXd& GetFrameFullJacobian(const std::string& frame_name, Eigen::MatrixXd& frame_jacobian);
  Eigen::Matrix3Xd& GetFramePositionJacobianTimeVariation(const std::string& frame_name, Eigen::Matrix3Xd& val);
  Eigen::Matrix3Xd& GetFrameRotationJacobianTimeVariation(const std::string& frame_name, Eigen::Matrix3Xd& val);
  Eigen::MatrixXd& GetFrameFullJacobianTimeVariation(const std::string& frame_name, Eigen::MatrixXd& val);
  Eigen::Vector3d& GetFramePositionJacobianDotTimesQd(const std::string& frame_name, Eigen::Vector3d& val);
  Eigen::Vector3d& GetFrameRotationJacobianDotTimesQd(const std::string& frame_name, Eigen::Vector3d& val);
  Eigen::VectorXd& GetFrameFullJacobianDotTimesQd(const std::string& frame_name, Eigen::VectorXd& val);

  Eigen::MatrixXd& GetInertiaMatrix(Eigen::MatrixXd& inertia_matrix) const;
  Eigen::VectorXd& GetNonlinearEffect(Eigen::VectorXd& none_linear) const;
  Eigen::Vector3d& GetComPositionInWorld(Eigen::Vector3d& com_position) const;
  Eigen::Vector3d& GetComPositionInBase(Eigen::Vector3d& com_position) const;
  Eigen::Vector3d& GetComVelocityInWorld(Eigen::Vector3d& com_velocity) const;
  Eigen::Vector3d& GetComVelocityInBase(Eigen::Vector3d& com_velocity) const;
  Eigen::Vector3d& GetComAccelerationInWorld(Eigen::Vector3d& com_acceleration) const;
  Eigen::Vector3d& GetComAccelerationInBase(Eigen::Vector3d& com_acceleration) const;
  Eigen::Matrix3Xd& GetComJacobianInWorld(Eigen::Matrix3Xd& com_jacobian);
  Eigen::Vector3d& GetComJacobianDotTimesQdInWorld(Eigen::Vector3d& com_jacobian_dot_times_qd);
  Eigen::VectorXd& GetCentroidalMomentum(Eigen::VectorXd& centroidal_momentum) const;
  Eigen::VectorXd& GetCentroidalMomentumTimeVariation(Eigen::VectorXd& val) const;
  Eigen::MatrixXd& GetCentroidalMomentumMatrix(Eigen::MatrixXd& val) const;
  Eigen::MatrixXd& GetCentroidalMomentumMatrixTimeVariation(Eigen::MatrixXd& val) const;

  double GetTotalMass() const;
  void GetLinkInertia(const std::string& joint_name, double& mass, Eigen::Vector3d& com_position,
                      Eigen::Matrix3d& inertia) const;
  void GetBaseInertia(double& mass, Eigen::Vector3d& com_position, Eigen::Matrix3d& inertia) const;

  Eigen::VectorXd& Integrate(double integrated_time, Eigen::VectorXd& q_integrate);
  Eigen::VectorXd& Integrate(double integrated_time, const Eigen::VectorXd& qd, Eigen::VectorXd& q_integrate);
  Eigen::VectorXd& UpdateInverseDynamics(Eigen::VectorXd& tau);
  Eigen::VectorXd& UpdateInverseDynamics(const Eigen::VectorXd& qdd, Eigen::VectorXd& tau);

  void UpdateKinematics();
  void UpdateDynamics();
  void UpdateKinematicsDynamics();

  // Interfaces for contact wrench constraint
  Eigen::MatrixXd& contact_wrench_jacobian() { return contact_wrench_jacobian_; }
  Eigen::VectorXd& contact_wrench_jacobian_product() { return contact_wrench_jacobian_product_; }
  Eigen::MatrixXd& contact_wrench_constraints() { return contact_wrench_constraints_; }
  Eigen::VectorXd& contact_wrench_constraints_lower_bounds() { return contact_wrench_constraints_lower_bounds_; }
  Eigen::VectorXd& contact_wrench_constraints_upper_bounds() { return contact_wrench_constraints_upper_bounds_; }

 private:
  bool CreateModelFromUrdf(const std::string& urdf_file_path, FloatingBaseType floating_base_type, bool verbose);

  static constexpr double kEpsilon = 1e-6;

  pinocchio::Model model_;
  pinocchio::DataTpl<double> data_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd qdd_;
  Eigen::MatrixXd contact_wrench_jacobian_;
  Eigen::VectorXd contact_wrench_jacobian_product_;
  Eigen::MatrixXd contact_wrench_constraints_;
  Eigen::VectorXd contact_wrench_constraints_lower_bounds_;
  Eigen::VectorXd contact_wrench_constraints_upper_bounds_;

  int num_joint_dimensions_ = 0;
  int num_floating_base_dimensions_in_q_ = 0;
  int num_floating_base_dimensions_in_qd_ = 0;
  FloatingBaseType floating_base_type_;

  bool is_reduced_ = false;
  std::vector<pinocchio::JointIndex> reduced_joint_ids_;
  Eigen::VectorXi reduced_q_indices_;
  Eigen::VectorXi reduced_qd_indices_;
  Eigen::VectorXi reduced_indices_in_joint_position_;
  Eigen::VectorXi reduced_indices_in_joint_velocity_;
  Eigen::VectorXi remaining_q_indices_;
  Eigen::VectorXi remaining_qd_indices_;
  Eigen::VectorXi remaining_indices_in_joint_position_;
  Eigen::VectorXi remaining_indices_in_joint_velocity_;
};
}  // namespace model
