#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "contact_info/contact_info.h"
#include "model/contact_wrench_cone.h"
#include "model/pinocchio_interface.h"

namespace model {

class Constraint {
 public:
  Constraint(const std::shared_ptr<model::PinocchioInterface>& pinocchio_interface,
             const data::ContactInfo& contact_info)
      : pinocchio_interface_(pinocchio_interface), contact_info_(contact_info) {}
  virtual ~Constraint() = default;

  virtual void UpdateJacobian() = 0;
  virtual void UpdateJacobianProduct() = 0;
  virtual void UpdateConstraints() = 0;

  void SetJacobianRowIndex(int index) { jacobian_row_index_ = index; }
  void SetConstraintsRowIndex(int index) { constraints_row_index_ = index; }
  void SetConstraintsColumnIndex(int index) { constraints_column_index_ = index; }

  Eigen::MatrixXd jacobian() const { return jacobian_; }
  Eigen::MatrixXd constraints() const { return constraints_; }

  Eigen::VectorXd jacobian_product() const { return jacobian_product_; }
  Eigen::VectorXd lower_bounds() const { return lower_bounds_; }
  Eigen::VectorXd upper_bounds() const { return upper_bounds_; }

  int jacobian_row_index() const { return jacobian_row_index_; }
  int constraints_row_index() const { return constraints_row_index_; }
  int constraints_column_index() const { return constraints_column_index_; }
  int num_wrenches() const { return num_wrenches_; }
  int num_constraints() const { return num_constraints_; }

 protected:
  std::shared_ptr<model::PinocchioInterface> pinocchio_interface_;
  data::ContactInfo contact_info_;

  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd constraints_;

  Eigen::VectorXd jacobian_product_;
  Eigen::VectorXd lower_bounds_;
  Eigen::VectorXd upper_bounds_;

  int num_wrenches_ = 0;
  int num_constraints_ = 0;
  int jacobian_row_index_ = 0;
  int constraints_row_index_ = 0;
  int constraints_column_index_ = 0;
};

// The point contact constraint calculates jacobian matrix and in-equalities for contact wrench constraints.
// Contact wrench is defined as [[fx, fy, fz]ᵂ]ᵀ, where [fx, fy, fz]ᵂ is in the world frame.
// Contact vertices are defined in the local frame as:
//  0       ^x
//  |       |
//  |   y<--o
//  1
class PointContactWrenchConstraint : public Constraint {
 public:
  PointContactWrenchConstraint(const std::shared_ptr<model::PinocchioInterface>& pinocchio_interface,
                               const data::ContactInfo& contact_info);
  ~PointContactWrenchConstraint() = default;

  void UpdateJacobian() final;
  void UpdateJacobianProduct() final;
  void UpdateConstraints() final;

 private:
  std::unique_ptr<PointContact> point_contact_;

  std::string frame_;
  Eigen::Matrix3Xd jacobian_pos_world_;
  Eigen::Vector3d jac_product_pos_world_;
  Eigen::Matrix3d rotation_local_to_world_;
};

// The line contact constraint calculates jacobian matrix and in-equalities for contact wrench constraints.
// Contact wrench is defined as [[fx, fy, fz]ᵂ, [my, mz]ᴸ]ᵀ, where [fx, fy, fz]ᵂ is in the world frame, [my, mz]ᴸ is in
// the local contact frame. The reason for this defintion is that the moment along roll axis in local contact frame
// should be zero, thus removing mxᴸ from contact wrench simplifies the constraints.
class LineContactWrenchConstraint : public Constraint {
 public:
  LineContactWrenchConstraint(const std::shared_ptr<model::PinocchioInterface>& pinocchio_interface,
                              const data::ContactInfo& contact_info);
  ~LineContactWrenchConstraint() = default;

  void UpdateJacobian() final;
  void UpdateJacobianProduct() final;
  void UpdateConstraints() final;

 private:
  static constexpr int kForceDims = 3;
  static constexpr int kMomentDims = 2;
  std::unique_ptr<LineContact> line_contact_;

  Eigen::Matrix3Xd jacobian_pos_world_;
  Eigen::Matrix3Xd jacobian_rot_world_;
  Eigen::Vector3d jac_product_pos_world_;
  Eigen::Vector3d jac_product_rot_world_;
  Eigen::Matrix3d rotation_local_to_world_;
};

// The rectangular surface contact constraint calculates jacobian matrix and in-equalities for contact wrench
// constraints. Contact wrench is defined as [[fx, fy, fz, mx, my, mz]ᵂ]ᵀ, where [fx, fy, fz, mx, my, mz]ᵂ is in the
// world frame. Contact vertices are defined in the local frame as:
//  0 --- 1      ^x
//  |     |      |
//  |     |  y<--o
//  3 --- 2
class RectangleContactWrenchConstraint : public Constraint {
 public:
  RectangleContactWrenchConstraint(const std::shared_ptr<model::PinocchioInterface>& pinocchio_interface,
                                   const data::ContactInfo& contact_info);
  ~RectangleContactWrenchConstraint() = default;

  void UpdateJacobian() final;
  void UpdateJacobianProduct() final;
  void UpdateConstraints() final;

 private:
  static constexpr int kForceDims = 3;
  static constexpr int kMomentDims = 3;
  std::unique_ptr<RectangularSurfaceContact> rectangular_surface_contact_;

  Eigen::Matrix3d rotation_local_to_world_;
};

class ContactWrenchConstraint {
 public:
  ContactWrenchConstraint(const std::shared_ptr<model::PinocchioInterface>& pinocchio_interface,
                          const std::vector<data::ContactInfo>& contact_info);
  ~ContactWrenchConstraint() = default;

  void UpdateContactWrenchList(const std::vector<std::string>& contact_wrench_list);
  void UpdateContactWrenchJacobian();
  void UpdateContactWrenchConstraints();

  Eigen::MatrixXd GetContactWrenchJacobian() const { return jacobian_; }
  Eigen::VectorXd GetContactWrenchJacobianProduct() const { return jacobian_product_; }
  int GetWrenchDimensions() const { return num_wrenches_; }

  Eigen::MatrixXd GetContactWrenchConstraints() const { return constraints_; }
  Eigen::VectorXd GetContactWrenchConstraintsLowerBounds() const { return lower_bounds_; }
  Eigen::VectorXd GetContactWrenchConstraintsUpperBounds() const { return upper_bounds_; }
  int GetConstraintsDimensions() const { return num_constraints_; }

 private:
  void UpdateContactWrenchList();
  bool Construct(const std::shared_ptr<model::PinocchioInterface>& pinocchio_interface,
                 const std::vector<data::ContactInfo>& contact_info);

  std::shared_ptr<model::PinocchioInterface> pinocchio_interface_;
  std::unordered_map<std::string, std::shared_ptr<Constraint>> constraint_list_;
  std::vector<std::string> contact_wrench_list_;

  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd constraints_;

  Eigen::VectorXd jacobian_product_;
  Eigen::VectorXd lower_bounds_;
  Eigen::VectorXd upper_bounds_;

  int num_wrenches_ = 0;
  int num_constraints_ = 0;
};
}  // namespace model
