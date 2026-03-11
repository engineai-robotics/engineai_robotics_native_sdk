#pragma once

#include <Eigen/Dense>

namespace model {

class ContactWrenchCone {
 public:
  ContactWrenchCone() = default;
  ~ContactWrenchCone() = default;

  Eigen::MatrixXd cone() const { return cone_; }
  Eigen::VectorXd upper_bounds() const { return upper_bounds_; }

 protected:
  Eigen::MatrixXd cone_;
  Eigen::VectorXd upper_bounds_;
};

class PointContact : public ContactWrenchCone {
 public:
  static constexpr int kNumWrenches = 3;
  static constexpr int kNumConstraints = 6;

  PointContact(double friction_coefficient, double fz_lb, double fz_ub) {
    double mu = friction_coefficient;
    cone_ = Eigen::Matrix<double, kNumConstraints, kNumWrenches>{{-1, 0, -mu}, {1, 0, -mu}, {0, -1, -mu},
                                                                 {0, 1, -mu},  {0, 0, -1},  {0, 0, 1}};
    upper_bounds_ = Eigen::Vector<double, kNumConstraints>{0, 0, 0, 0, -fz_lb, fz_ub};
  }
};

class LineContact : public ContactWrenchCone {
 public:
  static constexpr int kNumWrenches = 5;
  static constexpr int kNumConstraints = 10;

  // Contact wrench is defined as [[fx, fy, fz]ᵂ, [my, mz]ᴸ]ᵀ, where [fx, fy, fz]ᵂ is in the world frame, [my, mz]ᴸ is
  // in the local contact frame.
  LineContact(double toe_length, double heel_length, double friction_coefficient, double fz_lb, double fz_ub) {
    double mu = friction_coefficient;
    double lt = toe_length;
    double lh = heel_length;
    cone_ = Eigen::Matrix<double, kNumConstraints, kNumWrenches>{
        {-1, 0, -mu, 0, 0},       {1, 0, -mu, 0, 0},         {0, -1, -mu, 0, 0},         {0, 1, -mu, 0, 0},
        {0, 0, -1, 0, 0},         {0, 0, 1, 0, 0},           {0, lt, -mu * lt, -mu, -1}, {0, -lt, -mu * lt, -mu, 1},
        {0, lh, -mu * lh, mu, 1}, {0, -lh, -mu * lh, mu, -1}};
    upper_bounds_ = Eigen::Vector<double, kNumConstraints>{0, 0, 0, 0, -fz_lb, fz_ub, 0, 0, 0, 0};
  }
};

class RectangularSurfaceContact : public ContactWrenchCone {
 public:
  static constexpr int kNumWrenches = 6;
  static constexpr int kNumConstraints = 18;

  RectangularSurfaceContact(double half_length, double half_width, double friction_coefficient, double fz_lb,
                            double fz_ub) {
    double mu = friction_coefficient;
    double X = half_length;
    double Y = half_width;
    cone_ = Eigen::Matrix<double, kNumConstraints, kNumWrenches>{{-1, 0, -mu, 0, 0, 0},
                                                                 {+1, 0, -mu, 0, 0, 0},
                                                                 {0, -1, -mu, 0, 0, 0},
                                                                 {0, +1, -mu, 0, 0, 0},
                                                                 {0, 0, -Y, -1, 0, 0},
                                                                 {0, 0, -Y, +1, 0, 0},
                                                                 {0, 0, -X, 0, -1, 0},
                                                                 {0, 0, -X, 0, +1, 0},
                                                                 {0, 0, -1, 0, 0, 0},
                                                                 {0, 0, 1, 0, 0, 0},
                                                                 {-Y, -X, -(X + Y) * mu, +mu, +mu, -1},
                                                                 {-Y, +X, -(X + Y) * mu, +mu, -mu, -1},
                                                                 {+Y, -X, -(X + Y) * mu, -mu, +mu, -1},
                                                                 {+Y, +X, -(X + Y) * mu, -mu, -mu, -1},
                                                                 {+Y, +X, -(X + Y) * mu, +mu, +mu, +1},
                                                                 {+Y, -X, -(X + Y) * mu, +mu, -mu, +1},
                                                                 {-Y, +X, -(X + Y) * mu, -mu, +mu, +1},
                                                                 {-Y, -X, -(X + Y) * mu, -mu, -mu, +1}};
    upper_bounds_ =
        Eigen::Vector<double, kNumConstraints>{0, 0, 0, 0, 0, 0, 0, 0, -fz_lb, fz_ub, 0, 0, 0, 0, 0, 0, 0, 0};
  }
};
}  // namespace model
