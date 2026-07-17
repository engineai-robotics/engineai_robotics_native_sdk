#pragma once

#include <Eigen/Dense>
#include <vector>

namespace data {

struct SE3Pose {
  SE3Pose() { Zero(); }
  SE3Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& quaternion)
      : position(position), quaternion(quaternion) {}
  void Zero() {
    position.setZero();
    quaternion = Eigen::Quaterniond::Identity();
  }

  Eigen::Vector3d position;
  Eigen::Quaterniond quaternion;
};

struct SE3Velocity {
  SE3Velocity() { Zero(); }
  SE3Velocity(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) : linear(linear), angular(angular) {}
  void Zero() {
    linear.setZero();
    angular.setZero();
  }

  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct SE3Frame {
  using SE3Acceleration = SE3Velocity;
  SE3Frame(const SE3Pose& pose, const SE3Velocity& twist, const SE3Acceleration& acceleration)
      : pose(pose), twist(twist), acceleration(acceleration) {}
  SE3Frame() { Zero(); }
  void Zero() {
    pose.Zero();
    twist.Zero();
    acceleration.Zero();
  }

  SE3Pose pose;
  SE3Velocity twist;
  SE3Acceleration acceleration;
};

struct SE3Point {
  SE3Point() { Zero(); }
  SE3Point(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration)
      : position(position), velocity(velocity), acceleration(acceleration) {}
  void Zero() {
    position.setZero();
    velocity.setZero();
    acceleration.setZero();
  }

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
};

struct SE3PointTrajectory {
  SE3PointTrajectory() { Zero(); }

  void Zero() {
    times.clear();
    points.clear();
  }

  std::vector<double> times;
  std::vector<SE3Point> points;
};

struct SE3Wrench {
  SE3Wrench() { Zero(); }
  SE3Wrench(const Eigen::Vector3d& force, const Eigen::Vector3d& torque) : force(force), torque(torque) {}
  void Zero() {
    force.setZero();
    torque.setZero();
  }

  Eigen::Vector3d force;
  Eigen::Vector3d torque;
};

struct JointPoint {
  JointPoint() = default;
  JointPoint(int size)
      : position(size), velocity(size), acceleration(size), feed_forward_torque(size), stiffness(size), damping(size) {
    Zero(size);
  }

  void Zero(int size) {
    position.setZero(size);
    velocity.setZero(size);
    acceleration.setZero(size);
    feed_forward_torque.setZero(size);
    stiffness.setZero(size);
    damping.setZero(size);
  }

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd feed_forward_torque;
  Eigen::VectorXd stiffness;
  Eigen::VectorXd damping;
};
}  // namespace data
