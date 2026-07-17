#include "ros2_node/ros2_message_convertor.h"

#include <chrono>

#include "math/roll_pitch_yaw.h"

namespace ros2 {

void ToRos2Msg(const data::ImuInfo& imu_info, interface_protocol::msg::ImuInfo& msg) {
  msg.quaternion.w = imu_info.quaternion.w();
  msg.quaternion.x = imu_info.quaternion.x();
  msg.quaternion.y = imu_info.quaternion.y();
  msg.quaternion.z = imu_info.quaternion.z();
  msg.rpy.x = imu_info.rpy.x();
  msg.rpy.y = imu_info.rpy.y();
  msg.rpy.z = imu_info.rpy.z();
  msg.linear_acceleration.x = imu_info.linear_acceleration.x();
  msg.linear_acceleration.y = imu_info.linear_acceleration.y();
  msg.linear_acceleration.z = imu_info.linear_acceleration.z();
  msg.angular_velocity.x = imu_info.angular_velocity.x();
  msg.angular_velocity.y = imu_info.angular_velocity.y();
  msg.angular_velocity.z = imu_info.angular_velocity.z();
}

void ToRos2Msg(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& qd_cmd, const Eigen::VectorXd& tau_ff_cmd,
               const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& kp_cmd, const Eigen::VectorXd& kd_cmd,
               interface_protocol::msg::JointCommand& msg) {
  msg.position = std::vector<double>(q_cmd.data(), q_cmd.data() + q_cmd.size());
  msg.velocity = std::vector<double>(qd_cmd.data(), qd_cmd.data() + qd_cmd.size());
  msg.feed_forward_torque = std::vector<double>(tau_ff_cmd.data(), tau_ff_cmd.data() + tau_ff_cmd.size());
  msg.torque = std::vector<double>(tau_cmd.data(), tau_cmd.data() + tau_cmd.size());
  msg.stiffness = std::vector<double>(kp_cmd.data(), kp_cmd.data() + kp_cmd.size());
  msg.damping = std::vector<double>(kd_cmd.data(), kd_cmd.data() + kd_cmd.size());
}

void ToRos2Msg(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau,
               interface_protocol::msg::JointState& msg) {
  msg.position = std::vector<double>(q.data(), q.data() + q.size());
  msg.velocity = std::vector<double>(qd.data(), qd.data() + qd.size());
  msg.torque = std::vector<double>(tau.data(), tau.data() + tau.size());
}

void ToRos2Msg(const std::vector<std::string>& joint_names, const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
               const Eigen::VectorXd& tau, sensor_msgs::msg::JointState& msg) {
  msg.name = std::vector<std::string>(joint_names.data(), joint_names.data() + joint_names.size());
  msg.position = std::vector<double>(q.data(), q.data() + q.size());
  msg.velocity = std::vector<double>(qd.data(), qd.data() + qd.size());
  msg.effort = std::vector<double>(tau.data(), tau.data() + tau.size());
}

void ToRos2Msg(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau,
               sensor_msgs::msg::JointState& msg) {
  msg.position = std::vector<double>(q.data(), q.data() + q.size());
  msg.velocity = std::vector<double>(qd.data(), qd.data() + qd.size());
  msg.effort = std::vector<double>(tau.data(), tau.data() + tau.size());
}

void ToRos2Msg(const data::SE3Frame& frame, interface_protocol::msg::LinkInfo& msg) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = frame.pose.position.x();
  pose.position.y = frame.pose.position.y();
  pose.position.z = frame.pose.position.z();
  pose.orientation.w = frame.pose.quaternion.w();
  pose.orientation.x = frame.pose.quaternion.x();
  pose.orientation.y = frame.pose.quaternion.y();
  pose.orientation.z = frame.pose.quaternion.z();
  math::RollPitchYawd rpy = math::RollPitchYawd(frame.pose.quaternion);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = frame.twist.linear.x();
  twist.linear.y = frame.twist.linear.y();
  twist.linear.z = frame.twist.linear.z();
  twist.angular.x = frame.twist.angular.x();
  twist.angular.y = frame.twist.angular.y();
  twist.angular.z = frame.twist.angular.z();

  msg.pose = pose;
  msg.twist = twist;
}
}  // namespace ros2
