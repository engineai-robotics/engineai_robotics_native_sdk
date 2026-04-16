#ifndef ROS2_NODE_INCLUDE_ROS2_NODE_ROS2_MESSAGE_CONVERTOR_H_
#define ROS2_NODE_INCLUDE_ROS2_NODE_ROS2_MESSAGE_CONVERTOR_H_

#include <Eigen/Dense>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/link_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "basic_structure/basic_structure.h"
#include "imu_info/imu_info.h"

namespace ros2 {

void ToRos2Msg(const data::ImuInfo& imu_info, interface_protocol::msg::ImuInfo& msg);
void ToRos2Msg(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& qd_cmd, const Eigen::VectorXd& tau_ff_cmd,
               const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& kp_cmd, const Eigen::VectorXd& kd_cmd,
               interface_protocol::msg::JointCommand& msg);
void ToRos2Msg(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau,
               interface_protocol::msg::JointState& msg);
void ToRos2Msg(const std::vector<std::string>& joint_names, const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
               const Eigen::VectorXd& tau, sensor_msgs::msg::JointState& msg);
void ToRos2Msg(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau,
               sensor_msgs::msg::JointState& msg);
void ToRos2Msg(const data::SE3Frame& frame, interface_protocol::msg::LinkInfo& msg);

}  // namespace ros2

#endif  // ROS2_NODE_INCLUDE_ROS2_NODE_ROS2_MESSAGE_CONVERTOR_H_
