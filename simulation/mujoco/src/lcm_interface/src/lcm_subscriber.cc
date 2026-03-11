#include "lcm_interface/lcm_subscriber.h"

LcmSubscriber::LcmSubscriber(const data::LcmParam& param, const std::shared_ptr<LcmDataStore>& lcm_data_store)
    : lcm_interface::PeriodicTask("lcm_subscriber", /* period */ 0.002), lcm_data_store_(lcm_data_store) {
  std::string url;
  int ttl = param.multicast ? param.ttl : 0;
  url = "udpm://" + param.ip_port + "?ttl=" + std::to_string(ttl);
  LOG(INFO) << "LCM URL: " << url;
  lcm_ = std::make_shared<lcm::LCM>(url);
  if (!lcm_->good()) {
    LOG(FATAL) << "LCM is not good.";
  }

  lcm_->subscribe(channel_, &LcmSubscriber::HandleSimCommand, this);
}

void LcmSubscriber::TaskInit() {}

void LcmSubscriber::TaskRun() { lcm_->handle(); }

void LcmSubscriber::HandleSimCommand(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                                     const data::SimCommand* msg) {
  sim_command_.q = Eigen::Map<const Eigen::VectorXd>(msg->joint_position.data(), msg->num_ranges);
  sim_command_.qd = Eigen::Map<const Eigen::VectorXd>(msg->joint_velocity.data(), msg->num_ranges);
  sim_command_.tau_ff = Eigen::Map<const Eigen::VectorXd>(msg->joint_feed_forward_torque.data(), msg->num_ranges);
  sim_command_.kp = Eigen::Map<const Eigen::VectorXd>(msg->joint_stiffness.data(), msg->num_ranges);
  sim_command_.kd = Eigen::Map<const Eigen::VectorXd>(msg->joint_damping.data(), msg->num_ranges);

  lcm_data_store_->sim_command.Set(sim_command_);
}
