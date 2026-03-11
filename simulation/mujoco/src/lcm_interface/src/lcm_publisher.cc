#include "lcm_interface/lcm_publisher.h"

LcmPublisher::LcmPublisher(const data::LcmParam& param, const std::shared_ptr<LcmDataStore>& lcm_data_store)
    : lcm_interface::PeriodicTask("lcm_publisher", /* period */ 0.002), lcm_data_store_(lcm_data_store) {
  std::string url;
  int ttl = param.multicast ? param.ttl : 0;
  url = "udpm://" + param.ip_port + "?ttl=" + std::to_string(ttl);
  LOG(INFO) << "LCM URL: " << url;
  lcm_ = std::make_shared<lcm::LCM>(url);
  if (!lcm_->good()) {
    LOG(FATAL) << "LCM is not good.";
  }
}

void LcmPublisher::TaskInit() {}

void LcmPublisher::TaskRun() {
  sim_state_ = lcm_data_store_->sim_state();

  data::SimState msg;
  msg.num_ranges = static_cast<int32_t>(sim_state_.q.size());
  msg.joint_position = std::vector<double>(sim_state_.q.data(), sim_state_.q.data() + sim_state_.q.size());
  msg.joint_velocity = std::vector<double>(sim_state_.qd.data(), sim_state_.qd.data() + sim_state_.qd.size());
  msg.joint_torque = std::vector<double>(sim_state_.tau.data(), sim_state_.tau.data() + sim_state_.tau.size());

  std::copy(sim_state_.base_link_position.data(),
            sim_state_.base_link_position.data() + sim_state_.base_link_position.size(), msg.base_link_position);
  std::copy(sim_state_.base_link_linear_velocity.data(),
            sim_state_.base_link_linear_velocity.data() + sim_state_.base_link_linear_velocity.size(),
            msg.base_link_linear_velocity);
  msg.base_link_quaternion[0] = sim_state_.base_link_quaternion.w();
  msg.base_link_quaternion[1] = sim_state_.base_link_quaternion.x();
  msg.base_link_quaternion[2] = sim_state_.base_link_quaternion.y();
  msg.base_link_quaternion[3] = sim_state_.base_link_quaternion.z();
  std::copy(sim_state_.base_link_angular_velocity.data(),
            sim_state_.base_link_angular_velocity.data() + sim_state_.base_link_angular_velocity.size(),
            msg.base_link_angular_velocity);

  std::copy(sim_state_.imu_link_position.data(),
            sim_state_.imu_link_position.data() + sim_state_.imu_link_position.size(), msg.imu_link_position);
  std::copy(sim_state_.imu_link_linear_velocity.data(),
            sim_state_.imu_link_linear_velocity.data() + sim_state_.imu_link_linear_velocity.size(),
            msg.imu_link_linear_velocity);
  msg.imu_link_quaternion[0] = sim_state_.imu_link_quaternion.w();
  msg.imu_link_quaternion[1] = sim_state_.imu_link_quaternion.x();
  msg.imu_link_quaternion[2] = sim_state_.imu_link_quaternion.y();
  msg.imu_link_quaternion[3] = sim_state_.imu_link_quaternion.z();
  std::copy(sim_state_.imu_link_angular_velocity.data(),
            sim_state_.imu_link_angular_velocity.data() + sim_state_.imu_link_angular_velocity.size(),
            msg.imu_link_angular_velocity);

  msg.imu_sensor_quaternion[0] = sim_state_.imu_sensor_quaternion.w();
  msg.imu_sensor_quaternion[1] = sim_state_.imu_sensor_quaternion.x();
  msg.imu_sensor_quaternion[2] = sim_state_.imu_sensor_quaternion.y();
  msg.imu_sensor_quaternion[3] = sim_state_.imu_sensor_quaternion.z();
  std::copy(sim_state_.imu_sensor_linear_acceleration.data(),
            sim_state_.imu_sensor_linear_acceleration.data() + sim_state_.imu_sensor_linear_acceleration.size(),
            msg.imu_sensor_linear_acceleration);
  std::copy(sim_state_.imu_sensor_angular_velocity.data(),
            sim_state_.imu_sensor_angular_velocity.data() + sim_state_.imu_sensor_angular_velocity.size(),
            msg.imu_sensor_angular_velocity);

  int num_contacts = sim_state_.contact_force.size();
  int num_single_contact_dimensions = num_contacts ? sim_state_.contact_force[0].size() : 0;
  msg.num_contact_ranges = num_contacts * num_single_contact_dimensions;
  if (msg.num_contact_ranges) {
    msg.contact_force.reserve(num_contacts * num_single_contact_dimensions);
    double* ptr = msg.contact_force.data();
    for (const auto& vec : sim_state_.contact_force) {
      std::copy(vec.data(), vec.data() + vec.size(), ptr);
      ptr += vec.size();
    }
  } else {
    LOG(ERROR) << "Contact force dimension is 0.";
  }

  msg.timestamp = 0;
  if (lcm_->publish(channel_, &msg)) {
    LOG(ERROR) << "Failed to publish LCM message: " << channel_;
  }
}
