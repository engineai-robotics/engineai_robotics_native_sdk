#ifndef DATA_JOINT_INFO_JOINT_INFO_H_
#define DATA_JOINT_INFO_JOINT_INFO_H_

#include <Eigen/Dense>
#include <unordered_map>

#include "guarded_data/guarded_data.h"
#include "model_param/model_param.h"

namespace data {

class LimbCommand {
 public:
  LimbCommand() = default;
  LimbCommand(int size) { Zero(size); }
  ~LimbCommand() = default;

  void Zero(int size) {
    position.setZero(size);
    velocity.setZero(size);
    feed_forward_torque.setZero(size);
    torque.setZero(size);
    stiffness.setZero(size);
    damping.setZero(size);
  }

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd feed_forward_torque;
  Eigen::VectorXd torque;
  Eigen::VectorXd stiffness;
  Eigen::VectorXd damping;
};

class LimbState {
 public:
  LimbState() = default;
  LimbState(int size) { Zero(size); }
  ~LimbState() = default;

  void Zero(int size) {
    position.setZero(size);
    velocity.setZero(size);
    torque.setZero(size);
  }

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd torque;
};

class LimbLimit {
 public:
  LimbLimit() = default;
  LimbLimit(int size) { Zero(size); }
  ~LimbLimit() = default;

  void Zero(int size) {
    upper_position.setZero(size);
    lower_position.setZero(size);
    velocity.setZero(size);
    acceleration.setZero(size);
    torque.setZero(size);
  }

  Eigen::VectorXd upper_position;
  Eigen::VectorXd lower_position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd torque;
};

enum class JointInfoType {
  kPosition = 0,
  kVelocity,
  kFeedForwardTorque,
  kTorque,
  kStiffness,
  kDamping,
};
class JointInfo {
 public:
  JointInfo(const std::shared_ptr<ModelParam>& model_param) : model_param_(model_param) { Reset(); }
  ~JointInfo() = default;

  void Reset();
  void SetZeroCommand();
  void SetZeroState();
  void SetZeroLimit();

  Eigen::VectorXd& GetCommandByLimbId(JointInfoType type, LimbId id, Eigen::VectorXd& val);
  Eigen::VectorXd& GetCommand(JointInfoType type, Eigen::VectorXd& val);
  Eigen::VectorXd& GetLegCommand(JointInfoType type, Eigen::VectorXd& val);
  Eigen::VectorXd& GetCustomCommand(JointInfoType type, const std::vector<std::string>& joint_names,
                                    Eigen::VectorXd& val);

  Eigen::VectorXd& GetStateByLimbId(JointInfoType type, LimbId id, Eigen::VectorXd& val);
  Eigen::VectorXd& GetState(JointInfoType type, Eigen::VectorXd& val);
  Eigen::VectorXd& GetLegState(JointInfoType type, Eigen::VectorXd& val);
  Eigen::VectorXd& GetCustomState(JointInfoType type, const std::vector<std::string>& joint_names,
                                  Eigen::VectorXd& val);

  void SetCommandByLimbId(JointInfoType type, LimbId id, const Eigen::VectorXd& val);
  void SetCommand(JointInfoType type, const Eigen::VectorXd& val);
  void SetLegCommand(JointInfoType type, const Eigen::VectorXd& val);
  void SetCustomCommand(JointInfoType type, const std::vector<std::string>& joint_names, const Eigen::VectorXd& val);
  void SetCommand(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau_ff,
                  const Eigen::VectorXd& kp, const Eigen::VectorXd& kd);
  void SetCommandWithoutTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau_ff,
                               const Eigen::VectorXd& kp, const Eigen::VectorXd& kd);
  void SetCommandWithFullTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau_ff,
                                const Eigen::VectorXd& kp, const Eigen::VectorXd& kd, const Eigen::VectorXd& tau);
  void SetJointCommand(std::string_view joint_name, std::optional<double> q = std::nullopt,
                       std::optional<double> qd = std::nullopt, std::optional<double> tau_ff = std::nullopt,
                       std::optional<double> kp = std::nullopt, std::optional<double> kd = std::nullopt);

  void SetStateByLimbId(JointInfoType type, LimbId id, const Eigen::VectorXd& val);
  void SetState(JointInfoType type, const Eigen::VectorXd& val);
  void SetLegState(JointInfoType type, const Eigen::VectorXd& val);
  void SetCustomState(JointInfoType type, const std::vector<std::string>& joint_names, const Eigen::VectorXd& val);
  void SetState(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& tau);
  void SetJointState(std::string_view joint_name, std::optional<double> q = std::nullopt,
                     std::optional<double> qd = std::nullopt, std::optional<double> tau = std::nullopt);

  void SetLimit(const Eigen::VectorXd& upper_position, const Eigen::VectorXd& lower_position,
                const Eigen::VectorXd& velocity_limit, const Eigen::VectorXd& torque_limit);
  void SetUpperPositionLimit(const Eigen::VectorXd& val);
  void SetLowerPositionLimit(const Eigen::VectorXd& val);
  void SetVelocityLimit(const Eigen::VectorXd& val);
  void SetTorqueLimit(const Eigen::VectorXd& val);
  void GetLimit(Eigen::VectorXd& upper_position, Eigen::VectorXd& lower_position, Eigen::VectorXd& velocity_limit,
                Eigen::VectorXd& torque_limit);
  Eigen::VectorXd& GetUpperPositionLimit(Eigen::VectorXd& val);
  Eigen::VectorXd& GetLowerPositionLimit(Eigen::VectorXd& val);
  Eigen::VectorXd& GetVelocityLimit(Eigen::VectorXd& val);
  Eigen::VectorXd& GetTorqueLimit(Eigen::VectorXd& val);

  std::unordered_map<LimbId, data::GuardedData<LimbCommand>> command;
  std::unordered_map<LimbId, data::GuardedData<LimbState>> state;
  std::unordered_map<LimbId, data::GuardedData<LimbLimit>> limit;

 private:
  std::shared_ptr<ModelParam> model_param_;
};
}  // namespace data

#endif  // DATA_JOINT_INFO_JOINT_INFO_H_
