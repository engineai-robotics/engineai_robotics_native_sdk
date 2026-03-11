#pragma once
#include <Eigen/Dense>

namespace data {
enum class JointCommandType {
  kPosition = 0,
  kVelocity,
  kFeedForwardTorque,
  kTorque,
  kStiffness,
  kDamping,
};

class ControlOutput {
  class JointCommand {
   public:
    friend class ControlOutput;

    void SetPotionPDControl(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& kp, const Eigen::VectorXd& kd) {
      q_cmd_ = q_cmd;
      qd_cmd_.setZero(q_cmd_.size());
      tau_cmd_.setZero(q_cmd_.size());
      kp_ = kp;
      kd_ = kd;
      tau_ff_cmd_.setZero(q_cmd_.size());
    }

    void SetCommand(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& qd_cmd, const Eigen::VectorXd& kp,
                    const Eigen::VectorXd& kd, const Eigen::VectorXd& tau_ff_cmd) {
      q_cmd_ = q_cmd;
      qd_cmd_ = qd_cmd;
      tau_cmd_.setZero(q_cmd_.size());
      kp_ = kp;
      kd_ = kd;
      tau_ff_cmd_ = tau_ff_cmd;
    }

    void SetFullTorqueCommand(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& qd_cmd,
                              const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& kp, const Eigen::VectorXd& kd,
                              const Eigen::VectorXd& tau_ff_cmd) {
      q_cmd_ = q_cmd;
      qd_cmd_ = qd_cmd;
      tau_cmd_ = tau_cmd;
      kp_ = kp;
      kd_ = kd;
      tau_ff_cmd_ = tau_ff_cmd;
    }

    void SetSingleCommand(JointCommandType type, const Eigen::VectorXd& val) {
      if (type == JointCommandType::kPosition) {
        q_cmd_ = val;
      } else if (type == JointCommandType::kVelocity) {
        qd_cmd_ = val;
      } else if (type == JointCommandType::kFeedForwardTorque) {
        tau_ff_cmd_ = val;
      } else if (type == JointCommandType::kTorque) {
        tau_cmd_ = val;
      } else if (type == JointCommandType::kStiffness) {
        kp_ = val;
      } else if (type == JointCommandType::kDamping) {
        kd_ = val;
      }
    }

    void Reset() {
      q_cmd_.setZero();
      qd_cmd_.setZero();
      tau_cmd_.setZero();
      kp_.setZero();
      kd_.setZero();
      tau_ff_cmd_.setZero();
    }

    void Resize(int joint_size) {
      q_cmd_.setZero(joint_size);
      qd_cmd_.setZero(joint_size);
      tau_cmd_.setZero(joint_size);
      kp_.setZero(joint_size);
      kd_.setZero(joint_size);
      tau_ff_cmd_.setZero(joint_size);
    }

    Eigen::VectorXd q_cmd_;
    Eigen::VectorXd qd_cmd_;
    Eigen::VectorXd tau_cmd_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
    Eigen::VectorXd tau_ff_cmd_;
  };

 public:
  ControlOutput() { Reset(); }
  ControlOutput(int joint_size) { Resize(joint_size); }
  ~ControlOutput() = default;

  void Reset() {
    active_ = true;
    joint_command_.Reset();
  }

  void Resize(int joint_size) { joint_command_.Resize(joint_size); }

  void SetCommand(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& qd_cmd, const Eigen::VectorXd& kp,
                  const Eigen::VectorXd& kd, const Eigen::VectorXd& tau_ff_cmd) {
    joint_command_.SetCommand(q_cmd, qd_cmd, kp, kd, tau_ff_cmd);
  }

  void SetPositionPDControl(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& kp, const Eigen::VectorXd& kd) {
    joint_command_.SetPotionPDControl(q_cmd, kp, kd);
  }

  void SetCommandWithFullTorque(const Eigen::VectorXd& q_cmd, const Eigen::VectorXd& qd_cmd,
                                const Eigen::VectorXd& tau_cmd, const Eigen::VectorXd& kp, const Eigen::VectorXd& kd,
                                const Eigen::VectorXd& tau_ff_cmd) {
    joint_command_.SetFullTorqueCommand(q_cmd, qd_cmd, tau_cmd, kp, kd, tau_ff_cmd);
  }

  void SetSingleCommand(JointCommandType type, const Eigen::VectorXd& val) {
    joint_command_.SetSingleCommand(type, val);
  }

  const JointCommand& GetCommand() const { return joint_command_; }
  JointCommand& GetMutableCommand() { return joint_command_; }

  // 加法操作符重载
  ControlOutput operator+(const ControlOutput& other) const {
    ControlOutput result;
    result.joint_command_.q_cmd_ = this->joint_command_.q_cmd_ + other.joint_command_.q_cmd_;
    result.joint_command_.qd_cmd_ = this->joint_command_.qd_cmd_ + other.joint_command_.qd_cmd_;
    result.joint_command_.tau_cmd_ = this->joint_command_.tau_cmd_ + other.joint_command_.tau_cmd_;
    result.joint_command_.kp_ = this->joint_command_.kp_ + other.joint_command_.kp_;
    result.joint_command_.kd_ = this->joint_command_.kd_ + other.joint_command_.kd_;
    result.joint_command_.tau_ff_cmd_ = this->joint_command_.tau_ff_cmd_ + other.joint_command_.tau_ff_cmd_;
    return result;
  }

  // 复合赋值加法操作符重载
  ControlOutput& operator+=(const ControlOutput& other) {
    joint_command_.q_cmd_ += other.joint_command_.q_cmd_;
    joint_command_.qd_cmd_ += other.joint_command_.qd_cmd_;
    joint_command_.tau_cmd_ += other.joint_command_.tau_cmd_;
    joint_command_.kp_ += other.joint_command_.kp_;
    joint_command_.kd_ += other.joint_command_.kd_;
    joint_command_.tau_ff_cmd_ += other.joint_command_.tau_ff_cmd_;
    return *this;
  }

  void Activate() { active_ = true; }
  void Deactivate() { active_ = false; }
  bool IsActive() const { return active_; }

 private:
  bool active_{true};
  JointCommand joint_command_;
};
}  // namespace data
