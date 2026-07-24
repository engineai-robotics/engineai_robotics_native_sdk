#include "idle/idle_runner.h"

#include <glog/logging.h>
#include <iostream>

#include "parameter/global_config_initializer.h"
#include "tool/concatenate_vector.h"

namespace runner {
void IdleRunner::SetupContext() {}

void IdleRunner::TeardownContext() {}

bool IdleRunner::Enter() {
  num_try_exit_ = kNumTryExit;

  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::IdleParam>(param_tag_);
  }

  return true;
}

void IdleRunner::Run() {
  data_store_->enable_or_disable_motor.store(false);

  const int n = model_param_->num_total_joints;
  Eigen::VectorXd q_cur = Eigen::VectorXd::Zero(n);
  data_store_->joint_info.GetState(data::JointInfoType::kPosition, q_cur);

  if (!param_) {
    LOG_FIRST_N(WARNING, 1) << "[IdleRunner] param_ is null, output reset to zero.";
    GetMutableOutput().Reset();
    if (common::IsInMujoco()) {
      SetRunnerState(runner::RunnerState::kTryExit);
    }
    return;
  }

  const bool pre_motion_cmd_enable = param_->pre_motion_cmd_enable.value_or(false);

  if (pre_motion_cmd_enable) {
    Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd qd_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd kp_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd kd_cmd = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd tau_ff_cmd = Eigen::VectorXd::Zero(n);

    const Eigen::VectorXi enable_mask = param_->pre_cmd_enable_joints
                                            ? common::ConcatenateVectors(param_->pre_cmd_enable_joints.value())
                                            : Eigen::VectorXi::Zero(n);
    const Eigen::VectorXd kp =
        param_->stiffness ? common::ConcatenateVectors(param_->stiffness.value()) : Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd kd =
        param_->damping ? common::ConcatenateVectors(param_->damping.value()) : Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      if (enable_mask[i] == 1) {
        q_cmd[i] = q_cur[i];
        kp_cmd[i] = kp[i];
        kd_cmd[i] = kd[i];
      }
    }

    GetMutableOutput().SetCommand(q_cmd, qd_cmd, kp_cmd, kd_cmd, tau_ff_cmd);
  } else {
    GetMutableOutput().Reset();
  }

  if (common::IsInMujoco()) {
    SetRunnerState(runner::RunnerState::kTryExit);
  }
}

TransitionState IdleRunner::TryExit() {
  if (!common::IsInMujoco()) {
    data_store_->enable_or_disable_motor.store(true);
    if (!data_store_->enabled_motor_state.load() && num_try_exit_ < kNumTryExit) {
      ++num_try_exit_;
      this->Run();
      return TransitionState::kTrying;
    }
  }

  return TransitionState::kCompleted;
}

bool IdleRunner::Exit() { return true; }

void IdleRunner::End() {}
}  // namespace runner
