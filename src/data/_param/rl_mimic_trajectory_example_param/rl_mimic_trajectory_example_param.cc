#include "rl_mimic_trajectory_example_param.h"

namespace data {

RlMinicTrajectoryParam::RlMinicTrajectoryParam(std::string_view tag) : BasicParam(tag) {
  LOAD_PARAM(observations);
  LOAD_PARAM(active_joint_names);
  // 加载多组 default_joint_q 配置
  {
    std::vector<Eigen::VectorXd> tmp;
    tmp = common::ScopedParameterGetter<std::vector<Eigen::VectorXd>>::Get(scope_, "default_joint_q");
    default_joint_q_groups["default_joint_q"] = tmp;
    default_joint_q = tmp;  // 使用 default_joint_q 作为默认值
    tmp = common::ScopedParameterGetterWithDefault<std::vector<Eigen::VectorXd>>::Get(scope_, "default_joint_q_1",
                                                                                      std::vector<Eigen::VectorXd>{});
    if (!tmp.empty()) default_joint_q_groups["default_joint_q_1"] = tmp;
  }
  // 允许仅使用每motion_state的组配置；若未提供全局kp/kd/action_scale，则给空默认
  LOAD_PARAM_DEFAULT(joint_kp, std::vector<Eigen::VectorXd>{});
  LOAD_PARAM_DEFAULT(joint_kd, std::vector<Eigen::VectorXd>{});
  LOAD_PARAM_DEFAULT(action_scale, std::vector<Eigen::VectorXd>{});
  LOAD_PARAM(qd_mask);
  LOAD_PARAM(imu_install_delta_bias);
  LOAD_PARAM(linear_vel_delta_bias);
  LOAD_PARAM(action_clip);
  LOAD_PARAM(transition_time);
  LOAD_PARAM(torque_limit);
  LOAD_PARAM(max_torque_joint);
  LOAD_PARAM(max_lower_body_torque);
  LOAD_PARAM(motion_state);

  // 构建统一的observation scale向量
  const auto& scale_config = observations.observation_scale;

  // 本体感知各项配置：维度和scale值
  struct ProprioItem {
    int dim;
    double scale;
  };
  // Base proprioceptive items (yaw forward vector always included but scaled to 0 when not needed)
  std::vector<ProprioItem> proprio_items = {
      {static_cast<int>(active_joint_names.size()), scale_config.observation_scale_dof_pos},
      {static_cast<int>(active_joint_names.size()), scale_config.observation_scale_dof_vel},
      {static_cast<int>(active_joint_names.size()), 1.0},  // action
      {3, scale_config.observation_scale_angular_vel},     // angular vel
      {3, scale_config.observation_scale_quat},            // gravity
      {2, scale_config.observation_scale_yaw_forward}      // yaw forward vector (scaled to 0 when not needed)
  };

  // 计算单步本体感知维度
  int proprio_single_dim = 0;
  for (const auto& item : proprio_items) proprio_single_dim += item.dim;

  // 找到最大的time steps来确定scale向量的最大可能大小
  int max_proprio_time_steps = 0;
  for (const auto& [goal_type, goal_cfg] : observations.observation_type) {
    max_proprio_time_steps = std::max(max_proprio_time_steps, goal_cfg.num_include_obs_steps);
  }

  // 不再需要预构建scale向量，改为动态构建
  observation_scale_vec = Eigen::VectorXd::Zero(1);  // 占位符

  LOG(INFO) << "Scale configuration loaded: single_step_dim=" << proprio_single_dim
            << ", max_time_steps=" << max_proprio_time_steps;

  num_actions = active_joint_names.size();
}

}  // namespace data
