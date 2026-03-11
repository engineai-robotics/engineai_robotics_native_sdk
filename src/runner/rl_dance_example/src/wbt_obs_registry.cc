// WBT observation registry implementation.
// To add a new observation, append an entry inside GetRegistry()'s if (registry.empty()) block:
//   registry["obs_name"] = { [](int na){ return dim; }, [](const ObsContext& ctx){ ... return out; } };
// If a new data source is needed, add it to ObsContext in wbt_obs_registry.h
// and populate it in WbtRunner::fillObsContextConstantPart.

#include "rl_dance_example/wbt_obs_registry.h"

#include <glog/logging.h>
#include <functional>
#include <unordered_map>

#include "data_store/data_store.h"
#include "math/roll_pitch_yaw.h"
#include "math/rotation_matrix.h"

namespace wbt_obs {

namespace {

struct ObsEntry {
  std::function<int(int num_actions)> get_dim;
  std::function<Eigen::VectorXd(const ObsContext&)> get_value;
};

static void ComputeImuInBody(const ObsContext& ctx, Eigen::Matrix3d* R_real, Eigen::Vector3d* w_real,
                             Eigen::Vector3d* projected_gravity) {
  if (!ctx.data_store || !R_real || !w_real || !projected_gravity) return;
  Eigen::Matrix3d R_install = math::RotationMatrixd(math::RollPitchYawd(ctx.imu_install_bias)).matrix();
  Eigen::Matrix3d R_local = math::RotationMatrixd(ctx.data_store->imu_info.Get()->quaternion).matrix();
  *R_real = R_local * R_install.transpose();
  *w_real = R_real->transpose() * R_local * ctx.data_store->imu_info.Get()->angular_velocity;
  *projected_gravity = -R_real->transpose() * Eigen::Vector3d::UnitZ();
}

std::unordered_map<std::string, ObsEntry>& GetRegistry() {
  static std::unordered_map<std::string, ObsEntry> registry;

  if (registry.empty()) {
    registry["command"] = {
        [](int na) { return 2 * na; },
        [](const ObsContext& ctx) {
          if (!ctx.ref_joint_pos_all || !ctx.ref_joint_vel_all) {
            LOG(ERROR) << "[obs:command] ref_joint_pos_all or ref_joint_vel_all is null";
            return Eigen::VectorXd();
          }
          int r = ctx.ref_joint_pos_all->rows();
          int step = r > 0 ? std::min(ctx.policy_step + ctx.future_cmd_step, r - 1) : 0;
          Eigen::VectorXd ref_pos = ctx.ref_joint_pos_all->row(step);
          if (ctx.data_store && ctx.soft_joint_pos_limit && ctx.policy2deploy_joint_idx &&
              ctx.soft_joint_pos_limit->size() == ref_pos.size()) {
            int n = ctx.data_store->model_param->num_total_joints;
            Eigen::VectorXd upper_limit(n), lower_limit(n);
            ctx.data_store->joint_info.GetUpperPositionLimit(upper_limit);
            ctx.data_store->joint_info.GetLowerPositionLimit(lower_limit);
            Eigen::VectorXd upper_policy = upper_limit(*ctx.policy2deploy_joint_idx);
            Eigen::VectorXd lower_policy = lower_limit(*ctx.policy2deploy_joint_idx);
            Eigen::VectorXd soft_upper = upper_policy.cwiseProduct(*ctx.soft_joint_pos_limit);
            Eigen::VectorXd soft_lower = lower_policy.cwiseProduct(*ctx.soft_joint_pos_limit);
            ref_pos = ref_pos.cwiseMax(soft_lower).cwiseMin(soft_upper);
          }
          Eigen::VectorXd ref_vel = ctx.ref_joint_vel_all->row(step);
          Eigen::VectorXd cmd(ref_pos.size() + ref_vel.size());
          cmd << ref_pos, ref_vel;
          return cmd;
        },
    };

    registry["motion_anchor_ori_b"] = {
        [](int) { return 6; },
        [](const ObsContext& ctx) {
          if (!ctx.ref_body_quat_w_all || ctx.ref_body_quat_w_all->rows() == 0) {
            LOG(ERROR) << "[obs:motion_anchor_ori_b] ref_body_quat_w_all is null or empty";
            return Eigen::VectorXd();
          }
          int r = ctx.ref_body_quat_w_all->rows();
          int step = std::min(ctx.policy_step, r - 1);
          Eigen::Quaterniond ref_quat_w((*ctx.ref_body_quat_w_all)(step, 0), (*ctx.ref_body_quat_w_all)(step, 1),
                                        (*ctx.ref_body_quat_w_all)(step, 2), (*ctx.ref_body_quat_w_all)(step, 3));
          Eigen::Matrix3d ref_anchor_ori_rot_w = math::RotationMatrixd(ref_quat_w).matrix();

          Eigen::Matrix3d R_real;
          Eigen::Vector3d w_unused, proj_g_unused;
          ComputeImuInBody(ctx, &R_real, &w_unused, &proj_g_unused);
          Eigen::Matrix3d body_anchor_ori_rot_w = R_real;

          Eigen::Matrix3d ref_aligned = ctx.ref_init_yaw_rot.transpose() * ref_anchor_ori_rot_w;
          Eigen::Matrix3d body_aligned = ctx.body_init_yaw_rot.transpose() * body_anchor_ori_rot_w;
          Eigen::Matrix3d anchor_ori_rot_b = body_aligned.transpose() * ref_aligned;
          Eigen::VectorXd out(6);
          out = anchor_ori_rot_b.leftCols(2).reshaped<Eigen::RowMajor>();
          return out;
        },
    };

    registry["base_ang_vel"] = {
        [](int) { return 3; },
        [](const ObsContext& ctx) {
          Eigen::Matrix3d R_real;
          Eigen::Vector3d w_real, proj_g_unused;
          ComputeImuInBody(ctx, &R_real, &w_real, &proj_g_unused);
          return w_real;
        },
    };

    registry["joint_pos"] = {
        [](int na) { return na; },
        [](const ObsContext& ctx) {
          if (!ctx.data_store || !ctx.default_joint_q || !ctx.policy2deploy_joint_idx) {
            LOG(ERROR) << "[obs:joint_pos] data_store/default_joint_q/policy2deploy_joint_idx is null";
            return Eigen::VectorXd();
          }
          int n = ctx.data_store->model_param->num_total_joints;
          Eigen::VectorXd q_real(n);
          ctx.data_store->joint_info.GetState(data::JointInfoType::kPosition, q_real);
          Eigen::VectorXd upper_limit(n), lower_limit(n);
          ctx.data_store->joint_info.GetUpperPositionLimit(upper_limit);
          ctx.data_store->joint_info.GetLowerPositionLimit(lower_limit);
          q_real = q_real.cwiseMax(lower_limit).cwiseMin(upper_limit);
          Eigen::VectorXd out = (q_real - *ctx.default_joint_q)(*ctx.policy2deploy_joint_idx);
          return out;
        },
    };

    registry["joint_vel"] = {
        [](int na) { return na; },
        [](const ObsContext& ctx) {
          if (!ctx.data_store || !ctx.policy2deploy_joint_idx) {
            LOG(ERROR) << "[obs:joint_vel] data_store or policy2deploy_joint_idx is null";
            return Eigen::VectorXd();
          }
          int n = ctx.data_store->model_param->num_total_joints;
          Eigen::VectorXd qd_real(n);
          ctx.data_store->joint_info.GetState(data::JointInfoType::kVelocity, qd_real);
          Eigen::VectorXd qd_masked = qd_real;
          if (ctx.joint_vel_mask_indices) {
            for (int idx : *ctx.joint_vel_mask_indices) {
              if (idx < n) qd_masked(idx) = 0.0;
            }
          }
          Eigen::VectorXd out = qd_masked(*ctx.policy2deploy_joint_idx);
          return out;
        },
    };

    registry["actions"] = {
        [](int na) { return na; },
        [](const ObsContext& ctx) {
          if (!ctx.actions) {
            LOG(ERROR) << "[obs:actions] actions is null";
            return Eigen::VectorXd();
          }
          return *ctx.actions;
        },
    };

    registry["projected_gravity"] = {
        [](int) { return 3; },
        [](const ObsContext& ctx) {
          Eigen::Matrix3d R_real;
          Eigen::Vector3d w_unused, projected_gravity;
          ComputeImuInBody(ctx, &R_real, &w_unused, &projected_gravity);
          return projected_gravity;
        },
    };
  }

  return registry;
}

}  // namespace

int GetObservationDim(const std::string& name, int num_actions) {
  auto& reg = GetRegistry();
  auto it = reg.find(name);
  if (it == reg.end()) {
    LOG(ERROR) << "[GetObservationDim] Unknown observation: " << name;
    return 0;
  }
  return it->second.get_dim(num_actions);
}

Eigen::VectorXd GetObservation(const std::string& name, const ObsContext& ctx) {
  auto& reg = GetRegistry();
  auto it = reg.find(name);
  if (it == reg.end()) {
    LOG(ERROR) << "[GetObservation] Unknown observation: " << name;
    return Eigen::VectorXd();
  }
  Eigen::VectorXd result = it->second.get_value(ctx);
  if (result.size() == 0) {
    LOG(WARNING) << "[GetObservation] " << name << " returned empty vector";
  }
  return result;
}

}  // namespace wbt_obs
