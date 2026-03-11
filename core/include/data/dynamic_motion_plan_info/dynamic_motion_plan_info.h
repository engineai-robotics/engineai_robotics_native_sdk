#pragma once

#include <Eigen/Dense>
#include <mutex>
#include <optional>

namespace data {

// 规划状态枚举（统一状态，包含使能和运行时状态）
enum class PlanningState {
  // 未启用（规划器未激活）
  DISABLED = 0,
  // 空闲状态（已启用，等待请求）
  IDLE,
  // 执行中（包括规划和执行）
  EXECUTING,
  // 退出归位中（禁止外部请求）
  EXITING
};

// 运动规划请求
struct DynamicMotionPlanRequest {
  bool use_gravity_compensation = false;
  Eigen::VectorXi joint_indices;
  Eigen::VectorXd target_positions;
  Eigen::VectorXd target_velocities;
  double execution_time = 0.0;
  Eigen::VectorXd stiffness;
  Eigen::VectorXd damping;

  void Reset() {
    use_gravity_compensation = false;
    joint_indices.resize(0);
    target_positions.resize(0);
    target_velocities.resize(0);
    execution_time = 0.0;
    stiffness.resize(0);
    damping.resize(0);
  }
};

// 运动规划反馈
struct DynamicMotionPlanFeedback {
  // 执行进度 [0.0, 1.0]
  double progress = 0.0;
  // 当前关节位置
  Eigen::VectorXd current_positions;
  // 剩余时间
  double time_remaining = 0.0;
  // 状态描述
  std::string status;
};

// 运动规划结果
struct DynamicMotionPlanResult {
  bool success = false;
  Eigen::VectorXd final_positions;
  double actual_time = 0.0;
  std::string message;
};

// 线程安全的运动规划信息类
class DynamicMotionPlanInfo {
 public:
  DynamicMotionPlanInfo() : state_(PlanningState::DISABLED) {}

  // 设置规划状态（统一状态管理，包含使能和运行时状态）
  void SetState(PlanningState state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
  }

  // 获取规划状态
  PlanningState GetState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  // 提交新的规划请求（始终使最后一个请求生效）
  void SubmitRequest(const DynamicMotionPlanRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_request_ = request;
  }

  // 获取当前请求（如果有）
  std::optional<DynamicMotionPlanRequest> GetCurrentRequest() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_request_.has_value()) {
      auto request = current_request_.value();
      // 获取后清空，避免重复处理
      current_request_.reset();
      return request;
    }
    return std::nullopt;
  }

  // 设置反馈信息
  void SetFeedback(const DynamicMotionPlanFeedback& feedback) {
    std::lock_guard<std::mutex> lock(mutex_);
    feedback_ = feedback;
  }

  // 获取反馈信息
  std::optional<DynamicMotionPlanFeedback> GetFeedback() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_;
  }

  // 设置结果信息
  void SetResult(const DynamicMotionPlanResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = result;
  }

  // 获取结果信息
  std::optional<DynamicMotionPlanResult> GetResult() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result_.has_value()) {
      auto result = result_.value();
      // 获取后清空
      result_.reset();
      return result;
    }
    return std::nullopt;
  }

  // 清除当前请求
  void ClearRequest() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_request_.reset();
  }

 private:
  mutable std::mutex mutex_;
  PlanningState state_;
  std::optional<DynamicMotionPlanRequest> current_request_;
  std::optional<DynamicMotionPlanFeedback> feedback_;
  std::optional<DynamicMotionPlanResult> result_;
};

// namespace data
}  // namespace data