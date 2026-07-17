#ifndef RUNNER_BASIC_INCLUDE_BASIC_RUNNER_FACTORY_H_
#define RUNNER_BASIC_INCLUDE_BASIC_RUNNER_FACTORY_H_

#include <memory>
#include <string_view>

#include "basic/basic_runner.h"
#include "basic/motion_runner.h"
#include "data_store/data_store.h"

namespace runner {

/**
 * @brief Runner工厂类，负责创建runner实例
 *
 * 该类封装了runner的创建逻辑，通过RunnerRegistry创建独立的runner实例。
 * 每次调用Create方法都会创建一个新的runner实例，确保不同motion之间的runner相互独立。
 */
class RunnerFactory {
 public:
  /**
   * @brief 创建Resident类型的runner实例
   *
   * @param runner_name runner类型名称（如"remote_handle_runner"）
   * @param instance_name runner实例名称（用于标识具体实例，如"task1::remote_handle_runner"）
   * @param data_store 共享的数据存储
   * @return std::shared_ptr<BasicRunner> 创建的runner实例，失败返回nullptr
   */
  static std::shared_ptr<BasicRunner> CreateResidentRunner(std::string_view runner_name, std::string_view instance_name,
                                                           const std::shared_ptr<data::DataStore>& data_store);

  /**
   * @brief 创建Motion类型的runner实例
   *
   * @param runner_name runner类型名称（如"rl_basic_runner"）
   * @param instance_name runner实例名称（用于标识具体实例，如"rl_basic::rl_basic_runner"）
   * @param data_store 共享的数据存储
   * @return std::shared_ptr<BasicRunner> 创建的runner实例，失败返回nullptr
   */
  static std::shared_ptr<MotionRunner> CreateMotionRunner(std::string_view runner_name, std::string_view instance_name,
                                                          const std::shared_ptr<data::DataStore>& data_store);
};

}  // namespace runner

#endif  // RUNNER_BASIC_INCLUDE_BASIC_RUNNER_FACTORY_H_
