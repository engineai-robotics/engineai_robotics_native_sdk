#ifndef RUNNER_REGISTRY_RUNNER_REGISTRY_H_
#define RUNNER_REGISTRY_RUNNER_REGISTRY_H_

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "basic/basic_runner.h"
#include "data_store/data_store.h"

namespace runner {

enum class RunnerType { kResident, kMotion };

// Runner创建函数类型
using RunnerCreator = std::function<std::shared_ptr<BasicRunner>(std::string_view name,
                                                                 const std::shared_ptr<data::DataStore>& data_store)>;

// Runner注册信息
struct RunnerInfo {
  std::string name;
  RunnerType type;
  RunnerCreator creator;
};

// Runner注册表单例
class RunnerRegistry {
 public:
  static RunnerRegistry& Instance();

  // 注册runner
  void RegisterRunner(const std::string& name, RunnerType type, RunnerCreator creator);

  // 创建runner实例
  std::shared_ptr<BasicRunner> CreateRunner(const std::string& name, std::string_view instance_name,
                                            const std::shared_ptr<data::DataStore>& data_store);

  // 获取所有runner信息
  const std::unordered_map<std::string, RunnerInfo>& GetAllRunners() const;

  // 检查runner是否存在
  bool HasRunner(const std::string& name) const;

  // 获取runner类型
  RunnerType GetRunnerType(const std::string& name) const;

 private:
  RunnerRegistry() = default;
  std::unordered_map<std::string, RunnerInfo> runners_;
};
}  // namespace runner

// =============================================================================
// 编译期显式注册宏
// 用于头文件中定义工厂函数，导出到 .so 库
// 使用 extern "C" 确保符号名称不被 C++ 修饰，便于链接
// =============================================================================
#define REGISTER_RUNNER(ClassName, RunnerName, Type)                                                     \
  extern "C" __attribute__((weak)) void Register##ClassName(runner::RunnerRegistry& registry) {          \
    registry.RegisterRunner(                                                                             \
        RunnerName, runner::RunnerType::Type,                                                            \
        [](std::string_view name,                                                                        \
           const std::shared_ptr<data::DataStore>& data_store) -> std::shared_ptr<runner::BasicRunner> { \
          return std::make_shared<runner::ClassName>(name, data_store);                                  \
        });                                                                                              \
  }

#endif  // RUNNER_REGISTRY_RUNNER_REGISTRY_H_
