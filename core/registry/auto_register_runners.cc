// Auto-generated file - do not edit manually
// This file contains explicit runner registration calls

#include "basic/runner_registry.h"

// Factory function declarations
extern "C" void RegisterIdleRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterImuRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterInputCommandArbiterRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterJointMotorTransformRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterMotorRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterParamServerRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterSimPublishRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterSimSubscribeRunner(runner::RunnerRegistry& registry);
extern "C" void RegisterWholeBodyEstimateRunner(runner::RunnerRegistry& registry);

namespace runner {

void RegisterAllRunners() {
  auto& registry = RunnerRegistry::Instance();
  RegisterIdleRunner(registry);
  RegisterImuRunner(registry);
  RegisterInputCommandArbiterRunner(registry);
  RegisterJointMotorTransformRunner(registry);
  RegisterMotorRunner(registry);
  RegisterParamServerRunner(registry);
  RegisterSimPublishRunner(registry);
  RegisterSimSubscribeRunner(registry);
  RegisterWholeBodyEstimateRunner(registry);
}

}  // namespace runner
