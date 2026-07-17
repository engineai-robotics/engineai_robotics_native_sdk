#pragma once

namespace engineai_robotics {

class App {
 public:
  App() = default;
  ~App() = default;

  void RegisterApps();

  int RunMain(int argc, char* argv[]);

  static void GracefulExit(int signal);
};

}  // namespace engineai_robotics
