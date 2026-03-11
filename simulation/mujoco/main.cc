// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <glog/logging.h>
#include <mujoco/mujoco.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include "array_safety.h"
#include "glfw_adapter.h"
#include "simulate.h"

#include "lcm_interface/lcm_data_store.h"
#include "lcm_interface/lcm_publisher.h"
#include "lcm_interface/lcm_subscriber.h"
#include "lcm_param/lcm_param.h"
#include "model_param/model_param.h"
#include "parameter/global_config_initializer.h"
#include "parameter/parameter_loader.h"
#include "tool/string_join.h"
#include "tool/timer.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
#  include <windows.h>
#else
#  if defined(__APPLE__)
#    include <mach-o/dyld.h>
#  endif
#  include <sys/errno.h>
#  include <unistd.h>
#endif
}
using namespace std::chrono_literals;

namespace {
std::shared_ptr<LcmDataStore> lcm_data_store;
const constexpr auto kBusyWaitTime = 100us;
const constexpr int kDofFloatingBase = 6;
const constexpr int kNumFloatingBaseJoints = 7;
const constexpr int kDimQuaternion = 4;
const std::unordered_map<data::ContactType, int> single_contact_dimensions_map = {
    {data::ContactType::kPoint, 3},
    {data::ContactType::kRectangularSurface, 6},
};
}  // namespace

void UpdateSimState(const mjModel* m, mjData* d) {
  bool is_floating_base = (m->nv != m->nu);

  // Sets joint state
  SimState sim_state;
  Eigen::VectorXd q(m->nu);
  Eigen::VectorXd qd(m->nu);
  if (is_floating_base) {
    sim_state.q = Eigen::Map<const Eigen::VectorXd>(d->qpos + kNumFloatingBaseJoints, m->nu);
    sim_state.qd = Eigen::Map<const Eigen::VectorXd>(d->qvel + kDofFloatingBase, m->nu);
  } else {
    sim_state.q = Eigen::Map<const Eigen::VectorXd>(d->qpos, m->nu);
    sim_state.qd = Eigen::Map<const Eigen::VectorXd>(d->qvel, m->nu);
  }
  sim_state.tau = Eigen::Map<const Eigen::VectorXd>(d->actuator_force, m->nu);

  int index = 0;
  // Sets base link state, and the sequences are determined from `*sensors.xml`
  sim_state.base_link_quaternion.w() = d->sensordata[index + 0];
  sim_state.base_link_quaternion.x() = d->sensordata[index + 1];
  sim_state.base_link_quaternion.y() = d->sensordata[index + 2];
  sim_state.base_link_quaternion.z() = d->sensordata[index + 3];
  index += kDimQuaternion;

  sim_state.base_link_position =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.base_link_position.size());
  index += sim_state.base_link_position.size();

  sim_state.base_link_linear_velocity =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.base_link_linear_velocity.size());
  index += sim_state.base_link_linear_velocity.size();

  sim_state.base_link_angular_velocity =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.base_link_angular_velocity.size());
  index += sim_state.base_link_angular_velocity.size();

  // Sets imu link state, and the sequences are determined from `*sensors.xml`
  sim_state.imu_link_quaternion.w() = d->sensordata[index + 0];
  sim_state.imu_link_quaternion.x() = d->sensordata[index + 1];
  sim_state.imu_link_quaternion.y() = d->sensordata[index + 2];
  sim_state.imu_link_quaternion.z() = d->sensordata[index + 3];
  index += kDimQuaternion;

  sim_state.imu_link_position =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.imu_link_position.size());
  index += sim_state.imu_link_position.size();

  sim_state.imu_link_linear_velocity =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.imu_link_linear_velocity.size());
  index += sim_state.imu_link_linear_velocity.size();

  sim_state.imu_link_angular_velocity =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.imu_link_angular_velocity.size());
  index += sim_state.imu_link_angular_velocity.size();

  // Sets imu sensor state, and the sequences are determined from `*sensors.xml`
  sim_state.imu_sensor_quaternion.w() = d->sensordata[index + 0];
  sim_state.imu_sensor_quaternion.x() = d->sensordata[index + 1];
  sim_state.imu_sensor_quaternion.y() = d->sensordata[index + 2];
  sim_state.imu_sensor_quaternion.z() = d->sensordata[index + 3];
  index += kDimQuaternion;

  sim_state.imu_sensor_linear_acceleration =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.imu_sensor_linear_acceleration.size());
  index += sim_state.imu_sensor_linear_acceleration.size();

  sim_state.imu_sensor_angular_velocity =
      Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, sim_state.imu_sensor_angular_velocity.size());
  index += sim_state.imu_sensor_angular_velocity.size();

  // Sets contact force and torque state, and the sequences are determined from `*sensors.xml`
  sim_state.contact_force = std::vector<Eigen::VectorXd>(
      lcm_data_store->num_contacts, Eigen::VectorXd::Zero(lcm_data_store->num_single_contact_dimensions));
  for (int i = 0; i < lcm_data_store->num_contacts; ++i) {
    sim_state.contact_force[i] =
        Eigen::Map<const Eigen::VectorXd>(d->sensordata + index, lcm_data_store->num_single_contact_dimensions);
    sim_state.contact_force[i] *= -1.0;
    index += lcm_data_store->num_single_contact_dimensions;
  }
  lcm_data_store->sim_state.Set(sim_state);
}

void TorqueController(const mjModel* m, mjData* d) {
  bool is_floating_base = (m->nv != m->nu);

  Eigen::VectorXd q(m->nu);
  Eigen::VectorXd qd(m->nu);
  if (is_floating_base) {
    q = Eigen::Map<const Eigen::VectorXd>(d->qpos + kNumFloatingBaseJoints, m->nu);
    qd = Eigen::Map<const Eigen::VectorXd>(d->qvel + kDofFloatingBase, m->nu);
  } else {
    q = Eigen::Map<const Eigen::VectorXd>(d->qpos, m->nu);
    qd = Eigen::Map<const Eigen::VectorXd>(d->qvel, m->nu);
  }

  SimCommand sim_command = lcm_data_store->sim_command();
  Eigen::VectorXd tau_cmd(m->nu);
  tau_cmd = sim_command.kp.cwiseProduct(sim_command.q - q) + sim_command.kd.cwiseProduct(sim_command.qd - qd) +
            sim_command.tau_ff;

  Eigen::Map<Eigen::VectorXd>(d->ctrl, m->nu) = tau_cmd;
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

using Seconds = std::chrono::duration<double>;

//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *= 2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#  if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#  else
  const char* path = "/proc/self/exe";
#  endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif

  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename) > 4 && !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                                                     mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n') {
        loadError[error_length - 1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  if (!loadError[0] && load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;
  common::Timer loop_timer;
  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(kBusyWaitTime);
    }

    loop_timer.StartTimer();
    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned = std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            UpdateSimState(m, d);
            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction / sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // inject noise
              sim.InjectNoise();

              // call mj_step
              mj_step(m, d);
              UpdateSimState(m, d);
              const char* message = Diverged(m->opt.disableflags, d);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
    auto elapsed = loop_timer.GetElapsedSeconds();
    if (elapsed > m->opt.timestep) {
      LOG(WARNING) << "Physics loop took too long: " << elapsed << " s"
                   << ", Current timestep: " << m->opt.timestep << " s";
    }
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop

bool InitMain(int argc, char** argv) {
  std::string default_robot_name = argc > 1 ? argv[1] : "";
  if (!common::InitializeGlobalConfigPath(default_robot_name)) {
    std::cout << "[ERROR] Terminated since failed to initialize global config path." << std::endl;
    return false;
  }

  google::InitGoogleLogging(argv[0]);
  LOG(INFO) << "Loaded config files from: " << common::GlobalPathManager::GetInstance().GetConfigPath();

  return true;
}

int main(int argc, char** argv) {
  // Prints version and checks compatibility
  LOG(INFO) << "MuJoCo version " << mj_versionString();
  if (mjVERSION_HEADER != mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // Initializes global config path
  InitMain(argc, argv);
  data::ModelParam model_param;
  std::string filename = common::PathJoin(common::GlobalPathManager::GetInstance().GetResourcePath(), model_param.xml);

  int num_contacts = model_param.contact.size();
  int num_single_contact_dimensions = single_contact_dimensions_map.at(model_param.contact[0].type);
  lcm_data_store =
      std::make_shared<LcmDataStore>(model_param.num_total_joints, num_contacts, num_single_contact_dimensions);

  // Initializes LCM subscriber and publisher
  data::LcmParam lcm_param;
  std::shared_ptr<LcmSubscriber> lcm_subscriber = std::make_shared<LcmSubscriber>(lcm_param, lcm_data_store);
  lcm_subscriber->TaskCreate();
  lcm_subscriber->TaskInit();
  lcm_subscriber->TaskStart();

  std::shared_ptr<LcmPublisher> lcm_publisher = std::make_shared<LcmPublisher>(lcm_param, lcm_data_store);
  lcm_publisher->TaskCreate();
  lcm_publisher->TaskInit();
  lcm_publisher->TaskStart();

  // Scans for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // install control callback
  mjcb_control = TorqueController;

  // Simulates object encapsulates the UI
  auto sim =
      std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert, /* is_passive = */ false);

  // Starts physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());

  // Starts simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
