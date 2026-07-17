/* Copyright (C) 2021 - ~, SCUT-RobotLab Development Team
 * @file    Logitech_controller.cpp
 * @brief   罗技遥控器的驱动库，库目前支持罗技遥控器型号：F710、F310。
 * @note
 *         - 注意按键值所对应的意义。
 *         - 当手柄 Mode为黄灯时，左边上下左右按键与左边遥感值互换，按键按下为对应遥感边界值（+-1）
 *           遥感推至极限为对应按键值。
 *         - 一般情况下使用非黄灯模式。
 * @method
 *         - 1. 手柄驱动路径     ： char path[] = "/dev/input/js0";
 *         - 2. 定义一个遥控器类  : LogitechGamepadDriver LogitechGamepadDriver(path);
 *         - 3. 遥控器类初始化    : LogitechGamepadDriver.init();
 *         - 4. 线程中进行数据接收 : LogitechGamepadDriver.listen_input();
 * @warning
 *         - At least C++11 is required.
 *         - Only linux platform is supported for now.
 */

// TODO(qsy): Format the code
#include "gamepad/gamepad_driver.h"
#include <glog/logging.h>
#include <sys/stat.h>

#define RANGE_MAX 1000.0
using namespace std;

namespace hardware {
LogitechGamepadDriver::LogitechGamepadDriver() : dev("/dev/input/js0") { memset(buf, 0, sizeof buf); }

int LogitechGamepadDriver::Init() {
  VLOG(1) << "[INFO] Initializing gamepad driver";
  fd = open(dev.c_str(), O_RDONLY);
  if (fd == -1) {
    VLOG(1) << "[ERROR] Cannot open " << dev << ": " << strerror(errno);
    return EXIT_FAILURE;
  }
  Zeros();

  return EXIT_SUCCESS;
}

void LogitechGamepadDriver::Zeros() {
  /*Key Status*/

  /* 0 is released */
  /* 1 is press    */
  Keystate_map[JSKEY_A] = 0;
  Keystate_map[JSKEY_B] = 0;
  Keystate_map[JSKEY_X] = 0;
  Keystate_map[JSKEY_Y] = 0;

  /* 0 is released */
  /* 1 is press    */
  Keystate_map[JSKEY_LB] = 0;
  Keystate_map[JSKEY_RB] = 0;

  /* 0 is released */
  /* 1 is press    */
  Keystate_map[JSKEY_BACK] = 0;
  Keystate_map[JSKEY_START] = 0;
  Keystate_map[JSKEY_HOME] = 0;

  /*  0 is released */
  /* -1 is the left or up button is pressed */
  /*  1 is the right or down button is pressed*/
  Keystate_map[JSKEY_CROSS_X] = 0;
  Keystate_map[JSKEY_CROSS_Y] = 0;

  /* the result is the value of the key(0~99)*/
  Keystate_map[JSKEY_LT] = 0;
  Keystate_map[JSKEY_RT] = 0;

  /* the result is the value of the key(-100~100)*/
  Keystate_map[JSKEY_LEFTSTICK_X] = 0;
  Keystate_map[JSKEY_LEFTSTICK_Y] = 0;
  Keystate_map[JSKEY_RIGHTSTICK_X] = 0;
  Keystate_map[JSKEY_RIGHTSTICK_Y] = 0;

  logitech_data_res.A = 0;
  logitech_data_res.B = 0;
  logitech_data_res.X = 0;
  logitech_data_res.Y = 0;

  logitech_data_res.HOME = 0;
  logitech_data_res.START = 0;
  logitech_data_res.BACK = 0;

  logitech_data_res.LB = 0;
  logitech_data_res.RB = 0;
  logitech_data_res.LT = 0;
  logitech_data_res.RT = 0;

  logitech_data_res.CROSS_X = 0;
  logitech_data_res.CROSS_Y = 0;

  logitech_data_res.leftStickButton = 0;
  logitech_data_res.leftStickXAnalog = 0;
  logitech_data_res.leftStickYAnalog = 0;

  logitech_data_res.rightStickButton = 0;
  logitech_data_res.rightStickXAnalog = 0;
  logitech_data_res.rightStickYAnalog = 0;
}

int LogitechGamepadDriver::ListenInput() {
  loop_count = 0;

  while (loop_count < 5) {
    loop_count++;

    int flags = fcntl(fd, F_GETFL, 0);
    flags |= O_NONBLOCK;

    if (fcntl(fd, F_SETFL, flags) < 0) {
      VLOG(1) << "[ERROR] Error setting non-blocking mode";
      Zeros();
      return EXIT_FAILURE;
    }

    memset(buf, 0, sizeof buf);

    // 保存当前errno值
    errno = 0;
    n = read(fd, &buf, sizeof buf);
    int read_errno = errno;  // 立即保存read操作的errno

    // 使用fstat检查文件描述符的状态
    struct stat fd_stat;
    bool device_ok = true;
    if (fstat(fd, &fd_stat) < 0) {
      // 无法获取文件状态，设备可能已断开
      device_ok = false;
      VLOG(1) << "[ERROR] Cannot get gamepad device status, possible disconnection";
    }

    // 检查读取操作是否失败
    if (n < 0) {
      if (read_errno == ENODEV || read_errno == ENOENT || read_errno == EIO) {
        VLOG(1) << "[ERROR] Error reading from gamepad: device may have been disconnected, error: " << strerror(read_errno);
        continue;
      }
    }

    if (fcntl(fd, F_SETFL, flags) < 0) {
      VLOG(1) << "[ERROR] Error setting non-blocking mode";
      Zeros();
      return EXIT_FAILURE;
    }

    n = n / sizeof(int);
    if (n == (ssize_t)-1) {
      if (errno == EINTR) {
        continue;
      } else {
        break;
      }
    }

    unsigned short btn = buf[1] >> 16;
    short val = (short)(buf[1] & 0xffff);

    if (btn == JSKEY_LT || btn == JSKEY_RT) {
      unsigned short prs_val = val + 32768;
      val = (unsigned short)(((long)prs_val) * RANGE_MAX / 65536);
      Keystate_map[btn] = val;

      if (btn == JSKEY_LT) {
        logitech_data_res.LT = Keystate_map[btn] / RANGE_MAX;
      } else {
        logitech_data_res.RT = Keystate_map[btn] / RANGE_MAX;
      }
    } else if (btn == JSKEY_LEFTSTICK_X || btn == JSKEY_LEFTSTICK_Y || btn == JSKEY_RIGHTSTICK_X ||
               btn == JSKEY_RIGHTSTICK_Y) {
      /* y-axis reverse */
      val = (-1) * val;

      val = val * RANGE_MAX / 32767;
      Keystate_map[btn] = val;

      if (btn == JSKEY_LEFTSTICK_X) {
        logitech_data_res.leftStickYAnalog = Keystate_map[btn] / RANGE_MAX;
      } else if (btn == JSKEY_LEFTSTICK_Y) {
        logitech_data_res.leftStickXAnalog = Keystate_map[btn] / RANGE_MAX;
      } else if (btn == JSKEY_RIGHTSTICK_X) {
        logitech_data_res.rightStickYAnalog = Keystate_map[btn] / RANGE_MAX;
      } else {
        logitech_data_res.rightStickXAnalog = Keystate_map[btn] / RANGE_MAX;
      }
    } else {
      switch (val) {
        case JSKEY_PRESS:
          Keystate_map[btn] = 1;
          break;
        case JSKEY_RELEASE:
          Keystate_map[btn] = 0;
          break;
        case static_cast<int>(JSKEY_CROSS_LOW_VALUE):
          Keystate_map[btn] = -1;
          break;
        case JSKEY_CROSS_HIGH_VALUE:
          Keystate_map[btn] = 1;
          break;
        default:
          break;
      }
      /* y-axis reverse */
      if (btn == JSKEY_CROSS_X || btn == JSKEY_CROSS_Y) {
        Keystate_map[btn] = (-1) * Keystate_map[btn];
      }

      if (btn == JSKEY_LB)
        logitech_data_res.LB = Keystate_map[btn];
      else if (btn == JSKEY_RB)
        logitech_data_res.RB = Keystate_map[btn];
      else if (btn == JSKEY_BACK)
        logitech_data_res.BACK = Keystate_map[btn];
      else if (btn == JSKEY_START)
        logitech_data_res.START = Keystate_map[btn];
      else if (btn == JSKEY_HOME)
        logitech_data_res.HOME = Keystate_map[btn];
      else if (btn == JSKEY_A)
        logitech_data_res.A = Keystate_map[btn];
      else if (btn == JSKEY_B)
        logitech_data_res.B = Keystate_map[btn];
      else if (btn == JSKEY_X)
        logitech_data_res.X = Keystate_map[btn];
      else if (btn == JSKEY_Y)
        logitech_data_res.Y = Keystate_map[btn];
      else if (btn == JSKEY_CROSS_X)
        logitech_data_res.CROSS_Y = Keystate_map[btn];
      else if (btn == JSKEY_CROSS_Y)
        logitech_data_res.CROSS_X = Keystate_map[btn];
      else {
      }
    }
  }
  // 在循环结束后，如果读取失败或有其他指示设备可能断开的情况，尝试重新连接
  if (n < 0) {
    int saved_errno = errno;  // 保存当前错误码

    // 检查文件描述符的有效性
    struct stat fd_stat;
    bool device_disconnected = false;

    if (fstat(fd, &fd_stat) < 0 || saved_errno == ENODEV || saved_errno == ENOENT || saved_errno == EIO) {
      device_disconnected = true;
    }

    // 尝试重新打开特定的ioctl调用来检查设备是否真的存在
    if (device_disconnected) {
      errno = 0;
      VLOG(1) << "[ERROR] Failed to read from gamepad: device appears to be disconnected, error: " << strerror(saved_errno);

      // 关闭当前文件描述符
      if (fd >= 0) {
        close(fd);
        fd = -1;
      }

      // 重试打开设备
      if (Init() != EXIT_SUCCESS) {
        VLOG(1) << "[ERROR] Failed to reinitialize gamepad device";
        return EXIT_FAILURE;
      } else {
        VLOG(1) << "[INFO] Successfully reconnected to gamepad device";
      }
    }
  }
  return EXIT_SUCCESS;
}
}  // namespace hardware
