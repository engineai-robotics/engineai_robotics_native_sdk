#include "gamepad/gamepad_driver.h"

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

TEST(TestGamepad, TestGamepadOut) {
  hardware::LogitechGamepadDriver logitech_gamepad_driver;
  std::cout << logitech_gamepad_driver.Init() << std::endl;

  while (true) {
    logitech_gamepad_driver.ListenInput();

    std::cout << "[INFO] ----------------------------------------------------------------------------" << std::endl;
    std::cout << "[INFO] A: " << logitech_gamepad_driver.logitech_data_res.A << std::endl;
    std::cout << "[INFO] B: " << logitech_gamepad_driver.logitech_data_res.B << std::endl;
    std::cout << "[INFO] X: " << logitech_gamepad_driver.logitech_data_res.X << std::endl;
    std::cout << "[INFO] Y: " << logitech_gamepad_driver.logitech_data_res.Y << std::endl;
    std::cout << "[INFO] LB: " << logitech_gamepad_driver.logitech_data_res.LB << std::endl;
    std::cout << "[INFO] RB: " << logitech_gamepad_driver.logitech_data_res.RB << std::endl;
    std::cout << "[INFO] BACK: " << logitech_gamepad_driver.logitech_data_res.BACK << std::endl;
    std::cout << "[INFO] START: " << logitech_gamepad_driver.logitech_data_res.START << std::endl;
    std::cout << "[INFO] HOME: " << logitech_gamepad_driver.logitech_data_res.HOME << std::endl;
    std::cout << "[INFO] LT: " << logitech_gamepad_driver.logitech_data_res.LT << std::endl;
    std::cout << "[INFO] RT: " << logitech_gamepad_driver.logitech_data_res.RT << std::endl;
    std::cout << "[INFO] CROSS_X: " << logitech_gamepad_driver.logitech_data_res.CROSS_X << std::endl;
    std::cout << "[INFO] CROSS_Y: " << logitech_gamepad_driver.logitech_data_res.CROSS_Y << std::endl;
    std::cout << "[INFO] leftStickXAnalog: " << logitech_gamepad_driver.logitech_data_res.leftStickXAnalog << std::endl;
    std::cout << "[INFO] leftStickYAnalog: " << logitech_gamepad_driver.logitech_data_res.leftStickYAnalog << std::endl;
    std::cout << "[INFO] rightStickXAnalog: " << logitech_gamepad_driver.logitech_data_res.rightStickXAnalog
              << std::endl;
    std::cout << "[INFO] rightStickYAnalog: " << logitech_gamepad_driver.logitech_data_res.rightStickYAnalog
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
