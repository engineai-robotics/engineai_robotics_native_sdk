#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace data {

class MotorDebug {
 public:
  MotorDebug();
  ~MotorDebug();

  // 拷贝构造函数
  MotorDebug(const MotorDebug& other);

  // 拷贝赋值运算符
  MotorDebug& operator=(const MotorDebug& other);

  // 移动构造函数
  MotorDebug(MotorDebug&& other) noexcept;

  // 移动赋值运算符
  MotorDebug& operator=(MotorDebug&& other) noexcept;

  /**
   * @brief 预分配所有向量的容量
   * @param capacity 每个向量的容量
   * @note 调用此方法可避免后续多次内存分配
   */
  void Reserve(size_t capacity);

  /**
   * @brief 调整所有向量的大小
   * @param size 新的大小
   */
  void Resize(size_t size);

  /**
   * @brief 高效交换两个对象的内容
   * @param other 另一个对象
   */
  void Swap(MotorDebug& other) noexcept;

  /**
   * @brief 获取当前向量大小
   */
  size_t Size() const;

  /**
   * @brief 清空所有向量
   */
  void Clear();

  /**
   * @brief 从C库的motor_debug_t和motor_error_t拷贝数据
   * @param motor_debug 电机调试数据指针
   * @param motor_error 电机错误数据指针
   * @note 自动处理维度和bool到uint8_t的转换
   */
  void CopyFromCDriver(const void* motor_debug, const void* motor_error);

  std::vector<float> mos_temperature;
  std::vector<float> motor_temperature;
  std::vector<float> voltage;
  std::vector<float> current;
  std::vector<int> error_code;
  std::vector<uint8_t> offline;
  std::vector<uint8_t> enable;
};

}  // namespace data