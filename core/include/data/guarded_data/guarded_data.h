#ifndef DATA_SAFE_SHARED_DATA_SAFE_SHARED_DATA_H_
#define DATA_SAFE_SHARED_DATA_SAFE_SHARED_DATA_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>

/**
 * @class GuardedData
 * @brief A thread-safe wrapper for shared data.
 *
 * The `GuardedData` class provides a thread-safe way to access and modify shared data. It uses a shared mutex to ensure
 * that multiple threads can read the data simultaneously, while only allowing one thread to modify the data at a time.
 *
 * @tparam T The type of the shared data.
 */
namespace data {

template <typename T>
class GuardedData {
 public:
  GuardedData() { data_ = std::make_shared<T>(); }
  explicit GuardedData(const T& data) { data_ = std::make_shared<T>(data); }

  GuardedData(const GuardedData&) = delete;
  GuardedData& operator=(GuardedData&&) = delete;
  GuardedData(GuardedData&&) = delete;
  GuardedData& operator=(const GuardedData&) = delete;
  ~GuardedData() = default;

  void operator=(const T& data) {
    std::unique_lock lock(mutex_);
    data_ = std::make_shared<T>(data);
  }

  void operator=(T&& data) {
    std::unique_lock lock(mutex_);
    data_ = std::make_shared<T>(std::move(data));
  }

  T operator()() const {
    std::shared_lock lock(mutex_);
    return *data_;
  }

  void Set(const T& data) {
    std::unique_lock lock(mutex_);
    data_ = std::make_shared<T>(data);
  }

  void Set(T&& data) {
    std::unique_lock lock(mutex_);
    data_ = std::make_shared<T>(std::move(data));
  }

  std::shared_ptr<const T> Get() const {
    std::shared_lock lock(mutex_);
    return data_;
  }

  void Clear() {
    std::unique_lock lock(mutex_);
    data_.reset();
  }

 private:
  mutable std::shared_mutex mutex_;
  std::shared_ptr<T> data_;
};
}  // namespace data
#endif  // DATA_SAFE_SHARED_DATA_SAFE_SHARED_DATA_H_
