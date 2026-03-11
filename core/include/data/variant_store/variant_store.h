#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <typeindex>
#include <vector>

namespace data {

// 前向声明
template <typename T>
class Publisher;

template <typename T>
class Subscriber;

template <typename T>
class GuardedData;

/**
 * @brief 类型安全的变体存储
 *
 * 特性：
 * 1. 支持任意类型的数据存储（同一类型可以有多个key）
 * 2. 线程安全的分段锁设计，减少锁竞争
 * 3. 类型安全的数据访问
 * 4. 版本跟踪支持
 *
 * 使用示例：
 *   auto& store = VariantStore::GetInstance();
 *   store.Set("sensor_1", SensorData{...});
 *   store.Set("sensor_2", SensorData{...});  // 同一类型，不同key
 *   store.Set("config", Config{...});         // 不同类型
 */
class VariantStore {
 public:
  // 桶的数量，使用2的幂次方以便位运算优化
  static constexpr size_t BUCKET_COUNT = 64;

  struct Stats {
    size_t total_entries = 0;
    std::vector<std::pair<std::string, std::string>> key_types;
    std::vector<size_t> bucket_sizes;

    Stats();
    ~Stats();
  };

 private:
  class Impl;
  std::unique_ptr<Impl> data_;

 public:
  /**
   * @brief 获取单例实例
   * @return VariantStore的单例引用
   */
  static VariantStore& GetInstance();

  // 禁止拷贝和移动
  VariantStore(const VariantStore&) = delete;
  VariantStore& operator=(const VariantStore&) = delete;
  VariantStore(VariantStore&&) = delete;
  VariantStore& operator=(VariantStore&&) = delete;

 private:
  /**
   * @brief 私有构造函数，防止外部实例化
   */
  VariantStore();

  /**
   * @brief 析构函数
   */
  ~VariantStore();

 private:
  // 内部辅助函数声明
  void SetSharedImpl(const std::string& key, std::shared_ptr<void> data, const std::type_index& type_info);

 public:
  /**
   * @brief 设置数据（创建副本）
   * @tparam T 数据类型
   * @param key 数据键名
   * @param data 数据值
   */
  template <typename T>
  void Set(const std::string& key, const T& data) {
    SetShared(key, std::make_shared<T>(data));
  }

  /**
   * @brief 设置共享数据（零拷贝）
   * @tparam T 数据类型
   * @param key 数据键名
   * @param data 数据的共享指针
   */
  template <typename T>
  void SetShared(const std::string& key, std::shared_ptr<T> data);

  /**
   * @brief 获取共享数据
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 数据的共享指针，如果不存在或类型不匹配返回nullptr
   */
  template <typename T>
  std::shared_ptr<const T> Get(const std::string& key) const;

  /**
   * @brief 获取数据副本
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 数据的可选值，如果不存在或类型不匹配返回nullopt
   */
  template <typename T>
  std::optional<T> GetCopy(const std::string& key) const {
    auto ptr = Get<T>(key);
    return ptr ? std::make_optional(*ptr) : std::nullopt;
  }

  /**
   * @brief 获取可修改的共享数据（允许修改）
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 数据的共享指针（非const），如果不存在或类型不匹配返回nullptr
   * @note 注意：直接修改返回的数据不会增加版本号
   */
  template <typename T>
  std::shared_ptr<T> GetMutable(const std::string& key) const;

  /**
   * @brief 原子更新操作
   * @tparam T 数据类型
   * @param key 数据键名
   * @param updater 更新函数
   * @return 是否更新成功
   */
  template <typename T>
  bool Update(const std::string& key, std::function<void(T&)> updater);

  /**
   * @brief 获取或创建数据（按需初始化）
   *
   * 如果数据不存在则自动创建并注册到 VariantStore
   * 适用于需要长期持有指针的场景（如 RequestSlot）
   *
   * 使用示例：
   *   auto slot = store.GetOrCreate<common::RequestSlot<std::string>>("motion_state_override_command");
   *   slot->Submit(data);  // 后续直接使用，零查找开销
   *
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 数据的共享指针
   */
  template <typename T>
  std::shared_ptr<T> GetOrCreate(const std::string& key);

  /**
   * @brief 批量设置数据
   * @tparam T 数据类型
   * @param items 键值对列表
   */
  template <typename T>
  void SetBatch(const std::vector<std::pair<std::string, T>>& items);

  /**
   * @brief 检查数据是否存在且类型匹配
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 是否存在且类型匹配
   */
  template <typename T>
  bool HasData(const std::string& key) const;

  /**
   * @brief 检查键是否存在（不检查类型）
   * @param key 数据键名
   * @return 是否存在
   */
  bool HasKey(const std::string& key) const;

  /**
   * @brief 获取数据版本号
   * @param key 数据键名
   * @return 版本号，如果不存在返回0
   */
  uint64_t GetVersion(const std::string& key) const;

  /**
   * @brief 获取存储的类型信息
   * @param key 数据键名
   * @return 类型索引，如果不存在返回nullopt
   */
  std::optional<std::type_index> GetTypeInfo(const std::string& key) const;

  /**
   * @brief 删除数据
   * @param key 数据键名
   * @return 是否成功删除
   */
  bool Remove(const std::string& key);

  /**
   * @brief 清空所有数据
   */
  void Clear();

  /**
   * @brief 获取所有键名
   * @return 键名列表
   */
  std::vector<std::string> GetAllKeys() const;

  /**
   * @brief 获取统计信息
   * @return 统计数据
   */
  Stats GetStats() const;

  /**
   * @brief 获取负载分布（调试用）
   * @return 每个桶的数据量
   */
  std::vector<size_t> GetLoadDistribution() const;

  /**
   * @brief 创建发布者
   *
   * 发布者用于向指定 key 发布数据，避免重复哈希查找
   *
   * 使用示例：
   *   auto pub = store.CreatePublisher<ImuInfo>("imu_info");
   *   pub.Publish(imu_data);  // 不需要每次传 key
   *
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 发布者对象
   */
  template <typename T>
  Publisher<T> CreatePublisher(const std::string& key);

  /**
   * @brief 创建订阅者（无默认值）
   *
   * 订阅者用于从指定 key 读取数据，零哈希开销
   *
   * 使用示例：
   *   auto sub = store.CreateSubscriber<ImuInfo>("imu_info");
   *   const auto& data = sub.Get();  // 直接读取，返回引用，零拷贝
   *   if (auto new_data = sub.GetIfUpdated()) {
   *       // 处理更新的数据
   *   }
   *
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 订阅者对象
   * @note 如果数据不存在，Get() 会返回默认构造的 T{}
   */
  template <typename T>
  Subscriber<T> CreateSubscriber(const std::string& key);

  /**
   * @brief 创建订阅者（带本地默认值）
   *
   * 使用示例：
   *   ImuInfo default_imu{};
   *   auto sub = store.CreateSubscriber<ImuInfo>("imu_info", default_imu);
   *   const auto& data = sub.Get();  // 数据不存在时返回默认值
   *
   * @tparam T 数据类型
   * @param key 数据键名
   * @param default_value 本地默认值（仅影响该订阅者，不发布到 VariantStore）
   * @return 订阅者对象
   */
  template <typename T>
  Subscriber<T> CreateSubscriber(const std::string& key, const T& default_value);

  /**
   * @brief 创建订阅者（自动构造并发布默认值）
   *
   * 如果数据不存在，会自动构造 T{} 并发布到 VariantStore，
   * 这样其他代码也能读到这个默认值
   *
   * 使用示例：
   *   // 如果 "imu_info" 不存在，自动创建 ImuInfo{} 并发布
   *   auto sub = store.CreateSubscriberAutoConstruct<ImuInfo>("imu_info");
   *   const auto& data = sub.Get();  // 始终有效
   *
   * @tparam T 数据类型
   * @param key 数据键名
   * @return 订阅者对象
   */
  template <typename T>
  Subscriber<T> CreateSubscriberAutoConstruct(const std::string& key);

 private:
  // 内部辅助函数
  std::shared_ptr<void> GetImpl(const std::string& key, const std::type_index& type_info) const;
  bool HasDataImpl(const std::string& key, const std::type_index& type_info) const;
  bool UpdateImpl(const std::string& key, const std::type_index& type_info,
                  std::function<std::shared_ptr<void>(const std::shared_ptr<void>&)> updater);
  void SetBatchImpl(const std::vector<std::pair<std::string, std::shared_ptr<void>>>& items,
                    const std::type_index& type_info);
};

/**
 * @brief 键访问器 - 针对单个key提供便捷访问接口（可选工具）
 *
 * 作用：简化对特定key的重复访问，自动跟踪版本变化
 *
 * 使用示例：
 *   KeyAccessor<SensorData> sensor1("sensor_1");
 *
 *   // 简化的访问
 *   sensor1.Set(data);
 *   auto data = sensor1.Get();
 *
 *   // 自动跟踪版本
 *   if (auto new_data = sensor1.GetIfUpdated()) {
 *       // 只在数据更新时才处理
 *   }
 *
 * 注意：这是可选的便利工具，直接使用 VariantStore 也完全可以
 */
template <typename T>
class KeyAccessor {
 public:
  explicit KeyAccessor(std::string key) : key_(std::move(key)), last_version_(0) {}

  // 基本操作（不需要重复传key）
  void Set(const T& data) { VariantStore::GetInstance().Set(key_, data); }
  void SetShared(std::shared_ptr<T> data) { VariantStore::GetInstance().SetShared(key_, data); }

  std::shared_ptr<const T> Get() const { return VariantStore::GetInstance().Get<T>(key_); }
  std::shared_ptr<T> GetMutable() const { return VariantStore::GetInstance().GetMutable<T>(key_); }
  std::optional<T> GetCopy() const { return VariantStore::GetInstance().GetCopy<T>(key_); }

  bool Update(std::function<void(T&)> updater) { return VariantStore::GetInstance().Update<T>(key_, updater); }

  bool Exists() const { return VariantStore::GetInstance().HasData<T>(key_); }
  uint64_t GetVersion() const { return VariantStore::GetInstance().GetVersion(key_); }

  // 版本跟踪功能
  bool HasUpdate() {
    uint64_t current = GetVersion();
    if (current > last_version_) {
      last_version_ = current;
      return true;
    }
    return false;
  }

  std::optional<T> GetIfUpdated() { return HasUpdate() ? GetCopy() : std::nullopt; }

  const std::string& GetKey() const { return key_; }

 private:
  std::string key_;
  mutable uint64_t last_version_;
};

// 模板函数实现
template <typename T>
void VariantStore::SetShared(const std::string& key, std::shared_ptr<T> data) {
  SetSharedImpl(key, std::static_pointer_cast<void>(data), std::type_index(typeid(T)));
}

template <typename T>
std::shared_ptr<const T> VariantStore::Get(const std::string& key) const {
  auto ptr = GetImpl(key, std::type_index(typeid(T)));
  return ptr ? std::static_pointer_cast<const T>(ptr) : nullptr;
}

template <typename T>
std::shared_ptr<T> VariantStore::GetMutable(const std::string& key) const {
  auto ptr = GetImpl(key, std::type_index(typeid(T)));
  return ptr ? std::static_pointer_cast<T>(ptr) : nullptr;
}

template <typename T>
bool VariantStore::Update(const std::string& key, std::function<void(T&)> updater) {
  return UpdateImpl(key, std::type_index(typeid(T)),
                    [updater](const std::shared_ptr<void>& ptr) -> std::shared_ptr<void> {
                      auto typed_ptr = std::static_pointer_cast<T>(ptr);
                      auto new_data = std::make_shared<T>(*typed_ptr);
                      updater(*new_data);
                      return std::static_pointer_cast<void>(new_data);
                    });
}

template <typename T>
std::shared_ptr<T> VariantStore::GetOrCreate(const std::string& key) {
  // 尝试获取已存在的数据
  auto data = GetMutable<T>(key);

  // 如果不存在则创建并注册
  if (!data) {
    data = std::make_shared<T>();
    SetShared(key, data);
  }

  return data;
}

template <typename T>
void VariantStore::SetBatch(const std::vector<std::pair<std::string, T>>& items) {
  std::vector<std::pair<std::string, std::shared_ptr<void>>> void_items;
  void_items.reserve(items.size());

  for (const auto& [key, data] : items) {
    void_items.emplace_back(key, std::static_pointer_cast<void>(std::make_shared<T>(data)));
  }

  SetBatchImpl(void_items, std::type_index(typeid(T)));
}

template <typename T>
bool VariantStore::HasData(const std::string& key) const {
  return HasDataImpl(key, std::type_index(typeid(T)));
}

template <typename T>
Publisher<T> VariantStore::CreatePublisher(const std::string& key) {
  // 尝试获取已存在的 GuardedData
  auto data = GetMutable<GuardedData<T>>(key);

  // 如果不存在，创建新的 GuardedData 并注册
  if (!data) {
    data = std::make_shared<GuardedData<T>>();
    SetShared(key, data);
  }

  return Publisher<T>(key, data);
}

template <typename T>
Subscriber<T> VariantStore::CreateSubscriber(const std::string& key) {
  // 尝试获取已存在的 GuardedData
  auto data = GetMutable<GuardedData<T>>(key);

  // 如果不存在，创建新的 GuardedData 并注册
  if (!data) {
    data = std::make_shared<GuardedData<T>>();
    SetShared(key, data);
  }

  return Subscriber<T>(key, data);
}

template <typename T>
Subscriber<T> VariantStore::CreateSubscriber(const std::string& key, const T& default_value) {
  // 尝试获取已存在的 GuardedData
  auto data = GetMutable<GuardedData<T>>(key);

  // 如果不存在，创建新的 GuardedData 并注册
  if (!data) {
    data = std::make_shared<GuardedData<T>>();
    SetShared(key, data);
  }

  return Subscriber<T>(key, data, default_value);
}

template <typename T>
Subscriber<T> VariantStore::CreateSubscriberAutoConstruct(const std::string& key) {
  // 尝试获取已存在的 GuardedData
  auto data = GetMutable<GuardedData<T>>(key);

  // 如果不存在，创建新的 GuardedData 并发布默认值
  if (!data) {
    data = std::make_shared<GuardedData<T>>();
    SetShared(key, data);

    // 发布默认构造值到 VariantStore
    data->Set(T{});
  }

  // 使用默认构造值作为订阅者的本地默认值
  return Subscriber<T>(key, data, T{});
}

// ==================== 订阅发布辅助类实现 ====================

/**
 * @brief 发布者 - 用于发布数据到 VariantStore
 *
 * 使用场景：
 *   auto pub = store.CreatePublisher<ImuInfo>("imu_info");
 *   pub.Publish(imu_data);  // 直接发布，不需要每次传 key
 *
 * @tparam T 数据类型
 */
template <typename T>
class Publisher {
 public:
  Publisher() : data_(nullptr) {}

  /**
   * @brief 发布数据（拷贝）
   * @param data 要发布的数据
   */
  void Publish(const T& data) {
    if (!data_) {
      return;
    }
    data_->Set(data);
  }

  /**
   * @brief 发布数据（移动）
   * @param data 要发布的数据
   */
  void Publish(T&& data) {
    if (!data_) {
      return;
    }
    data_->Set(std::move(data));
  }

  /**
   * @brief 获取当前发布的数据（只读）
   * @return 当前数据的副本
   */
  std::optional<T> GetCurrent() const {
    if (!data_) {
      return std::nullopt;
    }
    return data_->Get();
  }

  /**
   * @brief 更新数据（原子操作）
   * @param updater 更新函数
   */
  void Update(std::function<void(T&)> updater) {
    if (!data_) {
      return;
    }
    data_->Update(updater);
  }

  /**
   * @brief 检查发布者是否有效
   */
  bool IsValid() const { return data_ != nullptr; }

  /**
   * @brief 获取订阅的 key
   */
  const std::string& GetKey() const { return key_; }

 private:
  friend class VariantStore;

  Publisher(const std::string& key, std::shared_ptr<GuardedData<T>> data) : key_(key), data_(std::move(data)) {}

  std::string key_;
  std::shared_ptr<GuardedData<T>> data_;
};

/**
 * @brief 订阅者 - 用于从 VariantStore 读取数据
 *
 * 使用场景：
 *   auto sub = store.CreateSubscriber<ImuInfo>("imu_info");
 *   const auto& data = sub.Get();  // 直接读取引用，零哈希开销，零拷贝
 *
 * @tparam T 数据类型
 */
template <typename T>
class Subscriber {
 public:
  Subscriber() : data_(nullptr), default_value_(std::make_shared<T>(T{})) {}

  /**
   * @brief 获取数据（直接从 GuardedData 读取最新数据）
   * @return 数据的 shared_ptr
   * @note 性能：shared_lock 开销（约 20-50ns）
   * @note 始终返回最新数据，Publisher 发布后立即可见
   * @note 使用方式：
   *   auto data = sub.Get();
   *   Process(*data);  // 通过 shared_ptr 保证数据生命周期
   */
  std::shared_ptr<const T> Get() const {
    if (!data_) {
      return default_value_;
    }
    auto ptr = data_->Get();
    return ptr ? ptr : default_value_;
  }

  /**
   * @brief 检查订阅者是否有效
   */
  bool IsValid() const { return data_ != nullptr; }

  /**
   * @brief 获取订阅的 key
   */
  const std::string& GetKey() const { return key_; }

 private:
  friend class VariantStore;

  // 无默认值构造
  Subscriber(const std::string& key, std::shared_ptr<GuardedData<T>> data)
      : key_(key), data_(std::move(data)), default_value_(std::make_shared<T>(T{})) {}

  // 带默认值构造
  Subscriber(const std::string& key, std::shared_ptr<GuardedData<T>> data, const T& default_value)
      : key_(key), data_(std::move(data)), default_value_(std::make_shared<T>(default_value)) {}

  std::string key_;
  std::shared_ptr<GuardedData<T>> data_;
  std::shared_ptr<const T> default_value_;
};

}  // namespace data