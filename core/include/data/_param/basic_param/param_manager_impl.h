#ifndef DATA__PARAM_BASIC_PARAM_PARAM_MANAGER_IMPL_H_
#define DATA__PARAM_BASIC_PARAM_PARAM_MANAGER_IMPL_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include "basic_param_impl.h"

namespace data {

class ParamManager {
 public:
  template <typename T>
  struct Deleter {
    void operator()(T* ptr) const {
      if (params_.find(ptr->GetTag()) != params_.end()) {
        params_[ptr->GetTag()].erase(static_cast<BasicParam*>(ptr));
      }
      delete ptr;
    }
  };

  /**
   * Creates a new shared pointer to an instance of type T, initializing it with the provided arguments.
   * The created instance is added to a set of instances for tracking purposes.
   * When the shared pointer is deleted, the instance is also removed from the tracking set and deleted.
   *
   * @tparam T The type of the object to create.
   * @tparam Args The types of the arguments to forward to the constructor of T.
   * @param args The arguments to forward to the constructor of T.
   * @return A shared pointer to the newly created instance of T.
   */
  template <typename T, typename... Args>
  static std::shared_ptr<T> create(Args&&... args) {
    static_assert(std::is_base_of_v<BasicParam, T>, "T must be derived from BasicParam");
    std::shared_ptr<T> instance(new T(std::forward<Args>(args)...), Deleter<T>());
    params_[instance->GetTag()].insert(static_cast<BasicParam*>(instance.get()));

    return instance;
  }

  /**
   * Gets a shared_ptr to a parameter with the specified tag
   * If the parameter doesn't exist and create_if_not_exist is true, creates a new instance of type T
   *
   * @tparam T The type of parameter to create (must be derived from BasicParam)
   * @param tag The tag of the parameter
   * @param create_if_not_exist Whether to create the parameter if it doesn't exist
   * @return A shared_ptr to the parameter, or nullptr if not found and create_if_not_exist is false
   */
  template <typename T>
  static std::shared_ptr<T> GetParamByTag(const std::string& tag, bool create_if_not_exist = true) {
    static_assert(std::is_base_of_v<BasicParam, T>, "T must be derived from BasicParam");

    auto iter = params_.find(tag);
    if (iter == params_.end() || iter->second.empty()) {
      if (create_if_not_exist) {
        return create<T>(tag);
      }
      return nullptr;
    }

    // Try to find a valid BasicParam in the set
    for (auto* param_ptr : iter->second) {
      if (param_ptr != nullptr) {
        // Try to cast to the requested type
        auto* cast_ptr = dynamic_cast<T*>(param_ptr);
        if (cast_ptr != nullptr) {
          return std::shared_ptr<T>(cast_ptr, [](T*) {});
        }
      }
    }

    if (create_if_not_exist) {
      return create<T>(tag);
    }
    return nullptr;
  }

  // Keep the original non-template version for backward compatibility
  static std::shared_ptr<BasicParam> GetParamByTag(const std::string& tag, bool create_if_not_exist = true);

  static std::map<std::string, std::unordered_set<BasicParam*>>& GetParams() { return params_; }

 private:
  static std::map<std::string, std::unordered_set<BasicParam*>> params_;
};

}  // namespace data

#endif  // DATA__PARAM_BASIC_PARAM_PARAM_MANAGER_IMPL_H_
