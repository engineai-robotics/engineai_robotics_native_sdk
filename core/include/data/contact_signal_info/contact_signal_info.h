#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "guarded_data/guarded_data.h"
#include "model_param/model_param.h"

namespace data {

enum class ContactSignal {
  kNonContact = 0,
  kContact = 1,
};

class ContactSignalInfo {
 public:
  ContactSignalInfo() = default;
  ContactSignalInfo(const std::shared_ptr<data::ModelParam>& model_param);
  ~ContactSignalInfo() = default;

  void Reset();

  void SetContactSignal(const Eigen::VectorX<data::ContactSignal>& contact_signal);
  void SetContactPhase(const Eigen::VectorXd& contact_phase);
  void SetContactSignal(const std::string& contact_name, data::ContactSignal contact_signal);
  void SetContactPhase(const std::string& contact_name, double contact_phase);
  void SetAllContactSignal();
  void SetAllLegContactSignal();
  void SetAllArmContactSignal();
  void SetAllNonContactSignal();
  void SetAllLegNonContactSignal();
  void SetAllArmNonContactSignal();

  const Eigen::VectorX<data::ContactSignal> GetContactSignal();
  const Eigen::VectorXd GetContactPhase();
  const Eigen::VectorXd GetContactConfidenceFactor();

  data::ContactSignal GetContactSignal(const std::string& contact_name);
  double GetContactPhase(const std::string& contact_name);
  double GetContactConfidenceFactor(const std::string& contact_name);

 private:
  static constexpr double kMinimumOffset = 0.01;
  static constexpr const char* kLegKeyword = "leg";
  static constexpr const char* kArmKeyword = "arm";

  std::shared_ptr<data::ModelParam> model_param_;

  data::GuardedData<Eigen::VectorX<data::ContactSignal>> contact_signal_;
  data::GuardedData<Eigen::VectorXd> contact_phase_;
  data::GuardedData<Eigen::VectorXd> contact_confidence_factor_;

  std::unordered_map<std::string, int> contact_name_map_;
  std::vector<std::string> leg_contact_name_;
  std::vector<std::string> arm_contact_name_;
  std::vector<std::string> other_contact_name_;
  std::vector<double> contact_confidence_interval_;
  std::vector<double> contact_confidence_value_;
  int num_contacts_ = 0;
};
}  // namespace data
