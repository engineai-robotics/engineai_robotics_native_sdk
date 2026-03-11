#ifndef DATA__PARAM_MODEL_PARAM_MODEL_PARAM_H_
#define DATA__PARAM_MODEL_PARAM_MODEL_PARAM_H_

#include "basic_param/basic_param.h"
#include "contact_param/contact_param.h"

namespace data {

using LimbId = int;

class LimbInfo {
 public:
  LimbInfo() { Reset(); }
  ~LimbInfo() = default;

  void Reset() {
    name = "";
    joints.clear();
  }

  std::string name = "";
  std::vector<std::string> joints;
};

class ModelParam : public BasicParam {
 public:
  ModelParam(std::string_view tag = "model") : BasicParam(tag) { SetModelInfo(); };

  bool SetModelInfo();
  std::string GetJointNameByJointId(int joint_id);
  std::vector<std::string> GetJointNamesByJointIds(const std::vector<int>& joint_ids);
  void Log();

  DEFINE_PARAM_SCOPE(scope_);
  std::string LOAD_PARAM(urdf);
  std::string LOAD_PARAM(xml);
  std::string LOAD_PARAM(floating_base_type);
  bool LOAD_PARAM(verbose);
  std::vector<data::LimbInfo> LOAD_PARAM(limbs);
  std::optional<std::vector<data::LimbInfo>> LOAD_PARAM(parallel_groups);
  std::vector<std::string> LOAD_PARAM(end_effector_frames);
  std::vector<data::ContactInfo> LOAD_PARAM(contact);

  // TODO(qsy): Add check for `end_effector_frames` and `limbs` size, use virtual/read-only
  void Update() {
    LOAD_PARAM(urdf);
    LOAD_PARAM(xml);
    LOAD_PARAM(floating_base_type);
    LOAD_PARAM(verbose);
    LOAD_PARAM(limbs);
    LOAD_PARAM(parallel_groups);
    LOAD_PARAM(end_effector_frames);
    LOAD_PARAM(contact);
  }

  // A mapping for strings of limb name to its LimbId(int)
  //  e.g. "left_leg" -> 0, "right_leg" -> 1, "left_arm" -> 2, "right_arm" -> 3
  std::unordered_map<std::string, LimbId> limb_id;
  // A mapping for strings of joint name to pairs of its LimbId and joint index in its limb
  // e.g. "J01_HIP_ROLL_L" -> {0, 0}, "J06_ANKLE_ROLL_L" -> {0, 5}, "J07_HIP_ROLL_R" -> {1, 0}
  std::unordered_map<std::string, std::pair<LimbId, int>> joint_id_in_single_limb;
  // A mapping for strings of joint name to its joint index in total limb
  // e.g. "J01_HIP_ROLL_L" -> 0, "J06_ANKLE_ROLL_L" -> 5, "J07_HIP_ROLL_R" -> 6
  std::unordered_map<std::string, int> joint_id_in_total_limb;
  // A mapping for LimbId to its number of joints
  // e.g. LimbId 0 -> 6, LimbId 1 -> 6, LimbId 2 -> 4, LimbId 3 -> 4
  std::unordered_map<LimbId, int> num_joints_per_limb;
  // A mapping for LimbId to its front joint index in total joints
  // e.g. LimbId 0 -> 0, LimbId 1 -> 6, LimbId 2 -> 12, LimbId 3 -> 16
  std::unordered_map<LimbId, int> first_joint_id_per_limb;
  // A mapping for strings of end effector frame name to its end effector id
  // e.g. "LINK_FOOT_L" -> 0, "LINK_FOOT_R" -> 1
  std::unordered_map<std::string, int> end_effector_id;
  std::vector<std::string> limb_names;
  std::vector<std::string> leg_names;
  std::vector<std::string> arm_names;
  std::vector<std::string> waist_names;
  std::vector<std::string> head_names;
  std::vector<std::string> contact_names;

  int num_limbs = 0;
  int num_legs = 0;
  int num_arms = 0;
  int num_waists = 0;
  int num_heads = 0;
  int num_total_joints = 0;
  int num_total_leg_joints = 0;
  int num_total_arm_joints = 0;
  int num_total_waist_joints = 0;
  int num_total_head_joints = 0;
  int num_total_end_effectors = 0;
  int num_contacts = 0;
};
}  // namespace data

namespace YAML {

template <>
struct convert<data::LimbInfo> {
  static bool decode(const Node& node, data::LimbInfo& param);
};
}  // namespace YAML

#endif  // DATA__PARAM_MODEL_PARAM_MODEL_PARAM_H_
