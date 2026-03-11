#pragma once

#include <string>
#include <unordered_map>

#include "basic_structure/basic_structure.h"
#include "guarded_data/guarded_data.h"

namespace data {
class MocapRigidBody {
 public:
  MocapRigidBody() = default;
  MocapRigidBody(const std::string& name, const SE3Frame& frame) : name(name), frame(frame) {}
  std::string name;
  SE3Frame frame;
  std::vector<Eigen::Vector3d> markers;
};

class MocapInfo {
 public:
  MocapInfo() = default;
  ~MocapInfo() = default;
  void Log() const;
  std::unordered_map<std::string, MocapRigidBody> rigid_bodies;
};

}  // namespace  data
