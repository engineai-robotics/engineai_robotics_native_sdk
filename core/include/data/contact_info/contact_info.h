#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

namespace data {

enum class ContactType {
  kPoint,
  kLine,
  kRectangularSurface,
};

static const std::unordered_map<std::string, ContactType> contact_type_map(
    {std::pair<std::string, ContactType>("point", ContactType::kPoint),
     std::pair<std::string, ContactType>("line", ContactType::kLine),
     std::pair<std::string, ContactType>("rectangular", ContactType::kRectangularSurface)});

class ContactInfo {
 public:
  ContactInfo() { Reset(); }
  ~ContactInfo() = default;

  void Reset() {
    name = "";
    belonging = "";
    type = ContactType::kPoint;

    frame = "";
    vertices_frame.clear();
    vertices_offset.clear();

    friction = 0.0;
    fz_lb = 0.0;
    fz_ub = std::numeric_limits<double>::infinity();

    confidence_interval = 0.0;

    verbose = false;
  }

  std::string name = "";
  std::string belonging = "";
  data::ContactType type = data::ContactType::kPoint;

  std::string frame = "";
  std::vector<std::string> vertices_frame;
  std::vector<Eigen::Vector3d> vertices_offset;

  double friction = 0.0;
  double fz_lb = 0.0;
  double fz_ub = std::numeric_limits<double>::infinity();

  double confidence_interval = 0.0;

  bool verbose = false;
};
}  // namespace data
