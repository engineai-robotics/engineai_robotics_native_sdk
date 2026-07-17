#ifndef DATA_LINK_INFO_LINK_INFO_H_
#define DATA_LINK_INFO_LINK_INFO_H_

#include "basic_structure/basic_structure.h"
#include "guarded_data/guarded_data.h"

namespace data {

class LinkInfo {
 public:
  LinkInfo() { Reset(); }
  ~LinkInfo() = default;

  void Reset() { frame.Zero(); }

  SE3Frame frame;
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd jacobian_dot_times_qd;
};
}  // namespace data

#endif  // DATA_LINK_INFO_LINK_INFO_H_
