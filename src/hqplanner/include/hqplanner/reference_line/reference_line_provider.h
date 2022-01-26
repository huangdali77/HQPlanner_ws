#ifndef HQPLANNER_REFERENCE_LINE_PROVIDER_H_
#define HQPLANNER_REFERENCE_LINE_PROVIDER_H_
#include <list>

#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/vehicle_state.h"
#include "hqplanner/reference_line/reference_line.h"

namespace hqplanner {

class ReferenceLineProvider {
 public:
  explicit ReferenceLineProvider(
      std::vector<std::vector<hqplanner::forproto::AnchorPoint>> anchor_points);
  bool AddReferenceLine(ReferenceLine ref_line) {
    reference_lines_.emplace_back(ref_line);
  }

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines);
  static double LookForwardDistance(const VehicleState& state);

 private:
  std::list<ReferenceLine> reference_lines_;
  // Subscribe subscribe_info_;
  std::vector<std::vector<hqplanner::forproto::AnchorPoint>> anchor_points_;
};

}  // namespace hqplanner

#endif