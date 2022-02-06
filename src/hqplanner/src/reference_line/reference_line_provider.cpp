#include "hqplanner/reference_line/reference_line_provider.h"
namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::VehicleState;

ReferenceLineProvider::ReferenceLineProvider(
    std::vector<std::vector<AnchorPoint>> anchor_points)
    : anchor_points_(anchor_points) {
  for (auto anchor_points : anchor_points_) {
    ReferenceLine ref(anchor_points);
    AddReferenceLine(ref);
  }
  is_initialized_ = true;
}

bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine>* reference_lines) {
  if (reference_lines_.empty()) {
    return false;
  }
  for (auto ref_line : reference_lines_) {
    reference_lines->push_back(ref_line);
  }
  return true;
}

double ReferenceLineProvider::LookForwardDistance(const VehicleState& state) {
  auto forward_distance = state.linear_velocity *
                          ConfigParam::instance()->FLAGS_look_forward_time_sec;

  if (forward_distance >
      ConfigParam::instance()->FLAGS_look_forward_short_distance) {
    return ConfigParam::instance()->FLAGS_look_forward_long_distance;
  }

  return ConfigParam::instance()->FLAGS_look_forward_short_distance;
}

void ReferenceLineProvider::Stop() { is_stop_ = true; }
bool ReferenceLineProvider::Start() {
  if (!is_initialized_) {
    return false;
  }

  return true;
}
}  // namespace hqplanner
