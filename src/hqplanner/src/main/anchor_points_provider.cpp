#include "hqplanner/main/anchor_points_provider.h"

namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
AnchorPointsProvider::AnchorPointsProvider(
    std::vector<std::vector<hqplanner::forproto::AnchorPoint>> anchor_points)
    : anchor_points_(anchor_points) {}

std::vector<std::vector<forproto::AnchorPoint>>
AnchorPointsProvider::GetAnchorPoints() const {
  return anchor_points_;
}
void AnchorPointsProvider::SetAnchorPoints(
    std::vector<std::vector<hqplanner::forproto::AnchorPoint>> anchor_points) {
  anchor_points_ = anchor_points;
}
}  // namespace hqplanner
