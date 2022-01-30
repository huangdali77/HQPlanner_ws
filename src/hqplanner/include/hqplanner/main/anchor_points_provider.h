#ifndef HQPLANNING_MAIN_ANCHOR_POINTS_PROVIDER_H_
#define HQPLANNING_MAIN_ANCHOR_POINTS_PROVIDER_H_
#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/util/macro.h"
namespace hqplanner {

class AnchorPointsProvider {
 public:
  //   AnchorPointsProvider() = default;
  explicit AnchorPointsProvider(
      std::vector<std::vector<hqplanner::forproto::AnchorPoint>> anchor_points);
  std::vector<std::vector<forproto::AnchorPoint>> GetAnchorPoints() const;
  void SetAnchorPoints(
      std::vector<std::vector<hqplanner::forproto::AnchorPoint>> anchor_points);

 private:
  std::vector<std::vector<forproto::AnchorPoint>> anchor_points_;

  DECLARE_SINGLETON(AnchorPointsProvider);
};
}  // namespace hqplanner

#endif