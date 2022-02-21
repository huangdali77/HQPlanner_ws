#ifndef HQPLANNING_MAIN_GLOBAL_NUMBER_PROVIDER_H_
#define HQPLANNING_MAIN_GLOBAL_NUMBER_PROVIDER_H_

#include "hqplanner/util/macro.h"
namespace hqplanner {

class GlobalNumberProvider {
 public:
  // GlobalNumberProvider() = default;

  std::uint32_t GetSequenceNum();
  std::int32_t GetPerceptionObstacleId();
  int GetMarkerId();

 private:
  std::uint32_t sequence_num_ = 0;
  std::int32_t perception_obstacle_id_ = 0;
  int marker_id_ = 0;
  DECLARE_SINGLETON(GlobalNumberProvider);
};
}  // namespace hqplanner

#endif