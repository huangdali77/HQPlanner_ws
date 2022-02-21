#include "hqplanner/main/global_number_provider.h"
namespace hqplanner {
GlobalNumberProvider::GlobalNumberProvider() {}
std::uint32_t GlobalNumberProvider::GetSequenceNum() {
  ++sequence_num_;
  return sequence_num_;
}
std::int32_t GlobalNumberProvider::GetPerceptionObstacleId() {
  ++perception_obstacle_id_;
  return perception_obstacle_id_;
}

int GlobalNumberProvider::GetMarkerId() {
  ++marker_id_;
  return marker_id_;
}

}  // namespace hqplanner