#include "hqplanner/tasks/dp_st_speed/st_graph_point.h"

namespace hqplanner {
namespace tasks {
using hqplanner::speed::STPoint;
std::uint32_t StGraphPoint::index_s() const { return index_s_; }

std::uint32_t StGraphPoint::index_t() const { return index_t_; }

const STPoint& StGraphPoint::point() const { return point_; }

const StGraphPoint* StGraphPoint::pre_point() const { return pre_point_; }

float StGraphPoint::reference_cost() const { return reference_cost_; }

float StGraphPoint::obstacle_cost() const { return obstacle_cost_; }

float StGraphPoint::total_cost() const { return total_cost_; }

void StGraphPoint::Init(const std::uint32_t index_t,
                        const std::uint32_t index_s, const STPoint& st_point) {
  index_t_ = index_t;
  index_s_ = index_s;
  point_ = st_point;
}

void StGraphPoint::SetReferenceCost(const float reference_cost) {
  reference_cost_ = reference_cost;
}

void StGraphPoint::SetObstacleCost(const float obs_cost) {
  obstacle_cost_ = obs_cost;
}

void StGraphPoint::SetTotalCost(const float total_cost) {
  total_cost_ = total_cost;
}

void StGraphPoint::SetPrePoint(const StGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}

}  // namespace tasks
}  // namespace hqplanner
