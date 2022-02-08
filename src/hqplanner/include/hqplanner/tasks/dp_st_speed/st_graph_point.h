#ifndef HQPLANNER_TASKS_ST_GRAPH_ST_GRAPH_POINT_H_
#define HQPLANNER_TASKS_ST_GRAPH_ST_GRAPH_POINT_H_

#include <limits>

#include "hqplanner/speed/st_point.h"

namespace hqplanner {
namespace tasks {

class StGraphPoint {
 public:
  std::uint32_t index_s() const;
  std::uint32_t index_t() const;

  const hqplanner::speed::STPoint& point() const;
  const StGraphPoint* pre_point() const;

  float reference_cost() const;
  float obstacle_cost() const;
  float total_cost() const;

  void Init(const std::uint32_t index_t, const std::uint32_t index_s,
            const hqplanner::speed::STPoint& st_point);

  // given reference speed profile, reach the cost, including position
  void SetReferenceCost(const float reference_cost);

  // given obstacle info, get the cost;
  void SetObstacleCost(const float obs_cost);

  // total cost
  void SetTotalCost(const float total_cost);

  void SetPrePoint(const StGraphPoint& pre_point);

 private:
  hqplanner::speed::STPoint point_;
  const StGraphPoint* pre_point_ = nullptr;
  std::uint32_t index_s_ = 0;
  std::uint32_t index_t_ = 0;

  float reference_cost_ = 0.0;
  float obstacle_cost_ = 0.0;
  float total_cost_ = std::numeric_limits<float>::infinity();
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_POINT_H_
