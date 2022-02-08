#ifndef HQPLANNER_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_
#define HQPLANNER_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_

#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/speed/speed_limit.h"
#include "hqplanner/speed/st_boundary.h"

namespace hqplanner {
namespace tasks {

class StGraphData {
 public:
  StGraphData(
      const std::vector<const hqplanner::speed::StBoundary*>& st_boundaries,
      const hqplanner::forproto::TrajectoryPoint& init_point,
      const hqplanner::speed::SpeedLimit& speed_limit,
      const double path_data_length);
  StGraphData() = default;

  const std::vector<const hqplanner::speed::StBoundary*>& st_boundaries() const;

  const hqplanner::forproto::TrajectoryPoint& init_point() const;

  const hqplanner::speed::SpeedLimit& speed_limit() const;

  double path_data_length() const;

 private:
  std::vector<const hqplanner::speed::StBoundary*> st_boundaries_;
  hqplanner::forproto::TrajectoryPoint init_point_;

  hqplanner::speed::SpeedLimit speed_limit_;
  double path_data_length_ = 0.0;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_GRAPH_DATA_H_
