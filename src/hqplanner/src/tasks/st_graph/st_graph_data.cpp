#include "hqplanner/tasks/st_graph/st_graph_data.h"
namespace hqplanner {
namespace tasks {

using hqplanner::forproto::TrajectoryPoint;
using hqplanner::speed::SpeedLimit;
using hqplanner::speed::StBoundary;
StGraphData::StGraphData(const std::vector<const StBoundary*>& st_boundaries,
                         const TrajectoryPoint& init_point,
                         const SpeedLimit& speed_limit,
                         const double path_data_length)
    : st_boundaries_(st_boundaries),
      init_point_(init_point),
      speed_limit_(speed_limit),
      path_data_length_(path_data_length) {}

const std::vector<const StBoundary*>& StGraphData::st_boundaries() const {
  return st_boundaries_;
}

const TrajectoryPoint& StGraphData::init_point() const { return init_point_; }

const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }

double StGraphData::path_data_length() const { return path_data_length_; }

}  // namespace tasks
}  // namespace hqplanner
