#ifndef MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_
#define MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "hqplanner/common/obstacle.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/sl_boundary.h"
#include "hqplanner/for_proto/st_boundary_config.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/speed/speed_limit.h"

namespace hqplanner {
namespace tasks {

class SpeedLimitDecider {
 public:
  SpeedLimitDecider(const hqplanner::forproto::SLBoundary& adc_sl_boundary,
                    const hqplanner::forproto::StBoundaryConfig& config,
                    const hqplanner::ReferenceLine& reference_line,
                    const hqplanner::path::PathData& path_data);

  virtual ~SpeedLimitDecider() = default;

  virtual bool GetSpeedLimits(
      const std::map<std::string, std::shared_ptr<PathObstacle>>&
          path_obstacles,
      hqplanner::speed::SpeedLimit* const speed_limit_data) const;

 private:
  //   FRIEND_TEST(SpeedLimitDeciderTest, get_centric_acc_limit);
  double GetCentricAccLimit(const double kappa) const;

  void GetAvgKappa(
      const std::vector<hqplanner::forproto::PathPoint>& path_points,
      std::vector<double>* kappa) const;

 private:
  const hqplanner::forproto::SLBoundary& adc_sl_boundary_;
  const hqplanner::forproto::StBoundaryConfig& st_boundary_config_;
  const hqplanner::ReferenceLine& reference_line_;
  const hqplanner::path::PathData& path_data_;
  const hqplanner::forproto::VehicleParam& vehicle_param_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_SPEED_LIMIT_DECIDER_H_
