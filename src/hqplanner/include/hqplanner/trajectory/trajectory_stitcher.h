#ifndef HQPLANNER_TRAJECTORY_TRAJECTORY_STITCHER_H_
#define HQPLANNER_TRAJECTORY_TRAJECTORY_STITCHER_H_

#include <utility>
#include <vector>

// #include "modules/common/proto/pnc_point.pb.h"
#include "hqplanner/for_proto/pnc_point.h"
// #include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "hqplanner/for_proto/vehicle_state.h"
// #include "modules/planning/reference_line/reference_line.h"
#include "hqplanner/reference_line/reference_line.h"
// #include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "hqplanner/trajectory/publishable_trajectory.h"
namespace hqplanner {
namespace trajectory {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static void TransformLastPublishedTrajectory(
      const double x_diff, const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);

  static std::vector<hqplanner::forproto::TrajectoryPoint>
  ComputeStitchingTrajectory(
      const hqplanner::forproto::VehicleState& vehicle_state,
      const double current_timestamp, const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory, bool* is_replan);

 private:
  static std::pair<double, double> ComputePositionProjection(
      const double x, const double y,
      const PublishableTrajectory& prev_trajectory);

  static std::vector<hqplanner::forproto::TrajectoryPoint>
  ComputeReinitStitchingTrajectory(
      const hqplanner::forproto::VehicleState& vehicle_state);
};

}  // namespace trajectory
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
