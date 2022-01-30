

#include <utility>

// #include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "hqplanner/trajectory/publishable_trajectory.h"
namespace hqplanner {
namespace trajectory {
using hqplanner::forproto::ADCTrajectory;

PublishableTrajectory::PublishableTrajectory(
    const double header_time,
    const DiscretizedTrajectory& discretized_trajectory)
    : DiscretizedTrajectory(discretized_trajectory),
      header_time_(header_time) {}

PublishableTrajectory::PublishableTrajectory(const ADCTrajectory& trajectory_pb)
    : DiscretizedTrajectory(trajectory_pb),
      header_time_(trajectory_pb.header_time) {}

double PublishableTrajectory::header_time() const { return header_time_; }

void PublishableTrajectory::PopulateTrajectoryProtobuf(
    ADCTrajectory* trajectory_pb) const {
  trajectory_pb->header_time = header_time_;

  trajectory_pb->trajectory_point.assign(trajectory_points_.begin(),
                                         trajectory_points_.end());

  if (!trajectory_points_.empty()) {
    const auto& last_tp = trajectory_points_.back();
    trajectory_pb->total_path_length = last_tp.path_point.s;
    trajectory_pb->total_path_time = last_tp.relative_time;
  }
}

}  // namespace trajectory
}  // namespace hqplanner
