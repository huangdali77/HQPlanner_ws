

#ifndef HQPLANNER_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
#define HQPLANNER_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_

#include <vector>
// #include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "hqplanner/trajectory/discretized_trajectory.h"
// #include "modules/planning/proto/planning.pb.h"
#include "hqplanner/for_proto/adc_trajectory.h"
namespace hqplanner {
namespace trajectory {

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  /**
   * Create a publishable trajectory based on a trajectory protobuf
   */
  explicit PublishableTrajectory(
      const hqplanner::forproto::ADCTrajectory& trajectory_pb);

  double header_time() const;

  void PopulateTrajectoryProtobuf(
      hqplanner::forproto::ADCTrajectory* trajectory_pb) const;

 private:
  double header_time_ = 0.0;
};

}  // namespace trajectory
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
