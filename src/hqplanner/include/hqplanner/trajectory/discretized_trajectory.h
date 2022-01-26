#ifndef HQPLANNER_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define HQPLANNER_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "hqplanner/math/linear_interpolation.h"
#include "hqplanner/math/vec2d.h"
#include "hqplanner/trajectory/trajectory.h"

namespace hqplanner {
namespace trajectory {

class DiscretizedTrajectory : public Trajectory {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  //   explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const std::vector<hqplanner::forproto::TrajectoryPoint>&
          trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<hqplanner::forproto::TrajectoryPoint>&
          trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  hqplanner::forproto::TrajectoryPoint StartPoint() const override;

  double GetTemporalLength() const override;

  double GetSpatialLength() const override;

  hqplanner::forproto::TrajectoryPoint Evaluate(
      const double relative_time) const override;

  virtual uint32_t QueryNearestPoint(const double relative_time) const;

  virtual uint32_t QueryNearestPoint(
      const hqplanner::math::Vec2d& position) const;

  virtual void AppendTrajectoryPoint(
      const hqplanner::forproto::TrajectoryPoint& trajectory_point);

  template <typename Iter>
  void PrependTrajectoryPoints(Iter begin, Iter end) {
    if (!trajectory_points_.empty() && begin != end) {
      CHECK((end - 1)->relative_time() <
            trajectory_points_.front().relative_time());
    }
    trajectory_points_.insert(trajectory_points_.begin(), begin, end);
  }

  const hqplanner::forproto::TrajectoryPoint& TrajectoryPointAt(
      const std::uint32_t index) const;

  uint32_t NumOfPoints() const;

  const std::vector<hqplanner::forproto::TrajectoryPoint>& trajectory_points()
      const;
  std::vector<hqplanner::forproto::TrajectoryPoint>& trajectory_points();

  virtual void Clear();

 protected:
  std::vector<hqplanner::forproto::TrajectoryPoint> trajectory_points_;
};

}  // namespace trajectory
}  // namespace hqplanner

#endif