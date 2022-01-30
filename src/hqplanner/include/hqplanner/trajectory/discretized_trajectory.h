#ifndef HQPLANNER_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define HQPLANNER_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "hqplanner/for_proto/adc_trajectory.h"
#include "hqplanner/for_proto/pnc_point.h"
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
  explicit DiscretizedTrajectory(
      const hqplanner::forproto::ADCTrajectory& trajectory);

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

  virtual std::uint32_t QueryNearestPoint(const double relative_time) const;

  virtual std::uint32_t QueryNearestPoint(
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

  std::uint32_t NumOfPoints() const;

  const std::vector<hqplanner::forproto::TrajectoryPoint>& trajectory_points()
      const;
  std::vector<hqplanner::forproto::TrajectoryPoint>& trajectory_points();

  virtual void Clear();

 protected:
  std::vector<hqplanner::forproto::TrajectoryPoint> trajectory_points_;
};

inline std::uint32_t DiscretizedTrajectory::NumOfPoints() const {
  return trajectory_points_.size();
}

inline const std::vector<hqplanner::forproto::TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() const {
  return trajectory_points_;
}

inline std::vector<hqplanner::forproto::TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() {
  return trajectory_points_;
}

inline void DiscretizedTrajectory::SetTrajectoryPoints(
    const std::vector<hqplanner::forproto::TrajectoryPoint>&
        trajectory_points) {
  trajectory_points_ = trajectory_points;
}

inline void DiscretizedTrajectory::Clear() { trajectory_points_.clear(); }
}  // namespace trajectory
}  // namespace hqplanner

#endif