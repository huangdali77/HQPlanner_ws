#ifndef HQPLANNER_OBSTACLE_H_
#define HQPLANNER_OBSTACLE_H_

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/prediction_obstacle.h"
#include "hqplanner/math/box2d.h"
#include "hqplanner/math/polygon2d.h"
#include "hqplanner/math/vec2d.h"

namespace hqplanner {

/**
 * @class Obstacle
 *
 * @brief Obstacle represents one perception obstacle.
 */
class Obstacle {
 public:
  Obstacle() = default;

  Obstacle(const std::string &id,
           const hqplanner::forproto::PerceptionObstacle &perception_obstacle);

  Obstacle(const std::string &id,
           const hqplanner::forproto::PerceptionObstacle &perception,
           const hqplanner::forproto::Trajectory &trajectory);

  const std::string &Id() const;
  void SetId(const std::string &id) { id_ = id; }

  std::int32_t PerceptionId() const;

  double Speed() const;

  bool IsStatic() const;
  bool IsVirtual() const;

  hqplanner::forproto::TrajectoryPoint GetPointAtTime(const double time) const;

  hqplanner::math::Box2d GetBoundingBox() const;
  hqplanner::math::Box2d GetBoundingBox(
      const hqplanner::forproto::TrajectoryPoint &point) const;
  /**
   * @brief get the perception bounding box
   */
  const hqplanner::math::Box2d &PerceptionBoundingBox() const;

  /**
   * @brief get the perception polygon for the obstacle. It is more precise than
   * bounding box
   */
  const hqplanner::math::Polygon2d &PerceptionPolygon() const;

  const hqplanner::forproto::Trajectory &Trajectory() const;
  //   TrajectoryPoint *AddTrajectoryPoint();
  //   bool HasTrajectory() const;

  //   const PerceptionObstacle &Perception() const;

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const hqplanner::forproto::PredictionObstacles &predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string &id, const hqplanner::math::Box2d &obstacle_box);

  static bool IsStaticObstacle(
      const hqplanner::forproto::PerceptionObstacle &perception_obstacle);

  static bool IsVirtualObstacle(
      const hqplanner::forproto::PerceptionObstacle &perception_obstacle);

  //   static bool IsValidTrajectoryPoint(const common::TrajectoryPoint &point);

  const hqplanner::forproto::PerceptionObstacle &Perception() const;

 private:
  std::string id_;
  std::int32_t perception_id_ = 0;
  bool is_static_ = true;
  bool is_virtual_ = false;
  double speed_ = 0.0;
  // std::vector<TrajectoryPoint> trajectory_;
  hqplanner::forproto::Trajectory trajectory_;
  hqplanner::forproto::PerceptionObstacle perception_obstacle_;
  hqplanner::math::Box2d perception_bounding_box_;
  hqplanner::math::Polygon2d perception_polygon_;
};

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_OBSTACLE_H_
