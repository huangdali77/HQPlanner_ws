#ifndef HQPLANNER_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
#define HQPLANNER_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_

#include <string>
#include <vector>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/st_boundary_config.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/speed/speed_limit.h"
#include "hqplanner/speed/st_boundary.h"
namespace hqplanner {
namespace tasks {

class StBoundaryMapper {
 public:
  StBoundaryMapper(const hqplanner::forproto::SLBoundary& adc_sl_boundary,
                   const hqplanner::forproto::StBoundaryConfig& config,
                   const ReferenceLine& reference_line,
                   const hqplanner::path::PathData& path_data,
                   const double planning_distance, const double planning_time,
                   const bool is_change_lane);

  virtual ~StBoundaryMapper() = default;

  bool CreateStBoundary(PathDecision* path_decision) const;

  //   bool CreateStBoundaryWithHistory(
  //       const hqplanner::forproto::ObjectDecisions& history_decisions,
  //       PathDecision* path_decision) const;

  bool CreateStBoundary(
      hqplanner::PathObstacle* path_obstacle,
      const hqplanner::forproto::ObjectDecisionType& external_decision) const;

 private:
  //   FRIEND_TEST(StBoundaryMapperTest, check_overlap_test);
  bool CheckOverlap(const hqplanner::forproto::PathPoint& path_point,
                    const hqplanner::math::Box2d& obs_box,
                    const double buffer) const;

  /**
   * Creates valid st boundary upper_points and lower_points
   * If return true, upper_points.size() > 1 and
   * upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(
      const std::vector<hqplanner::forproto::PathPoint>& path_points,
      const Obstacle& obstacle,
      std::vector<hqplanner::speed::STPoint>* upper_points,
      std::vector<hqplanner::speed::STPoint>* lower_points) const;

  bool MapWithoutDecision(PathObstacle* path_obstacle) const;

  bool MapStopDecision(
      PathObstacle* stop_obstacle,
      const hqplanner::forproto::ObjectDecisionType& decision) const;

  bool MapWithDecision(
      PathObstacle* path_obstacle,
      const hqplanner::forproto::ObjectDecisionType& decision) const;

 private:
  const hqplanner::forproto::SLBoundary& adc_sl_boundary_;
  const hqplanner::forproto::StBoundaryConfig& st_boundary_config_;
  const ReferenceLine& reference_line_;
  const hqplanner::path::PathData& path_data_;
  const hqplanner::forproto::VehicleParam& vehicle_param_;
  const double planning_distance_;  // 149m
  const double planning_time_;
  bool is_change_lane_ = false;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_ST_GRAPH_ST_BOUNDARY_MAPPER_H_
