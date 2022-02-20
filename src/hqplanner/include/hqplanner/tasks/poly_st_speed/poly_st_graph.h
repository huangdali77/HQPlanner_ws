#ifndef HQPLANNER_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_
#define HQPLANNER_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/poly_st_speed_config.h"
#include "hqplanner/math/curve1d/quartic_polynomial_curve1d.h"
#include "hqplanner/math/curve1d/quartic_polynomial_curve1d_pro.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/reference_line/reference_line_info.h"
#include "hqplanner/speed/speed_data.h"
#include "hqplanner/speed/speed_limit.h"
#include "hqplanner/trajectory/discretized_trajectory.h"
namespace hqplanner {
namespace tasks {

class PolyStGraph {
 public:
  explicit PolyStGraph(const forproto::PolyStSpeedConfig &config,
                       const ReferenceLineInfo *reference_line_info,
                       const speed::SpeedLimit &speed_limit);

  ~PolyStGraph() = default;

  bool FindStTunnel(
      const forproto::TrajectoryPoint &init_point,
      const std::vector<const hqplanner::PathObstacle *> &obstacles,
      hqplanner::speed::SpeedData *const speed_data);

 private:
  struct PolyStGraphNode {
   public:
    PolyStGraphNode() = default;

    PolyStGraphNode(const speed::STPoint &point_st, const double speed,
                    const double accel)
        : st_point(point_st), speed(speed), accel(accel) {}

    speed::STPoint st_point;
    double speed = 0.0;
    double accel = 0.0;
    hqplanner::math::QuarticPolynomialCurve1dPro speed_profile;
  };

  bool GenerateMinCostSpeedProfile(
      const std::vector<std::vector<speed::STPoint>> &points,
      const std::vector<const PathObstacle *> &obstacles,
      PolyStGraphNode *const min_cost_node);

  bool SampleStPoints(
      std::vector<std::vector<hqplanner::speed::STPoint>> *const points);

 private:
  hqplanner::forproto::PolyStSpeedConfig config_;
  hqplanner::forproto::TrajectoryPoint init_point_;
  const ReferenceLineInfo *reference_line_info_ = nullptr;
  const ReferenceLine &reference_line_;
  const hqplanner::speed::SpeedLimit &speed_limit_;

  double unit_t_ = 1.0;
  double unit_s_ = 5.0;
  double planning_distance_ = 100.0;
  double planning_time_ = 6.0;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_POLY_ST_SPEED_POLY_ST_GRAPH_H_
