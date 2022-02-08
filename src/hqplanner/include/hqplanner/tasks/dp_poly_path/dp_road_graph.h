#ifndef HQPLANNER_TASKS_DP_ROAD_GRAPH_H_
#define HQPLANNER_TASKS_DP_ROAD_GRAPH_H_

#include <limits>
#include <list>
#include <vector>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/dp_poly_path_config.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/math/curve1d/quintic_polynomial_curve1d.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/reference_line/reference_line_info.h"
#include "hqplanner/speed/speed_data.h"
#include "hqplanner/tasks/dp_poly_path/trajectory_cost.h"
#include "hqplanner/trajectory/discretized_trajectory.h"
namespace hqplanner {
namespace tasks {

class DPRoadGraph {
 public:
  explicit DPRoadGraph(const hqplanner::forproto::DpPolyPathConfig &config,
                       const hqplanner::ReferenceLineInfo &reference_line_info,
                       const hqplanner::speed::SpeedData &speed_data);

  ~DPRoadGraph() = default;

  bool FindPathTunnel(
      const hqplanner::forproto::TrajectoryPoint &init_point,
      const std::vector<const hqplanner::PathObstacle *> &obstacles,
      hqplanner::path::PathData *const path_data);

 private:
  /**
   * an private inner struct for the dp algorithm
   */
  struct DPRoadGraphNode {
   public:
    DPRoadGraphNode() = default;

    DPRoadGraphNode(const hqplanner::forproto::SLPoint point_sl,
                    const DPRoadGraphNode *node_prev)
        : sl_point(point_sl), min_cost_prev_node(node_prev) {}

    DPRoadGraphNode(const hqplanner::forproto::SLPoint point_sl,
                    const DPRoadGraphNode *node_prev,
                    const ComparableCost &cost)
        : sl_point(point_sl), min_cost_prev_node(node_prev), min_cost(cost) {}

    void UpdateCost(const DPRoadGraphNode *node_prev,
                    const math::QuinticPolynomialCurve1d &curve,
                    const ComparableCost &cost) {
      if (cost <= min_cost) {
        min_cost = cost;
        min_cost_prev_node = node_prev;
        min_cost_curve = curve;
      }
    }

    hqplanner::forproto::SLPoint sl_point;
    const DPRoadGraphNode *min_cost_prev_node = nullptr;
    ComparableCost min_cost = {true, true, true,
                               std::numeric_limits<float>::infinity(),
                               std::numeric_limits<float>::infinity()};
    math::QuinticPolynomialCurve1d min_cost_curve;
  };

  bool GenerateMinCostPath(
      const std::vector<const hqplanner::PathObstacle *> &obstacles,
      std::vector<DPRoadGraphNode> *min_cost_path);

  bool SamplePathWaypoints(
      const hqplanner::forproto::TrajectoryPoint &init_point,
      std::vector<std::vector<hqplanner::forproto::SLPoint>> *const points);

  bool CalculateFrenetPoint(
      const hqplanner::forproto::TrajectoryPoint &traj_point,
      hqplanner::forproto::FrenetFramePoint *const frenet_frame_point);

  bool IsValidCurve(const math::QuinticPolynomialCurve1d &curve) const;

  void GetCurveCost(TrajectoryCost trajectory_cost,
                    const math::QuinticPolynomialCurve1d &curve,
                    const float start_s, const float end_s,
                    const uint32_t curr_level, const uint32_t total_level,
                    ComparableCost *cost);

  void UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                  const uint32_t level, const uint32_t total_level,
                  TrajectoryCost *trajectory_cost, DPRoadGraphNode *front,
                  DPRoadGraphNode *cur_node);
  bool HasSidepass();

 private:
  hqplanner::forproto::DpPolyPathConfig config_;
  hqplanner::forproto::TrajectoryPoint init_point_;
  const hqplanner::ReferenceLineInfo &reference_line_info_;
  const hqplanner::ReferenceLine &reference_line_;
  hqplanner::speed::SpeedData speed_data_;
  hqplanner::forproto::SLPoint init_sl_point_;
  hqplanner::forproto::FrenetFramePoint init_frenet_frame_point_;

  hqplanner::forproto::ObjectSidePass sidepass_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif