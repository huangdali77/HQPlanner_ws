#include "hqplanner/tasks/poly_st_speed/poly_st_graph.h"

#include <algorithm>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/math/curve1d/quartic_polynomial_curve1d.h"
#include "hqplanner/tasks/poly_st_speed/speed_profile_cost.h"
#include "hqplanner/util/util.h"

namespace hqplanner {
namespace tasks {
namespace {
constexpr double kEpsilon = 1e-6;
}
using hqplanner::PathObstacle;
using hqplanner::ReferenceLineInfo;
using hqplanner::forproto::PolyStSpeedConfig;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::math::QuarticPolynomialCurve1d;
using hqplanner::speed::SpeedData;
using hqplanner::speed::SpeedLimit;
using hqplanner::speed::STPoint;

PolyStGraph::PolyStGraph(const PolyStSpeedConfig &config,
                         const ReferenceLineInfo *reference_line_info,
                         const SpeedLimit &speed_limit)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info->reference_line()),
      speed_limit_(speed_limit) {}

bool PolyStGraph::FindStTunnel(
    const TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    SpeedData *const speed_data) {
  assert(speed_data != nullptr);
  //   CHECK_NOTNULL(speed_data);

  // set init point
  init_point_ = init_point;
  unit_s_ = std::fmax(0.1, init_point_.v / 4.0);

  // sample end points
  std::vector<std::vector<STPoint>> points;
  if (!SampleStPoints(&points)) {
    ROS_INFO("Fail to sample st points.");
    assert(0);
    return false;
  }

  PolyStGraphNode min_cost_node;
  if (!GenerateMinCostSpeedProfile(points, obstacles, &min_cost_node)) {
    // AERROR << "Fail to search min cost speed profile.";
    return false;
  }
  //   ADEBUG << "min_cost_node s = " << min_cost_node.st_point.s()
  //          << ", t = " << min_cost_node.st_point.t();
  speed_data->Clear();
  constexpr double delta_t = 0.1;  // output resolution, in seconds
  const auto curve = min_cost_node.speed_profile;
  for (double t = 0.0; t < planning_time_; t += delta_t) {
    const double s = curve.Evaluate(0, t);
    const double v = curve.Evaluate(1, t);
    const double a = curve.Evaluate(2, t);
    const double da = curve.Evaluate(3, t);
    speed_data->AppendSpeedPoint(s, t, v, a, da);
  }
  return true;
}

bool PolyStGraph::GenerateMinCostSpeedProfile(
    const std::vector<std::vector<STPoint>> &points,
    const std::vector<const PathObstacle *> &obstacles,
    PolyStGraphNode *const min_cost_node) {
  assert(min_cost_node != nullptr);
  //   CHECK_NOTNULL(min_cost_node);
  PolyStGraphNode start_node = {STPoint(0.0, 0.0), init_point_.v,
                                init_point_.a};
  SpeedProfileCost cost(config_, obstacles, speed_limit_, init_point_);
  double min_cost = std::numeric_limits<double>::max();
  for (const auto &level : points) {
    for (const auto &st_point : level) {
      const double speed_limit = speed_limit_.GetSpeedLimitByS(st_point.s());
      constexpr int num_speed = 10;
      for (double v = 0; v < speed_limit + kEpsilon;
           v += speed_limit / num_speed) {
        PolyStGraphNode node = {st_point, v, 0.0};
        node.speed_profile = QuarticPolynomialCurve1d(
            0.0, start_node.speed, start_node.accel, node.st_point.s(),
            node.speed, node.st_point.t());
        const double c =
            cost.Calculate(node.speed_profile, st_point.t(), min_cost);
        if (c < min_cost) {
          *min_cost_node = node;
          min_cost = c;
        }
      }
    }
  }
  return true;
}

bool PolyStGraph::SampleStPoints(
    std::vector<std::vector<STPoint>> *const points) {
  assert(points != nullptr);
  //   CHECK_NOTNULL(points);
  constexpr double start_t = 6.0;
  constexpr double start_s = 0.0;

  for (double t = start_t; t <= planning_time_; t += unit_t_) {
    std::vector<STPoint> level_points;
    for (double s = start_s; s < planning_distance_ + kEpsilon; s += unit_s_) {
      level_points.emplace_back(s, t);
    }
    points->push_back(std::move(level_points));
  }
  return true;
}

}  // namespace tasks
}  // namespace hqplanner
