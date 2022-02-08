#include "hqplanner/tasks/st_graph/speed_limit_decider.h"

#include <assert.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/util/util.h"
namespace hqplanner {
namespace tasks {
using hqplanner::ReferenceLine;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::ObjectNudge;
using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::StBoundaryConfig;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedLimit;
SpeedLimitDecider::SpeedLimitDecider(const SLBoundary& adc_sl_boundary,
                                     const StBoundaryConfig& config,
                                     const ReferenceLine& reference_line,
                                     const PathData& path_data)
    : adc_sl_boundary_(adc_sl_boundary),
      st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(
          VehicleConfigHelper::instance()->GetConfig().vehicle_param) {}

void SpeedLimitDecider::GetAvgKappa(const std::vector<PathPoint>& path_points,
                                    std::vector<double>* kappa) const {
  assert(kappa != nullptr);

  const int kHalfNumPoints = st_boundary_config_.num_points_to_avg_kappa / 2;
  assert(kHalfNumPoints > 0);
  //   CHECK_GT(kHalfNumPoints, 0);
  kappa->clear();
  kappa->resize(path_points.size());
  double sum = 0.0;
  int start = 0;
  int end = 0;
  while (end < static_cast<int>(path_points.size()) &&
         end - start < kHalfNumPoints + 1) {
    sum += path_points[end].kappa;
    ++end;
  }

  int iter = 0;
  while (iter < static_cast<int>(path_points.size())) {
    kappa->at(iter) = sum / (end - start);
    if (start < iter - kHalfNumPoints) {
      sum -= path_points[start].kappa;
      ++start;
    }
    if (end < static_cast<int>(path_points.size())) {
      sum += path_points[end].kappa;
      ++end;
    }
    ++iter;
  }
}

bool SpeedLimitDecider::GetSpeedLimits(
    const std::map<std::string, std::shared_ptr<PathObstacle>>& path_obstacles,
    SpeedLimit* const speed_limit_data) const {
  assert(speed_limit_data != nullptr);
  //   CHECK_NOTNULL(speed_limit_data);

  std::vector<double> avg_kappa;
  GetAvgKappa(path_data_.discretized_path().path_points(), &avg_kappa);
  const auto& discretized_path_points =
      path_data_.discretized_path().path_points();
  const auto& frenet_path_points = path_data_.frenet_frame_path().points();
  for (uint32_t i = 0; i < discretized_path_points.size(); ++i) {
    const double path_s = discretized_path_points.at(i).s;
    const double frenet_point_s = frenet_path_points.at(i).s;
    if (frenet_point_s > reference_line_.Length()) {
      return false;
    }

    // (1) speed limit from map
    double speed_limit_on_reference_line =
        reference_line_.GetSpeedLimitFromS(frenet_point_s);

    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration)
    const double centri_acc_speed_limit = std::sqrt(
        GetCentricAccLimit(std::fabs(avg_kappa[i])) /
        std::fmax(std::fabs(avg_kappa[i]), st_boundary_config_.minimal_kappa));

    // -- 2.2: limit by centripetal jerk
    double centri_jerk_speed_limit = std::numeric_limits<double>::max();
    if (i + 1 < discretized_path_points.size()) {
      const double ds =
          discretized_path_points.at(i + 1).s - discretized_path_points.at(i).s;
      assert(ds >= 0.0);

      const double kEpsilon = 1e-9;
      const double centri_jerk =
          std::fabs(avg_kappa[i + 1] - avg_kappa[i]) / (ds + kEpsilon);
      centri_jerk_speed_limit = std::fmax(
          10.0, st_boundary_config_.centri_jerk_speed_coeff / centri_jerk);
    }

    // (3) speed limit from nudge obstacles
    double nudge_obstacle_speed_limit = std::numeric_limits<double>::max();
    for (const auto& const_path_obstacle : path_obstacles) {
      if (const_path_obstacle.second->obstacle()->IsVirtual()) {
        continue;
      }
      if (!const_path_obstacle.second->LateralDecision().has_nudge()) {
        continue;
      }

      /* ref line:
       * -------------------------------
       *    start_s   end_s
       * ------|  adc   |---------------
       * ------------|  obstacle |------
       */
      if (frenet_point_s + vehicle_param_.front_edge_to_center <
              const_path_obstacle.second->PerceptionSLBoundary().start_s ||
          frenet_point_s - vehicle_param_.back_edge_to_center >
              const_path_obstacle.second->PerceptionSLBoundary().end_s) {
        continue;
      }
      constexpr double kRange = 1.0;  // meters
      const auto& nudge = const_path_obstacle.second->LateralDecision().nudge();

      // Please notice the differences between adc_l and frenet_point_l
      const double frenet_point_l = frenet_path_points.at(i).l;

      // obstacle is on the right of ego vehicle (at path point i)
      bool is_close_on_left =
          (nudge.type == ObjectNudge::LEFT_NUDGE) &&
          (frenet_point_l - vehicle_param_.right_edge_to_center - kRange <
           const_path_obstacle.second->PerceptionSLBoundary().end_l);

      // obstacle is on the left of ego vehicle (at path point i)
      bool is_close_on_right =
          (nudge.type == ObjectNudge::RIGHT_NUDGE) &&
          (const_path_obstacle.second->PerceptionSLBoundary().start_l - kRange <
           frenet_point_l + vehicle_param_.left_edge_to_center);

      if (is_close_on_left || is_close_on_right) {
        double nudge_speed_ratio = 1.0;
        if (const_path_obstacle.second->obstacle()->IsStatic()) {
          nudge_speed_ratio = st_boundary_config_.static_obs_nudge_speed_ratio;
        } else {
          nudge_speed_ratio = st_boundary_config_.dynamic_obs_nudge_speed_ratio;
        }
        nudge_obstacle_speed_limit =
            nudge_speed_ratio * speed_limit_on_reference_line;
        break;
      }
    }

    double curr_speed_limit = 0.0;
    if (ConfigParam::instance()->FLAGS_enable_nudge_slowdown) {
      curr_speed_limit =
          std::fmax(st_boundary_config_.lowest_speed,
                    hqplanner::util::MinElement(std::vector<double>{
                        speed_limit_on_reference_line, centri_acc_speed_limit,
                        centri_jerk_speed_limit, nudge_obstacle_speed_limit}));
    } else {
      curr_speed_limit =
          std::fmax(st_boundary_config_.lowest_speed,
                    hqplanner::util::MinElement(std::vector<double>{
                        speed_limit_on_reference_line, centri_acc_speed_limit,
                        centri_jerk_speed_limit}));
    }

    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
  }
  return true;
}

double SpeedLimitDecider::GetCentricAccLimit(const double kappa) const {
  // this function uses a linear model with upper and lower bound to determine
  // centric acceleration limit

  // suppose acc = k1 * v + k2
  // consider acc = v ^ 2 * kappa
  // we determine acc by the two functions above, with uppper and lower speed
  // bounds
  const double v_high = st_boundary_config_.high_speed_threshold;
  const double v_low = st_boundary_config_.low_speed_threshold;

  const double h_v_acc =
      st_boundary_config_.high_speed_centric_acceleration_limit;
  const double l_v_acc =
      st_boundary_config_.low_speed_centric_acceleration_limit;

  if (std::fabs(v_high - v_low) < 1.0) {
    // AERROR << "High speed and low speed threshold are too close to each
    // other. "
    //           "Please check config file."
    //        << " Current high speed threshold = " << v_high
    //        << ", current low speed threshold = " << v_low;
    return h_v_acc;
  }
  const double kMinKappaEpsilon = 1e-9;
  if (kappa < kMinKappaEpsilon) {
    return h_v_acc;
  }

  const double k1 = (h_v_acc - l_v_acc) / (v_high - v_low);
  const double k2 = h_v_acc - v_high * k1;

  const double v = (k1 + std::sqrt(k1 * k1 + 4.0 * kappa * k2)) / (2.0 * kappa);

  if (v > v_high) {
    return h_v_acc;
  } else if (v < v_low) {
    return l_v_acc;
  } else {
    return v * k1 + k2;
  }
}

}  // namespace tasks
}  // namespace hqplanner
