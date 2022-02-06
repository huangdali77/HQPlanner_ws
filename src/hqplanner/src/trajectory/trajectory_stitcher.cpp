// #include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "hqplanner/trajectory/trajectory_stitcher.h"

#include <algorithm>
#include <list>
#include <utility>

// #include "modules/common/configs/config_gflags.h"
// #include "modules/common/log.h"
// #include "modules/common/math/quaternion.h"
// #include "modules/common/util/util.h"
#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/util/util.h"
// #include "modules/planning/common/planning_gflags.h"
#include "hqplanner/math/vec2d.h"
namespace hqplanner {
namespace trajectory {

using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleState;
using hqplanner::math::Vec2d;
// using hqplanner::util::DistanceXY;

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;
  init_point.path_point.x = vehicle_state.x;
  init_point.path_point.y = vehicle_state.y;
  init_point.path_point.z = vehicle_state.z;
  init_point.path_point.theta = vehicle_state.heading;
  init_point.path_point.kappa = vehicle_state.kappa;
  init_point.v = vehicle_state.linear_velocity;
  init_point.a = vehicle_state.linear_acceleration;
  init_point.relative_time = 0.0;
  //   init_point.mutable_path_point()->set_y(vehicle_state.y());
  //   init_point.mutable_path_point()->set_z(vehicle_state.z());
  //   init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  //   init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  //   init_point.set_v(vehicle_state.linear_velocity());
  //   init_point.set_a(vehicle_state.linear_acceleration());
  //   init_point.set_relative_time(0.0);

  return std::vector<TrajectoryPoint>(1, init_point);
}

// Planning from current vehicle state:
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory, bool* is_replan) {
  *is_replan = true;
  if (!ConfigParam::instance()->FLAGS_enable_trajectory_stitcher) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (!prev_trajectory) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  //   if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE)
  //   {
  //     return ComputeReinitStitchingTrajectory(vehicle_state);
  //   }

  std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    //   前一个规划周期的轨迹为空，说明是刚刚开始规划的第一个周期
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  std::size_t matched_index = prev_trajectory->QueryNearestPoint(veh_rel_time);

  if (matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (matched_index + 1 >= prev_trajectory_size) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  auto matched_point = prev_trajectory->Evaluate(veh_rel_time);

  //   if (!matched_point.has_path_point()) {
  //     return ComputeReinitStitchingTrajectory(vehicle_state);
  //   }

  auto frenet_sd = ComputePositionProjection(vehicle_state.x, vehicle_state.y,
                                             *prev_trajectory);

  auto lon_diff = std::fabs(frenet_sd.first);
  auto lat_diff = std::fabs(frenet_sd.second);

  if (lat_diff >
          ConfigParam::instance()->FLAGS_replan_lateral_distance_threshold ||
      lon_diff > ConfigParam::instance()
                     ->FLAGS_replan_longitudinal_distance_threshold) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  double forward_rel_time =
      prev_trajectory->TrajectoryPointAt(matched_index).relative_time +
      planning_cycle_time;

  std::size_t forward_index =
      prev_trajectory->QueryNearestPoint(forward_rel_time);

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->trajectory_points().begin() +
          std::max(0, static_cast<int>(matched_index - 1)),
      prev_trajectory->trajectory_points().begin() + forward_index + 1);

  const double zero_s = matched_point.path_point.s;

  for (auto& tp : stitching_trajectory) {
    // if (!tp.has_path_point()) {
    //   return ComputeReinitStitchingTrajectory(vehicle_state);
    // }
    tp.relative_time =
        tp.relative_time + prev_trajectory->header_time() - current_timestamp;
    tp.path_point.s = tp.path_point.s - zero_s;
  }
  *is_replan = false;
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y,
    const PublishableTrajectory& prev_trajectory) {
  auto index = prev_trajectory.QueryNearestPoint({x, y});
  auto p = prev_trajectory.TrajectoryPointAt(index);

  Vec2d v(x - p.path_point.x, y - p.path_point.y);
  Vec2d n(std::cos(p.path_point.theta), std::sin(p.path_point.theta));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point.s;
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace trajectory
}  // namespace hqplanner
