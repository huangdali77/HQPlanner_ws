#include "hqplanner/tasks/em/em_planner.h"

#include <assert.h>
#include <ros/ros.h>

#include <fstream>
#include <limits>
#include <utility>

#include "hqplanner/common/frame.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_state.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/math/math_utils.h"
#include "hqplanner/tasks/dp_poly_path/dp_poly_path_optimizer.h"
#include "hqplanner/tasks/path_decider/path_decider.h"
#include "hqplanner/tasks/poly_st_speed/poly_st_speed_optimizer.h"
#include "hqplanner/tasks/speed_decider/speed_decider.h"
#include "hqplanner/util/util.h"
namespace hqplanner {
namespace tasks {
using hqplanner::FrameHistory;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PathPoint;
using hqplanner::forproto::SLPoint;
using hqplanner::forproto::SpeedPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::math::QuinticPolynomialCurve1d;
using hqplanner::math::Vec2d;
using hqplanner::path::DiscretizedPath;
using hqplanner::path::PathData;
using hqplanner::speed::SpeedData;
using hqplanner::tasks::DpPolyPathOptimizer;
using hqplanner::trajectory::DiscretizedTrajectory;
namespace {
constexpr double kPathOptimizationFallbackClost = 2e4;
constexpr double kSpeedOptimizationFallbackClost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

// bool EMPlanner::Init(const PlanningConfig& config) {
//   // RegisterTasks();
//   for (const auto task : config.em_planner_config.task) {
//     tasks_.emplace_back(
//         task_factory_.CreateObject(static_cast<TaskType>(task)));
//     AINFO << "Created task:" << tasks_.back()->Name();
//   }
//   for (auto& task : tasks_) {
//     if (!task->Init(config)) {
//       std::string msg(
//           common::util::StrCat("Init task[", task->Name(), "] failed."));
//       AERROR << msg;
//       return Status(ErrorCode::PLANNING_ERROR, msg);
//     }
//   }
//   return Status::OK();
// }

EMPlanner::EMPlanner() {
  std::unique_ptr<DpPolyPathOptimizer> dp_poly_path_optimizer_ptr(
      new DpPolyPathOptimizer);
  tasks_.emplace_back(dp_poly_path_optimizer_ptr);
}

bool EMPlanner::Plan(const TrajectoryPoint& planning_start_point,
                     Frame* frame) {
  bool has_drivable_reference_line = false;
  bool disable_low_priority_path = false;
  // auto status =
  //     Status(ErrorCode::PLANNING_ERROR, "reference line not drivable");
  for (auto& reference_line_info : frame->reference_line_info()) {
    if (disable_low_priority_path) {
      reference_line_info.SetDrivable(false);
    }
    if (!reference_line_info.IsDrivable()) {
      continue;
    }
    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);
    if (cur_status && reference_line_info.IsDrivable()) {
      has_drivable_reference_line = true;
      disable_low_priority_path = true;
      // if (FLAGS_prioritize_change_lane &&
      //     reference_line_info.IsChangeLanePath() &&
      //     reference_line_info.Cost() < kStraightForwardLineCost) {
      //   disable_low_priority_path = true;
      // }
    } else {
      reference_line_info.SetDrivable(false);
    }
  }
  return has_drivable_reference_line;
}

bool EMPlanner::PlanOnReferenceLine(const TrajectoryPoint& planning_start_point,
                                    Frame* frame,
                                    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }

  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
  auto speed_profile =
      GenerateInitSpeedProfile(planning_start_point, reference_line_info);
  if (speed_profile.empty()) {
    speed_profile = GenerateSpeedHotStart(planning_start_point);
    ROS_INFO("Using dummy hot start for speed vector");
  }
  heuristic_speed_data->set_speed_vector(speed_profile);

  bool ret;

  for (auto& optimizer : tasks_) {
    const double start_timestamp = ros::Time::now().toSec();
    ret = optimizer->Execute(frame, reference_line_info);
    if (!ret) {
      ROS_INFO("Failed to run tasks[%s]", optimizer->Name());
      break;
    }
    const double end_timestamp = ros::Time::now().toSec();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ROS_INFO("after optimizer [%s] time spend: %f", optimizer->Name(),
             time_diff_ms);
  }

  if (reference_line_info->path_data().Empty()) {
    ROS_INFO("Path fallback.");

    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackClost);
  }

  if (!ret || reference_line_info->speed_data().Empty()) {
    ROS_INFO("Speed fallback.");
    GenerateFallbackSpeedProfile(reference_line_info,
                                 reference_line_info->mutable_speed_data());
    reference_line_info->AddCost(kSpeedOptimizationFallbackClost);
  }

  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time, planning_start_point.path_point.s,
          &trajectory)) {
    ROS_INFO("Fail to aggregate planning trajectory.");

    return false;
  }

  for (const auto* path_obstacle :
       reference_line_info->path_decision()->path_obstacle_items()) {
    if (path_obstacle->obstacle()->IsVirtual()) {
      continue;
    }
    if (!path_obstacle->obstacle()->IsStatic()) {
      continue;
    }
    if (path_obstacle->LongitudinalDecision().has_stop()) {
      constexpr double kRefrenceLineStaticObsCost = 1e3;
      reference_line_info->AddCost(kRefrenceLineStaticObsCost);
    }
  }

  // if (FLAGS_enable_trajectory_check) {
  //   if (!ConstraintChecker::ValidTrajectory(trajectory)) {
  //     std::string msg("Failed to validate current planning trajectory.");
  //     AERROR << msg;
  //     return Status(ErrorCode::PLANNING_ERROR, msg);
  //   }
  // }

  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  return true;
}

std::vector<SpeedPoint> EMPlanner::GenerateInitSpeedProfile(
    const TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info) {
  std::vector<SpeedPoint> speed_profile;
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    ROS_INFO("last frame is empty");
    return speed_profile;
  }
  const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();
  if (!last_reference_line_info) {
    ROS_INFO("last reference line info is empty");
    return speed_profile;
  }

  // ====================NoNeed===============
  // if (!reference_line_info->IsStartFrom(*last_reference_line_info)) {
  //   ROS_INFO("Current reference line is not started previous drived line");
  //   return speed_profile;
  // }
  // ====================NoNeed===============
  const auto& last_speed_vector =
      last_reference_line_info->speed_data().speed_vector();

  if (!last_speed_vector.empty()) {
    const auto& last_init_point = last_frame->PlanningStartPoint().path_point;
    Vec2d last_xy_point(last_init_point.x, last_init_point.y);
    SLPoint last_sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(last_xy_point,
                                                           &last_sl_point)) {
      ROS_INFO("Fail to transfer xy to sl when init speed profile");
    }

    Vec2d xy_point(planning_init_point.path_point.x,
                   planning_init_point.path_point.y);
    SLPoint sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(xy_point,
                                                           &sl_point)) {
      ROS_INFO("Fail to transfer xy to sl when init speed profile");
    }

    double s_diff = sl_point.s - last_sl_point.s;
    double start_time = 0.0;
    double start_s = 0.0;
    bool is_updated_start = false;
    for (const auto& speed_point : last_speed_vector) {
      if (speed_point.s < s_diff) {
        continue;
      }
      if (!is_updated_start) {
        start_time = speed_point.t;
        start_s = speed_point.s;
        is_updated_start = true;
      }
      SpeedPoint refined_speed_point;
      refined_speed_point.s = speed_point.s - start_s;
      refined_speed_point.t = speed_point.t - start_time;
      refined_speed_point.v = speed_point.v;
      refined_speed_point.a = speed_point.a;
      refined_speed_point.da = speed_point.da;
      speed_profile.push_back(std::move(refined_speed_point));
    }
  }
  return speed_profile;
}

// This is a dummy simple hot start, need refine later
std::vector<SpeedPoint> EMPlanner::GenerateSpeedHotStart(
    const TrajectoryPoint& planning_init_point) {
  std::vector<SpeedPoint> hot_start_speed_profile;
  double s = 0.0;
  double t = 0.0;
  double v = hqplanner::math::Clamp(
      planning_init_point.v, 5.0,
      ConfigParam::instance()->FLAGS_planning_upper_speed_limit);
  while (t < ConfigParam ::instance()->FLAGS_trajectory_time_length) {
    SpeedPoint speed_point;
    speed_point.s = s;
    speed_point.t = t;
    speed_point.v = v;

    hot_start_speed_profile.push_back(std::move(speed_point));

    t += ConfigParam::instance()->FLAGS_trajectory_time_min_interval;
    s += v * ConfigParam::instance()->FLAGS_trajectory_time_min_interval;
  }
  return hot_start_speed_profile;
}

void EMPlanner::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  auto adc_point = reference_line_info->AdcPlanningPoint();
  double adc_s = reference_line_info->AdcSlBoundary().end_s;
  const double max_s = 150.0;
  const double unit_s = 1.0;

  // projection of adc point onto reference line
  const auto& adc_ref_point =
      reference_line_info->reference_line().GetReferencePoint(adc_s);

  if (adc_point.path_point.x == 0 && adc_point.path_point.y == 0 &&
      adc_point.path_point.s == 0) {
    assert(0);
  }
  const double dx = adc_point.path_point.x - adc_ref_point.x;
  const double dy = adc_point.path_point.y - adc_ref_point.y;

  std::vector<PathPoint> path_points;
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point =
        reference_line_info->reference_line().GetReferencePoint(adc_s);
    PathPoint path_point = hqplanner::util::MakePathPoint(
        ref_point.x + dx, ref_point.y + dy, 0.0, ref_point.heading,
        ref_point.kappa, ref_point.dkappa, 0.0);
    path_point.s = s;

    path_points.push_back(std::move(path_point));
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

void EMPlanner::GenerateFallbackSpeedProfile(
    const ReferenceLineInfo* reference_line_info, SpeedData* speed_data) {
  *speed_data = GenerateStopProfileFromPolynomial(
      reference_line_info->AdcPlanningPoint().v,
      reference_line_info->AdcPlanningPoint().a);

  if (speed_data->Empty()) {
    *speed_data =
        GenerateStopProfile(reference_line_info->AdcPlanningPoint().v,
                            reference_line_info->AdcPlanningPoint().a);
  }
}

SpeedData EMPlanner::GenerateStopProfile(const double init_speed,
                                         const double init_acc) const {
  ROS_INFO("Slowing down the car.");

  SpeedData speed_data;

  const double kFixedJerk = -1.0;
  const double first_point_acc = std::fmin(0.0, init_acc);

  const double max_t = 3.0;
  const double unit_t = 0.02;

  double pre_s = 0.0;
  const double t_mid =
      (ConfigParam::instance()->FLAGS_slowdown_profile_deceleration -
       first_point_acc) /
      kFixedJerk;
  const double s_mid = init_speed * t_mid +
                       0.5 * first_point_acc * t_mid * t_mid +
                       1.0 / 6.0 * kFixedJerk * t_mid * t_mid * t_mid;
  const double v_mid =
      init_speed + first_point_acc * t_mid + 0.5 * kFixedJerk * t_mid * t_mid;

  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    if (t <= t_mid) {
      s = std::fmax(pre_s, init_speed * t + 0.5 * first_point_acc * t * t +
                               1.0 / 6.0 * kFixedJerk * t * t * t);
      v = std::fmax(
          0.0, init_speed + first_point_acc * t + 0.5 * kFixedJerk * t * t);
      const double a = first_point_acc + kFixedJerk * t;
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      pre_s = s;
    } else {
      s = std::fmax(
          pre_s,
          s_mid + v_mid * (t - t_mid) +
              0.5 *
                  ConfigParam::instance()->FLAGS_slowdown_profile_deceleration *
                  (t - t_mid) * (t - t_mid));
      v = std::fmax(
          0.0,
          v_mid +
              (t - t_mid) *
                  ConfigParam::instance()->FLAGS_slowdown_profile_deceleration);
      speed_data.AppendSpeedPoint(
          s, t, v, ConfigParam::instance()->FLAGS_slowdown_profile_deceleration,
          0.0);
    }
    pre_s = s;
  }
  return speed_data;
}

SpeedData EMPlanner::GenerateStopProfileFromPolynomial(
    const double init_speed, const double init_acc) const {
  ROS_INFO("Slowing down the car with polynomial.");

  constexpr double kMaxT = 4.0;
  for (double t = 2.0; t <= kMaxT; t += 0.5) {
    for (double s = 0.0; s < 50.0; s += 1.0) {
      QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, s, 0.0, 0.0, t);
      if (!IsValidProfile(curve)) {
        continue;
      }
      constexpr double kUnitT = 0.02;
      SpeedData speed_data;
      for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
        const double curve_s = curve.Evaluate(0, curve_t);
        const double curve_v = curve.Evaluate(1, curve_t);
        const double curve_a = curve.Evaluate(2, curve_t);
        const double curve_da = curve.Evaluate(3, curve_t);
        speed_data.AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a,
                                    curve_da);
      }
      return speed_data;
    }
  }
  return SpeedData();
}

bool EMPlanner::IsValidProfile(const QuinticPolynomialCurve1d& curve) const {
  for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength();
       evaluate_t += 0.2) {
    const double v = curve.Evaluate(1, evaluate_t);
    const double a = curve.Evaluate(2, evaluate_t);
    constexpr double kEpsilon = 1e-3;
    if (v < -kEpsilon || a < -5.0) {
      return false;
    }
  }
  return true;
}

}  // namespace tasks
}  // namespace hqplanner
