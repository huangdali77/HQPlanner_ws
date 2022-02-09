#include "hqplanner/main/planning.h"

#include <assert.h>
#include <ros/ros.h>

#include <algorithm>
#include <list>
#include <vector>
// #include "modules/planning/common/trajectory/trajectory_stitcher.h"
// #include "modules/planning/planner/em/em_planner.h"
#include "hqplanner/path/discretized_path.h"
#include "hqplanner/trajectory/discretized_trajectory.h"
// #include "modules/planning/reference_line/reference_line_provider.h"
#include "hqplanner/common/frame.h"
#include "hqplanner/common/obstacle.h"
#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/main/anchor_points_provider.h"
#include "hqplanner/main/global_number_provider.h"
#include "hqplanner/main/prediction_obstacles_provider.h"
#include "hqplanner/reference_line/reference_line_provider.h"
#include "hqplanner/tasks/em/em_planner.h"
#include "hqplanner/trajectory/trajectory_stitcher.h"
namespace hqplanner {
using hqplanner::Frame;
using hqplanner::GlobalNumberProvider;
using hqplanner::Obstacle;
using hqplanner::PathDecision;
using hqplanner::PathObstacle;
using hqplanner::forproto::ADCTrajectory;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::Trajectory;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleState;
using hqplanner::forproto::VehicleStateProvider;
using hqplanner::path::DiscretizedPath;
using hqplanner::tasks::EMPlanner;
using hqplanner::tasks::Planner;
using hqplanner::trajectory::DiscretizedTrajectory;
using hqplanner::trajectory::PublishableTrajectory;
using hqplanner::trajectory::TrajectoryStitcher;

Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

bool Planning::Init() {
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(
      AnchorPointsProvider::instance()->GetAnchorPoints());
  planner_ = new EMPlanner();
  return true；
  // return planner_->Init(config_);
}

bool Planning::InitFrame(const std::uint32_t sequence_num,
                         const TrajectoryPoint& planning_start_point,
                         const double start_time,
                         const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, planning_start_point, start_time,
                         vehicle_state, reference_line_provider_.get()));
  bool status = frame_->Init();
  if (!status) {
    return status;
  }
  return true;
}

bool Planning::IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x) || std::isnan(vehicle_state.y) ||
      std::isnan(vehicle_state.z) || std::isnan(vehicle_state.heading) ||
      std::isnan(vehicle_state.kappa) ||
      std::isnan(vehicle_state.linear_velocity) ||
      std::isnan(vehicle_state.linear_acceleration)) {
    return false;
  }
  return true;
}

void Planning::RunOnce() {
  const double start_timestamp = ros::Time::now().toSec();
  VehicleState vehicle_state =
      VehicleStateProvider::instance()->vehicle_state();

  assert(start_timestamp >= vehicle_state.timestamp);

  const double planning_cycle_time =
      1.0 / ConfigParam::FLAGS_planning_loop_rate;

  bool is_replan = false;
  std::vector<TrajectoryPoint> stitching_trajectory;
  stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      last_publishable_trajectory_.get(), &is_replan);

  const std::uint32_t frame_num =
      GlobalNumberProvider::instance()->GetSequenceNum();
  // 初始化本次规划周期内的Frame信息
  InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,
            vehicle_state);

  auto* trajectory_pb = frame_->mutable_trajectory();

  Plan(start_timestamp, stitching_trajectory, trajectory_pb);

  // 更新下一个规划周期的adc状态信息
  TrajectoryPoint& next_cycle_trajectory_point =
      trajectory_pb->trajectory_point.at(1);

  VehicleState next_cycle_state;
  next_cycle_state.x = next_cycle_trajectory_point.path_point.x;
  next_cycle_state.y = next_cycle_trajectory_point.path_point.y;
  next_cycle_state.z = next_cycle_trajectory_point.path_point.z;
  next_cycle_state.timestamp =
      vehicle_state.timestamp + next_cycle_trajectory_point.relative_time;

  next_cycle_state.heading = next_cycle_trajectory_point.path_point.theta;
  next_cycle_state.kappa = next_cycle_trajectory_point.path_point.kappa;
  next_cycle_state.linear_velocity = next_cycle_trajectory_point.v;
  next_cycle_state.linear_acceleration = next_cycle_trajectory_point.a;

  // 更新下一个规划周期的信息
  // 先更新adc状态信息
  VehicleStateProvider::instance()->UpdateNextCycleVehicleState(
      next_cycle_state);

  // 再更新障碍物信息（因为障碍物的出现与消失与adc的位置有关）
  PredictionObstaclesProvider::instance()->UpdataNextCyclePredictionObstacles();
  // PublishPlanningPb(trajectory_pb, start_timestamp);

  // auto seq_num = frame_->SequenceNum();
}
// ===================================测试planning所用到的工具======================
// ===================================================================================
// void Planning::RunOnce() {
//   const double start_timestamp = ros::Time::now().toSec();

//   VehicleState vehicle_state =
//       VehicleStateProvider::instance()->vehicle_state();
//   ROS_INFO("s:%f", start_timestamp);
//   ROS_INFO("v:%f", vehicle_state.timestamp);
//   assert(start_timestamp >= vehicle_state.timestamp);

//   const double planning_cycle_time =
//       1.0 / ConfigParam::instance()->FLAGS_planning_loop_rate;

//   bool is_replan = false;
//   std::vector<TrajectoryPoint> stitching_trajectory;
//   stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
//       vehicle_state, start_timestamp, planning_cycle_time,
//       last_publishable_trajectory_.get(), &is_replan);

//   const std::uint32_t frame_num =
//       GlobalNumberProvider::instance()->GetSequenceNum();
//   // 初始化本次规划周期内的Frame信息
//   InitFrame(frame_num, stitching_trajectory.back(), start_timestamp,
//             vehicle_state);

//   auto* trajectory_pb = frame_->mutable_trajectory();

//   Plan(start_timestamp, stitching_trajectory, trajectory_pb);
//   // 更新下一个规划周期的信息
//   // 先更新adc状态信息
//   // 更新下一个规划周期的adc状态信息
//   const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();

//   const PathObstacle* path_obs_adc =
//   best_ref_info->path_decision().Find("0"); const auto& adc_traj =
//   path_obs_adc->obstacle()->Trajectory(); if
//   (adc_traj.trajectory_point.size() > 2) {
//     const TrajectoryPoint& next_cycle_trajectory_point =
//         adc_traj.trajectory_point.at(1);
//     VehicleState next_cycle_state;
//     next_cycle_state.x = next_cycle_trajectory_point.path_point.x;
//     next_cycle_state.y = next_cycle_trajectory_point.path_point.y;
//     next_cycle_state.z = next_cycle_trajectory_point.path_point.z;
//     next_cycle_state.timestamp =
//         vehicle_state.timestamp + next_cycle_trajectory_point.relative_time;

//     next_cycle_state.heading = next_cycle_trajectory_point.path_point.theta;
//     next_cycle_state.kappa = next_cycle_trajectory_point.path_point.kappa;
//     next_cycle_state.linear_velocity = next_cycle_trajectory_point.v;
//     next_cycle_state.linear_acceleration = next_cycle_trajectory_point.a;
//     VehicleStateProvider::instance()->UpdateNextCycleVehicleState(
//         next_cycle_state);
//   }

//   // 再更新障碍物信息（因为障碍物的出现与消失与adc的位置有关）
//   PredictionObstaclesProvider::instance()->UpdataNextCyclePredictionObstacles();
//   // PublishPlanningPb(trajectory_pb, start_timestamp);

//   // auto seq_num = frame_->SequenceNum();
// }

// ===================================测试planning所用到的工具======================
// ===================================================================================
void Planning::Stop() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
}

void Planning::SetLastPublishableTrajectory(
    const hqplanner::forproto::ADCTrajectory& adc_trajectory) {
  last_publishable_trajectory_.reset(new PublishableTrajectory(adc_trajectory));
}

bool Planning::Plan(const double current_time_stamp,
                    const std::vector<TrajectoryPoint>& stitching_trajectory,
                    ADCTrajectory* trajectory_pb) {
  bool status = planner_->Plan(stitching_trajectory.back(), frame_.get());
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (!best_ref_info) {
    ROS_INFO("planner failed to make a driving plan");
    return false;
  }
  last_publishable_trajectory_.reset(new PublishableTrajectory(
      current_time_stamp, best_ref_info->trajectory()));

  last_publishable_trajectory_->PrependTrajectoryPoints(
      stitching_trajectory.begin(), stitching_trajectory.end() - 1);

  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

  return true;
}

// ===================================测试planning所用到的工具======================
// ===================================================================================
// bool Planning::Plan(const double current_time_stamp,
//                     const std::vector<TrajectoryPoint>& stitching_trajectory,
//                     ADCTrajectory* trajectory_pb) {
//   // bool status = planner_->Plan(stitching_trajectory.back(), frame_.get());
//   // const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
//   // if (!best_ref_info) {
//   //   ROS_INFO("planner failed to make a driving plan");
//   //   return false;
//   // }
//   // 获得下一个adc规划信息
//   auto& reference_line_info = frame_->reference_line_info().front();
//   // const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();

//   const PathObstacle* path_obs_adc =
//       reference_line_info.path_decision()->Find("0");

//   const auto& adc_traj = path_obs_adc->obstacle()->Trajectory();
//   DiscretizedTrajectory adc_trajectory;
//   for (auto traj_point : adc_traj.trajectory_point) {
//     adc_trajectory.AppendTrajectoryPoint(traj_point);
//   }

//   // 将adc轨迹信息存放到ref line info
//   reference_line_info.SetTrajectory(adc_trajectory);

//   const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();

//   last_publishable_trajectory_.reset(new PublishableTrajectory(
//       current_time_stamp, best_ref_info->trajectory()));

//   last_publishable_trajectory_->PrependTrajectoryPoints(
//       stitching_trajectory.begin(), stitching_trajectory.end() - 1);

//   last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

//   return true;
// }
// ===================================测试planning所用到的工具======================
// ===================================================================================

}  // namespace hqplanner
