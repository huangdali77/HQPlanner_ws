#include "hqplanner/main/prediction_obstacles_provider.h"

#include <math.h>

#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/reference_line/reference_line.h"
namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::PredictionObstacle;
using hqplanner::forproto::PredictionObstacles;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleState;
using hqplanner::forproto::VehicleStateProvider;
// ====================PotentialPredictionObstacle==============
PotentialPredictionObstacle::PotentialPredictionObstacle(
    hqplanner::forproto::PerceptionObstacle perception_obstacle,
    std::vector<hqplanner::forproto::AnchorPoint> anchor_points)
    : perception_obstacle_(perception_obstacle), anchor_points_(anchor_points) {
  obstacle_reference_trajectory_ = ReferenceLine(anchor_points);
  // 障碍物全部轨迹
  obstacle_reference_points_ =
      obstacle_reference_trajectory_.GetReferenceLinePoints();
  //   障碍物预测轨迹起点状态
  perception_obstacle_.position.x = obstacle_reference_points_.front().x;
  perception_obstacle_.position.y = obstacle_reference_points_.front().y;
  perception_obstacle_.theta = obstacle_reference_points_.front().heading;
  //   障碍物5s轨迹
  double obs_speed = std::sqrt(
      perception_obstacle_.velocity.x * perception_obstacle_.velocity.x +
      perception_obstacle_.velocity.y * perception_obstacle_.velocity.y);
  int trajectory_sample_step =
      int(0.1 * obs_speed / ConfigParam::FLAGS_reference_line_sample_step);

  int trajectory_sample_points_num = ConfigParam::FLAGS_prediction_total_time *
                                     ConfigParam::FLAGS_planning_loop_rate;
  trajectory_point_.clear();
  double relative_time = 0.0;
  for (int i = 0; i < obstacle_reference_points_.size();) {
    auto const& obstacle_reference_point = obstacle_reference_points_[i];
    if (trajectory_point_.size() > 50) {
      break;
    }
    TrajectoryPoint tp;
    tp.path_point.x = obstacle_reference_point.x;
    tp.path_point.y = obstacle_reference_point.y;
    tp.path_point.theta = obstacle_reference_point.heading;
    tp.path_point.s = obstacle_reference_point.s;
    tp.path_point.kappa = obstacle_reference_point.kappa;
    tp.path_point.dkappa = obstacle_reference_point.dkappa;

    tp.v = obs_speed;
    tp.v_x = perception_obstacle_.velocity.x;
    tp.v_y = perception_obstacle_.velocity.y;
    tp.relative_time = relative_time;
    trajectory_point_.emplace_back(std::move(tp));

    relative_time += 0.1;
    i += trajectory_sample_step;
  }
}

// ==============================PredictionObstaclesProvider===============
PredictionObstaclesProvider::PredictionObstaclesProvider() {}
void PredictionObstaclesProvider::Init(
    std::unordered_map<std::int32_t, PotentialPredictionObstacle>
        potential_prediction_obstacles) {
  potential_prediction_obstacles_ = potential_prediction_obstacles;
}

void PredictionObstaclesProvider::UpdataNextCyclePredictionObstacles() {
  // 获得下个规划周期adc的状态
  const VehicleState& vehicle_state =
      VehicleStateProvider::instance()->vehicle_state();
  //   清除远离的障碍物
  std::vector<std::int32_t> clear_obs_id;
  for (const auto& publish_prediction_obstacle :
       publish_prediction_obstacles_) {
    std::int32_t obs_id =
        publish_prediction_obstacle.second.perception_obstacle_.id;

    if (publish_prediction_obstacle.second.trajectory_point_.size() < 20) {
      clear_obs_id.push_back(obs_id);
      continue;
    }
    double dx = std::abs(
        vehicle_state.x -
        publish_prediction_obstacle.second.perception_obstacle_.position.x);
    double dy = std::abs(
        vehicle_state.y -
        publish_prediction_obstacle.second.perception_obstacle_.position.y);
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist >
        potential_prediction_obstacles_[obs_id].disappear_distance_threshold) {
      clear_obs_id.push_back(obs_id);
    }
  }

  for (auto& clear_id : clear_obs_id) {
    publish_prediction_obstacles_.erase(clear_id);
  }

  //添加要显示的障碍物
  for (const auto& potential_prediction_obstacle :
       potential_prediction_obstacles_) {
    auto obs_id = potential_prediction_obstacle.second.perception_obstacle_.id;
    double dx = std::abs(
        vehicle_state.x -
        potential_prediction_obstacle.second.perception_obstacle_.position.x);
    double dy = std::abs(
        vehicle_state.y -
        potential_prediction_obstacle.second.perception_obstacle_.position.y);
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < potential_prediction_obstacle.second.appear_distance_threshold) {
      if (publish_prediction_obstacles_.find(obs_id) ==
          publish_prediction_obstacles_.end()) {
        publish_prediction_obstacles_.insert(
            std::make_pair(obs_id, potential_prediction_obstacle.second));
      }
    }
  }

  // 更新
}
}  // namespace hqplanner
