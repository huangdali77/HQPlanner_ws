#include "hqplanner/main/prediction_obstacles_provider.h"

#include <math.h>
#include <ros/ros.h>

#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/reference_line/reference_line.h"
namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::PredictionObstacle;
using hqplanner::forproto::PredictionObstacles;
using hqplanner::forproto::ReferencePoint;
using hqplanner::forproto::Trajectory;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleState;
using hqplanner::forproto::VehicleStateProvider;
// ====================PotentialPredictionObstacle==============
PotentialPredictionObstacle::PotentialPredictionObstacle(
    hqplanner::forproto::PerceptionObstacle perception_obstacle,
    std::vector<hqplanner::forproto::AnchorPoint> anchor_points)
    : perception_obstacle_(perception_obstacle), anchor_points_(anchor_points) {
  if (anchor_points_.empty()) {
    InitPolygonPoint(perception_obstacle_.polygon_point);
  } else {
    obstacle_reference_trajectory_ = ReferenceLine(anchor_points);
    obstacle_reference_points_ =
        obstacle_reference_trajectory_.reference_points();
    //   障碍物预测轨迹起点状态
    perception_obstacle_.position.x = obstacle_reference_points_.front().x;
    perception_obstacle_.position.y = obstacle_reference_points_.front().y;
    perception_obstacle_.theta = obstacle_reference_points_.front().heading;

    InitPolygonPoint(perception_obstacle_.polygon_point);
    //   障碍物5s轨迹
    double obs_speed = std::sqrt(
        perception_obstacle_.velocity.x * perception_obstacle_.velocity.x +
        perception_obstacle_.velocity.y * perception_obstacle_.velocity.y);
    double planning_duration_time =
        1.0 / ConfigParam::instance()->FLAGS_planning_loop_rate;
    int trajectory_sample_step =
        int(planning_duration_time * obs_speed /
            ConfigParam::instance()->FLAGS_reference_line_sample_step);

    int trajectory_sample_points_num =
        ConfigParam::instance()->FLAGS_prediction_total_time /
        planning_duration_time;
    trajectory_point_.clear();
    double relative_time = 0.0;

    for (int i = 0; i < obstacle_reference_points_.size();) {
      auto const& obstacle_reference_point = obstacle_reference_points_[i];
      if (trajectory_point_.size() > trajectory_sample_points_num) {
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
}

void PotentialPredictionObstacle::InitPolygonPoint(
    std::vector<hqplanner::forproto::Point>& polygon_point) {
  double sin_heading = std::sin(perception_obstacle_.theta);
  double cos_heading = std::cos(perception_obstacle_.theta);
  double half_length = perception_obstacle_.length / 2;
  double half_width = perception_obstacle_.width / 2;

  const double dx1 = cos_heading * half_length;
  const double dy1 = sin_heading * half_length;
  const double dx2 = sin_heading * half_width;
  const double dy2 = -cos_heading * half_width;
  polygon_point.clear();

  hqplanner::forproto::Point point1;
  point1.x = perception_obstacle_.position.x + dx1 + dx2;
  point1.y = perception_obstacle_.position.y + dy1 + dy2;
  polygon_point.emplace_back(std::move(point1));

  hqplanner::forproto::Point point2;
  point2.x = perception_obstacle_.position.x + dx1 - dx2;
  point2.y = perception_obstacle_.position.y + dy1 - dy2;
  polygon_point.emplace_back(std::move(point2));

  hqplanner::forproto::Point point3;
  point3.x = perception_obstacle_.position.x - dx1 - dx2;
  point3.y = perception_obstacle_.position.y - dy1 - dy2;
  polygon_point.emplace_back(std::move(point3));

  hqplanner::forproto::Point point4;
  point4.x = perception_obstacle_.position.x - dx1 + dx2;
  point4.y = perception_obstacle_.position.y - dy1 + dy2;
  polygon_point.emplace_back(std::move(point4));
}

// ==============================PredictionObstaclesProvider===============
PredictionObstaclesProvider::PredictionObstaclesProvider() {}

void PredictionObstaclesProvider::Init(
    std::map<std::int32_t, PotentialPredictionObstacle>
        potential_prediction_obstacles) {
  potential_prediction_obstacles_ = potential_prediction_obstacles;
  UpdataNextCyclePredictionObstacles();
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

    // if (publish_prediction_obstacle.second.trajectory_point_.size() < 20) {
    //   clear_obs_id.push_back(obs_id);
    //   continue;
    // }
    double dx = std::abs(
        vehicle_state.x -
        publish_prediction_obstacle.second.perception_obstacle_.position.x);
    double dy = std::abs(
        vehicle_state.y -
        publish_prediction_obstacle.second.perception_obstacle_.position.y);
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > 150) {
      clear_obs_id.push_back(obs_id);
    }
  }

  for (auto& clear_id : clear_obs_id) {
    publish_prediction_obstacles_.erase(clear_id);
  }

  // shrink publish_prediction_obstacles_中障碍物的trajectory_point_
  ShrinkObstacleTrajectory();

  //添加要显示的障碍物
  std::vector<std::int32_t> add_obs_id;
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
    if (dist < 150) {
      if (publish_prediction_obstacles_.find(obs_id) ==
          publish_prediction_obstacles_.end()) {
        add_obs_id.push_back(obs_id);
        publish_prediction_obstacles_.insert(
            std::make_pair(obs_id, potential_prediction_obstacle.second));
      }
    }
  }
  // 将添加到publish_prediction_obstacles_中的障碍物从potential_prediction_obstacles_中删除
  for (auto add_id : add_obs_id) {
    potential_prediction_obstacles_.erase(add_id);
  }

  // 更新prediction_obstacles_
  prediction_obstacles_.start_timestamp = ros::Time::now().toSec();
  prediction_obstacles_.start_timestamp =
      ros::Time::now().toSec() +
      ConfigParam::instance()->FLAGS_prediction_total_time;
  prediction_obstacles_.prediction_obstacle.clear();
  for (auto& publish_prediction_obstacle : publish_prediction_obstacles_) {
    PredictionObstacle pred_obs;
    pred_obs.perception_obstacle =
        publish_prediction_obstacle.second.perception_obstacle_;
    pred_obs.timestamp = prediction_obstacles_.start_timestamp;
    pred_obs.predicted_period =
        ConfigParam::instance()->FLAGS_prediction_total_time;
    Trajectory traj;
    traj.probability = 1.0;
    traj.trajectory_point =
        publish_prediction_obstacle.second.trajectory_point_;
    pred_obs.trajectory.emplace_back(std::move(traj));
    prediction_obstacles_.prediction_obstacle.emplace_back(pred_obs);
  }
}

void PredictionObstaclesProvider::ShrinkObstacleTrajectory() {
  for (auto& publish_prediction_obstacle : publish_prediction_obstacles_) {
    //  获得一个规划周期轨迹重采样的步长大小
    double obs_speed = std::sqrt(
        publish_prediction_obstacle.second.perception_obstacle_.velocity.x *
            publish_prediction_obstacle.second.perception_obstacle_.velocity.x +
        publish_prediction_obstacle.second.perception_obstacle_.velocity.y *
            publish_prediction_obstacle.second.perception_obstacle_.velocity.y);
    double planning_duration_time =
        1.0 / ConfigParam::instance()->FLAGS_planning_loop_rate;
    int trajectory_sample_step =
        int(planning_duration_time * obs_speed /
            ConfigParam::instance()->FLAGS_reference_line_sample_step);

    // 清除obstacle_reference_points_中上个周期的点
    if (publish_prediction_obstacle.second.obstacle_reference_points_.size() <=
        trajectory_sample_step + 5) {
      continue;
    }
    auto iter =
        publish_prediction_obstacle.second.obstacle_reference_points_.begin();
    publish_prediction_obstacle.second.obstacle_reference_points_.erase(
        iter, iter + trajectory_sample_step);

    //   障碍物预测轨迹起点状态
    publish_prediction_obstacle.second.perception_obstacle_.position.x =
        publish_prediction_obstacle.second.obstacle_reference_points_.front().x;
    publish_prediction_obstacle.second.perception_obstacle_.position.y =
        publish_prediction_obstacle.second.obstacle_reference_points_.front().y;
    publish_prediction_obstacle.second.perception_obstacle_.theta =
        publish_prediction_obstacle.second.obstacle_reference_points_.front()
            .heading;
    //   障碍物5s轨迹

    int trajectory_sample_points_num =
        ConfigParam::instance()->FLAGS_prediction_total_time /
        planning_duration_time;

    publish_prediction_obstacle.second.trajectory_point_.clear();
    double relative_time = 0.0;
    for (int i = 0; i < publish_prediction_obstacle.second
                            .obstacle_reference_points_.size();) {
      auto const& obstacle_reference_point =
          publish_prediction_obstacle.second.obstacle_reference_points_[i];
      if (publish_prediction_obstacle.second.trajectory_point_.size() >
          trajectory_sample_points_num) {
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
      tp.v_x =
          publish_prediction_obstacle.second.perception_obstacle_.velocity.x;
      tp.v_y =
          publish_prediction_obstacle.second.perception_obstacle_.velocity.y;
      tp.relative_time = relative_time;
      publish_prediction_obstacle.second.trajectory_point_.emplace_back(
          std::move(tp));

      relative_time += 0.1;
      i += trajectory_sample_step;
    }
  }
}
}  // namespace hqplanner
