// #include <ros/ros.h>

#include <unordered_map>

#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_state.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/main/anchor_points_provider.h"
#include "hqplanner/main/global_number_provider.h"
#include "hqplanner/main/planning.h"
#include "hqplanner/main/prediction_obstacles_provider.h"
// #include"hqplanner/main/planning.h"
using hqplanner::AnchorPointsProvider;
using hqplanner::GlobalNumberProvider;
using hqplanner::Planning;
using hqplanner::PotentialPredictionObstacle;
using hqplanner::PredictionObstaclesProvider;
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::VehicleState;
using hqplanner::forproto::VehicleStateProvider;
PotentialPredictionObstacle CreatPotentialPredictionObstacle(
    PerceptionObstacle perception_obstacle, AnchorPoint start_anchor) {
  double pred_traj_length = 300.0;
  double traj_step = 20;
  std::vector<hqplanner::forproto::AnchorPoint> anchor_points;
  anchor_points.emplace_back(std::move(start_anchor));
  double speed = std::sqrt(
      perception_obstacle.velocity.x * perception_obstacle.velocity.x +
      perception_obstacle.velocity.y * perception_obstacle.velocity.y);
  double dx = traj_step * perception_obstacle.velocity.x / speed;
  double dy = traj_step * perception_obstacle.velocity.y / speed;
  while (anchor_points.back().frenet_s < pred_traj_length) {
    AnchorPoint anchor;

    anchor.cartesian_x = anchor_points.back().cartesian_x + dx;
    anchor.cartesian_y = anchor_points.back().cartesian_y + dy;
    anchor.frenet_s = anchor_points.back().frenet_s + traj_step;
    anchor_points.emplace_back(std::move(anchor));
  }
  return PotentialPredictionObstacle(std::move(perception_obstacle),
                                     std::move(anchor_points));
}

void GetPotentialPredictionObstacles(
    std::unordered_map<std::int32_t, PotentialPredictionObstacle>
        &potential_prediction_obstacles) {
  // 障碍物1沿x轴直行========================================================
  //   障碍物感知信息
  PerceptionObstacle perception_obstacle;
  perception_obstacle.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle.velocity.x = 5.0;
  perception_obstacle.length = 4.0;
  perception_obstacle.width = 2.0;
  perception_obstacle.length = 1.7;
  perception_obstacle.type = PerceptionObstacle::VEHICLE;
  perception_obstacle.timestamp = ros::Time::now().toSec();

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor;
  start_anchor.cartesian_x = 10.0;
  start_anchor.cartesian_y = 2.0;
  start_anchor.frenet_s = 0.0;

  // 障碍物2沿y轴直行========================================================
  //   障碍物感知信息
  PerceptionObstacle perception_obstacle2;
  perception_obstacle2.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle2.velocity.y = 6.0;
  perception_obstacle2.length = 4.0;
  perception_obstacle2.width = 2.0;
  perception_obstacle2.length = 1.7;
  perception_obstacle2.type = PerceptionObstacle::VEHICLE;
  perception_obstacle2.timestamp = ros::Time::now().toSec();

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor2;
  start_anchor2.cartesian_x = 70.0;
  start_anchor2.cartesian_y = 10;
  start_anchor2.frenet_s = 0.0;

  // adc沿x轴直行========================================================
  //   障碍物感知信息
  PerceptionObstacle adc;
  adc.id = 0;
  adc.velocity.x = 4.0;
  adc.length = 4.0;
  adc.width = 2.0;
  adc.length = 1.7;
  adc.type = PerceptionObstacle::VEHICLE;
  adc.timestamp = ros::Time::now().toSec();

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor_adc;
  start_anchor_adc.cartesian_x = 0.0;
  start_anchor_adc.cartesian_y = 0.0;
  start_anchor_adc.frenet_s = 0.0;
  //   ===================================================

  auto ptt_pred_obs =
      CreatPotentialPredictionObstacle(perception_obstacle, start_anchor);
  ptt_pred_obs.appear_distance_threshold = 50;
  ptt_pred_obs.disappear_distance_threshold = 200;

  auto ptt_pred_obs2 =
      CreatPotentialPredictionObstacle(perception_obstacle2, start_anchor2);
  ptt_pred_obs2.appear_distance_threshold = 50;
  ptt_pred_obs2.disappear_distance_threshold = 200;

  auto ptt_pred_obs_adc =
      CreatPotentialPredictionObstacle(adc, start_anchor_adc);
  ptt_pred_obs_adc.appear_distance_threshold = 99999;
  ptt_pred_obs_adc.disappear_distance_threshold = 99999;

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle.id, std::move(ptt_pred_obs)));

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle2.id, std::move(ptt_pred_obs2)));
  potential_prediction_obstacles.insert(
      std::make_pair(adc.id, std::move(ptt_pred_obs_adc)));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hqplanner_test");
  ros::NodeHandle n;
  // 0、提供全局路由锚点
  std::vector<std::vector<AnchorPoint>> anchor_points;
  std::vector<AnchorPoint> anchor_point;
  for (int i = 0; i <= 20; ++i) {
    AnchorPoint ap;
    ap.cartesian_x = i * 20.0;
    ap.cartesian_y = 0.0;
    ap.frenet_s = 0.0;
    anchor_point.emplace_back(std::move(ap));
  }
  anchor_points.emplace_back(std::move(anchor_point));
  AnchorPointsProvider::instance()->SetAnchorPoints(anchor_points);
  //   1、先初始化adc状态
  VehicleState adc_state;
  VehicleStateProvider::instance()->Init(adc_state);
  //   2、再初始化障碍物信息
  std::unordered_map<std::int32_t, PotentialPredictionObstacle>
      potential_prediction_obstacles;
  GetPotentialPredictionObstacles(potential_prediction_obstacles);
  PredictionObstaclesProvider::instance()->Init(potential_prediction_obstacles);
  // 实例化规划对象
  Planning planning;
  planning.Init();

  ros::Rate r(10);
  while (ros::ok()) {
    planning.RunOnce();
  }
}
