#include "hqplanner/main/manual_setting.h"

#include "hqplanner/main/global_number_provider.h"
namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::PerceptionObstacle;
void ManualSetting::SetAnchorPiont(std::vector<AnchorPoint> &anchor_point) {
  for (int i = 0; i <= 50; ++i) {
    AnchorPoint ap;
    ap.cartesian_x = i * 20.0;
    ap.cartesian_y = 0.0;
    ap.frenet_s = 0.0;
    anchor_point.emplace_back(std::move(ap));
  }
}

void ManualSetting::SetPotentialPredictionObstacles(
    std::map<std::int32_t, PotentialPredictionObstacle>
        &potential_prediction_obstacles) {
  // 障碍物1沿x轴直行========================================================
  //   障碍物感知信息
  PerceptionObstacle perception_obstacle;
  perception_obstacle.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle.velocity.x = 5.0;
  perception_obstacle.length = 4.0;
  perception_obstacle.width = 2.0;
  perception_obstacle.height = 1.7;
  perception_obstacle.type = PerceptionObstacle::VEHICLE;
  //   perception_obstacle.timestamp = ros::Time::now().toSec();

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
  perception_obstacle2.height = 1.7;
  perception_obstacle2.type = PerceptionObstacle::VEHICLE;
  //   perception_obstacle2.timestamp = ros::Time::now().toSec();

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor2;
  start_anchor2.cartesian_x = 70.0;
  start_anchor2.cartesian_y = 10;
  start_anchor2.frenet_s = 0.0;

  // 静态障碍物障碍物3沿y轴直行========================================================
  //   障碍物感知信息
  PerceptionObstacle perception_obstacle3;
  perception_obstacle3.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();

  perception_obstacle3.length = 2.0;
  perception_obstacle3.width = 2.0;
  perception_obstacle3.height = 1.7;
  perception_obstacle3.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  //   perception_obstacle3.timestamp = ros::Time::now().toSec();

  perception_obstacle3.position.x = 30;
  perception_obstacle3.position.y = 50;
  perception_obstacle3.theta = 0.0;

  // adc沿x轴直行========================================================
  //   障碍物感知信息
  PerceptionObstacle adc;
  adc.id = GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  ;
  adc.velocity.x = 4.0;
  adc.length = 4.0;
  adc.width = 2.0;
  adc.height = 1.7;
  adc.type = PerceptionObstacle::VEHICLE;
  //   adc.timestamp = ros::Time::now().toSec();

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor_adc;
  start_anchor_adc.cartesian_x = 3.0;
  start_anchor_adc.cartesian_y = 0.0;
  start_anchor_adc.frenet_s = 0.0;
  //   ===================================================

  auto ptt_pred_obs =
      CreatPotentialPredictionObstacle(perception_obstacle, start_anchor);
  ptt_pred_obs.appear_when_adc_at_x_ = 50;
  ptt_pred_obs.appear_when_adc_at_y_ = 200;

  auto ptt_pred_obs2 =
      CreatPotentialPredictionObstacle(perception_obstacle2, start_anchor2);
  ptt_pred_obs2.appear_when_adc_at_x_ = 50;
  ptt_pred_obs2.appear_when_adc_at_y_ = 200;

  auto ptt_pred_obs_adc =
      CreatPotentialPredictionObstacle(adc, start_anchor_adc);
  ptt_pred_obs_adc.appear_when_adc_at_x_ = 99999;
  ptt_pred_obs_adc.appear_when_adc_at_y_ = 99999;

  std::vector<hqplanner::forproto::AnchorPoint> null_anchor_points;
  null_anchor_points.clear();
  PotentialPredictionObstacle ptt_pred_obs3(std::move(perception_obstacle3),
                                            std::move(null_anchor_points));

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle.id, std::move(ptt_pred_obs)));
  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle2.id, std::move(ptt_pred_obs2)));
  potential_prediction_obstacles.insert(
      std::make_pair(adc.id, std::move(ptt_pred_obs_adc)));

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle3.id, std::move(ptt_pred_obs3)));
}

PotentialPredictionObstacle ManualSetting::CreatPotentialPredictionObstacle(
    PerceptionObstacle &perception_obstacle, AnchorPoint start_anchor) {
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
  return PotentialPredictionObstacle(perception_obstacle, anchor_points);
}

visualization_msgs::Marker ManualSetting::GetObstacleMarker(
    const PerceptionObstacle &perception_obstacle) {
  static int id = 1;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "obsframe";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hqplanner";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = perception_obstacle.position.x;
  marker.pose.position.y = perception_obstacle.position.y;
  marker.pose.position.z = perception_obstacle.position.z;
  // orientation应该适合heading有关
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = perception_obstacle.length;
  marker.scale.y = perception_obstacle.width;
  marker.scale.z = perception_obstacle.height;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();  // marker 存在的时间

  ++id;
  return marker;
}

}  // namespace hqplanner