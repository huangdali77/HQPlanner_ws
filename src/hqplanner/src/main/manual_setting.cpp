#include "hqplanner/main/manual_setting.h"

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "hqplanner/main/global_number_provider.h"
#include "hqplanner/math/math_utils.h"
namespace hqplanner {
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::PerceptionObstacle;
void ManualSetting::SetAnchorPiont(std::vector<AnchorPoint> &anchor_point) {
  for (int i = 0; i <= 35; ++i) {
    AnchorPoint ap;
    ap.cartesian_x = i * 20.0;
    ap.cartesian_y = 0.0;
    ap.frenet_s = 0.0;
    anchor_point.emplace_back(std::move(ap));
  }

  double turn_left = anchor_point.back().cartesian_x;
  //   700m之后左转
  AnchorPoint ap1;
  ap1.cartesian_x = turn_left + 5.0;
  ap1.cartesian_y = 1.4;
  ap1.frenet_s = 0.0;
  anchor_point.emplace_back(std::move(ap1));
  AnchorPoint ap2;
  ap2.cartesian_x = turn_left + 10.0;
  ap2.cartesian_y = 5.4;
  ap2.frenet_s = 0.0;
  anchor_point.emplace_back(std::move(ap2));

  AnchorPoint ap3;
  ap3.cartesian_x = turn_left + 15.0;
  ap3.cartesian_y = 11.2;
  ap3.frenet_s = 0.0;
  anchor_point.emplace_back(std::move(ap3));

  for (int i = 1; i <= 10; ++i) {
    AnchorPoint ap;
    ap.cartesian_x = turn_left + 20;
    ap.cartesian_y = i * 20;
    ap.frenet_s = 0.0;
    anchor_point.emplace_back(std::move(ap));
  }
}

void ManualSetting::SetPotentialPredictionObstacles(
    std::map<std::int32_t, PotentialPredictionObstacle>
        &potential_prediction_obstacles) {
  SetStaticPotentialPredictionObstacles(potential_prediction_obstacles);
  SetDynamicPotentialPredictionObstacles(potential_prediction_obstacles);
}

PotentialPredictionObstacle
ManualSetting::CreatDynamicPotentialPredictionObstacle(
    PerceptionObstacle &perception_obstacle, AnchorPoint start_anchor) {
  double pred_traj_length = 200.0;
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

void ManualSetting::SetDynamicPotentialPredictionObstacles(
    std::map<std::int32_t, PotentialPredictionObstacle>
        &potential_prediction_obstacles) {
  //   障碍物1沿y轴高速直行========================================================
  //     障碍物感知信息
  PerceptionObstacle perception_obstacle;
  perception_obstacle.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle.velocity.y = 18.0;
  perception_obstacle.length = 4.0;
  perception_obstacle.width = 2.0;
  perception_obstacle.height = 1.7;
  perception_obstacle.type = PerceptionObstacle::VEHICLE;

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor;
  start_anchor.cartesian_x = 296.0;
  start_anchor.cartesian_y = -70.0;
  start_anchor.frenet_s = 0.0;
  auto ptt_pred_obs = CreatDynamicPotentialPredictionObstacle(
      perception_obstacle, start_anchor);
  ptt_pred_obs.appear_when_adc_at_x_ = 210.0;
  ptt_pred_obs.appear_when_adc_at_y_ = 0.0;
  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle.id, std::move(ptt_pred_obs)));

  // 障碍物2沿y轴低速直行========================================================
  //   障碍物感知信息
  PerceptionObstacle perception_obstacle2;
  perception_obstacle2.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle2.velocity.y = 7;
  perception_obstacle2.length = 4.0;
  perception_obstacle2.width = 2.0;
  perception_obstacle2.height = 1.7;
  perception_obstacle2.type = PerceptionObstacle::VEHICLE;

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor2;
  start_anchor2.cartesian_x = 300.0;
  start_anchor2.cartesian_y = -80;
  start_anchor2.frenet_s = 0.0;

  auto ptt_pred_obs2 = CreatDynamicPotentialPredictionObstacle(
      perception_obstacle2, start_anchor2);
  ptt_pred_obs2.appear_when_adc_at_x_ = 210.0;
  ptt_pred_obs2.appear_when_adc_at_y_ = 0.0;

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle2.id, std::move(ptt_pred_obs2)));
  // 障碍物3 cut_in========================================================
  //   障碍物感知信息
  PerceptionObstacle perception_obstacle3;
  perception_obstacle3.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle3.velocity.y = 2.0;
  perception_obstacle3.velocity.x = 7.0;
  perception_obstacle3.length = 4.0;
  perception_obstacle3.width = 2.0;
  perception_obstacle3.height = 1.7;
  perception_obstacle3.type = PerceptionObstacle::VEHICLE;

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor3;
  start_anchor3.cartesian_x = 400.0;
  start_anchor3.cartesian_y = -10.0;
  start_anchor3.frenet_s = 0.0;

  double pred_traj_length = 200.0;
  double traj_step = 10;

  std::vector<hqplanner::forproto::AnchorPoint> anchor_points;
  anchor_points.emplace_back(start_anchor3);

  double speed = std::sqrt(
      perception_obstacle3.velocity.x * perception_obstacle3.velocity.x +
      perception_obstacle3.velocity.y * perception_obstacle3.velocity.y);
  double dx = traj_step * perception_obstacle3.velocity.x / speed;
  double dy = traj_step * perception_obstacle3.velocity.y / speed;
  while (anchor_points.back().frenet_s < 25) {
    AnchorPoint anchor;
    anchor.cartesian_x = anchor_points.back().cartesian_x + dx;
    anchor.cartesian_y = anchor_points.back().cartesian_y + dy;
    anchor.frenet_s = anchor_points.back().frenet_s + traj_step;
    anchor_points.emplace_back(std::move(anchor));
  }

  AnchorPoint anchor1;
  anchor1.cartesian_x = 435.0;
  anchor1.cartesian_y = 0;
  double dx1 = anchor1.cartesian_x - anchor_points.back().cartesian_x;
  double dy1 = 0.0 - anchor_points.back().cartesian_y;
  double dist = std::sqrt(dx1 * dx1 + dy1 * dy1);
  anchor1.frenet_s = anchor_points.back().frenet_s + dist;
  anchor_points.emplace_back(std::move(anchor1));

  while (anchor_points.back().frenet_s < 100) {
    AnchorPoint anchor;
    anchor.cartesian_x =
        anchor_points.back().cartesian_x + std::sqrt(dx * dx + dy * dy);
    anchor.cartesian_y = 0;
    anchor.frenet_s = anchor_points.back().frenet_s + traj_step;
    anchor_points.emplace_back(std::move(anchor));
  }

  AnchorPoint anchor2;
  anchor2.cartesian_x = 520.0;
  anchor2.cartesian_y = -3;
  double dx2 = anchor2.cartesian_x - anchor_points.back().cartesian_x;
  double dy2 = 0.0 - anchor_points.back().cartesian_y;
  double dist2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
  anchor2.frenet_s = anchor_points.back().frenet_s + dist2;
  anchor_points.emplace_back(std::move(anchor2));

  AnchorPoint anchor3;
  anchor3.cartesian_x = 530.0;
  anchor3.cartesian_y = -5;
  double dx3 = anchor3.cartesian_x - anchor_points.back().cartesian_x;
  double dy3 = 0.0 - anchor_points.back().cartesian_y;
  double dist3 = std::sqrt(dx3 * dx3 + dy3 * dy3);
  anchor3.frenet_s = anchor_points.back().frenet_s + dist3;
  anchor_points.emplace_back(std::move(anchor3));

  PotentialPredictionObstacle ptt_pred_obs3(perception_obstacle3,
                                            anchor_points);
  ptt_pred_obs3.appear_when_adc_at_x_ = 370.0;
  ptt_pred_obs3.appear_when_adc_at_y_ = 0.0;
  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle3.id, std::move(ptt_pred_obs3)));
  // 障碍物4沿x轴高速直行,adc会让行它========================================================
  //   障碍物感知信息
  PerceptionObstacle perception_obstacle4;
  perception_obstacle4.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle4.velocity.x = 20.0;
  perception_obstacle4.length = 4.0;
  perception_obstacle4.width = 2.0;
  perception_obstacle4.height = 1.7;
  perception_obstacle4.type = PerceptionObstacle::VEHICLE;

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor4;
  start_anchor4.cartesian_x = 460.0;
  start_anchor4.cartesian_y = 5;
  start_anchor4.frenet_s = 0.0;

  // ========================================

  std::vector<hqplanner::forproto::AnchorPoint> anchor_points4;
  anchor_points4.emplace_back(std::move(start_anchor4));

  //   障碍物直行
  while (anchor_points4.back().frenet_s < 170) {
    AnchorPoint anchor;

    anchor.cartesian_x = anchor_points4.back().cartesian_x + 20;
    anchor.cartesian_y = anchor_points4.back().cartesian_y;
    anchor.frenet_s = anchor_points4.back().frenet_s + 20;
    anchor_points4.emplace_back(std::move(anchor));
  }

  //   障碍物右转
  double turn_right = anchor_points4.back().cartesian_x;

  AnchorPoint ap1;
  ap1.cartesian_x = turn_right + 5.0;
  ap1.cartesian_y = -1.4;
  ap1.frenet_s = 0.0;
  anchor_points4.emplace_back(std::move(ap1));

  AnchorPoint ap2;
  ap2.cartesian_x = turn_right + 10.0;
  ap2.cartesian_y = -5.4;
  ap2.frenet_s = 0.0;
  anchor_points4.emplace_back(std::move(ap2));

  AnchorPoint ap3;
  ap3.cartesian_x = turn_right + 15.0;
  ap3.cartesian_y = -11.2;
  ap3.frenet_s = 0.0;
  anchor_points4.emplace_back(std::move(ap3));

  for (int i = 1; i <= 10; ++i) {
    AnchorPoint ap;
    ap.cartesian_x = turn_right + 20;
    ap.cartesian_y = -i * 20;
    ap.frenet_s = 0.0;
    anchor_points4.emplace_back(std::move(ap));
  }

  // ===============================
  PotentialPredictionObstacle ptt_pred_obs4(perception_obstacle4,
                                            anchor_points4);

  ptt_pred_obs4.appear_when_adc_at_x_ = 520;
  ptt_pred_obs4.appear_when_adc_at_y_ = 0.0;
  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle4.id, std::move(ptt_pred_obs4)));

  //   障碍物5沿x轴高速直行在adc左转的时候========================================================
  //     障碍物感知信息
  PerceptionObstacle perception_obstacle5;
  perception_obstacle5.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();
  perception_obstacle5.velocity.x = 18.0;
  perception_obstacle5.length = 4.0;
  perception_obstacle5.width = 2.0;
  perception_obstacle5.height = 1.7;
  perception_obstacle5.type = PerceptionObstacle::VEHICLE;

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor5;
  start_anchor5.cartesian_x = 600.0;
  start_anchor5.cartesian_y = 4.0;
  start_anchor5.frenet_s = 0.0;
  auto ptt_pred_obs5 = CreatDynamicPotentialPredictionObstacle(
      perception_obstacle5, start_anchor5);
  ptt_pred_obs5.appear_when_adc_at_x_ = 630;
  ptt_pred_obs5.appear_when_adc_at_y_ = 0.0;
  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle5.id, std::move(ptt_pred_obs5)));
}

void ManualSetting::SetStaticPotentialPredictionObstacles(
    std::map<std::int32_t, PotentialPredictionObstacle>
        &potential_prediction_obstacles) {
  PerceptionObstacle perception_obstacle1;
  perception_obstacle1.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();

  perception_obstacle1.length = 2.0;
  perception_obstacle1.width = 2.0;
  perception_obstacle1.height = 1.7;
  perception_obstacle1.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  //   perception_obstacle3.timestamp = ros::Time::now().toSec();

  perception_obstacle1.position.x = 100;
  perception_obstacle1.position.y = -1.0;
  perception_obstacle1.theta = 0.0;

  std::vector<hqplanner::forproto::AnchorPoint> null_anchor_points1;
  null_anchor_points1.clear();
  PotentialPredictionObstacle ptt_pred_obs1(perception_obstacle1,
                                            std::move(null_anchor_points1));
  ptt_pred_obs1.appear_when_adc_at_x_ = 10;
  ptt_pred_obs1.appear_when_adc_at_y_ = 0;

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle1.id, std::move(ptt_pred_obs1)));
  //   ===================================================

  PerceptionObstacle perception_obstacle2;
  perception_obstacle2.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();

  perception_obstacle2.length = 2.0;
  perception_obstacle2.width = 2.0;
  perception_obstacle2.height = 1.7;
  perception_obstacle2.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  //   perception_obstacle3.timestamp = ros::Time::now().toSec();

  perception_obstacle2.position.x = 140;
  perception_obstacle2.position.y = 1.0;
  perception_obstacle2.theta = 0.0;

  //
  std::vector<hqplanner::forproto::AnchorPoint> null_anchor_points2;
  null_anchor_points2.clear();
  PotentialPredictionObstacle ptt_pred_obs2(perception_obstacle2,
                                            std::move(null_anchor_points2));
  ptt_pred_obs2.appear_when_adc_at_x_ = 10;
  ptt_pred_obs2.appear_when_adc_at_y_ = 0;

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle2.id, std::move(ptt_pred_obs2)));

  // ======================================================
  PerceptionObstacle perception_obstacle3;
  perception_obstacle3.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();

  perception_obstacle3.length = 2.0;
  perception_obstacle3.width = 2.0;
  perception_obstacle3.height = 1.7;
  perception_obstacle3.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  //   perception_obstacle3.timestamp = ros::Time::now().toSec();

  perception_obstacle3.position.x = 180;
  perception_obstacle3.position.y = -1.5;
  perception_obstacle3.theta = 0.0;

  //
  std::vector<hqplanner::forproto::AnchorPoint> null_anchor_points3;
  null_anchor_points3.clear();
  PotentialPredictionObstacle ptt_pred_obs3(perception_obstacle3,
                                            std::move(null_anchor_points3));
  ptt_pred_obs3.appear_when_adc_at_x_ = 10;
  ptt_pred_obs3.appear_when_adc_at_y_ = 0;

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle3.id, std::move(ptt_pred_obs3)));
  //   ============================================================
  PerceptionObstacle perception_obstacle4;
  perception_obstacle4.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();

  perception_obstacle4.length = 2.0;
  perception_obstacle4.width = 2.0;
  perception_obstacle4.height = 1.7;
  perception_obstacle4.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  //   perception_obstacle3.timestamp = ros::Time::now().toSec();

  perception_obstacle4.position.x = 200;
  perception_obstacle4.position.y = 0.0;
  perception_obstacle4.theta = 0.0;
  //
  std::vector<hqplanner::forproto::AnchorPoint> null_anchor_points4;
  null_anchor_points4.clear();
  PotentialPredictionObstacle ptt_pred_obs4(perception_obstacle4,
                                            std::move(null_anchor_points4));
  ptt_pred_obs4.appear_when_adc_at_x_ = 10;
  ptt_pred_obs4.appear_when_adc_at_y_ = 0;

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle4.id, std::move(ptt_pred_obs4)));

  // =====================================================

  PerceptionObstacle perception_obstacle5;
  perception_obstacle5.id =
      GlobalNumberProvider::instance()->GetPerceptionObstacleId();

  perception_obstacle5.length = 3.0;
  perception_obstacle5.width = 3.0;
  perception_obstacle5.height = 1.7;
  perception_obstacle5.type = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  //   perception_obstacle3.timestamp = ros::Time::now().toSec();

  perception_obstacle5.position.x = 580;
  perception_obstacle5.position.y = -1.2;
  perception_obstacle5.theta = 0.0;
  //
  std::vector<hqplanner::forproto::AnchorPoint> null_anchor_points5;
  null_anchor_points5.clear();
  PotentialPredictionObstacle ptt_pred_obs5(perception_obstacle5,
                                            std::move(null_anchor_points5));
  ptt_pred_obs5.appear_when_adc_at_x_ = 450;
  ptt_pred_obs5.appear_when_adc_at_y_ = 0;

  potential_prediction_obstacles.insert(
      std::make_pair(perception_obstacle5.id, std::move(ptt_pred_obs5)));
}

visualization_msgs::Marker ManualSetting::GetObstacleMarker(
    const PerceptionObstacle &perception_obstacle) {
  //   static int id = 1;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "obsframe";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hqplanner";
  marker.id = perception_obstacle.id;
  ROS_INFO("theta:%f", perception_obstacle.theta);
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = perception_obstacle.position.x;
  marker.pose.position.y = perception_obstacle.position.y;
  marker.pose.position.z = perception_obstacle.position.z;
  // orientation应该适合heading有关

  double q_x;
  double q_y;
  double q_z;
  double q_w;

  hqplanner::math::EulerToQuaternion(0.0, 0.0, perception_obstacle.theta, q_x,
                                     q_y, q_z, q_w);

  marker.pose.orientation.x = q_x;
  marker.pose.orientation.y = q_y;
  marker.pose.orientation.z = q_z;
  marker.pose.orientation.w = q_w;

  marker.scale.x = perception_obstacle.length;
  marker.scale.y = perception_obstacle.width;
  marker.scale.z = perception_obstacle.height;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();  // marker 存在的时间

  return marker;
}

visualization_msgs::Marker ManualSetting::GetReferenceLineMarker(
    const hqplanner::ReferenceLine &ref_line) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "obsframe";
  marker.header.stamp = ros::Time::now();
  marker.ns = "hqplanner";
  marker.id = 999;

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;

  marker.color.g = 1.0f;

  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();  // marker 存在的时间

  const auto ref_line_points = ref_line.reference_points();
  for (const auto &ref_line_point : ref_line_points) {
    geometry_msgs::Point temp;

    temp.x = ref_line_point.x;
    temp.y = ref_line_point.y;
    temp.z = 0;
    marker.points.push_back(temp);
  }
  return marker;
}

}  // namespace hqplanner