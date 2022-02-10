#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <map>

#include "hqplanner/common/frame.h"
#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_state.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
#include "hqplanner/main/anchor_points_provider.h"
#include "hqplanner/main/global_number_provider.h"
#include "hqplanner/main/planning.h"
#include "hqplanner/main/prediction_obstacles_provider.h"

// #include"hqplanner/main/planning.h"
using hqplanner::AnchorPointsProvider;
using hqplanner::Frame;
using hqplanner::GlobalNumberProvider;
using hqplanner::Planning;
using hqplanner::PotentialPredictionObstacle;
using hqplanner::PredictionObstaclesProvider;
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::VehicleConfig;
using hqplanner::forproto::VehicleConfigHelper;
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
  return PotentialPredictionObstacle(perception_obstacle, anchor_points);
}

void GetPotentialPredictionObstacles(
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
  perception_obstacle2.height = 1.7;
  perception_obstacle2.type = PerceptionObstacle::VEHICLE;
  perception_obstacle2.timestamp = ros::Time::now().toSec();

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
  perception_obstacle3.type = PerceptionObstacle::VEHICLE;
  perception_obstacle3.timestamp = ros::Time::now().toSec();

  perception_obstacle3.position.x = 30;
  perception_obstacle3.position.y = 50;
  perception_obstacle3.theta = 0.0;

  // adc沿x轴直行========================================================
  //   障碍物感知信息
  PerceptionObstacle adc;
  adc.id = 0;
  adc.velocity.x = 4.0;
  adc.length = 4.0;
  adc.width = 2.0;
  adc.height = 1.7;
  adc.type = PerceptionObstacle::VEHICLE;
  adc.timestamp = ros::Time::now().toSec();

  // 障碍物的预测轨迹起始锚点
  AnchorPoint start_anchor_adc;
  start_anchor_adc.cartesian_x = 3.0;
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

visualization_msgs::Marker GetObsMarker(
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
//   ==================================================================
// geometry_msgs::Quaternion odom_quat;
//   ==================================================================

int main(int argc, char **argv) {
  ROS_INFO("start main()");
  ros::init(argc, argv, "hqplanner_test");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //   ==================================================================
  //   tf2_ros::TransformBroadcaster odom_broadcaster;
  //   ==================================================================
  // 0、提供全局路由锚点
  std::vector<std::vector<AnchorPoint>> anchor_points;
  std::vector<AnchorPoint> anchor_point;
  for (int i = 0; i <= 30; ++i) {
    AnchorPoint ap;
    ap.cartesian_x = i * 20.0;
    ap.cartesian_y = 0.0;
    ap.frenet_s = 0.0;
    anchor_point.emplace_back(std::move(ap));
  }
  anchor_points.emplace_back(std::move(anchor_point));
  AnchorPointsProvider::instance()->SetAnchorPoints(anchor_points);

  //   2、再初始化障碍物信息
  std::map<std::int32_t, PotentialPredictionObstacle>
      potential_prediction_obstacles;
  GetPotentialPredictionObstacles(potential_prediction_obstacles);
  PredictionObstaclesProvider::instance()->Init(potential_prediction_obstacles);

  // 实例化规划对象
  Planning planning;
  //   初始化全局路由的anchor和planning config
  planning.Init();

  // 初始化VehicleConfigHelper
  VehicleConfig vehicle_param;
  VehicleConfigHelper::Init(vehicle_param);
  VehicleConfig veh_conf = VehicleConfigHelper::instance()->GetConfig();
  ROS_INFO("before ros::ok()");

  //   1、先初始化adc状态
  VehicleState adc_state;
  adc_state.x = 3.0;
  adc_state.timestamp = ros::Time::now().toSec();
  VehicleStateProvider::instance()->Init(adc_state);

  ros::Rate r(10);
  while (ros::ok()) {
    ROS_INFO("time:%f", ros::Time::now().toSec());
    planning.RunOnce();
    ROS_INFO("after runonce");

    VehicleState veh_state = VehicleStateProvider::instance()->vehicle_state();

    Frame *frame = planning.GetFrame();
    if (frame->IsNearDestination()) {
      break;
    }

    const auto path_obs_items = frame->FindDriveReferenceLineInfo()
                                    ->path_decision()
                                    .path_obstacle_items();

    // adc maker
    visualization_msgs::Marker adc_marker;
    adc_marker.header.frame_id = "obsframe";
    adc_marker.header.stamp = ros::Time::now();
    adc_marker.ns = "hqplanner";
    adc_marker.id = 0;
    adc_marker.type = visualization_msgs::Marker::CUBE;
    adc_marker.action = visualization_msgs::Marker::ADD;

    adc_marker.pose.position.x = veh_state.x;
    adc_marker.pose.position.y = veh_state.y;
    adc_marker.pose.position.z = veh_state.z;
    // orientation应该适合heading有关
    adc_marker.pose.orientation.x = 0.0;
    adc_marker.pose.orientation.y = 0.0;
    adc_marker.pose.orientation.z = 0.0;
    adc_marker.pose.orientation.w = 1.0;

    adc_marker.scale.x = veh_conf.vehicle_param.length;
    adc_marker.scale.y = veh_conf.vehicle_param.width;
    adc_marker.scale.z = veh_conf.vehicle_param.height;

    adc_marker.color.r = 0.0f;
    adc_marker.color.g = 1.0f;
    adc_marker.color.b = 0.0f;
    adc_marker.color.a = 1.0;

    adc_marker.lifetime = ros::Duration();  // marker 存在的时间

    // obstacle marker
    std::vector<visualization_msgs::Marker> obs_markers;
    for (const auto &path_obs : path_obs_items) {
      const PerceptionObstacle perception_obstacle =
          path_obs->obstacle()->Perception();
      visualization_msgs::Marker obs_marker = GetObsMarker(perception_obstacle);
      obs_markers.emplace_back(std::move(obs_marker));
    }
    marker_pub.publish(adc_marker);
    for (auto &marker : obs_markers) {
      marker_pub.publish(marker);
    }

    //   ==================================================================
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.stamp = ros::Time::now();
    // odom_trans.header.frame_id = "obsframe";
    // odom_trans.child_frame_id = "adcframe";
    // odom_quat = adc_marker.pose.orientation;
    // odom_trans.transform.translation.x = veh_state.x;
    // odom_trans.transform.translation.y = veh_state.x;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_quat;

    // odom_broadcaster.sendTransform(odom_trans);
    //   ==================================================================

    // ros::spinOnce();
    r.sleep();
  }
}
