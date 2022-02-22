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
#include "hqplanner/main/manual_setting.h"
#include "hqplanner/main/planning.h"
#include "hqplanner/main/prediction_obstacles_provider.h"

using hqplanner::AnchorPointsProvider;
using hqplanner::Frame;
using hqplanner::FrameHistory;
using hqplanner::GlobalNumberProvider;
using hqplanner::ManualSetting;
using hqplanner::Planning;
using hqplanner::PotentialPredictionObstacle;
using hqplanner::PredictionObstaclesProvider;
using hqplanner::ReferenceLine;
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::PerceptionObstacle;
using hqplanner::forproto::VehicleConfig;
using hqplanner::forproto::VehicleConfigHelper;
using hqplanner::forproto::VehicleState;
using hqplanner::forproto::VehicleStateProvider;

int main(int argc, char **argv) {
  ROS_INFO("start main()");
  ros::init(argc, argv, "hqplanner_test");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  //   ==================================================================
  //   tf2_ros::TransformBroadcaster odom_broadcaster;
  //   ==================================================================
  ManualSetting manual_setting;

  // 0、提供全局路由锚点
  std::vector<std::vector<AnchorPoint>> anchor_points;
  std::vector<AnchorPoint> anchor_point;
  manual_setting.SetAnchorPiont(anchor_point);
  anchor_points.emplace_back(std::move(anchor_point));
  AnchorPointsProvider::instance()->SetAnchorPoints(anchor_points);
  // 在rviz中发布参考线信息
  ReferenceLine ref_line(anchor_points.front());
  visualization_msgs::Marker ref_line_points_pub =
      manual_setting.GetReferenceLineMarker(ref_line);
  // marker_pub.publish(ref_line_points_pub);
  //   2、再初始化障碍物信息
  std::map<std::int32_t, PotentialPredictionObstacle>
      potential_prediction_obstacles;
  manual_setting.SetPotentialPredictionObstacles(
      potential_prediction_obstacles);
  PredictionObstaclesProvider::instance()->Init(potential_prediction_obstacles);
  ROS_INFO("potential_prediction_obstacles size:%d",
           static_cast<int>(potential_prediction_obstacles.size()));
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
  adc_state.x = 1.0;
  adc_state.timestamp = ros::Time::now().toSec();
  VehicleStateProvider::instance()->Init(adc_state);
  ROS_INFO("x:%f,y:%f,v:%f", adc_state.x, adc_state.y,
           adc_state.linear_velocity);
  ros::Rate r(10);
  while (ros::ok()) {
    ROS_INFO("=============================================================");
    ROS_INFO("time:%f", ros::Time::now().toSec());
    planning.RunOnce();
    ROS_INFO("after runonce");

    VehicleState veh_state = VehicleStateProvider::instance()->vehicle_state();

    // Frame *frame = planning.GetFrame();

    const auto *frame = FrameHistory::instance()->Latest();
    if (frame->is_near_destination()) {
      break;
    }

    if (frame == nullptr) {
      ROS_INFO("frame==nullptr");
    }
    if (frame->DriveReferenceLineInfo() == nullptr) {
      ROS_INFO("frame->drive_reference_line_info_ == nullptr");
    }
    ROS_INFO("x:%f,y:%f,v:%f", veh_state.x, veh_state.y,
             veh_state.linear_velocity);
    const auto path_obs_items =
        frame->DriveReferenceLineInfo()->path_decision().path_obstacle_items();
    ROS_INFO("obs size:%d", static_cast<int>(path_obs_items.size()));
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
    double q_x;
    double q_y;
    double q_z;
    double q_w;

    hqplanner::math::EulerToQuaternion(0.0, 0.0, veh_state.heading, q_x, q_y,
                                       q_z, q_w);

    adc_marker.pose.orientation.x = q_x;
    adc_marker.pose.orientation.y = q_y;
    adc_marker.pose.orientation.z = q_z;
    adc_marker.pose.orientation.w = q_w;

    // adc_marker.pose.orientation.x = 0.0;
    // adc_marker.pose.orientation.y = 0.0;
    // adc_marker.pose.orientation.z = 0.0;
    // adc_marker.pose.orientation.w = 1.0;

    adc_marker.scale.x = veh_conf.vehicle_param.length;
    adc_marker.scale.y = veh_conf.vehicle_param.width;
    adc_marker.scale.z = veh_conf.vehicle_param.height;

    adc_marker.color.r = 0.0f;
    adc_marker.color.g = 0.0f;
    adc_marker.color.b = 1.0f;
    adc_marker.color.a = 1.0;

    adc_marker.lifetime = ros::Duration();  // marker 存在的时间

    // obstacle marker
    // std::vector<visualization_msgs::Marker> obs_markers;
    for (const auto &path_obs : path_obs_items) {
      const PerceptionObstacle perception_obstacle =
          path_obs->obstacle()->Perception();
      visualization_msgs::Marker obs_marker =
          manual_setting.GetObstacleMarker(perception_obstacle);
      marker_pub.publish(obs_marker);
      // obs_markers.emplace_back(std::move(obs_marker));
    }

    marker_pub.publish(ref_line_points_pub);
    marker_pub.publish(adc_marker);

    // for (auto &marker : obs_markers) {
    //   marker_pub.publish(marker);
    // }
    // 发布参考线

    ros::spinOnce();
    r.sleep();
  }
}
