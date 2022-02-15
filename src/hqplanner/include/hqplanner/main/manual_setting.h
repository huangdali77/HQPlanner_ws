#ifndef HQPLANNING_MAIN_MANUAL_SETTING_H_
#define HQPLANNING_MAIN_MANUAL_SETTING_H_
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <map>
#include <vector>

#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/main/prediction_obstacles_provider.h"

namespace hqplanner {

class ManualSetting {
  ManualSetting() = default;

 public:
  //  设置全局锚点
  void SetAnchorPiont(
      std::vector<hqplanner::forproto::AnchorPoint> &anchor_point);
  //   设置障碍物信息
  void SetPotentialPredictionObstacles(
      std::map<std::int32_t, PotentialPredictionObstacle>
          &potential_prediction_obstacles);
  PotentialPredictionObstacle CreatPotentialPredictionObstacle(
      hqplanner::forproto::PerceptionObstacle &perception_obstacle,
      hqplanner::forproto::AnchorPoint start_anchor);
  // 设置Marker信息
  visualization_msgs::Marker GetObstacleMarker(
      const hqplanner::forproto::PerceptionObstacle &perception_obstacle);
};

}  // namespace hqplanner

#endif