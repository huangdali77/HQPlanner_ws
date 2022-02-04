#ifndef HQPLANNING_MAIN_PREDICTION_OBSTACLES_PROVIDER_H_
#define HQPLANNING_MAIN_PREDICTION_OBSTACLES_PROVIDER_H_

#include <unordered_map>
#include <vector>

#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/prediction_obstacle.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/util/macro.h"
namespace hqplanner {
struct PotentialPredictionObstacle {
 public:
  PotentialPredictionObstacle() = default;
  PotentialPredictionObstacle(
      hqplanner::forproto::PerceptionObstacle perception_obstacle,
      std::vector<hqplanner::forproto::AnchorPoint> anchor_points);
  void InitPolygonPoint(std::vector<hqplanner::forproto::Point> &polygon_point);

 public:
  double appear_distance_threshold = 0.0;
  double disappear_distance_threshold = 0.0;
  hqplanner::forproto::PerceptionObstacle perception_obstacle_;
  std::vector<hqplanner::forproto::AnchorPoint> anchor_points_;
  ReferenceLine obstacle_reference_trajectory_;
  //   障碍物全部轨迹
  std::vector<hqplanner::forproto::ReferencePoint> obstacle_reference_points_;
  //   障碍物5s轨迹
  std::vector<hqplanner::forproto::TrajectoryPoint> trajectory_point_;
};

class PredictionObstaclesProvider {
 public:
  void Init(std::unordered_map<std::int32_t, PotentialPredictionObstacle>
                potential_prediction_obstacles);
  void Init();
  void UpdataNextCyclePredictionObstacles();

  hqplanner::forproto::PredictionObstacles GetPredictionObstacles() const {
    return prediction_obstacles_;
  };

 private:
  void ShrinkObstacleTrajectory();

 private:
  hqplanner::forproto::PredictionObstacles prediction_obstacles_;
  //   std::unordered_map<std::int32_t, double> disappear_distance_threshold_;
  std::unordered_map<std::int32_t, PotentialPredictionObstacle>
      publish_prediction_obstacles_;
  std::unordered_map<std::int32_t, PotentialPredictionObstacle>
      potential_prediction_obstacles_;

  DECLARE_SINGLETON(PredictionObstaclesProvider);
};

}  // namespace hqplanner

#endif