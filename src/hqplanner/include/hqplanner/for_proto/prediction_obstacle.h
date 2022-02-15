#ifndef HQPLANNER_FOR_PROTO_PREDICTION_OBSTACLE_H_
#define HQPLANNER_FOR_PROTO_PREDICTION_OBSTACLE_H_
#include <vector>

#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
namespace hqplanner {
namespace forproto {
struct Trajectory {
  double probability = 0.0;  // probability of this trajectory
  std::vector<TrajectoryPoint> trajectory_point;
};

struct PredictionObstacle {
  PerceptionObstacle perception_obstacle;
  double timestamp = 0.0;  // GPS time in seconds
  // the length of the time for this prediction (e.g. 10s)
  double predicted_period = 0.0;
  // can have multiple trajectories per obstacle
  std::vector<Trajectory> trajectory;
};

struct PredictionObstacles {
  // make prediction for multiple obstacles
  std::vector<PredictionObstacle> prediction_obstacle;
  // start timestamp
  double start_timestamp = 0.0;
  // end timestamp
  double end_timestamp = 0.0;
};

}  // namespace forproto
}  // namespace hqplanner
#endif