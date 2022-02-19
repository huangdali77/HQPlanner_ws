#ifndef HQPLANNER_FOR_PROTO_PLANNING_CONFIG_H_
#define HQPLANNER_FOR_PROTO_PLANNING_CONFIG_H_
#include <vector>

#include "hqplanner/for_proto/cubic_st_speed_config.h"
#include "hqplanner/for_proto/dp_poly_path_config.h"
#include "hqplanner/for_proto/dp_st_speed_config.h"
#include "hqplanner/for_proto/poly_st_speed_config.h"
namespace hqplanner {
namespace forproto {

enum TaskType {
  DP_POLY_PATH_OPTIMIZER = 0,
  DP_ST_SPEED_OPTIMIZER = 1,
  QP_SPLINE_PATH_OPTIMIZER = 2,
  QP_SPLINE_ST_SPEED_OPTIMIZER = 3,
  PATH_DECIDER = 4,
  SPEED_DECIDER = 5,
  POLY_ST_SPEED_OPTIMIZER = 6,
  CUBIC_ST_SPEED_OPTIMIZER = 7,
  NAVI_PATH_DECIDER = 8,
  NAVI_SPEED_DECIDER = 9,
  NAVI_OBSTACLE_DECIDER = 10,
};

struct EMPlannerConfig {
  std::vector<TaskType> task;
  DpPolyPathConfig dp_poly_path_config;
  DpStSpeedConfig dp_st_speed_config;
  PolyStSpeedConfig poly_st_speed_config;
  CubicStSpeedConfig cubic_st_speed_config;
};

struct PlanningConfig {
  enum PlannerType { RTK = 0, EM = 1, LATTICE = 2, NAVI = 3 };
  PlannerType planner_type = EM;
  EMPlannerConfig em_planner_config;
};

}  // namespace forproto
}  // namespace hqplanner

#endif