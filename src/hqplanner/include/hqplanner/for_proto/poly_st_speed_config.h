#ifndef HQPLANNER_FORPROTO_POLY_ST_SPEED_CONFIG_H_
#define HQPLANNER_FORPROTO_POLY_ST_SPEED_CONFIG_H_
#include "hqplanner/for_proto/st_boundary_config.h"
namespace hqplanner {
namespace forproto {
struct PolyStSpeedConfig {
  double total_path_length = 150;
  double total_time = 8.0;
  double preferred_accel = 2.5;
  double preferred_decel = -3.3;
  double max_accel = 2.5;
  double min_decel = -4.5;
  double speed_limit_buffer = 0.05;
  double speed_weight = 1e2;
  double accelerate_weight = 1.0;
  double jerk_weight = 1.0;
  double obstacle_weight = 10.0;
  double unblocking_obstacle_cost = 1e3;
  StBoundaryConfig st_boundary_config;
  // st_boundary_config.boundary_buffer= 0.1;
};

}  // namespace forproto
}  // namespace hqplanner

#endif
