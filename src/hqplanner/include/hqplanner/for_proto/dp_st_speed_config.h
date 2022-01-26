#ifndef HQPLANNER_FORPROTO_DP_ST_SPEED_CONFIG_H_
#define HQPLANNER_FORPROTO_DP_ST_SPEED_CONFIG_H_
#include "hqplanner/for_proto/st_boundary_config.h"
namespace hqplanner {
namespace forproto {
struct DpStSpeedConfig {
  double total_path_length = 149;
  double total_time = 7.0;
  int matrix_dimension_s = 150;
  int matrix_dimension_t = 8;

  double speed_weight = 0.0;
  double accel_weight = 10.0;
  double jerk_weight = 10.0;
  double obstacle_weight = 1.0;
  double reference_weight = 0.0;
  double go_down_buffer = 5.0;
  double go_up_buffer = 5.0;

  // obstacle cost config
  double default_obstacle_cost = 1e3;

  // speed cost config
  double default_speed_cost = 1.0e3;
  double exceed_speed_penalty = 10.0;
  double low_speed_penalty = 10.0;
  double keep_clear_low_speed_penalty = 10.0;

  // accel cost config
  double accel_penalty = 1.0;
  double decel_penalty = 1.0;

  // jerk cost config
  double positive_jerk_coeff = 1.0;
  double negative_jerk_coeff = 1.0;

  // other constraint
  double max_acceleration = 3.0;
  double max_deceleration = -4.0;

  StBoundaryConfig st_boundary_config;
};

}  // namespace forproto
}  // namespace hqplanner

#endif