#ifndef HQPLANNER_FOR_PROTO_CONFIG_PARAM_H_
#define HQPLANNER_FOR_PROTO_CONFIG_PARAM_H_

#include <string>

namespace hqplanner {
namespace forproto {
struct ConfigParam {
  /* data */
  static const double FLAGS_st_max_s;
  static const double FLAGS_st_max_t;
  static const double FLAGS_trajectory_time_min_interval;  // second
  static const double FLAGS_trajectory_time_max_interval;
  static const double FLAGS_trajectory_time_high_density_period;
  static const std::size_t FLAGS_max_history_frame_num;
  static const double FLAGS_lane_left_width;
  static const double FLAGS_lane_right_width;
  static const double FLAGS_virtual_stop_wall_length;
  static const double FLAGS_virtual_stop_wall_height;
  static const double FLAGS_max_collision_distance;
  static const double FLAGS_look_forward_time_sec;
  static const double FLAGS_look_forward_short_distance;
  static const double FLAGS_look_forward_long_distance;
  static const double FLAGS_look_backward_distance;
  static const double FLAGS_reference_line_sample_step;
  static const int FLAGS_num_reference_points_near_destination;
  static const double FLAGS_prediction_total_time;
  static const double FLAGS_lateral_ignore_buffer;
  static const double FLAGS_static_decision_nudge_l_buffer;
  static const bool FLAGS_use_navigation_mode;
  static const double FLAGS_max_stop_speed;
  static const double FLAGS_max_stop_distance_obstacle;
  static const double FLAGS_min_stop_distance_obstacle;
  static const bool FLAGS_enable_nudge_decision;
  static const double FLAGS_nudge_distance_obstacle;
  static const double FLAGS_planning_upper_speed_limit;
  static const bool FLAGS_enable_nudge_slowdown;
  static const bool FLAGS_enable_side_vehicle_st_boundary;

  // stop distance from stop line
  static const double FLAGS_stop_line_stop_distance;
  // min following time in st region before considering a valid follow
  static const double FLAGS_follow_min_time_sec;
  // follow time buffer (in second) to calculate the following distance
  static const double FLAGS_follow_time_buffer;
  // min follow distance for vehicles/bicycles/moving objects
  static const double FLAGS_follow_min_distance;
  // min yield distance for vehicles/moving objects
  static const double FLAGS_yield_distance;
  // min yield distance for pedestrians/bicycles
  static const double FLAGS_yield_distance_pedestrian_bycicle;

  static const std::string FLAGS_destination_obstacle_id;
};

const double ConfigParam::FLAGS_st_max_s = 40.0;
const double ConfigParam::FLAGS_st_max_t = 8.0;
const double ConfigParam::FLAGS_trajectory_time_min_interval = 0.02;
const double ConfigParam::FLAGS_trajectory_time_max_interval = 0.1;
const double ConfigParam::FLAGS_trajectory_time_high_density_period = 1.0;
const std::size_t ConfigParam::FLAGS_max_history_frame_num = 2;
const double ConfigParam::FLAGS_lane_left_width = 3.0;
const double ConfigParam::FLAGS_lane_right_width = 3.0;
const double ConfigParam::FLAGS_virtual_stop_wall_length = 0.1;
const double ConfigParam::FLAGS_virtual_stop_wall_height = 2.0;
const double ConfigParam::FLAGS_max_collision_distance = 0.1;
const double ConfigParam::FLAGS_look_forward_time_sec = 8.0;
const double ConfigParam::FLAGS_look_forward_short_distance = 150.0;
const double ConfigParam::FLAGS_look_forward_long_distance = 250.0;
const double ConfigParam::FLAGS_look_backward_distance = 30.0;
const double ConfigParam::FLAGS_reference_line_sample_step = 0.1;
const int ConfigParam::FLAGS_num_reference_points_near_destination = 50;
const double ConfigParam::FLAGS_prediction_total_time = 5.0;
const double ConfigParam::FLAGS_lateral_ignore_buffer = 3.0;
const double ConfigParam::FLAGS_static_decision_nudge_l_buffer = 0.5;
const bool ConfigParam::FLAGS_use_navigation_mode = false;
const double ConfigParam::FLAGS_max_stop_speed = 0.2;  //被定义为停车的最大速度
const double ConfigParam::FLAGS_max_stop_distance_obstacle = 10.0;
const double ConfigParam::FLAGS_min_stop_distance_obstacle = 6.0;
const bool ConfigParam::FLAGS_enable_nudge_decision = true;
const double ConfigParam::FLAGS_nudge_distance_obstacle = 0.5;
const double ConfigParam::FLAGS_planning_upper_speed_limit = 16.66667;
const bool ConfigParam::FLAGS_enable_nudge_slowdown = true;
const bool ConfigParam::FLAGS_enable_side_vehicle_st_boundary = false;
const double ConfigParam::FLAGS_stop_line_stop_distance = 1.0;
const double ConfigParam::FLAGS_follow_min_time_sec = 5.0;
const double ConfigParam::FLAGS_follow_time_buffer = 2.5;
const double ConfigParam::FLAGS_follow_min_distance = 3.0;
const double ConfigParam::FLAGS_yield_distance = 3.0;
const double ConfigParam::FLAGS_yield_distance_pedestrian_bycicle = 5.0;
const std::string ConfigParam::FLAGS_destination_obstacle_id = "DEST";
}  // namespace forproto
}  // namespace hqplanner

#endif