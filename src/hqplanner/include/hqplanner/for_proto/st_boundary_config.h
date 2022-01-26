#ifndef HQPLANNER_FORPROTO_ST_BOUNDARY_CONFIG_H_
#define HQPLANNER_FORPROTO_ST_BOUNDARY_CONFIG_H_

namespace hqplanner {
namespace forproto {
struct StBoundaryConfig {
  double boundary_buffer = 0.1;
  double high_speed_centric_acceleration_limit = 1.2;
  double low_speed_centric_acceleration_limit = 1.4;
  double high_speed_threshold = 11.18;
  double low_speed_threshold = 4.5;
  double minimal_kappa = 0.00001;
  double point_extension = 1.0;
  double lowest_speed = 2.5;
  int num_points_to_avg_kappa = 2;

  double static_obs_nudge_speed_ratio = 0.6;
  double dynamic_obs_nudge_speed_ratio = 0.8;
  double centri_jerk_speed_coeff = 1.0;
};

}  // namespace forproto
}  // namespace hqplanner

#endif