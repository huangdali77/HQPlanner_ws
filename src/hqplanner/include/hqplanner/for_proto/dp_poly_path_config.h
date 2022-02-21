#ifndef HQPLANNER_FORPROTO_DP_POLY_PATH_CONFIG_H_
#define HQPLANNER_FORPROTO_DP_POLY_PATH_CONFIG_H_
namespace hqplanner {
namespace forproto {
struct DpPolyPathConfig {
  int sample_points_num_each_level = 7;
  double step_length_max = 40.0;
  double step_length_min = 20.0;
  double lateral_sample_offset = 0.5;
  double lateral_adjust_coeff = 0.5;
  // Trajectory Cost Config
  double eval_time_interval = 0.1;
  double path_resolution = 1.0;
  double obstacle_ignore_distance = 20.0;
  double obstacle_collision_distance = 0.2;
  double obstacle_risk_distance = 2.0;
  double obstacle_collision_cost = 1e4;  // 1e8;
  double path_l_cost = 1e3;              // 6.5;
  double path_dl_cost = 8e3;
  double path_ddl_cost = 5e1;
  double path_l_cost_param_l0 = 1.50;
  double path_l_cost_param_b = 0.40;
  double path_l_cost_param_k = 1.50;
  double path_out_lane_cost = 1e8;
  double path_end_l_cost = 1.0e4;
  double sidepass_distance = 2.8;
  int navigator_sample_num_each_level = 3;
};

}  // namespace forproto
}  // namespace hqplanner

#endif