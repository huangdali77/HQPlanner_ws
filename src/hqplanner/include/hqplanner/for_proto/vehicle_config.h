#ifndef HQPLANNER_FORPROTO_VEHICLE_CONFIG_H_
#define HQPLANNER_FORPROTO_VEHICLE_CONFIG_H_
namespace hqplanner {
namespace forproto {
struct VehicleParam {
  // Car center point is car referencefront_edge_to_centerpoint, i.e., center of
  // rear axle.
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;

  double length = 4.933;
  double width = 2.11;
  double height = 1.48;

  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.0;
  double max_deceleration = -4.0;

  // The following items are used to compute trajectory constraints in
  // planning/control/canbus, vehicle max steer angle
  double max_steer_angle = 8.20304748437;  //前轮最大转角8.2/16=0.512rad=30度
  // vehicle max steer rate; how fast can the steering wheel turn.
  //  double max_steer_angle_rate = 6.98131700798;
  // vehicle min steer rate;
  //  double min_steer_angle_rate = 14;
  // ratio between the turn of steering wheel and the turn of wheels
  double steer_ratio = 16.0;
  // the distance between the front and back wheels
  double wheel_base = 2.8448;
  // Tire effective rolling radius (vertical distance between the wheel center
  // and the ground).
  double wheel_rolling_radius = 0.335;

  // minimum differentiable vehicle speed, in m/s
  float max_abs_speed_when_stopped = 0.15;  //或0.2
};

struct VehicleConfig {
  VehicleParam vehicle_param;
};

}  // namespace forproto
}  // namespace hqplanner

#endif