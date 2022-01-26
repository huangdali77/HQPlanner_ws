#ifndef HQPLANNER_FORPROTO_VEHICLE_STATE_H_
#define HQPLANNER_FORPROTO_VEHICLE_STATE_H_
namespace hqplanner {
namespace forproto {

struct VehicleState {
 public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double timestamp = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double heading = 0.0;
  double kappa = 0.0;
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  double linear_acceleration = 0.0;
};
}  // namespace forproto
}  // namespace hqplanner

#endif