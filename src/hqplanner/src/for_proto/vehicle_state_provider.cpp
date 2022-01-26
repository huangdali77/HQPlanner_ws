#include "hqplanner/for_proto/vehicle_state_provider.h"

#include <cmath>

#include "eigen3/Eigen/Core"

namespace hqplanner {
namespace forproto {

VehicleStateProvider::VehicleStateProvider() {}

void VehicleStateProvider::UpdateNextCycleVehicleState(
    const VehicleState &vehicle_state) {
  vehicle_state_ = vehicle_state;
}

double VehicleStateProvider::x() const { return vehicle_state_.x; }

double VehicleStateProvider::y() const { return vehicle_state_.y; }

double VehicleStateProvider::z() const { return vehicle_state_.z; }

double VehicleStateProvider::roll() const { return vehicle_state_.roll; }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch; }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw; }

double VehicleStateProvider::heading() const { return vehicle_state_.heading; }

double VehicleStateProvider::kappa() const { return vehicle_state_.kappa; }

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity;
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity;
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration;
}

// double VehicleStateProvider::gear() const { return vehicle_state_.gear(); }

double VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp;
}

// const localization::Pose &VehicleStateProvider::pose() const {
//   return vehicle_state_.pose();
// }

// const localization::Pose &VehicleStateProvider::original_pose() const {
//   return original_localization_.pose();
// }

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.linear_velocity = linear_velocity;
}

const VehicleState &VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

void VehicleStateProvider::set_vehicle_config(const double x, const double y,
                                              const double heading) {
  vehicle_state_.x = x;
  vehicle_state_.y = y;
  vehicle_state_.heading = heading;
}

math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state_.linear_velocity;

  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity *
                      (1.0 - std::cos(vehicle_state_.angular_velocity * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity * t) * v /
                      vehicle_state_.angular_velocity;
  }

  return math::Vec2d(vec_distance[0] + vehicle_state_.x,
                     vec_distance[1] + vehicle_state_.y);
}

}  // namespace forproto
}  // namespace hqplanner
