#ifndef HQPLANNER_FORPROTO_VEHICLE_STATE_PROVIDER_H_
#define HQPLANNER_FORPROTO_VEHICLE_STATE_PROVIDER_H_

#include <memory>
#include <string>

#include "hqplanner/for_proto/vehicle_state.h"
#include "hqplanner/math/box2d.h"
#include "hqplanner/math/vec2d.h"
#include "hqplanner/util/macro.h"

namespace hqplanner {
namespace forproto {

/**
 * @class VehicleStateProvider
 * @brief The class of vehicle state.
 *        It includes basic information and computation
 *        about the state of the vehicle.
 */
class VehicleStateProvider {
 public:
  VehicleStateProvider();
  void UpdateNextCycleVehicleState(const VehicleState& vehicle_state);
  void Init(const VehicleState& vehicle_state);
  double timestamp() const;

  // const localization::Pose& pose() const;
  // const localization::Pose& original_pose() const;

  /**
   * @brief Default destructor.
   */
  virtual ~VehicleStateProvider() = default;

  /**
   * @brief Get the x-coordinate of vehicle position.
   * @return The x-coordinate of vehicle position.
   */
  double x() const;

  /**
   * @brief Get the y-coordinate of vehicle position.
   * @return The y-coordinate of vehicle position.
   */
  double y() const;

  /**
   * @brief Get the z coordinate of vehicle position.
   * @return The z coordinate of vehicle position.
   */
  double z() const;

  double kappa() const;

  /**
   * @brief Get the vehicle roll angle.
   * @return The euler roll angle.
   */
  double roll() const;

  /**
   * @brief Get the vehicle pitch angle.
   * @return The euler pitch angle.
   */
  double pitch() const;

  /**
   * @brief Get the vehicle yaw angle.
   *  As of now, use the heading instead of yaw angle.
   *  Heading angle with East as zero, yaw angle has North as zero
   * @return The euler yaw angle.
   */
  double yaw() const;

  /**
   * @brief Get the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @return The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  double heading() const;

  /**
   * @brief Get the vehicle's linear velocity.
   * @return The vehicle's linear velocity.
   */
  double linear_velocity() const;

  /**
   * @brief Get the vehicle's angular velocity.
   * @return The vehicle's angular velocity.
   */
  double angular_velocity() const;

  /**
   * @brief Get the vehicle's linear acceleration.
   * @return The vehicle's linear acceleration.
   */
  double linear_acceleration() const;

  // /**
  //  * @brief Get the vehicle's gear position.
  //  * @return The vehicle's gear position.
  //  */
  // double gear() const;

  /**
   * @brief Set the vehicle's linear velocity.
   * @param linear_velocity The value to set the vehicle's linear velocity.
   */
  void set_linear_velocity(const double linear_velocity);

  /**
   * @brief Estimate future position from current position and heading,
   *        along a period of time, by constant linear velocity,
   *        linear acceleration, angular velocity.
   * @param t The length of time period.
   * @return The estimated future position in time t.
   */
  math::Vec2d EstimateFuturePosition(const double t) const;

  const VehicleState& vehicle_state() const;

  void set_vehicle_config(const double x, const double y, const double heading);

 private:
  VehicleState vehicle_state_;
  // localization::LocalizationEstimate original_localization_;

  DECLARE_SINGLETON(VehicleStateProvider);
};

}  // namespace forproto
}  // namespace hqplanner

#endif  // MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_
