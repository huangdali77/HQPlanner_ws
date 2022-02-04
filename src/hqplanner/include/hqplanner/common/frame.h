#ifndef HQPLANNER_FRAME_H_
#define HQPLANNER_FRAME_H_
// #include <fsd_common_msgs/CarState.h>
// #include <fsd_common_msgs/CarStateDt.h>
// #include <fsd_common_msgs/TrajectoryPoint.h>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "hqplanner/for_proto/adc_trajectory.h"
#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/prediction_obstacle.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_state.h"
#include "hqplanner/math/box2d.h"
#include "hqplanner/math/math_utils.h"
// #include "hqplanner/math/indexed_queue.h"
#include "hqplanner/common/obstacle.h"
#include "hqplanner/math/line_segment2d.h"
#include "hqplanner/math/vec2d.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/reference_line/reference_line_info.h"
#include "hqplanner/reference_line/reference_line_provider.h"
// #include "subscribe.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
namespace hqplanner {

class Frame {
 public:
  Frame() = default;
  explicit Frame(
      std::uint32_t sequence_num,
      const hqplanner::forproto::TrajectoryPoint &planning_start_point,
      const double start_time,
      const hqplanner::forproto::VehicleState &vehicle_state,
      ReferenceLineProvider *reference_line_provider);

  const hqplanner::forproto::TrajectoryPoint &PlanningStartPoint() const;
  bool Init();

  std::uint32_t SequenceNum() const;

  // std::string DebugString() const;

  // const PublishableTrajectory &ComputedTrajectory() const;

  // void RecordInputDebug(planning_internal::Debug *debug);

  std::list<ReferenceLineInfo> &reference_line_info();

  Obstacle *Find(const std::string &id);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacle_items() const;
  const std::unordered_map<std::string, std::unique_ptr<Obstacle>> obstacles()
      const;
  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

  bool Rerouting();

  const hqplanner::forproto::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      hqplanner::forproto::PredictionObstacles *prediction_obstacles);

  hqplanner::forproto::ADCTrajectory *mutable_trajectory() {
    return &trajectory_;
  }

  const hqplanner::forproto::ADCTrajectory &trajectory() const {
    return trajectory_;
  }

  const bool is_near_destination() const { return is_near_destination_; }

 private:
  bool CreateReferenceLineInfo();

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such
   * obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;

  /**
   * @brief create a static virtual obstacle
   */
  const Obstacle *CreateStaticVirtualObstacle(
      const std::string &id, const hqplanner::math::Box2d &box);

  void AddObstacle(const Obstacle &obstacle);

 private:
  // Subscribe subscribe_info_;
  std::uint32_t sequence_num_;
  hqplanner::forproto::TrajectoryPoint planning_start_point_;
  double start_time_;
  hqplanner::forproto::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;
  bool is_near_destination_ = false;
  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;
  hqplanner::forproto::PredictionObstacles prediction_;

  std::unordered_map<std::string, std::unique_ptr<Obstacle>> obstacles_;

  // std::unordered_map<std::string, Obstacle> obstacles_;
  // std::vector<const Obstacle *> obstacle_items_;
  hqplanner::forproto::ADCTrajectory trajectory_;  // last published trajectory

  ReferenceLineProvider *reference_line_provider_ = nullptr;
};

}  // namespace hqplanner

#endif