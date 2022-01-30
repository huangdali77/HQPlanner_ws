#ifndef HQPLANNING_MAIN_PLANNING_H_
#define HQPLANNING_MAIN_PLANNING_H_

// #include <ros/ros.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>
// #include "modules/common/proto/pnc_point.pb.h"
#include "hqplanner/for_proto/pnc_point.h"

// #include "modules/planning/proto/planning.pb.h"
#include "hqplanner/for_proto/adc_trajectory.h"
// #include "modules/planning/proto/planning_config.pb.h"
#include "hqplanner/for_proto/planning_config.h"
// #include "modules/planning/proto/traffic_rule_config.pb.h"

// #include "modules/common/adapters/adapter_manager.h"
// #include "modules/common/apollo_app.h"
// #include "modules/common/status/status.h"
// #include "modules/common/util/factory.h"
// #include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "hqplanner/for_proto/vehicle_state_provider.h"
// #include "modules/planning/common/frame.h"
#include "hqplanner/common/frame.h"
// #include "modules/planning/common/trajectory/publishable_trajectory.h"
// #include "modules/planning/planner/planner.h"
#include "hqplanner/reference_line/reference_line_provider.h"
#include "hqplanner/trajectory/publishable_trajectory.h"

namespace hqplanner {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class Planning {
 public:
  Planning() = default;
  virtual ~Planning();
  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init();

  /**
   * @brief module start function
   * @return start status
   */
  bool Start();

  /**
   * @brief module stop function
   */
  void Stop();

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce();

  /**
   * @brief record last planning trajectory
   */
  void SetLastPublishableTrajectory(
      const hqplanner::forproto::ADCTrajectory& adc_trajectory);

 private:
  // Watch dog timer
  //   void OnTimer(const ros::TimerEvent&);

  void PublishPlanningPb(hqplanner::forproto::ADCTrajectory* trajectory_pb,
                         double timestamp);

  /**
   * @brief Fill the header and publish the planning message.
   */
  //   void Publish(hqplanner::forproto:: ADCTrajectory* trajectory) {
  //     using apollo::common::adapter::AdapterManager;
  //     AdapterManager::FillPlanningHeader(Name(), trajectory);
  //     AdapterManager::PublishPlanning(*trajectory);
  //   }

  void RegisterPlanners();

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  bool Plan(const double current_time_stamp,
            const std::vector<forproto::TrajectoryPoint>& stitching_trajectory,
            forproto::ADCTrajectory* trajectory);

  bool InitFrame(const uint32_t sequence_num,
                 const forproto::TrajectoryPoint& planning_start_point,
                 const double start_time,
                 const forproto::VehicleState& vehicle_state);

  bool IsVehicleStateValid(const forproto::VehicleState& vehicle_state);
  //   void ExportReferenceLineDebug(planning_internal::Debug* debug);

  void SetFallbackCruiseTrajectory(forproto::ADCTrajectory* cruise_trajectory);

  /**
   * Reset pull over mode whenever received new routing
   */
  //   void ResetPullOver(const routing::RoutingResponse& response);

  double start_time_ = 0.0;

  //   common::util::Factory<PlanningConfig::PlannerType, Planner>
  //   planner_factory_;

  //   PlanningConfig config_;

  //   TrafficRuleConfigs traffic_rule_configs_;

  //   const hdmap::HDMap* hdmap_ = nullptr;

  std::unique_ptr<Frame> frame_;

  //   std::unique_ptr<Planner> planner_;

  std::unique_ptr<hqplanner::trajectory::PublishableTrajectory>
      last_publishable_trajectory_;

  forproto::VehicleState last_vehicle_state_abs_pos_;

  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;

  // ros::Timer timer_;

  //   routing::RoutingResponse last_routing_;
};

}  // namespace hqplanner

#endif /* MODULES_PLANNING_PLANNING_H_ */
