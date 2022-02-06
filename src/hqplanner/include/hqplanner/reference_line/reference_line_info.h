#ifndef HQPLANNER_REFERENCE_LINE_INFO_H_
#define HQPLANNER_REFERENCE_LINE_INFO_H_

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/sl_boundary.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/for_proto/vehicle_state.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/speed/speed_data.h"
#include "hqplanner/trajectory/discretized_trajectory.h"

namespace hqplanner {

class ReferenceLineInfo {
  /* data */
 public:
  explicit ReferenceLineInfo(
      const hqplanner::forproto::VehicleState& vehicle_state,
      const hqplanner::forproto::TrajectoryPoint& adc_planning_point,
      const ReferenceLine& reference_line);

  bool Init(const std::vector<const Obstacle*>& obstacles);

  bool IsInited() const;

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  PathObstacle* AddObstacle(const Obstacle* obstacle);

  PathDecision* path_decision();
  const PathDecision& path_decision() const;
  const ReferenceLine& reference_line() const;
  const hqplanner::forproto::TrajectoryPoint& AdcPlanningPoint() const;

  bool ReachedDestination() const;

  void SetTrajectory(
      const hqplanner::trajectory::DiscretizedTrajectory& trajectory);

  const hqplanner::trajectory::DiscretizedTrajectory& trajectory() const;
  double TrajectoryLength() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  // For lattice planner'speed planning target
  // void SetStopPoint(const StopPoint& stop_point);
  void SetCruiseSpeed(double speed);
  // const PlanningTarget& planning_target() const { return planning_target_; }

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  // planning_internal::Debug* mutable_debug() { return &debug_; }
  // const planning_internal::Debug& debug() const { return debug_; }
  // LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  // const LatencyStats& latency_stats() const { return latency_stats_; }

  const hqplanner::path::PathData& path_data() const;
  const hqplanner::speed::SpeedData& speed_data() const;
  hqplanner::path::PathData* mutable_path_data();
  hqplanner::speed::SpeedData* mutable_speed_data();
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      hqplanner::trajectory::DiscretizedTrajectory* discretized_trajectory);

  const hqplanner::forproto::SLBoundary& AdcSlBoundary() const;
  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const { return false; };

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool IsNeighborLanePath() const;

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  // void ExportEngageAdvice(EngageAdvice* engage_advice) const;

  bool IsSafeToChangeLane() const { return is_safe_to_change_lane_; }

  // const hdmap::RouteSegments& Lanes() const;
  // const std::list<hdmap::Id> TargetLaneId() const;

  // void ExportDecision(DecisionResult* decision_result) const;

  // void SetJunctionRightOfWay(double junction_s, bool is_protected);

  // ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  // bool IsRightTurnPath() const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }
  bool IsUnrelaventObstacle(PathObstacle* path_obstacle);

 private:
  const hqplanner::forproto::VehicleState vehicle_state_;
  const hqplanner::forproto::TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_;

  double cost_ = 0.0;

  bool is_inited_ = false;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  hqplanner::path::PathData path_data_;
  hqplanner::speed::SpeedData speed_data_;

  hqplanner::trajectory::DiscretizedTrajectory discretized_trajectory_;

  hqplanner::forproto::SLBoundary adc_sl_boundary_;

  // planning_internal::Debug debug_;
  // LatencyStats latency_stats_;

  // hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = true;

  bool is_safe_to_change_lane_ = false;

  // ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;
};

}  // namespace hqplanner

#endif