#ifndef HQPLANNER_PATH_OBSTACLE_H_
#define HQPLANNER_PATH_OBSTACLE_H_

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include "hqplanner/common/obstacle.h"
#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/perception_obstacle.h"
#include "hqplanner/for_proto/sl_boundary.h"
#include "hqplanner/for_proto/vehicle_config.h"
#include "hqplanner/for_proto/vehicle_config_helper.h"
#include "hqplanner/math/vec2d.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/speed/st_boundary.h"
#include "hqplanner/speed/st_point.h"

namespace hqplanner {

class PathObstacle {
 public:
  PathObstacle() = default;
  explicit PathObstacle(const Obstacle* obstacle);

  const std::string& Id() const;

  const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const hqplanner::forproto::ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const hqplanner::forproto::ObjectDecisionType& LongitudinalDecision() const;

  const hqplanner::forproto::SLBoundary& PerceptionSLBoundary() const;

  const hqplanner::speed::StBoundary& reference_line_st_boundary() const;

  const hqplanner::speed::StBoundary& st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<hqplanner::forproto::ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(
      const std::string& decider_tag,
      const hqplanner::forproto::ObjectDecisionType& decision);

  void AddLateralDecision(
      const std::string& decider_tag,
      const hqplanner::forproto::ObjectDecisionType& decision);

  bool HasLateralDecision() const;

  void SetStBoundary(const hqplanner::speed::StBoundary& boundary);

  void SetStBoundaryType(const hqplanner::speed::StBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const hqplanner::speed::StBoundary& boundary);

  void SetReferenceLineStBoundaryType(
      const hqplanner::speed::StBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(ReferenceLine& reference_line,
                                    const double adc_start_s);

  void SetPerceptionSlBoundary(
      const hqplanner::forproto::SLBoundary& sl_boundary);

  /**
   * @brief check if a ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(
      const hqplanner::forproto::ObjectDecisionType& decision);

  /**
   * @brief check if a ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(
      const hqplanner::forproto::ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }
  double MinRadiusStopDistance(
      const hqplanner::forproto::VehicleParam& vehicle_param) const;

 private:
  static hqplanner::forproto::ObjectDecisionType MergeLongitudinalDecision(
      const hqplanner::forproto::ObjectDecisionType& lhs,
      const hqplanner::forproto::ObjectDecisionType& rhs);
  static hqplanner::forproto::ObjectDecisionType MergeLateralDecision(
      const hqplanner::forproto::ObjectDecisionType& lhs,
      const hqplanner::forproto::ObjectDecisionType& rhs);
  bool BuildTrajectoryStBoundary(
      ReferenceLine& reference_line, const double adc_start_s,
      hqplanner::speed::StBoundary* const st_boundary);
  bool IsValidObstacle(
      const hqplanner::forproto::PerceptionObstacle& perception_obstacle);
  std::string id_;
  const Obstacle* obstacle_ = nullptr;
  std::vector<hqplanner::forproto::ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;

  hqplanner::forproto::SLBoundary perception_sl_boundary_;
  hqplanner::speed::StBoundary reference_line_st_boundary_;
  hqplanner::speed::StBoundary st_boundary_;

  hqplanner::forproto::ObjectDecisionType lateral_decision_;
  hqplanner::forproto::ObjectDecisionType longitudinal_decision_;

  bool is_blocking_obstacle_ = false;
  double min_radius_stop_distance_ = -1.0;
};

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_
