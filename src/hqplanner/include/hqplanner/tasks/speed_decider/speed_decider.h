#ifndef HQPLANNER_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_
#define HQPLANNER_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_

#include <string>

#include "hqplanner/common/path_decision.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/dp_st_speed_config.h"
#include "hqplanner/for_proto/st_boundary_config.h"
#include "hqplanner/tasks/task.h"
namespace hqplanner {
namespace tasks {

class SpeedDecider : public Task {
 public:
  SpeedDecider();
  ~SpeedDecider() = default;

  bool Init(const hqplanner::forproto::PlanningConfig& config) override;

  bool Execute(hqplanner::Frame* frame,
               hqplanner::ReferenceLineInfo* reference_line_info) override;

 private:
  //  StPosition是st曲线相对于障碍物的位置
  enum StPosition {
    ABOVE = 1,
    BELOW = 2,
    CROSS = 3,
  };

  StPosition GetStPosition(
      const hqplanner::speed::SpeedData& speed_profile,
      const hqplanner::speed::StBoundary& st_boundary) const;
  /**
   * @brief check if the ADC should follow an obstacle by examing the
   *StBoundary of the obstacle.
   * @param boundary The boundary of the obstacle.
   * @return true if the ADC believe it should follow the obstacle, and
   *         false otherwise.
   **/
  bool CheckIsFollowByT(const hqplanner::speed::StBoundary& boundary) const;

  bool CreateStopDecision(
      const hqplanner::PathObstacle& path_obstacle,
      hqplanner::forproto::ObjectDecisionType* const stop_decision,
      double stop_distance) const;

  /**
   * @brief create follow decision based on the boundary
   **/
  bool CreateFollowDecision(
      const hqplanner::PathObstacle& path_obstacle,
      hqplanner::forproto::ObjectDecisionType* const follow_decision) const;

  /**
   * @brief create yield decision based on the boundary
   **/
  bool CreateYieldDecision(
      const hqplanner::PathObstacle& path_obstacle,
      hqplanner::forproto::ObjectDecisionType* const yield_decision) const;

  /**
   * @brief create overtake decision based on the boundary
   **/
  bool CreateOvertakeDecision(
      const hqplanner::PathObstacle& path_obstacle,
      hqplanner::forproto::ObjectDecisionType* const overtake_decision) const;

  bool MakeObjectDecision(const hqplanner::speed::SpeedData& speed_profile,
                          hqplanner::PathDecision* const path_decision) const;

  void AppendIgnoreDecision(hqplanner::PathObstacle* path_obstacle) const;

  /**
   * @brief "too close" is determined by whether ego vehicle will hit the front
   * obstacle if the obstacle drive at current speed and ego vehicle use some
   * reasonable deceleration
   **/
  bool IsFollowTooClose(const hqplanner::PathObstacle& path_obstacle) const;

 private:
  hqplanner::forproto::DpStSpeedConfig dp_st_speed_config_;
  hqplanner::forproto::StBoundaryConfig st_boundary_config_;
  hqplanner::forproto::SLBoundary adc_sl_boundary_;
  hqplanner::forproto::TrajectoryPoint init_point_;
  const hqplanner::ReferenceLine* reference_line_ = nullptr;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_
